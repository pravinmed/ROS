#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>


// Define a global client that can request services
ros::ServiceClient client;
// If already brakes are applied , do not apply again.
bool brakes_applied;

// If it was a full turn , then do not turn again give the camera a chance.
bool last_was_full_turn = false;




// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
	ROS_INFO_STREAM("Moving the robot .....");
	ball_chaser::DriveToTarget drive;
	drive.request.linear_x = lin_x;
	drive.request.angular_z = ang_z;

	if (!client.call(drive))
		ROS_ERROR("Failed to call service drive");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

	// Initialize
	int desired_pixel = 255;
	// % of side view to be taken for left and right
	float side_view = 0.15;
	bool found_sphere = false;
	
	int foundin_r, foundin_c;
        int total_pixel = img.height * img.step/3;
	
	// % of left side
	int left = (img.step)*side_view;
	// % of right side
	int right = (1 - side_view)*(img.step);

	bool in_left = false;
	bool in_right = false;
	bool in_front = false;
	float linx = 0.0;
	float angz = 0.0;
	int pix_count = 0;
	int right_pix_count = 0;
	int left_pix_count = 0;
        int width = img.step/3;
	int first_right = 99999;
	int last_left = -1;
	// Loop through all pixel and find how much is in the left,center and right
        for (int i = 0; i < img.height; i++) {
		
		for (int j = 0; j < img.step; j+=3)
		{
			if (img.data[i*img.step + j] == desired_pixel && 
			    img.data[i*img.step + j + 1] == desired_pixel &&
			    img.data[i*img.step + j + 2] == desired_pixel) {

				foundin_c = j;
				pix_count++;
				
				// check if in the left
				if (foundin_c <= left)
				{
					in_left = true;
					if (foundin_c > last_left)
					{
						last_left = foundin_c;
					}
					left_pix_count++;
				}
				// check if in the middle
				if (foundin_c > left && foundin_c < right)
				{
					in_front = true;
				}
				// check if in the right
				if (foundin_c >= right)
				{
					in_right = true;
					if (foundin_c < first_right)
					{
						first_right = foundin_c;
					}
					right_pix_count++;
				}
				
				found_sphere = true;

			}
		}
		if (in_left && in_front && in_right)
		{
			// if in all the view then it is too close ,
			// so quit and apply brake.
			found_sphere = false;		
			break;
		}
	}
	
	if (last_was_full_turn && (in_right || in_left))
	{
	   // Slow down , dont turn or else it keeps turning with 2 consecutive turns.
	   last_was_full_turn = false;
	   found_sphere = false;
	}
	
	
	// If the percentage in the middle is > 5% then STOP.
	if ( (pix_count/(total_pixel*(1-side_view)))*100.0 > 5.0 && in_front && !(in_left || in_right) )
	{
		// If too close then Apply the brakes.
		ROS_INFO_STREAM("STOP......");
		found_sphere = false;
		brakes_applied = false;
	} else if ( (pix_count/(total_pixel*(1-side_view)))*100.0 > 10.0 && in_front)
	{
		// If too close then Apply the brakes.
		ROS_INFO_STREAM("STOP IN THE FRONT ......");
		found_sphere = false;
		brakes_applied = false;
	}
	
	if (found_sphere) {
		std::stringstream info2;
		info2 << "Found in left : " << in_left << " right : " << in_right << " front: " << in_front;
		//ROS_INFO_STREAM(info2.str());
		if (in_left)
		{
			float percent_visible_left = (left_pix_count / (side_view*total_pixel))*100.0;
			float ang = 0.25 - (((0.25 -0.03)*last_left)/left);	
			
			std::stringstream info_left;
			info_left << "Percent in Left % : " << percent_visible_left << " Angle "<<ang<< " last_left : "<<last_left<< " left : "<<left;
			//ROS_INFO_STREAM(info_left.str());

			linx = 0.0;
			angz = ang;  							
			if (percent_visible_left > 50)
			{
				// too close 
				angz = 0.0;
			}

		}
		else if (in_right)
		{
			float percent_visible_right = (right_pix_count / (side_view*total_pixel))*100.0;
			float ang = 0.25 - ((0.25 -0.03)*(img.step -first_right)/(img.step -right));
			
			std::stringstream info_right;
			info_right << "Percent in Right % : " << percent_visible_right << "Angle :"<<ang<< "first_right : "<<first_right<< " right : "<<right;
			//ROS_INFO_STREAM(info_right.str());
			// find the angle and rotate that much % of angle
			linx = 0.0;
			angz = -ang;					
			if (percent_visible_right > 50) {
			    // too close
			    angz = 0.0;
			}
		}	
		else {
			linx = 0.05;
			angz = 0.0;
	
		}
		if (std::abs(angz) >= 0.1)
		{
		      // Too much turn , slow down in next cycle.
	    	      last_was_full_turn = true;
		}	
	}
		

	if (found_sphere == true)
	{
		std::stringstream info;
		info << "Moving the robot with linx " << linx << " ang " << angz;
		ROS_INFO_STREAM(info.str());
		brakes_applied = false;		
		drive_robot(linx, angz);
	}
	else {
		
		if (!brakes_applied) {
			ROS_INFO_STREAM(" Applying the brakes ...");
			drive_robot(0, 0);
		}
		brakes_applied = true;
	}
}

int main(int argc, char** argv)
{
	// Initialize the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	brakes_applied = false;
	// Define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	// Handle ROS communication events
	ros::spin();

	return 0;
}
