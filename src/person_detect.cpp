#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

float avg_image_p_time = 0;
float counter = 1;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
	for (int i = 0; i < msg->bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());
	};
	ros::Duration t = msg->header.stamp - msg->image_header.stamp;
	double avg_t = t.toSec();
	double sum = counter * avg_image_p_time + avg_t;
	counter++;
	avg_image_p_time = sum / counter;
}

float getAvgImageProcessingTime()
{
	return avg_image_p_time;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detection_sub");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);

	ros::spin();

	return 0;
}
