#include "ros/ros.h"

#include "tangible_msgs/GetMovements.h"
#include "moveit_msgs/Grasp.h"

// run this with rosservice call /control_arm 0 or rosservice call /control_arm 1

int main (int argc, char** argv)
{
	ros::init(argc, argv, "arm_motion_test");

	ros::NodeHandle node;
	ros::ServiceClient client = node.serviceClient<tangible_msgs::GetMovements>("move_arm");
	

	tangible_msgs::GetMovements move_srv;

	move_srv.request.type = tangible_msgs::GetMovements::Request::PICK;

	moveit_msgs::Grasp grasp_move;
	grasp_move.grasp_pose.pose.position.x = 0.559;
	grasp_move.grasp_pose.pose.position.y = 0.027;
	grasp_move.grasp_pose.pose.position.z = 1.031;
	grasp_move.grasp_pose.pose.orientation.x = -0.073;
	grasp_move.grasp_pose.pose.orientation.y = 0.621;
	grasp_move.grasp_pose.pose.orientation.z = 0.034;
	grasp_move.grasp_pose.pose.orientation.w = 0.779;

	grasp_move.pre_grasp_approach.direction.vector.x = 0;
	grasp_move.pre_grasp_approach.direction.vector.y = 0;
	grasp_move.pre_grasp_approach.direction.vector.z = -1;
	grasp_move.pre_grasp_approach.desired_distance = 0.1;

	grasp_move.post_grasp_retreat.direction.vector.x = 0;
	grasp_move.post_grasp_retreat.direction.vector.y = 0;
	grasp_move.post_grasp_retreat.direction.vector.z = 1;
	grasp_move.post_grasp_retreat.desired_distance = 0.1;

	move_srv.request.poses.push_back(grasp_move);

	bool success = false;

	ros::Rate loop_rate(10);
	do
	{
		if (client.call(move_srv))
		{
			ROS_INFO("motion was %s", move_srv.response.status ? "SUCCESSFUL" : "UNSUCCESSFUL");
			success = move_srv.response.status;
		}
		else
			ROS_ERROR("failed to call service move_arm");
		loop_rate.sleep();
	} while (!success);

	return 0;
}