#include "ros/ros.h"

#include "tangible_msgs/GetMovements.h"
#include "moveit_msgs/Grasp.h"

int main (int argc, char** argv)
{
	ros::init(argc, argv, "arm_motion_test");

	ros::NodeHandle node;
	ros::ServiceClient client = node.serviceClient<tangible_msgs::GetMovements>("move_arm");
	

	tangible_msgs::GetMovements move_srv;

	move_srv.request.type = tangible_msgs::GetMovements::Request::PICK;

	moveit_msgs::Grasp grasp_move;
	grasp_move.grasp_pose.pose.position.x = 0.553;
	grasp_move.grasp_pose.pose.position.y = -0.126;
	grasp_move.grasp_pose.pose.position.z = 1.106;
	grasp_move.grasp_pose.pose.orientation.x = 0.747;
	grasp_move.grasp_pose.pose.orientation.y = -0.066;
	grasp_move.grasp_pose.pose.orientation.z = -0.638;
	grasp_move.grasp_pose.pose.orientation.w = -0.638;

	grasp_move.pre_grasp_approach.direction.vector.x = 0;
	grasp_move.pre_grasp_approach.direction.vector.y = 0;
	grasp_move.pre_grasp_approach.direction.vector.z = -1;
	grasp_move.pre_grasp_approach.desired_distance = 0.03;

	grasp_move.post_grasp_retreat.direction.vector.x = 0;
	grasp_move.post_grasp_retreat.direction.vector.y = 0;
	grasp_move.post_grasp_retreat.direction.vector.z = 1;
	grasp_move.post_grasp_retreat.desired_distance = 0.03;

	move_srv.request.poses.push_back(grasp_move);

	if(client.call(move_srv))
	{
		ROS_INFO("motion was %s", move_srv.response.status ? "SUCCESSFUL" : "UNSUCCESSFUL");
	}
	else
	{
		ROS_ERROR("failed to call service move_arm");
	}

	return 0;
}