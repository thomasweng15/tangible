#include "tangible/arm_motion.h"

namespace tangible
{

ArmMotion::ArmMotion(ros::NodeHandle& n) : right_arm("right_arm"), right_gripper('r')
{
	node_handle = n;
	srv_move = node_handle.advertiseService("move_arm", &ArmMotion::move_callback, this);
	srv_stop = node_handle.advertiseService("stop_arm", &ArmMotion::stop_callback, this);

	status = SUCCESSUL;
	stopped = false;
}

ArmMotion::~ArmMotion() {}

bool ArmMotion::move_callback(tangible_msgs::GetMovements::Request& req, tangible_msgs::GetMovements::Response& res)
{
	ROS_INFO("planning motion");
	stopped = false;
	status = UNSUCCESSFUL;

	moveit::planning_interface::MoveGroup::Plan plan;

	for(int i = 0; i < req.poses.size(); i++)
	{
		
		if(stopped)
			break;

		geometry_msgs::Pose grasp, pre_grasp, post_grasp;
		
		grasp = req.poses[i].grasp_pose.pose;
		
		pre_grasp.orientation = grasp.orientation;
		pre_grasp.position = get_relative_point(grasp.position, 
												req.poses[i].pre_grasp_approach.direction, 
												req.poses[i].pre_grasp_approach.desired_distance);
		
		post_grasp.orientation = grasp.orientation;
		post_grasp.position = get_relative_point(grasp.position, 
												req.poses[i].pre_grasp_approach.direction, 
												req.poses[i].pre_grasp_approach.desired_distance);
		
		right_arm.setPoseTarget(pre_grasp);
		if(!right_arm.plan(plan))
		{
			ROS_INFO("no plan to pre grasp");
			continue;
		}

		right_arm.setPoseTarget(grasp);
		if(!right_arm.plan(plan))
		{
			ROS_INFO("no plan to pre grasp");
			continue;
		}

		right_arm.setPoseTarget(post_grasp);
		if(!right_arm.plan(plan))
		{
			ROS_INFO("no plan to pre grasp");
			continue;
		}

		if(stopped)
			break;

		right_arm.setPoseTarget(pre_grasp);
		right_arm.move();

		if(stopped)
			break;

		if(req.type == tangible_msgs::GetMovements::Request::PICK)
			right_gripper.open();
		// TO-DO handle the case when oppening gripper is not successful

		if(stopped)
			break;

		right_arm.setPoseTarget(grasp);
		right_arm.move();

		if(stopped)
			break;

		if(req.type == tangible_msgs::GetMovements::Request::PICK)
			right_gripper.close();
		else if(req.type == tangible_msgs::GetMovements::Request::RELEASE)
			right_gripper.open();
		// TO-DO handle the case when gripping (pick) or openning gripper (release) is not successful

		if(stopped)
			break;

		right_arm.setPoseTarget(post_grasp);
		right_arm.move();

		// NOTE: no need to check again whether moving to post_grasp was successful

		status = SUCCESSUL;
		break;

	}

	res.status = status;
	return true;
}

geometry_msgs::Point ArmMotion::get_relative_point(geometry_msgs::Point org, geometry_msgs::Vector3Stamped vec, double mag)
{
	geometry_msgs::Point relative_point;
	relative_point.x = org.x + vec.vector.x * mag;
	relative_point.y = org.y + vec.vector.y * mag;
	relative_point.z = org.z + vec.vector.z * mag;
	return relative_point;
}

bool ArmMotion::stop_callback(tangible_msgs::StopMovements::Request& req, tangible_msgs::StopMovements::Response& res)
{
	ROS_INFO("stopping all movements");
	stopped = true;
	right_arm.stop();
	return true;
}

}