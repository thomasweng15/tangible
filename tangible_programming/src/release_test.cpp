#include "tangible/release_test.h"
#include "tangible/gripper_marker.h"

namespace tangible {
float random(float low, float high){
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = high - low;
  float r = random * diff;
  return low + r;

}
ReleaseTest::ReleaseTest(ros::NodeHandle& n, std::string scene_service_name, std::string release_service_name){
  node_handle = n;
  marker_pub = node_handle.advertise<visualization_msgs::Marker>("release_markers", 100);
  scene_client = n.serviceClient<tangible_msgs::GetScene>(scene_service_name);
  release_client = n.serviceClient<tangible_msgs::GetReleases>(release_service_name);

}

ReleaseTest::~ReleaseTest() {}

void ReleaseTest::publishMarkers(std::vector<moveit_msgs::Grasp> releases){
  for (int i=0; i < releases.size(); i++){
    moveit_msgs::Grasp grasp = releases[i];
    GripperMarker gripper_marker = GripperMarker(node_handle);
    std_msgs::ColorRGBA colour;
    colour.r = random(0.0, 1.0);
    colour.g = random(0.0, 1.0);
    colour.b = random(0.0, 1.0);
    colour.a = 0.5;
    std::ostringstream oss;
    oss << "main_grasp_" <<0 + 15*i;
    std::vector<visualization_msgs::Marker> grasp_markers = gripper_marker.generateMarkerWithColour(0 + 15*i, grasp.grasp_pose, colour, "grasp_pose_frame", oss.str());
    /*geometry_msgs::PoseStamped pre_grasp_pose = poseFromVec(grasp.grasp_pose, grasp.pre_grasp_approach.direction.vector, grasp.pre_grasp_approach.desired_distance);
    std::ostringstream oss1;
    oss1 << "pre_grasp_" << 5 + 15*i;
    std::vector<visualization_msgs::Marker> pre_grasp_markers = gripper_marker.generateMarkerWithColour(5 + 15*i, pre_grasp_pose, colour, "pre_grasp_pose_frame",oss1.str());
    geometry_msgs::PoseStamped post_grasp_pose = poseFromVec(grasp.grasp_pose, grasp.post_grasp_retreat.direction.vector, grasp.post_grasp_retreat.desired_distance);    
    std::ostringstream oss2;
    oss2 << "post_grasp_" << 10 + 15*i;
    std::vector<visualization_msgs::Marker> post_grasp_markers = gripper_marker.generateMarkerWithColour(10 + 15*i, post_grasp_pose, colour, "post_grasp_pose_frame",  oss2.str());
    //ROS_INFO("publishing grasp markers");
    */
    ros::Duration(3.0).sleep();
    tf::TransformListener listener;
    listener.waitForTransform("base_footprint", "grasp_pose_frame",
                              ros::Time::now(), ros::Duration(5.0)); 
    visualization_msgs::Marker marker;
    marker.action = 3;
    marker_pub.publish(marker);

    //listener.waitForTransform("base_footprint", "pre_grasp_pose_frame",
    //                          ros::Time::now(), ros::Duration(5.0)); 
    //listener.waitForTransform("base_footprint", "post_grasp_pose_frame",
    //                          ros::Time::now(), ros::Duration(5.0)); 
    for(int j=0; j < grasp_markers.size(); j++){
      marker_pub.publish(grasp_markers[j]);
      //ROS_INFO("%d", grasp_markers[j].id);
    }
    ros::Duration(3.0).sleep();
        /*
    for(int j=0; j < pre_grasp_markers.size(); j++){
      marker_pub.publish(pre_grasp_markers[j]);
      ROS_INFO("%d", pre_grasp_markers[j].id);
    }
    ros::Duration(1.0).sleep();
    marker_pub.publish(marker);
    for(int j=0; j < post_grasp_markers.size(); j++){
      marker_pub.publish(post_grasp_markers[j]);
      ROS_INFO("%d", post_grasp_markers[j].id);
    }
    ros::Duration(1.0).sleep();
    marker_pub.publish(marker);
    */
  }
}
bool ReleaseTest::testCallback(tangible_msgs::ReleaseTest::Request& req,
                 tangible_msgs::ReleaseTest::Response& res)
{
  ROS_INFO("Got test service call, calling scene service");
  tangible_msgs::GetScene srv;
  scene_client.call(srv);
  ROS_INFO("Got scene");

  if (req.type == tangible_msgs::ReleaseTest::Request::OBJECT_SELECTOR){
    ROS_INFO("Object selector type");
    if (srv.response.scene.objects.size() > 1){
      ROS_INFO("Found at least 2 objects in scene");
      tangible_msgs::GetReleases release_srv;
      tangible_msgs::Target target;
      release_srv.request.type = tangible_msgs::GetReleases::Request::PLACE;
      tangible_msgs::SceneObject obj = srv.response.scene.objects[0];
      target.selected_object = srv.response.scene.objects[1];
      target.type = tangible_msgs::Target::OBJECT_SELECTOR;
      release_srv.request.target = target;
      release_srv.request.object = obj;
      release_srv.request.num_orientations = 4;
      release_client.call(release_srv);
      if (release_srv.response.releases.size() > 0){
        ROS_INFO("Got releases");
          publishMarkers(release_srv.response.releases);
      }
      else {
        ROS_INFO("No releases found");
      }
    }
    else {
      ROS_INFO("Not enough objects found in scene");
    }
  }
  else {

      if (srv.response.scene.objects.size() > 0){
        tangible_msgs::GetReleases release_srv;
      tangible_msgs::SceneObject obj = srv.response.scene.objects[0];
      release_srv.request.type = tangible_msgs::GetReleases::Request::PLACE;
      tangible_msgs::Target target;


        if (req.type == tangible_msgs::ReleaseTest::Request::POINT_LOCATION){
          target.type = tangible_msgs::Target::POINT_LOCATION;
          geometry_msgs::PointStamped point;
          point.header.frame_id = "base_footprint";
          point.point.x = 0.70;
          point.point.y = -0.70;
          point.point.z = obj.bounding_box.pose.pose.position.z - obj.bounding_box.dimensions.z/2.0;

          release_srv.request.num_orientations = 4;
          target.specified_point = point;
        }

        else if (req.type == tangible_msgs::ReleaseTest::Request::REGION){
          target.type = tangible_msgs::Target::REGION;
          geometry_msgs::PointStamped point;
          point.header.frame_id = "base_footprint";
          point.point.x = 0.70;
          point.point.y = 0.70;
          point.point.z = obj.bounding_box.pose.pose.position.z - obj.bounding_box.dimensions.z/2.0;
          target.region_corners.push_back(point);
          point.point.x += 0.20;
          target.region_corners.push_back(point);
          point.point.y -= 0.20;
          target.region_corners.push_back(point);
          point.point.x -= 0.20;
          target.region_corners.push_back(point);

          release_srv.request.num_orientations = 4;
          release_srv.request.region_sample_spacing = 0.05;
        }

        release_srv.request.target = target;
        release_srv.request.object = obj;
        release_client.call(release_srv);
        if (release_srv.response.releases.size() > 0){
          ROS_INFO("Got %d releases", (int) release_srv.response.releases.size());
          for (int j =0; j < release_srv.response.releases.size(); j ++){
            ROS_INFO("Release orientation: %f, %f, %f, %f", release_srv.response.releases[j].grasp_pose.pose.orientation.x, 
                                                 release_srv.response.releases[j].grasp_pose.pose.orientation.y,
                                                 release_srv.response.releases[j].grasp_pose.pose.orientation.z,
                                                 release_srv.response.releases[j].grasp_pose.pose.orientation.w);
          }
          publishMarkers(release_srv.response.releases);
        }
        else {
          ROS_INFO("No releases found");
        }

      // if (srv.response.scene.objects.size() > 0){
      //   ROS_INFO("Objects found in scene, calling grasp service");
      //   tangible_msgs::SceneObject obj = srv.response.scene.objects[0];
      //   tangible_msgs::GetGrasps grasp_srv;
      //   grasp_srv.request.object = obj;
      //   grasp_client.call(grasp_srv);
      //   if (grasp_srv.response.grasps.size() > 0) {
      //     ROS_INFO("Got grasps");
      //   }
      //   else {
      //     ROS_WARN("No grasps found");
      //   }

      // }
      // else{
      //   ROS_INFO("No objects found in scene");

      // }
    }
    else {
      ROS_INFO("No objects found in scene");
    }
  }
}

}
