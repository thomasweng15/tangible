#include "tangible/release_test.h"


namespace tangible {

ReleaseTest::ReleaseTest(ros::NodeHandle& n, std::string scene_service_name, std::string release_service_name){
  node_handle = n;
  // marker_pub = node_handle.advertise<visualization_msgs::Marker>("grasp_markers", 20);
  scene_client = n.serviceClient<tangible_msgs::GetScene>(scene_service_name);
  release_client = n.serviceClient<tangible_msgs::GetReleases>(release_service_name);

}

ReleaseTest::~ReleaseTest() {}

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
      release_client.call(release_srv);
      if (release_srv.response.releases.size() > 0){
        ROS_INFO("Got releases");
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
          point.point.y = 0.70;
          point.point.z = obj.bounding_box.pose.pose.position.z - obj.bounding_box.dimensions.z/2.0;

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


        }

        release_srv.request.target = target;
        release_srv.request.object = obj;
        release_client.call(release_srv);
        if (release_srv.response.releases.size() > 0){
          ROS_INFO("Got releases");
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
