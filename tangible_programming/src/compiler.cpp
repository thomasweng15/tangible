#include "tangible/compiler.h"

namespace tangible {

bool blockLessThan(const tangible_msgs::Block &a, const tangible_msgs::Block &b){
  return a.id < b.id;
}

Compiler::Compiler(ros::NodeHandle& n, int i_id, int r_id, int e_id, 
                  std::string block_service_name,
                  std::string scene_service_name) {
  node_handle = n;
  edit_id = e_id;
  run_id = r_id;
  idle_id = i_id;
  edit_state = false;
  run_state = false;
  idle_state = true;
  block_client = node_handle.serviceClient<tangible_msgs::GetBlocks>(block_service_name);
  scene_client = node_handle.serviceClient<tangible_msgs::GetScene>(scene_service_name);
  box_marker_pub = node_handle.advertise<visualization_msgs::Marker>("region_markers", 20);
  cloud_marker_pub = node_handle.advertise<sensor_msgs::PointCloud2>("selected_cloud_markers", 20);
}

Compiler::~Compiler() {}

void Compiler::modeCallback(const tangible_msgs::Mode::ConstPtr msg) {
  if (msg->mode == tangible_msgs::Mode::IDLE){
    idle_state = true;
    run_state = false;
    edit_state = false;
  }
  else if (msg->mode == tangible_msgs::Mode::EXECUTE){
    idle_state = false;
    run_state = true;
    edit_state = false;
  }
  else if (msg->mode == tangible_msgs::Mode::EDIT){
    idle_state = false;
    run_state = false;
    edit_state = true;
  }
}

bool Compiler::programCallback(tangible_msgs::GetProgram::Request& req,
                 tangible_msgs::GetProgram::Response& res)
{
    // TODO(sarah): return error if called when editing
    ROS_INFO("Got service call!");
    if (edit_state != true) {
  	 res.program = working_program;
    }
  	return true;
}

void Compiler::compile(){
  ros::Rate loop_rate(10);

  while (ros::ok()){
    if (edit_state == true) {
      tangible_msgs::GetScene scene_srv;
      if (scene_client.call(scene_srv))
      {
        //scene_srv.response;
      }
      else
      {
        ROS_ERROR("Failed to call scene service");
        continue;
      }

      tangible_msgs::GetBlocks block_srv;
      if (block_client.call(block_srv))
      {
        //block_srv.response;
      }
      else
      {
        ROS_ERROR("Failed to call block service");
        continue;
      }

      tangible_msgs::Scene scene = scene_srv.response.scene;
      std::vector<tangible_msgs::Block> blocks = block_srv.response.blocks;

      if (tags2program(blocks)){
        if (addObjects(scene)){
          working_program = program;
          publishMarkers();
          ROS_INFO("Published markers");
        }
      }
      
    }

    loop_rate.sleep();
  }
}

void Compiler::publishMarkers(){
  for(int i = 0; i < program.operations.size(); i++) {
    for (int j = 0; j < program.operations[i].instructions.size(); j++) {
      if (program.operations[i].instructions[j].type == tangible_msgs::Instruction::PICK){
        sensor_msgs::PointCloud2 pc2 = program.operations[i].instructions[j].target.selected_object.point_cloud;
          cloud_marker_pub.publish(pc2);
          ROS_INFO("CLoud size: %d", pc2.width*pc2.height);
          ROS_INFO("Published cloud");

      }
      else if (program.operations[i].instructions[j].type == tangible_msgs::Instruction::DROP
                || program.operations[i].instructions[j].type == tangible_msgs::Instruction::PLACE){
          ROS_INFO("It's a release");

        if (program.operations[i].instructions[j].target.type == tangible_msgs::Target::REGION){
          ROS_INFO("It's a region");
            if (program.operations[i].instructions[j].target.region_corners.size() == 4){
            publishCorner(program.operations[i].instructions[j].target.region_corners[0], 10);
            publishCorner(program.operations[i].instructions[j].target.region_corners[1], 11);
            publishCorner(program.operations[i].instructions[j].target.region_corners[2], 12);
            publishCorner(program.operations[i].instructions[j].target.region_corners[3], 13);
            ROS_INFO("Published corners");
          }

        }
        else if (program.operations[i].instructions[j].target.type == tangible_msgs::Target::POINT_LOCATION){
          publishCorner(program.operations[i].instructions[j].target.specified_point, 10);
        }

      }


    }
  }
}

void Compiler::publishCorner(geometry_msgs::PointStamped point, int id){
          visualization_msgs::Marker marker;
          // Set the frame ID and timestamp.  See the TF tutorials for information on these.
          marker.header.frame_id = "base_footprint";
          marker.header.stamp = ros::Time::now();

          // Set the namespace and id for this marker.  This serves to create a unique ID
          // Any marker sent with the same namespace and id will overwrite the old one
          marker.ns = "region";
          marker.id = id;
          uint32_t shape = visualization_msgs::Marker::CUBE;

          // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
          marker.type = shape;

          // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
          marker.action = visualization_msgs::Marker::ADD;

          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
          marker.pose.position.x = point.point.x;
          marker.pose.position.y = point.point.y;
          marker.pose.position.z = point.point.z;
          marker.pose.orientation.w = 1.0;

          // Set the scale of the marker -- 1x1x1 here means 1m on a side
          marker.scale.x = 0.02;
          marker.scale.y = 0.02;
          marker.scale.z = 0.02;

          // Set the color -- be sure to set alpha to something non-zero!
          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
          marker.color.a = 1.0;

          marker.lifetime = ros::Duration();
          ROS_INFO("About to publish a corner");
          box_marker_pub.publish(marker);
}

bool Compiler::addObjects(tangible_msgs::Scene scene){
  for(int i = 0; i < program.operations.size(); i++) {
    for (int j = 0; j < program.operations[i].instructions.size(); j++) {
      if (!addObject(scene, &program.operations[i].instructions[j])){
        return false;
      }
    }
  }
  return true;
}

void Compiler::setupFilterBox(pcl::CropBox<pcl::PointXYZRGB>& cbox, tangible_msgs::Instruction* instruction) {
  //std::cout << "selection at (" << selection.getCenter().x << ", "
  //                              << selection.getCenter().y << ", "
  //                              << selection.getCenter().z << ")\n";

  Eigen::Vector4f min_ (-1, -1, 0, 0);
  Eigen::Vector4f max_ ( 1,  1, 1, 0);
  //NOTE the box is symmetric about the origin so half of will be in front of the arrow
  //     under all transformations
  min_ *= OBJECT_SELECTION_BOX_SIZE; min_(3) = 1;
  max_ *= OBJECT_SELECTION_BOX_SIZE; max_(3) = 1;

  cbox.setMin(min_);
  cbox.setMax(max_);

  //std::cout << "min at (" << min_(0) << ", " << min_(1) << ", " << min_(2) << ")\n";
  //std::cout << "max at (" << max_(0) << ", " << max_(1) << ", " << max_(2) << ")\n";
  
  // Eigen::Vector3d x_axis = getXvect();
  // Eigen::Vector3d y_axis = getYvect();
  // Eigen::Vector3d z_axis = getZvect();

  //std::cout << "coordinate at arrow "
  //          << "x(" << x_axis.transpose() << ") "
  //          << "y(" << y_axis.transpose() << ") "
  //          << "z(" << z_axis.transpose() << ")\n";

  // float roll =  atan2(y_axis(2), z_axis(2));
  // float pitch = asin(-x_axis(2));
  // float yaw = atan2(x_axis(1), x_axis(0));

  // Eigen::Vector3f box_rotation (roll, pitch, yaw);
  // cbox.setRotation(box_rotation);

  // Position center = selection.getCenter();
  // Eigen::Vector3f y_axis_eigen (y_axis(0), y_axis(1), y_axis(2));
  Eigen::Vector3f tip (instruction->target.specified_point.point.x, 
                instruction->target.specified_point.point.y,
                instruction->target.specified_point.point.z);
  // tip += y_axis_eigen * Tag::ARROW_SELECTION_LEN;



  //std::cout << "tip at (" << tip(0) << ", " << tip(1) << ", " << tip(2) << ")\n";
  
  cbox.setTranslation(tip);

  min_ = cbox.getMin(); max_ = cbox.getMax();
  
  //box_rotation = cbox.getRotation(); tip = cbox.getTranslation();
  //std::cout << "rotated by (" << box_rotation(0) << ", " 
  //                                   << box_rotation(1) << ", " 
  //                                   << box_rotation(2) << ")\n";
  //std::cout << "translated by (" << tip(0) << ", " << tip(1) << ", " << tip(2) << ")\n";
}

int Compiler::filterObject(pcl::CropBox<pcl::PointXYZRGB>& cbox,
                        tangible_msgs::SceneObject& obj) {
  // std::stringstream ss;

  //std::cout << "(" << obj.pose().pose.position.x << ", "
    //                 << obj.pose().pose.position.y << ", "
    //                 << obj.pose().pose.position.z << ") ";
    //ss << "object (" << obj.pose().pose.position.x << ", "
    //                 << obj.pose().pose.position.y << ", "
    //                 << obj.pose().pose.position.z << ") ";

  sensor_msgs::PointCloud2 pc2 = obj.point_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (pc2, *cloud);
  // *cloud = *obj.GetCloud();
  //NOTE make a copy of the cloud to ensure the original cloud is intact 

  // int cloud_size = cloud->points.size();

    //std::cout << "cloud size (before crop: " << cloud_size << ") ";
    //ss << "cloud size (before crop: " << cloud_size << ") ";
  
  cbox.setInputCloud(cloud);
  cbox.filter(*cloud);

  int cloud_size = cloud->points.size();

  //std::cout << "cloud size (after crop: " << cloud_size << ")";
  //ss << "cloud size (after crop: " << cloud_size << ")";

  //ROS_INFO("\nfiltering info:\n%s", ss.str().c_str());

  return cloud_size;
}

bool Compiler::addObject(tangible_msgs::Scene scene, tangible_msgs::Instruction* instruction){
  if(instruction->target.type == tangible_msgs::Target::OBJECT_SELECTOR) {  

    pcl::CropBox<pcl::PointXYZRGB> cbox;
    setupFilterBox(cbox, instruction);

    int max_overlap = -1; int max_overlap_index = -1;
    for(int i = 0; i < scene.objects.size(); i++) {
      //std::cout << "object \\" << i;
      //ss << "object \\" << i;

        int cloud_size = filterObject(cbox, scene.objects[i]);

        //ss << " filtered cloud size: " << cloud_size << " | "; 
      
      //std::cout << " pass min_overlap? (" << MIN_POINT_OVERLAP << ") " << (cloud_size >= MIN_POINT_OVERLAP);
      //ss << " pass min_overlap? (" << MIN_POINT_OVERLAP << ") " << (cloud_size >= MIN_POINT_OVERLAP);
      //std::cout << " pass max_so_far? (" << max_overlap << ") " << (cloud_size > max_overlap);
      //ss << " pass max_so_far? (" << max_overlap << ") " << (cloud_size > max_overlap);
      
      if(cloud_size >= MIN_POINT_OVERLAP &&
         cloud_size > max_overlap) {
        max_overlap = cloud_size;
        max_overlap_index = i;
        
        //std::cout << " <--- new max. ";
        //ss << " <--- new max. ";
      }

      //std::cout << "\n";
      //ss << "\n";
    }

    //ROS_INFO("\n%s", ss.str().c_str());

    if(max_overlap_index == -1) {
      error_msg = "ERROR - TAG GROUPING - no object to select.";
      ROS_ERROR_STREAM(error_msg);
      return false;

    }
    instruction->target.selected_object = scene.objects[max_overlap_index];
    // matched.push_back(objects[max_overlap_index]);
  }
  return true;
  // } else if (selection.getID() == Tag::SELECT_OBJECTS_ID){
  //   Tag selection2nd = ins.selection2nd;

  //   pcl::CropBox<pcl::PointXYZRGB> cbox;
  //   setupFilterBox(cbox, selection, selection2nd);
    
  //   for(int i = 0; i < objects.size(); i++) {
  //     //std::cout << "object \\" << i;

  //     int size_before_crop = objects[i].GetCloud()->points.size();
      
  //     int size_after_crop = filterObject(cbox, objects[i]);

  //     //std::cout << " ratio = " << (size_after_crop * 1.0 / size_before_crop);

  //     if((size_after_crop * 1.0 / size_before_crop) < MIN_REGION_OVERLAP_RATIO) {
  //       //std::cout << "\n";
  //       continue;
  //     }
  //     //TO-DO this check does not address oversized segments

  //     //std::cout << " <--- within the region\n";

  //     matched.push_back(objects[i]);
  //   }

  //   if(matched.size() == 0) {
  //     error_msg = "ERROR - TAG GROUPING - no object in region to select.";
  //     return false;
  //   }
  // }
  
}


bool Compiler::inRange(int LB, int UB, int number) {
  if(number < LB) return false;
  if(number > UB) return false;
  return true;
}

bool Compiler::inRange(double LB, double UB, double number) {
  if(number < LB) return false;
  if(number > UB) return false;
  return true;
}

Eigen::Vector3d Compiler::getXvect(const tangible_msgs::Block &a) {
  Eigen::Vector3d v;
  v << a.x_axis.x, a.x_axis.y, a.x_axis.z;
  return v;
}

Eigen::Vector3d Compiler::getYvect(const tangible_msgs::Block &a) {
  Eigen::Vector3d v;
  v << a.y_axis.x, a.y_axis.y, a.y_axis.z;
  return v;
}

Eigen::Vector3d Compiler::getZvect(const tangible_msgs::Block &a) {
  Eigen::Vector3d v;
  v << a.z_axis.x, a.z_axis.y, a.z_axis.z;
  return v;
}

Eigen::Vector3d Compiler::vect(const tangible_msgs::Block &a, const tangible_msgs::Block &b) {
  Eigen::Vector3d out;
  out << b.pose.pose.position.x - a.pose.pose.position.x,
        b.pose.pose.position.y - a.pose.pose.position.y,
        b.pose.pose.position.z - a.pose.pose.position.z;
  return out;
}

int Compiler::blockToType(const tangible_msgs::Block &a){
  if (a.id == tangible_msgs::Block::SIDE_PICK_ID || a.id == tangible_msgs::Block::TOP_PICK_ID){
    return tangible_msgs::Instruction::PICK;    
  }
  else if (a.id == tangible_msgs::Block::DROP_ID){
    return tangible_msgs::Instruction::DROP;
  }
  else {
    return tangible_msgs::Instruction::PLACE;
  }
}

double Compiler::blockDist(const tangible_msgs::Block &a, const tangible_msgs::Block &b ){
  return vect(a,b).norm();
}

bool Compiler::getIntersection(std::vector<geometry_msgs::Point> line1, 
                              std::vector<geometry_msgs::Point> line2,
                              geometry_msgs::Point* p){
  double x1, x2, x3, x4;
  double y1, y2, y3, y4;
  x1 = line1[0].x;
  x2 = line1[1].x;
  x3 = line2[0].x;
  x4 = line2[1].x;
  y1 = line1[0].y;
  y2 = line1[1].y;
  y3 = line2[0].y;
  y4 = line2[1].y;
  ROS_INFO("line1:");
  ROS_INFO("x1: %f", x1);
  ROS_INFO("x2: %f", x2);
  ROS_INFO("y1: %f", y1);
  ROS_INFO("y2: %f", y2);

  ROS_INFO("line2:");
  ROS_INFO("x3: %f", x3);
  ROS_INFO("x4: %f", x4);
  ROS_INFO("y3: %f", y3);
  ROS_INFO("y4: %f", y4);

  double denom, x_num, y_num;
  x_num = (x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4); 
  denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);

  y_num = (x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4);
  // y_denom = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4);
  if (denom == 0.0){
    ROS_INFO("No denominator");

    return false;
  }

  p->x = x_num / denom;
  p->y = y_num / denom;

  return true;

}

double Compiler::getQuadArea(geometry_msgs::Point p1, geometry_msgs::Point p2, 
                            geometry_msgs::Point p3, geometry_msgs::Point p4){

  // area += 0.5 * (x[i]*y[j] -  x[j]*y[i]);
  std::vector<geometry_msgs::Point> points;
  points.push_back(p1);
  points.push_back(p3);
  points.push_back(p2);
  points.push_back(p4);
  int i,j;
  double area = 0;
  int N = points.size();
  for (i=0;i<N;i++) {
    j = (i + 1) % N;
    area += points[i].x * points[j].y;
    area -= points[i].y * points[j].x;
  }

  area /= 2;
  return fabs(area < 0 ? -area : area);
}

std::vector<geometry_msgs::PointStamped> Compiler::getRegionCorners(const tangible_msgs::Block &a, const tangible_msgs::Block &b ){
  std::vector<geometry_msgs::PointStamped> corners;

  std::vector<geometry_msgs::Point> a_line_1_points;
  geometry_msgs::Point a_point_1;
  a_point_1.x = a.pose.pose.position.x;  
  a_point_1.y = a.pose.pose.position.y;  
  geometry_msgs::Point a_point_2;
  a_point_2.x = a.pose.pose.position.x + a.x_axis.x;  
  a_point_2.y = a.pose.pose.position.y + a.x_axis.y;
  a_line_1_points.push_back(a_point_1);   
  a_line_1_points.push_back(a_point_2);   
  std::vector<geometry_msgs::Point> a_line_2_points;    
  geometry_msgs::Point a_point_3;  
  a_point_3.x = a.pose.pose.position.x + a.y_axis.x;  
  a_point_3.y = a.pose.pose.position.y + a.y_axis.y;
  a_line_2_points.push_back(a_point_1);   
  a_line_2_points.push_back(a_point_3);   

  std::vector<geometry_msgs::Point> b_line_1_points;  
  geometry_msgs::Point b_point_1;
  b_point_1.x = b.pose.pose.position.x;  
  b_point_1.y = b.pose.pose.position.y;  
  geometry_msgs::Point b_point_2;  
  b_point_2.x = b.pose.pose.position.x + b.x_axis.x;  
  b_point_2.y = b.pose.pose.position.y + b.x_axis.y;
  b_line_1_points.push_back(b_point_1);   
  b_line_1_points.push_back(b_point_2);   

  std::vector<geometry_msgs::Point> b_line_2_points; 
  geometry_msgs::Point b_point_3;  
  b_point_3.x = b.pose.pose.position.x + b.y_axis.x;  
  b_point_3.y = b.pose.pose.position.y + b.y_axis.y;
  b_line_2_points.push_back(b_point_1);   
  b_line_2_points.push_back(b_point_3);

  std::vector<geometry_msgs::Point> intersection_points;
  geometry_msgs::Point int_point_1;
  geometry_msgs::Point int_point_2;
  geometry_msgs::Point int_point_3;
  geometry_msgs::Point int_point_4;
  if (getIntersection(a_line_1_points, b_line_1_points, &int_point_1)){
    intersection_points.push_back(int_point_1);
    ROS_INFO("adding point");

  }
  if (getIntersection(a_line_1_points, b_line_2_points, &int_point_2)){
    intersection_points.push_back(int_point_2);
    ROS_INFO("adding point");
  }
  if (getIntersection(a_line_2_points, b_line_1_points, &int_point_3)){
    intersection_points.push_back(int_point_3);
    ROS_INFO("adding point");
  } 
  if (getIntersection(a_line_2_points, b_line_2_points, &int_point_4)){
    intersection_points.push_back(int_point_4);    
    ROS_INFO("adding point");
  }  

  double best_area = 1000.0; //large number

  for (int i=0; i < intersection_points.size(); i++){
    for (int j=0; j < intersection_points.size(); j++){
      if (i != j){
        double area = getQuadArea(a_point_1, b_point_1, intersection_points[i], intersection_points[j]);
        ROS_INFO("area: %f", area);
        if (area < best_area){
          best_area = area;
          corners.clear();
          geometry_msgs::PointStamped ps1, ps2, ps3, ps4;
          ps1.header.frame_id = a.pose.header.frame_id;
          ps2.header.frame_id = a.pose.header.frame_id;
          ps3.header.frame_id = a.pose.header.frame_id;
          ps4.header.frame_id = a.pose.header.frame_id;
          ps1.point = a_point_1;
          ps2.point = b_point_1;
          ps3.point = intersection_points[i];
          ps4.point = intersection_points[j];
          ps1.point.z = a.pose.pose.position.z;
          ps2.point.z = a.pose.pose.position.z;
          ps3.point.z = a.pose.pose.position.z;
          ps4.point.z = a.pose.pose.position.z;
          corners.push_back(ps1);
          corners.push_back(ps2);
          corners.push_back(ps3);
          corners.push_back(ps4);
        }
      }
    }
  }

  return corners;
}

geometry_msgs::PointStamped Compiler::getPoint(const tangible_msgs::Block &a){
  geometry_msgs::PointStamped point;
  // Position center = selection.getCenter();
  Eigen::Vector3f y_axis_eigen (a.y_axis.x, a.y_axis.y, a.y_axis.z);
  Eigen::Vector3f tip (a.pose.pose.position.x, a.pose.pose.position.y, a.pose.pose.position.z);
  tip += y_axis_eigen * tangible_msgs::Block::ARROW_SELECTION_LEN;
  point.header.frame_id = a.pose.header.frame_id;
  point.point.x = tip(0);
  point.point.y = tip(1);
  point.point.z = tip(2);
  return point;
}

bool Compiler::tags2program(std::vector<tangible_msgs::Block> blocks){
  program.operations.clear();
  error_msg.clear();
  //NOTE: this ensures no instruction is formed for invalid tag settings
  int num_blocks = blocks.size();

  if(num_blocks == 0) {
    error_msg = "ERROR - TAG GROUPING - no tags to group.";
    ROS_ERROR_STREAM(error_msg);
    return false;
  }

  // sort blocks
  std::sort(blocks.begin(), blocks.end(), blockLessThan);
  for (int i=0; i < blocks.size(); i ++){
    ROS_INFO("sorted tag: %d", blocks[i].id);
  }
  // std::sort(blocks.begin(), blocks.end(), [](tangible_msgs::Block const &a, tangible_msgs::Block const &b) { return a.id < b.id; });
  // std::sort(blocks.begin(), blocks.end(), blockLess{});
  // std::sort(blocks.begin(), blocks.end(), tangible_msgs::Compiler::blockLessThan);

  int selection_count = 0;
  int selection2nd_count = 0;
  int action_count = 0;
  int number_count = 0;
  int other_count = 0;
  int regionID_count = 0;
  int grouped[num_blocks];
  for(int i = 0; i < num_blocks; i++) {
    if(inRange(tangible_msgs::Block::SELECTION_ID_MIN, tangible_msgs::Block::SELECTION_ID_MAX, blocks[i].id))
      selection_count++;
    else if(blocks[i].id == tangible_msgs::Block::SELECTION_2ND_ID)
      selection2nd_count++;
    else if(inRange(tangible_msgs::Block::ACTION_ID_MIN, tangible_msgs::Block::ACTION_ID_MAX, blocks[i].id))
      action_count++;
    else if(inRange(tangible_msgs::Block::NUMBER_ID_MIN, tangible_msgs::Block::NUMBER_ID_MAX, blocks[i].id))
      number_count++;
    else if(blocks[i].id > tangible_msgs::Block::NUMBER_ID_MAX)
      other_count++;

    if(blocks[i].id == tangible_msgs::Block::SELECT_REGION_ID ||
       blocks[i].id == tangible_msgs::Block::SELECT_OBJECTS_ID)
      regionID_count++;
    
    grouped[i] = -1;
  }

  if(selection_count == 0) {
    error_msg = "ERROR - TAG GROUPING - no selection tag.";
    ROS_ERROR_STREAM(error_msg);
    return false;
  }

  if(action_count == 0) {
    error_msg = "ERROR - TAG GROUPING - no action tag.";
    ROS_ERROR_STREAM(error_msg);
    return false;
  }

  if(number_count == 0) {
    error_msg = "ERROR - TAG GROUPING - no number tag.";
    ROS_ERROR_STREAM(error_msg);
    return false;
  }

  if(action_count < selection_count) {
    error_msg = "ERROR - TAG GROUPING - too few action or two many selection tags.";
    ROS_ERROR_STREAM(error_msg);
    return false;
  }

  if(selection2nd_count != regionID_count) {
    error_msg = "ERROR - TAG GROUPING - too many or too few secondary selection tags.";
    ROS_ERROR_STREAM(error_msg);
    return false;
  }

  if(selection2nd_count + action_count != number_count) {
    error_msg = "ERROR - TAG GROUPING - too many or too few number tags.";
    ROS_ERROR_STREAM(error_msg);
    ROS_ERROR("selection2nd_count %d", selection2nd_count);
    ROS_ERROR("action_count %d", action_count);
    ROS_ERROR("number_count %d", number_count);
    return false;
  }

  // index where selection tags start
  int selection_ind = 0;
  // index where secondary selection tags start
  int selection2nd_ind = selection_ind + selection_count; 
  // index where action tags start
  int action_ind = selection2nd_ind + selection2nd_count;
  // index where number tags start
  int number_ind = action_ind + action_count;
  //TO-DO to handle additional tags (e.g. loop)
  int other_ind  = number_ind + number_count;

  if (number_ind  < 2){
    return false;
  }

  for(int i = number_ind; i < other_ind - 1; i++) {
    int prev = blocks[i-1].id;
    // NOTE: at this point there is at least one selection and one action tags so 
    // number_ind >= 2 and i-1 will be valid.
    int curr = blocks[i].id;
    if (blocks.size() <= i+1){
      return false;
    }
    int next = blocks[i+1].id;
    if(next > (curr+1)) {
      error_msg = "ERROR - TAG GROUPING - missing number tag.";
    ROS_ERROR_STREAM(error_msg);
      return false;
    }
    if(prev == next) {
      error_msg = "ERROR - TAG GROUPING - too many repeated number tags.";
    ROS_ERROR_STREAM(error_msg);
      return false;
    }
  }


  for(int i = number_ind; i < other_ind; i++) {
    tangible_msgs::Block number = blocks[i];
        
    for(int j = selection2nd_ind; j < number_ind; j++) {
      if(grouped[j] > -1) // action/secondary selection tag is already grouped
        continue;

      tangible_msgs::Block action_or_2ndary = blocks[j];
      
     
      double distance = blockDist(number, action_or_2ndary);
      
      
      if(!inRange(tangible_msgs::Block::EDGE_SIZE - DIST_ERR_MARGIN,
                tangible_msgs::Block::EDGE_SIZE + DIST_ERR_MARGIN,
                distance)) {// action/secondary selection tag is too close/far
        if (blocks[i].id == 10 && blocks[j].id == 7){
          ROS_ERROR("too close or far");
        }
        continue;
      }
      
      // normalized center-to-center vector
      Eigen::Vector3d n2a = vect(number, action_or_2ndary) / distance;

      Eigen::Vector3d ux = getXvect(number);
      
      double inner_product = ux.dot(n2a);
      
      if(!inRange(1 - ROTATE_ERR_MARGIN,
                  1 + ROTATE_ERR_MARGIN,
                  inner_product)){ //action/secondary selection tag is not aligned w/ x-axis
        if (blocks[i].id == 10 && blocks[j].id == 7){
          ROS_ERROR("not aligned");
        }
        continue;
      }
      
      // number tag is grouped with action/secondary selection tag
      grouped[i] = j; 
      grouped[j] = i;
      
      break;
    }

    if(grouped[i] == -1) {
      error_msg = "ERROR - TAG GROUPING - dangling number tag.";
      ROS_ERROR_STREAM(error_msg);
      ROS_ERROR("Block unpaired: %d", blocks[i].id);
      for(int gi=0; gi < num_blocks; gi++){
      ROS_ERROR("%d", grouped[gi]);

      }
      return false;
    }

    //NOTE: decided to enforce the following:
    //  - the even steps are pick actions and the odd steps are place actions.
    //    This allows us to consider each pick and the subsequent place as a block
    //    We cannot thus accpet nested blocks to support such cases as picking up a tool
    //    for later pick&place (later pick&place blocks are nested in tool pickup block)
    //TO-DO once decided to support nested blocks should remove this check

    if(number.id%2 == 1 && 
       (blocks[grouped[i]].id != tangible_msgs::Block::TOP_PICK_ID &&
        blocks[grouped[i]].id != tangible_msgs::Block::SIDE_PICK_ID &&
        blocks[grouped[i]].id != tangible_msgs::Block::SELECTION_2ND_ID)) {
      error_msg = "ERROR - TAG GROUPING - expected a pick action." ;
    ROS_ERROR_STREAM(error_msg);
      return false;
    }

    if(number.id%2 == 0 && 
       (blocks[grouped[i]].id != tangible_msgs::Block::POSITION_ID &&
        blocks[grouped[i]].id != tangible_msgs::Block::DROP_ID &&
        blocks[grouped[i]].id != tangible_msgs::Block::SELECTION_2ND_ID)) {
      error_msg = "ERROR - TAG GROUPING - expected a place action." ;
    ROS_ERROR_STREAM(error_msg);
      return false;
    }
  }

  //YSS cannot think of a case where a selection2nd is not grouped yet none of the earlier
  //    error cases is triggered. I think this check is redundant.
  //    - extra selection2nd ---> selection2nd_count != regionID_count
  //    - selection2nd not numbered ---> selection2nd_count + action_count != number_count
  //    - selection2nd and number improperly placed ---> dangling number tag
  //    but I leave it just in case
  for(int i = selection2nd_ind; i < action_ind; i++)
    if(grouped[i] == -1) {
      error_msg = "ERROR - TAG GROUPING - secondary selection tag not numbered (WEIRD).";
    ROS_ERROR_STREAM(error_msg);
      return false;
    }

  //YSS again cannot think of a case where a selection2nd is not grouped yet none of the 
  //    earlier error cases is triggered. I think this check is redundant.
  //    - extra action ---> selection2nd_count + action_count != number_count
  //    - action not numbered ---> selection2nd_count + action_count != number_count
  //    - action and number improperly placed ---> dangling number tag
  //    but I leave it just in case
  for(int i = action_ind; i < number_ind; i++)
    if(grouped[i] == -1) {
      error_msg = "ERROR - TAG GROUPING - action tag not numbered (WEIRD).";
    ROS_ERROR_STREAM(error_msg);
      return false;
    }

  for(int i = action_ind; i < number_ind; i++) {
    tangible_msgs::Block action = blocks[i];

    double minDist = MAX_WORKSPACE_DIST; int temp_grouped = -1;
    for(int j = selection_ind; j < selection2nd_ind; j++) {
      tangible_msgs::Block selection = blocks[j];

      double distance = blockDist(action, selection);

      Eigen::Vector3d a2s = vect(action, selection) / distance;

      double quantizedDist = round(distance/tangible_msgs::Block::EDGE_SIZE);    
      
      if(!inRange(quantizedDist*tangible_msgs::Block::EDGE_SIZE - DIST_ERR_MARGIN,
                  quantizedDist*tangible_msgs::Block::EDGE_SIZE + DIST_ERR_MARGIN,
                  distance)){ // distance of selection tag is not a multiple of EDGE_SIZE
      if (j==0){
          ROS_ERROR("distance not multiple of edge size");
          ROS_ERROR("%f", distance);
          ROS_ERROR("%d", blocks[i].id);
        }
        continue;

    }

      Eigen::Vector3d uy = getYvect(action);
      double inner_product = uy.dot(a2s);
      if(!inRange(1 - ROTATE_ERR_MARGIN,
                1 + ROTATE_ERR_MARGIN,
                inner_product)){ // selection tag is not aligned w/ y-axis
        
        continue;
    }
      
      if(distance < minDist) {
        minDist = distance;
        temp_grouped = j;
      }
    }

    if(temp_grouped == -1) {
      error_msg = "ERROR - TAG GROUPING - dangling action tag.";

      ROS_ERROR_STREAM(error_msg);
      ROS_ERROR("Block: %d", blocks[i].id);
      for(int gi=0; gi < num_blocks; gi++){
      ROS_ERROR("%d", grouped[gi]);

      }
      return false;
    }

    grouped[i] = temp_grouped;
    if(grouped[temp_grouped] == -1)
      grouped[temp_grouped] = i;
  }

  for(int i = selection_ind; i < selection2nd_ind; i++)
    if(grouped[i] == -1) {
      error_msg = "ERROR - TAG GROUPING - dangling selection tag.";
    ROS_ERROR_STREAM(error_msg);
      return false;
    }

  //TO-DO return false for the following error cases
  //   - the order of action tags grouped w/ the same selection tag does not follow
  //     their distances



  for(int i = number_ind; i < other_ind-1; i++) {
    tangible_msgs::Block num1 = blocks[i]; 
    int num1_action_or_2ndary_at = grouped[i];
    tangible_msgs::Block num2 = blocks[i+1]; 
    int num2_action_or_2ndary_at = grouped[i+1];
    if(num1.id == num2.id) {
      
      //std::cout << num1.printID() << " @" << i 
      //          << " --> " << tags[grouped[i]].printID() << " @" << grouped[i];
      //std::cout << " ---- ";
      //std::cout << num2.printID() << " @" << i+1 
      //          << " --> " << tags[grouped[i+1]].printID() << " @" << grouped[i+1];
      //std::cout << "\n";
      
      // of two successive number tags with the same id, one is grouped with an action
          // and another with a secondary selection tool. The action is grouped with a
          // region selection tool
      if((blocks[num1_action_or_2ndary_at].id == tangible_msgs::Block::SELECTION_2ND_ID &&
          blocks[num2_action_or_2ndary_at].id == tangible_msgs::Block::SELECTION_2ND_ID) ||
         (blocks[num1_action_or_2ndary_at].id != tangible_msgs::Block::SELECTION_2ND_ID &&
          blocks[num2_action_or_2ndary_at].id != tangible_msgs::Block::SELECTION_2ND_ID) ||
         blocks[grouped[num1_action_or_2ndary_at]].id == tangible_msgs::Block::SELECT_POSITION_ID ||
         blocks[grouped[num1_action_or_2ndary_at]].id == tangible_msgs::Block::SELECT_OBJECT_ID ||
         blocks[grouped[num2_action_or_2ndary_at]].id == tangible_msgs::Block::SELECT_POSITION_ID ||
         blocks[grouped[num2_action_or_2ndary_at]].id == tangible_msgs::Block::SELECT_OBJECT_ID) {
        error_msg = "ERROR - TAG GROUPING - tags inavlidly paired.";
    ROS_ERROR_STREAM(error_msg);
        return false;
      }

      //TO-DO return false for the following error cases
      //   - paired selection and 2ndary selection tags have very different z_axes
      //   - paired selection and 2ndary selection tags have far from orthogonal y_axes
      //   - paired selection and 2ndary selection tags have far from orthogonal x_axes
      //   - paired selection and 2ndary selection tags do not face each other
      //     (center-2-center vector is in 2nd quandrant)

      if(blocks[num1_action_or_2ndary_at].id == tangible_msgs::Block::SELECTION_2ND_ID) {
        grouped[num1_action_or_2ndary_at] = grouped[num2_action_or_2ndary_at];
        grouped[grouped[num2_action_or_2ndary_at]] = num1_action_or_2ndary_at;
        //std::cout << "secondary selection at " << i 
        //          << " grouped with selection at " << grouped[grouped[i+1]] << "\n";
      } else {
        grouped[num2_action_or_2ndary_at] = grouped[num1_action_or_2ndary_at];
        grouped[grouped[num1_action_or_2ndary_at]] = num2_action_or_2ndary_at;
        //std::cout << "secondary selection at " << i+1 
        //          << " grouped with selection at " << grouped[grouped[i]] << "\n";
      }
    }
  }

  //TO-DO return false for the following error cases
  //   - the number grouped w/ the secondary selection tag is not equal to the smallest
  //     number grouped w/ an action grouped with the primary selection
  std::vector<tangible_msgs::Instruction> instructions;
  int instruction_num = action_count;
  for(int i = 0; i < instruction_num; i++) {
    tangible_msgs::Instruction instruction;
    instructions.push_back(instruction);
  }
  // iterating through tags 
    ROS_INFO("Made it this far");

  for(int i = number_ind; i < other_ind; i++) {
    int index, action_at, selection_at;
    index = blocks[i].id - tangible_msgs::Block::NUMBER_ID_MIN;
    if (instructions.size() <= index){
      return false;
    }
    tangible_msgs::Instruction instruction = instructions[index];

    action_at = grouped[i];
    selection_at = grouped[action_at];

    if(blocks[action_at].id == tangible_msgs::Block::SELECTION_2ND_ID) {
      
      ;
      
    } else {
      // instruction.number = blocks[i];
      instruction.type = blockToType(blocks[action_at]);
      // instruction.selection = blocks[selection_at]; 
      // if(blocks[selection_at].id == tangible_msgs::Block::SELECT_REGION_ID ||
      //    blocks[selection_at].id == tangible_msgs::Block::SELECT_OBJECTS_ID) {
      //   int selection2nd_at = grouped[selection_at];
      //   instruction.selection2nd = blocks[selection2nd_at];
      // }
      if (blocks[selection_at].id == tangible_msgs::Block::SELECT_REGION_ID){
        tangible_msgs::Block other_region_block = blocks[grouped[selection_at]];
        instruction.target.region_corners = getRegionCorners(blocks[selection_at], other_region_block);
        instruction.target.type = tangible_msgs::Target::REGION;
        ROS_INFO("Region:");
        for (int f=0; f<instruction.target.region_corners.size(); f++){
          geometry_msgs::PointStamped corner = instruction.target.region_corners[f];
          ROS_INFO("corner x: %f", corner.point.x);
          ROS_INFO("corner y: %f", corner.point.y);
        }
      }
      else if (blocks[selection_at].id == tangible_msgs::Block::SELECT_OBJECT_ID){
        instruction.target.type = tangible_msgs::Target::OBJECT_SELECTOR;
        instruction.target.specified_point = getPoint(blocks[selection_at]);
        ROS_INFO("specified point:");
        ROS_INFO("x: %f", instruction.target.specified_point.point.x);
        ROS_INFO("y: %f", instruction.target.specified_point.point.y);

      }
      else if (blocks[selection_at].id == tangible_msgs::Block::SELECT_POSITION_ID){
        instruction.target.type = tangible_msgs::Target::POINT_LOCATION;
        instruction.target.specified_point = getPoint(blocks[selection_at]);
      }


    }
    ROS_INFO("before");
    instructions[index] = instruction;
    ROS_INFO("after");
  }
  if (instructions.size()%2 != 0){
    return false;
  }
  for(int i=0; i < instructions.size(); i++){
    ROS_INFO("About to add operation");

    tangible_msgs::Operation operation;
    operation.instructions.push_back(instructions[i]);
    i++;
    operation.instructions.push_back(instructions[i]);
    program.operations.push_back(operation);
    ROS_INFO("Adding operation");
  }
  ROS_INFO("Done adding");
  // //YSS this is already enforced by requiring all picks be on even steps
  // if(instructions[0].action.id != tangible_msgs::Block::SIDE_PICK_ID &&
  //    instructions[0].action.id != tangible_msgs::Block::TOP_PICK_ID) {
  //   error_msg = "ERROR - TAG GROUPING - invalid first action (not a pick).";
  //   instructions.clear();
  //   return false;
  // }

  // for(int i = 0; i < instructions.size(); i++) {
  //   if((instructions[i].selection.id == tangible_msgs::Block::SELECT_REGION_ID ||
  //       instructions[i].selection.id == tangible_msgs::Block::SELECT_OBJECTS_ID) &&
  //      instructions[i].selection2nd.id == -1) {
  //       error_msg = "ERROR - TAG GROUPING - missing secondary selection tag.";
  //     instructions.clear();
  //     return false;
  //   }
  // }

  //std::cout << printInstructionTags();

  //TO-DO handling additional other tags
  return true;

}


}
