#include "tangible/visualizer.h"

#include <vector>

#include "rapid_viz/markers.h"

namespace tangible {

Visualizer::Visualizer(ros::NodeHandle& n, std::string id) {
	node = n;

	frame_id = id;

	ar_label_pub = n.advertise<visualization_msgs::Marker>("ar_label_viz", 10);
	scene_pub = n.advertise<visualization_msgs::Marker>("scene_viz", 10);
	program_pub = n.advertise<visualization_msgs::Marker>("program_viz", 10);

	latest_instruction_num = 0;
	latest_object_num = 0;
}

Visualizer::~Visualizer() {}

void Visualizer::update(Program p) {
	std::vector<Instruction> instructions = p.getInstructions();
	latest_instruction_num = instructions.size();
	for(int i = 0; i < latest_instruction_num; i++) {
		visualization_msgs::Marker marker;
		setHeader(marker);
		setNamespace(marker, "tag_grouping");
		setID(marker, i);
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		setAction(marker, ADD);
		marker.pose.orientation.w = 1;
		geometry_msgs::Vector3 scale;
		scale.x = 0.01; scale.y = 0.01;
		setScale(marker, scale);
		setColor(marker, 0.2, 1, 1, 1);
		setPoints(marker, instructions[i]);
		program_pub.publish(marker);
	}
}

//TO-DO overload update function for AR tags and scene

void Visualizer::update(std::vector<rapid::perception::Object> objects) {
	if(objects.size() < latest_object_num)
		clearScene();

	latest_object_num = objects.size();
	for(int i = 0; i < latest_object_num; i++) {
		visualization_msgs::Marker box =
	        rapid::viz::Marker::Box(NULL, objects[i].pose(), objects[i].scale()).marker();
		setNamespace(box, "scene_visualization");
		setID(box, i+1);
		setColor(box, 0.5, 0, 0.5, 0.45);
		scene_pub.publish(box);
	}
}

void Visualizer::setHeader(visualization_msgs::Marker& marker) {
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = frame_id;

}

void Visualizer::setNamespace(visualization_msgs::Marker& marker, std::string name) {
	marker.ns = name;
}

void Visualizer::setID(visualization_msgs::Marker& marker, int ID) {
	marker.id = ID;
}

void Visualizer::setAction(visualization_msgs::Marker& marker, int action) {
	switch(action){
		case ADD:
			marker.action = visualization_msgs::Marker::ADD;
			break;
		case MODIFY:
			marker.action = visualization_msgs::Marker::MODIFY;
			break;
		case DELETE:
			marker.action = visualization_msgs::Marker::DELETE;
		default:
			marker.action = visualization_msgs::Marker::DELETE;
			break;
	}
}

void Visualizer::setPose(visualization_msgs::Marker& marker, geometry_msgs::PoseStamped ps) {
	marker.header = ps.header;
	marker.pose = ps.pose;
}

void Visualizer::setScale(visualization_msgs::Marker& marker, geometry_msgs::Vector3 scale) {
	marker.scale = scale;
}

void Visualizer::setColor(visualization_msgs::Marker& marker,
	                      double r, double g, double b, double a) {
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;
}

void Visualizer::setPoints(visualization_msgs::Marker& marker, Instruction& ins) {
	geometry_msgs::Point p;

	p.x = ins.number.getCenter().x;
	p.y = ins.number.getCenter().y;
	p.z = ins.number.getCenter().z;
	marker.points.push_back(p);

	p.x = ins.action.getCenter().x;
	p.y = ins.action.getCenter().y;
	p.z = ins.action.getCenter().z;
	marker.points.push_back(p);

	p.x = ins.selection.getCenter().x;
	p.y = ins.selection.getCenter().y;
	p.z = ins.selection.getCenter().z;
	marker.points.push_back(p);

	if(ins.selection2nd.getID() != -1) {
		p.x = ins.selection2nd.getCenter().x;
		p.y = ins.selection2nd.getCenter().y;
		p.z = ins.selection2nd.getCenter().z;
		marker.points.push_back(p);
	}
}

void Visualizer::clearScene() {
	for(int i = 0; i < latest_object_num; i++) {
			visualization_msgs::Marker old_box;
			setNamespace(old_box, "scene_visualization");
			setID(old_box, i);
			setAction(old_box, DELETE);
			scene_pub.publish(old_box);
		}
}

void Visualizer::clear() {
	//TO-DO
	//DELETE ALL markers
	//use latest_instruction_num latest_object_num
	clearScene();
}

}