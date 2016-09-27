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

	LABELS_TXT[0] = "@ POINT";
	LABELS_TXT[1] = "THE OBJECT";
	LABELS_TXT[2] = "IN REGION";
	LABELS_TXT[3] = "THE OBJECTS";
	LABELS_TXT[4] = "PAIRED";
	LABELS_TXT[5] = "PICK SIDE";
	LABELS_TXT[6] = "PICK TOP";
	LABELS_TXT[7] = "PLACE";
	LABELS_TXT[8] = "DROP";
	LABELS_TXT[9] = "STEP #1";
	LABELS_TXT[10] = "STEP #2";
	LABELS_TXT[11] = "STEP #3";
	LABELS_TXT[12] = "STEP #4";
	LABELS_TXT[13] = "STEP #5";
	LABELS_TXT[14] = "STEP #6";
	LABELS_TXT[15] = "STEP #7";
}

Visualizer::~Visualizer() {}

void Visualizer::update(Program p) {
	clearProgram(); // TO-DO test if program is properly cleared and re-drawed

	std::vector<Instruction> instructions = p.getInstructions();
	int marker_count = 0;
	for(int i = 0; i < instructions.size(); i++) {

		visualization_msgs::Marker grouping_marker;
		fillGroupingMarker(grouping_marker, instructions[i]);
		setID(grouping_marker, marker_count);
		marker_count++;
		program_elements.push_back(grouping_marker);
		program_pub.publish(grouping_marker);
		
		visualization_msgs::Marker label_marker_selection,
		                           label_marker_action,
		                           label_marker_number,
		                           label_marker_selection2nd;
		
		fillLabelMarker(label_marker_selection, instructions[i].selection);
		setID(label_marker_selection, marker_count);
		marker_count++;
		program_elements.push_back(label_marker_selection);
		program_pub.publish(label_marker_selection);

		fillLabelMarker(label_marker_action, instructions[i].action);
		setID(label_marker_action, marker_count);
		marker_count++;
		program_elements.push_back(label_marker_action);
		program_pub.publish(label_marker_action);

		fillLabelMarker(label_marker_number, instructions[i].number);
		setID(label_marker_number, marker_count);
		marker_count++;
		program_elements.push_back(label_marker_number);
		program_pub.publish(label_marker_number);

		if(instructions[i].selection2nd.getID() != -1) {
			fillLabelMarker(label_marker_selection2nd, instructions[i].selection2nd);
			setID(label_marker_selection2nd, marker_count);
			marker_count++;
			program_elements.push_back(label_marker_selection2nd);
			program_pub.publish(label_marker_selection2nd);
		}

	}
}

//TO-DO overload update function for AR tags and scene

void Visualizer::update(std::vector<rapid::perception::Object> objects) {
	clearScene(); // TO-DO test if scene is properly cleared and re-drawed

	for(int i = 0; i < objects.size(); i++) {
		visualization_msgs::Marker box =
	        rapid::viz::Marker::Box(NULL, objects[i].pose(), objects[i].scale()).marker();
		setNamespace(box, "scene_visualization");
		setID(box, i+1);
		setColor(box, 0.5, 0, 0.5, 0.45);
		scene_elements.push_back(box);
		scene_pub.publish(box);
	}
}

void Visualizer::fillGroupingMarker(visualization_msgs::Marker& marker, Instruction& ins) {
	setHeader(marker);
	setNamespace(marker, "tag_grouping");
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	setAction(marker, ADD);
	marker.pose.orientation.w = 1;
	geometry_msgs::Vector3 scale;
	scale.x = 0.01; scale.y = 0.01;
	setScale(marker, scale);
	setColor(marker, 0.2, 1, 1, 1);
	setPoints(marker, ins);
}

void Visualizer::fillLabelMarker(visualization_msgs::Marker& marker, Tag& tag) {
	setHeader(marker);
	setNamespace(marker, "tag_grouping");
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	setAction(marker, ADD);
	setPose(marker, tag);
	geometry_msgs::Vector3 scale;
	scale.z = 0.03;
	setScale(marker, scale);
	setColor(marker, 0, 0, 1, 1);
	marker.text = LABELS_TXT[tag.getID()];
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

void Visualizer::setPose(visualization_msgs::Marker& marker, Tag& tag) {
	marker.pose.position.x = tag.getCenter().x;
	marker.pose.position.y = tag.getCenter().y;
	marker.pose.position.z = tag.getCenter().z;
	marker.pose.orientation.x = tag.getOrientation().x;
	marker.pose.orientation.y = tag.getOrientation().y;
	marker.pose.orientation.z = tag.getOrientation().z;
	marker.pose.orientation.w = tag.getOrientation().w;
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

void Visualizer::clearProgram() {
	for(int i = 0; i < program_elements.size(); i++) {
		setAction(program_elements.at(i), DELETE);
		scene_pub.publish(program_elements.at(i));
	}
	program_elements.clear();
}

void Visualizer::clearScene() {
	for(int i = 0; i < scene_elements.size(); i++) {
		setAction(scene_elements.at(i), DELETE);
		scene_pub.publish(scene_elements.at(i));
	}
	scene_elements.clear();
}

void Visualizer::clear() {
	//TO-DO
	//DELETE ALL markers
	clearScene();
}

}