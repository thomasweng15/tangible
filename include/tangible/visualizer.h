#ifndef TANGIBLE_VISUALIZER
#define TANGIBLE_VISUALIZER

#include "ros/ros.h"
#include "tangible/program.h"

namespace tangible {

class Visualizer {
private:
	const static int ADD = 0;
	const static int MODIFY = 1;
	const static int DELETE = 2;

	const static int TAG_NUM = 18;
	int latest_instruction_num;

	ros::NodeHandle node;

	ros::Publisher ar_label_pub;
	ros::Publisher scene_pub;
	ros::Publisher program_pub;
public:
	Visualizer(ros::NodeHandle& n);
	~Visualizer();

	void update(Program p);
	// TO-DO update for AR tags and scene 
	
	void clear();
};

}

#endif