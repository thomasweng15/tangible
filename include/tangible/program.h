#ifndef TANGIBLE_PROGRAM
#define TANGIBLE_PROGRAM

#include <vector>
#include <string>

#include "pcl/filters/crop_box.h"
#include "pcl/point_types.h"

#include "tangible/tag.h"
#include "rapid_perception/object.h"

namespace tangible {

struct Instruction {
	Tag selection;
	Tag selection2nd;
	Tag action;
	Tag number;
	std::vector<rapid::perception::Object> objects;
};

class Program {
private:
	std::vector<Tag> tags;
	std::vector<rapid::perception::Object> objects;
	std::vector<Instruction> instructions;
	std::string error_msg;

	bool tags2Instructions();
	bool instructionsWithObjects();
	bool matchObjects(Instruction ins, std::vector<rapid::perception::Object>& matched);

	void setupFilterBox(pcl::CropBox<pcl::PointXYZRGB>& cbox, Tag& selection);
	void setupFilterBox(pcl::CropBox<pcl::PointXYZRGB>& cbox, Tag& selection, Tag& selection2nd);

	int filterObject(pcl::CropBox<pcl::PointXYZRGB>& cbox, rapid::perception::Object& obj);

public:
	//TO-DO register these as ros params so you can modify them on the go
	const static double MAX_WORKSPACE_DIST = 1;
	const static double MAX_WORKSPACE_HEIGHT = 0.3;

	const static double DIST_ERR_MARGIN = 0.01;
	const static double ROTATE_ERR_MARGIN = 0.01;

	const static int MIN_POINT_OVERLAP = 15;
	const static double MIN_REGION_OVERLAP_RATIO = 0.5;
	const static double OBJECT_SELECTION_BOX_SIZE = 0.02;

	Program(std::vector<Tag>& tgs, std::vector<rapid::perception::Object> objs);
	//Program(std::vector<Tag>& tgs,
	//	    rapid::perception::HSurface tt,
	//	    std::vector<rapid::perception::Object> objs);
	~Program();

	void refresh(std::vector<Tag>& tgs, std::vector<rapid::perception::Object> objs);
	//void refresh(std::vector<Tag>& tgs,
	//	         rapid::perception::HSurface tt,
	//	         std::vector<rapid::perception::Object> objs);

	std::vector<Instruction> getInstructions();
	std::string error();

	std::string printInstructions();

	//TO-DO
	//run program

};

}

#endif