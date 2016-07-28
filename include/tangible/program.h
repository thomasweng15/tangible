#ifndef TANGIBLE_PROGRAM
#define TANGIBLE_PROGRAM

#include <vector>
#include <string>

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

	bool tag2Instruction();
	bool matchObjects();

public:
	//TO-DO register these as ros params so you can modify them on the go
	const static double MAX_WORKSPACE_DIST = 200;
	const static double DIST_ERR_MARGIN = 0.01;
	const static double ROTATE_ERR_MARGIN = 0.01;

	const static int MIN_POINTCLOUD_OVERLAP = 15;

	//Program(std::vector<Tag>& tgs);
	Program(std::vector<Tag>& tgs, std::vector<rapid::perception::Object> objs);
	//Program(std::vector<Tag>& tgs,
	//	    rapid::perception::HSurface tt,
	//	    std::vector<rapid::perception::Object> objs);
	~Program();

	//void refresh(std::vector<Tag>& tgs);
	void refresh(std::vector<Tag>& tgs, std::vector<rapid::perception::Object> objs);
	//void refresh(std::vector<Tag>& tgs,
	//	         rapid::perception::HSurface tt,
	//	         std::vector<rapid::perception::Object> objs);

	std::string error();

	//TO-DO
	//run program

};

}

#endif