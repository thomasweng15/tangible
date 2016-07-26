#ifndef TANGIBLE_PROGRAM
#define TANGIBLE_PROGRAM

#include <vector>
#include <string>

#include "tangible/tag.h"

namespace tangible {

struct Instruction {
	Tag selection;
	Tag selection2nd;
	Tag action;
	Tag number;
};

class Program {
private:
	std::vector<Tag> tags;
	std::vector<Instruction> instructions;
	std::string error_msg;

public:
	const static double MAX_WORKSPACE_DIST = 200;
	const static double DIST_ERR_MARGIN = 0.01;
	const static double ROTATE_ERR_MARGIN = 0.01;

	Program(std::vector<Tag>& tgs);//YSS should consider objs next
	~Program();

	bool refreshTags(std::vector<Tag>& tgs);

	bool groupTags();
	std::string error();

	//TO-DO
	//run program

};

}

#endif