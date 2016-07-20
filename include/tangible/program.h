#ifndef TANGIBLE_PROGRAM
#define TANGIBLE_PROGRAM

#include <vector>

#include "tangible/tag.h"

namespace tangible {

class Program {
private:
	std::vector<Tag> tags;
public:
	Program(std::vector<Tag>& tgs);//YSS should consider objs next
	~Program();

	//TO-DO
	//generate program
	//run program
};

}

#endif