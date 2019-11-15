//
// Created by stumbo on 18-5-10.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "tmapping/Tmapping.h"


using namespace std;
using namespace tmap;

int main(int argc, char **argv)
{
    TopoVec2 vec{1, 2};
    TopoVec2 vec2{1, 0};
    cout << vec + vec2 << endl;
    return 0;
}