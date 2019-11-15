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
    GateWay gate{{1, 2}, {2, 3}};
    cout << gate.getNormalVec() - gate.getPos() << endl;
}