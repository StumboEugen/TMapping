//
// Created by stumbo on 18-5-10.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "tmapping/Tmapping.h"

#include "TmapRosNode.h"

using namespace std;
using namespace tmap;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TmappingCore");
    TmapRosNode t;
    cout << "Tmapping Core started!" << endl;
    ros::spin();
}