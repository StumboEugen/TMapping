//
// Created by stumbo on 18-5-10.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "TmapRosNode.h"

using namespace std;
using namespace tmap;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TmappingCore");
    TmapRosNode t;
    cout << "Tmapping Core started!" << endl;
    /// 回调函数已经注册在class TmapRosNode中, 自动进行
    ros::spin();
}