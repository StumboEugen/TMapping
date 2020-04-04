//
// Created by stumbo on 2020/4/3.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "LocalSSH.h"

using namespace std;
using namespace tmap;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Converter_LocalSSH");
    LocalSSH t;
    cout << "Converter(Local SSH) started!" << endl;
    ros::spin();
}