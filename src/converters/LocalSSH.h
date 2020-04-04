//
// Created by stumbo on 2020/4/3.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_CONVERTERS_LOCALSSH_H
#define TMAPPING_INCLUDE_TMAPPING_CONVERTERS_LOCALSSH_H

#include "ros/ros.h"

#include "local_ssh/Features.h"
#include "local_ssh/Feature.h"

#include "tmapping/Tmapping.h"


namespace tmap
{
class LocalSSH
{
    ros::NodeHandle n;
    ros::Subscriber subLocalSSH;

    ros::ServiceClient RSC_newExp;
    ros::ServiceClient RSC_throughGate;

    ExpPtr lastExp;

public:
    LocalSSH();

    void cbFeatures(const local_ssh::Features& msg);
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_CONVERTERS_LOCALSSH_H
