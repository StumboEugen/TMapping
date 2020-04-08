//
// Created by stumbo on 2019/12/19.
//

#ifndef TMAPPING_SRC_TMAPROSNODE_H
#define TMAPPING_SRC_TMAPROSNODE_H

#include "tmapping/Tmapping.h"
#include "ros/ros.h"

#include <tmapping/NewExp.h>
#include <tmapping/GateMovement.h>
#include <tmapping/GetMaps.h>
#include <tmapping/SetSurviverMapsNum.h>


namespace tmap
{

class TmapRosNode
{
    ros::NodeHandle n;
    ros::ServiceServer srvNewExp;
    ros::ServiceServer srvGateMovement;
    ros::ServiceServer srvGetmaps;
    ros::ServiceServer srvSetSurviver;
    TopoMapping mTmappingCore;

private:
    bool cbSrvNewExp(tmapping::NewExpRequest& req,
            tmapping::NewExpResponse& res);
    bool cbSrvGateMovement(tmapping::GateMovementRequest& req,
            tmapping::GateMovementResponse& res);
    bool cbSrvGetMaps(tmapping::GetMapsRequest& req,
                      tmapping::GetMapsResponse& res);
    bool cbSrvSetSuriviers(tmapping::SetSurviverMapsNumRequest& req,
                           tmapping::SetSurviverMapsNumResponse& res);

public:
    TmapRosNode();

};
}


#endif //TMAPPING_SRC_TMAPROSNODE_H
