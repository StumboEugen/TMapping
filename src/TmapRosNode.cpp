//
// Created by stumbo on 2019/12/19.
//

#include "TmapRosNode.h"

using namespace tmap;
using namespace std;

tmap::TmapRosNode::TmapRosNode() :
        n(),
        srvNewExp(n.advertiseService(TMAP_STD_SERVICE_NAME_NEW_EXP,
                                     &TmapRosNode::cbSrvNewExp, this)),
        srvGateMovement(n.advertiseService(TMAP_STD_SERVICE_NAME_GATE_MOVE,
                                           &TmapRosNode::cbSrvGateMovement, this)),
        srvGetmaps(n.advertiseService(TMAP_STD_SERVICE_NAME_GET_MAPS,
                                      &TmapRosNode::cbSrvGetMaps, this))
{

}

bool TmapRosNode::cbSrvNewExp(tmapping::NewExpRequest& req,
                              tmapping::NewExpResponse& res)
{
    const auto& newExp = make_shared<Exp>(JsonHelper::Str2JS(req.jNewExp));
    mTmappingCore.arriveNewExp(newExp);
    return true;
}

bool TmapRosNode::cbSrvGateMovement(tmapping::GateMovementRequest& req,
                                    tmapping::GateMovementResponse& res)
{
    auto gate = JsonHelper::Str2JS(req.jGateMove).asInt();
    mTmappingCore.setLeftGate(gate);
    return true;
}

bool
TmapRosNode::cbSrvGetMaps(tmapping::GetMapsRequest& req,
                          tmapping::GetMapsResponse& res)
{
    res.jMaps = JsonHelper::JS2Str(mTmappingCore.getTopMaps(req.nMapRequired));
    return true;
}

