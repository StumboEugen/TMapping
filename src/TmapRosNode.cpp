//
// Created by stumbo on 2019/12/19.
//

#include "TmapRosNode.h"
#include <chrono>

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

static double temp;

bool TmapRosNode::cbSrvNewExp(tmapping::NewExpRequest& req,
                              tmapping::NewExpResponse& res)
{
    auto startTime = std::chrono::system_clock::now();
    const auto& newExp = make_shared<Exp>(JsonHelper::Str2JS(req.jNewExp));
    mTmappingCore.arriveNewExp(newExp);
    cout << "i get it " << newExp->expData()->typeStr() << endl;
    auto endTime = std::chrono::system_clock::now();
    std::chrono::duration<double> diff(endTime - startTime);
    temp = diff.count();
    cout << "\nTIME newExp: " << diff.count() << endl;
    return true;
}

bool TmapRosNode::cbSrvGateMovement(tmapping::GateMovementRequest& req,
                                    tmapping::GateMovementResponse& res)
{
    auto startTime = std::chrono::system_clock::now();
    auto gateID = JsonHelper::Str2JS(req.jGateMove).asInt();
    mTmappingCore.setLeftGate(gateID);
    auto endTime = std::chrono::system_clock::now();
    std::chrono::duration<double> diff(endTime - startTime);
    cout << "\nTIME gateMove: " << diff.count() << endl;
    cout << "TIME ALL: " << diff.count() + temp << endl;
    return true;
}

bool
TmapRosNode::cbSrvGetMaps(tmapping::GetMapsRequest& req,
                          tmapping::GetMapsResponse& res)
{
    res.jMaps = JsonHelper::JS2Str(mTmappingCore.getTopMaps(req.nMapRequired));
    return true;
}

