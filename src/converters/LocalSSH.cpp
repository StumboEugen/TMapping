//
// Created by stumbo on 2020/4/3.
//

#include "LocalSSH.h"

#include <tmapping/NewExp.h>
#include <tmapping/GateMovement.h>

#include <vector>

using namespace std;

void tmap::LocalSSH::cbFeatures(const local_ssh::Features& msg)
{
    /// TEMP
    if (msg.header.seq > 10)
        return;

    vector<GatePtr> gates;
    gates.reserve(msg.features.size());

    double maxDist = 0.0;

    int maxA{}, maxB{};

    for (const auto& oneFeature : msg.features) {
        TopoVec2 pos(oneFeature.y, oneFeature.x);
        TopoVec2 normalVec(1, 0);
        normalVec = normalVec.rotate(90 - (oneFeature.yaw * 180 / M_PI));
        GatePtr gate = make_shared<GateWay>(pos, normalVec);
        gate->setPossibility(oneFeature.confidence);
        for (int i = 0; i < gates.size(); ++i) {
            double currentDist = (gates[i]->getPos() - pos).len();
            if (currentDist > maxDist) {
                maxDist = currentDist;
                maxA = i;
                maxB = gates.size();
            }
        }
        gates.push_back(std::move(gate));
    }

    ExpDataPtr theExpData;
    if (maxDist > 4.5 && gates.size() < 3) {
        auto corridor = new Corridor();
        for (auto& gate: gates) {
            corridor->addGate(std::move(gate));
        }
        corridor->setEndGateA(maxA);
        corridor->setEndGateB(maxB);
        theExpData.reset(corridor);
    } else {
        theExpData.reset(new Intersection());
        for (auto& gate: gates) {
            theExpData->addGate(std::move(gate));
        }
    }

    GateID enterGate = GATEID_BEGINNING_POINT;
    if (lastExp) {
        auto lastLeftGate = lastExp->getLeaveGate();
        enterGate = theExpData->findTheCloestGate(
                lastExp->expData()->getGates()[lastLeftGate]->getPos());
    }

    ExpPtr exp = make_shared<Exp>(theExpData, enterGate);
    lastExp = exp;

    TopoVec2 exitPos(msg.passed_gateway.y, msg.passed_gateway.x);
    exp->setLeftGate(theExpData->findTheCloestGate(exitPos));

    tmapping::NewExp srvExp;
    srvExp.request.jNewExp = JsonHelper::JS2Str(exp->toJS(), false);
    if (!RSC_newExp.call(srvExp)) {
        cerr << "ROS service [newExp] call failed!" << endl;
        return;
    }

    cout << srvExp.request.jNewExp << endl;

    tmapping::GateMovement gateMovement;
    gateMovement.request.jGateMove = JsonHelper::JS2Str(exp->getLeaveGate());
    if (!RSC_throughGate.call(gateMovement)) {
        cerr << "ROS service [gateMove] call failed!" << endl;
    }
}

tmap::LocalSSH::LocalSSH() :
    n(),
    subLocalSSH(n.subscribe(
            "/features_in_last_map", 1000, &LocalSSH::cbFeatures, this))
{
    RSC_newExp = n.serviceClient<tmapping::NewExp>(TMAP_STD_SERVICE_NAME_NEW_EXP);
    RSC_throughGate = n.serviceClient<tmapping::GateMovement>(TMAP_STD_SERVICE_NAME_GATE_MOVE);
}
