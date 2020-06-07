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
    /// @note 最早添加的数据不够鲁棒, seq=11有问题,因此只处理seq<=10的
    if (msg.header.seq > 10)
        return;

    /// 每个Node由若干Gate组成,先制作Gate
    vector<GatePtr> gates;
    gates.reserve(msg.features.size());

    /// 根据节点中的最大距离决定把节点视作路口还是走廊
    double maxDist = 0.0;
    int maxA{}, maxB{};

    /// 开始遍历提供的featuer制作为Gate
    for (const auto& oneFeature : msg.features) {
        TopoVec2 pos(oneFeature.y, oneFeature.x);
        TopoVec2 normalVec(1, 0);
        normalVec = normalVec.rotate(90 - (oneFeature.yaw * 180 / M_PI));
        /// 全部认为是GateWay, 也就是没有门的路口
        GatePtr gate = make_shared<GateWay>(pos, normalVec);
        gate->setPossibility(oneFeature.confidence);
        /// 寻找节点中距离最大的两个Gate, 来判断是不是Corridor, 如果是的话把maxA和maxB视为路口
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

    /// 本次experience的节点数据
    ExpDataPtr theExpData;
    /// 一个粗浅的分类, 判断节点是路口还是走廊
    if (maxDist > 4.5 && gates.size() < 3) {
        /// 足够长,并且之鞥有两个路口,认为是走廊
        auto corridor = new Corridor();
        for (auto& gate: gates) {
            corridor->addGate(std::move(gate));
        }
        /// 对于走廊,我们要设置endGate(不然没法画出来)
        corridor->setEndGateA(maxA);
        corridor->setEndGateB(maxB);
        theExpData.reset(corridor);
    } else {
        /// 我们认为这个是路口
        theExpData.reset(new Intersection());
        for (auto& gate: gates) {
            theExpData->addGate(std::move(gate));
        }
    }

    GateID enterGate;
    if (lastExp) {
        /// 根据上一次的exp离开的gate的pos, 来确定机器人是从哪一个gate到达新exp的(即最近的)
        auto lastLeftGate = lastExp->getLeaveGate();
        enterGate = theExpData->findTheCloestGate(
                lastExp->expData()->getGates()[lastLeftGate]->getPos());
    } else {
        /// lastExp == nullptr, 这个是第一个exp, 我们认为入口门为GATEID_BEGINNING_POINT
        enterGate = GATEID_BEGINNING_POINT;
    }

    /// 封装出这次的experience
    ExpPtr exp = make_shared<Exp>(theExpData, enterGate);
    lastExp = exp;

    /// 根据exitPos的位置判断哪个口是离开的gateway
    TopoVec2 exitPos(msg.passed_gateway.y, msg.passed_gateway.x);
    exp->setLeftGate(theExpData->findTheCloestGate(exitPos));

    /// 调用service, 输出新观测的信息
    tmapping::NewExp srvExp;
    /// 统一使用json编码的字符串
    srvExp.request.jNewExp = JsonHelper::JS2Str(exp->toJS(), false);
    if (!RSC_newExp.call(srvExp)) {
        cerr << "ROS service [newExp] call failed!" << endl;
        return;
    }

    /// 调用service, 输出后续gatemove的信息
    tmapping::GateMovement gateMovement;
    gateMovement.request.jGateMove = JsonHelper::JS2Str(exp->getLeaveGate());
    if (!RSC_throughGate.call(gateMovement)) {
        cerr << "ROS service [gateMove] call failed!" << endl;
    }
    cout << "[Converter] exp information convert success! seq: " << msg.header.seq << endl;
}

tmap::LocalSSH::LocalSSH() :
    n(),
    subLocalSSH(n.subscribe(
            "/features_in_last_map", 1000, &LocalSSH::cbFeatures, this))
{
    RSC_newExp = n.serviceClient<tmapping::NewExp>(TMAP_STD_SERVICE_NAME_NEW_EXP);
    RSC_throughGate = n.serviceClient<tmapping::GateMovement>(TMAP_STD_SERVICE_NAME_GATE_MOVE);
}
