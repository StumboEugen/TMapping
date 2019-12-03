//
// Created by stumbo on 2019/12/3.
//

#include "TopoMapping.h"

void tmap::TopoMapping::addNewExp(tmap::ExpDataPtr newExpData)
{

}

void tmap::TopoMapping::setLeftGate(size_t gateID)
{
    experiences.setLeftGateOfCurrent(gateID);
}

void tmap::TopoMapping::setLeftGate(TopoVec2 gatePos)
{
    experiences.setLeftGateOfCurrent(gatePos);
}
