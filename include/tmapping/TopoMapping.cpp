//
// Created by stumbo on 2019/12/3.
//

#include "TopoMapping.h"
#include "MapTwig.h"
#include "tools/TopoParams.h"
#include "MergedExp.h"

#include <iostream>
#include <unordered_set>

using namespace std;

void tmap::TopoMapping::addNewExp(tmap::ExpDataPtr newExpData)
{

}

void tmap::TopoMapping::setLeftGate(size_t gateID)
{
    mExperiences.setLeftGateOfCurrent(gateID);
}

void tmap::TopoMapping::setLeftGate(TopoVec2 gatePos)
{
    mExperiences.setLeftGateOfCurrent(gatePos);
}

void tmap::TopoMapping::arriveNewExp(tmap::ExpPtr newExp)
{
    unordered_set<MapTwigPtr> explorers;
    mExperiences.registExp(newExp);

    for (auto & oneAliveTwig : twigCollection.getAliveMaps()) {
        switch (oneAliveTwig->getStatus()) {
            case MapTwigStatus::EXPIRED:
                cerr << FILE_AND_LINE << " A expired twig in alive container!" << endl;
                break;
            case MapTwigStatus::MOVE2NEW:
                explorers.insert(oneAliveTwig);
                break;
            case MapTwigStatus::MOVE2OLD:
                double coe = oneAliveTwig->getTheArrivingSimiliarExp()->alike(*newExp->expData());
                if (coe == 0.0) {
                    twigCollection.getAliveMaps().erase(oneAliveTwig);
                } else {
                    oneAliveTwig->xCoe(coe);
                }
                break;
        }
    }
}
