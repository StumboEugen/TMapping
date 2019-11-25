//
// Created by stumbo on 2019/11/17.
//

#include "MapTwig.h"
#include "Tmapping.h"
#include <iostream>

using namespace std;

tmap::MapBranchPtr tmap::MapTwig::getAdamBranch()
{
    MapBranchPtr adam(new MapTwig(0, nullptr, 0, 1.0));
    return adam;
}

tmap::MapTwig::MapTwig(size_t bornAt, MapBranchPtr father,
                       size_t nSerial, double confidence) : borndAt(bornAt),
                                                            father(std::move(father)),
                                                            nSerial(nSerial),
                                                            confidence(confidence)
{}

tmap::MapBranchPtr tmap::MapTwig::bornOne(size_t newSerial, double newConfidence)
{
    if (status != MapBranchStatus::EXPIRED) {
        cerr << FILE_AND_LINE << " You didn't born a branch from a EXPIRED branch!" << endl;
    }
    MapBranchPtr newTwig(
            new MapTwig(this->dieAt, shared_from_this(), newSerial, newConfidence));
    /// 在后代列表后插入这个新的后代
    this->children.emplace_back(newTwig);
    return newTwig;
}

void tmap::MapTwig::setExpired()
{
    if (status == MapBranchStatus::EXPIRED) {
        cerr << FILE_AND_LINE << " You expired a mapBranch more than once!" << endl;
    }
    status = MapBranchStatus::EXPIRED;
}

void tmap::MapTwig::setNTopoNode(size_t NTopoNode)
{
    MapTwig::nTopoNode = NTopoNode;
}
