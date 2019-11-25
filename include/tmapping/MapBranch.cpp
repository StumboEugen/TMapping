//
// Created by stumbo on 2019/11/17.
//

#include "MapBranch.h"
#include "Tmapping.h"
#include <iostream>

using namespace std;

tmap::MapBranchPtr tmap::MapBranch::getAdamBranch()
{
    MapBranchPtr adam(new MapBranch(0, nullptr, 0, 1.0));
    return adam;
}

tmap::MapBranch::MapBranch(size_t bornedAt, MapBranchPtr father,
                           size_t nSerial, double confidence) : bornedAt(bornedAt),
                                                                father(std::move(father)),
                                                                nSerial(nSerial),
                                                                confidence(confidence)
{}

tmap::MapBranchPtr tmap::MapBranch::bornOne(
        size_t newBornedAt, size_t newSerial, double newConfidence)
{
    MapBranchPtr newBranch(
            new MapBranch(newBornedAt, shared_from_this(), newSerial, newConfidence));
    newBranch->nTopoNode = this->nTopoNode - 1;
    /// 当前记录到的首位后代的位置
    size_t recordedFirstBornPlace = this->firstChildAtBirthExp.size() + this->bornedAt;
    if (newBornedAt > recordedFirstBornPlace) {
        /// 还没有记录到最新出生的后代位置
        /// 预分配空间
        if (this->firstChildAtBirthExp.capacity() < newBornedAt - this->bornedAt) {
            this->firstChildAtBirthExp.reserve((newBornedAt - this->bornedAt) * 2);
        }
        /// 对于中间空缺的代, 都是以新生出的后代为第一个
        for (size_t i = recordedFirstBornPlace; i < newBornedAt; ++i) {
            this->firstChildAtBirthExp.push_back(this->children.size());
        }
    }
    /// 在后代列表后插入这个新的后代
    this->children.emplace_back(newBranch);

    return newBranch;
}

void tmap::MapBranch::setExpired()
{
    if (status == MapBranchStatus::EXPIRED) {
        cerr << FILE_AND_LINE << " You expired a mapBranch more than once!" << endl;
    }
    status = MapBranchStatus::EXPIRED;
}
