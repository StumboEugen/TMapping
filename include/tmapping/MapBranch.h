//
// Created by stumbo on 2019/11/17.
//

#ifndef TMAPPING_MAPBRANCH_H
#define TMAPPING_MAPBRANCH_H

#include <vector>
#include <memory>
#include <unordered_set>
#include <map>

namespace tmap
{

class MapBranch;

using MapBranchPtr = std::shared_ptr<MapBranch>;
using MapBranchWePtr = std::weak_ptr<MapBranch>;
using MapBranchUnPtr = std::unique_ptr<MapBranch>;

enum class MapBranchStatus {EXPIRED, MOVE2NEW, MOVE2OLD};

class MapBranch : public std::enable_shared_from_this<MapBranch>
{
    /// 出生在哪个Exp
    const size_t bornedAt;
    /// 是哪个MapBranch生成的
    const MapBranchPtr father;
    /// Branch的UUID序号
    const size_t nSerial;

    /// 不包括 child's child
    std::vector<MapBranchWePtr> children;
    /// 记录后代在何处出生, children[bornPlace[k]] 以及之后的branch都是在exps[k + bornedAt]以及其之后出生的
    std::vector<size_t> firstChildAtBirthExp;
    /// 用于记录branch上哪些Exp已经经过回环匹配
    std::unordered_set<size_t> loopClosureHandled;
    /// 当 status 为 MapBranchStatus::MOVE2OLD 的时候记录相似的Exp在哪里
    size_t theArrivingSimiliarExp = 0;
    /// 当前MapBranch的状态
    MapBranchStatus status = MapBranchStatus::MOVE2NEW;
    /// 当前Branch的概率
    double confidence = 1.0;
    /// 与 confidence相关, 需要知道当前构型中有多少数量的TopoNode
    size_t nTopoNode = 1;

    MapBranch(size_t bornedAt, MapBranchPtr father, size_t nSerial, double confidence);
public:

    static MapBranchPtr getAdamBranch();

    MapBranchPtr bornOne(size_t newBornedAt, size_t newSerial, double newConfidence);

    void setExpired();
};

}


#endif //TMAPPING_MAPBRANCH_H
