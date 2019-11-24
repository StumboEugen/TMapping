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

class MapBranch
{
    /// 出生在哪个Exp
    const size_t bornedAt;
    /// 是哪个MapBranch生成的
    MapBranch* const father;
    /// Branch的UUID序号
    const size_t nSerial;

    /// 不包括 child's child
    std::vector<MapBranch*> children;
    /// 存活的后代数量
    size_t nAliveChildren = 0;
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

public:
    /// 构造函数, 构造时会自动在father注册自己相关的部分信息
    MapBranch(size_t bornedAt, MapBranch* father, size_t nSerial, double confidence);

    void setExpired();
};

}


#endif //TMAPPING_MAPBRANCH_H
