//
// Created by stumbo on 2019/11/17.
//

#ifndef TMAPPING_MAPTWIG_H
#define TMAPPING_MAPTWIG_H

#include <vector>
#include <memory>
#include <unordered_set>
#include <map>

#include "tools/TopoParams.h"

namespace tmap
{

/// 标明当前Twig的状态
enum class MapTwigStatus {
    /// 当前MapTwig已经停止了生长, 但是后代分支还存在, 需要this的连接与数据, 所以this还活着
    EXPIRED,
    /// 当前MapTwig正在走向一个没有去过的位置, 有可能能产生分支
    MOVE2NEW,
    /// 当前MapTwig正在走向一个去过的位置, 不可能产生分支, 但是可能产生矛盾从而死亡
    MOVE2OLD
};

class MapTwig : public std::enable_shared_from_this<MapTwig>
{
    /// 出生在哪个Exp
    const size_t borndAt;
    /// 是哪个MapBranch生成的
    const MapTwigPtr father;
    /// Branch的UUID序号
    const size_t nSerial;

    /// 不包括 child's child
    std::vector<MapTwigWePtr> mChildren;
    /// 记录分歧的位置, 在何处分歧分离的(和后代的borneAt是相同的)
    size_t dieAt = 0;
    /// 与过去经历重复的 exp
    std::vector<MergedExpPtr> mLoopClosures;
    /// 当 status 为 MapTwigStatus::MOVE2OLD 的时候记录相似的Exp在哪里
    // TODO 注意单独匹配的情况, 这里是个父类指针
    MergedExpPtr theArrivingSimiliarExp = nullptr;
    /// 当前MapBranch的状态
    MapTwigStatus status = MapTwigStatus::MOVE2NEW;
    /// 当前Branch的概率
    double mConfidence = 1.0;
    /// 与 confidence相关, 需要知道当前构型中有多少数量的TopoNode
    size_t nTopoNode = 1;

    MapTwig(size_t bornAt, MapTwigPtr father, size_t nSerial, double confidence);

public:

    static MapTwigPtr getAdamBranch();

    MapTwigPtr bornOne(size_t newSerial, double newConfidence);

    void setExpired();

    void setNTopoNode(size_t NTopoNode);

    MapTwigStatus getStatus() const;

    const MergedExpPtr& getTheArrivingSimiliarExp() const;

    double xCoe(double coe);

    const std::vector<MapTwigWePtr>& getChildren() const;

    const std::vector<MergedExpPtr>& getLoopClosures() const;
};

}


#endif //TMAPPING_MAPTWIG_H
