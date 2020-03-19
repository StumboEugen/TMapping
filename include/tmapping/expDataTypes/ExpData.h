//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_EXPDATA_H
#define TMAPPING_EXPDATA_H

#include <vector>
#include <memory>

#include "SubNode.h"
#include "GeoHash.h"

#include "../tools/TopoTools.h"
#include "../gateTypes/GateTypes.h"
#include "../landmarkTypes/LandmarkTypes.h"

namespace tmap
{

enum class ExpDataType{Intersection, Corridor, Stair, Room};

class ExpData;
using ExpDataPtr = std::shared_ptr<ExpData>;
using ExpDataUnPtr = std::unique_ptr<ExpData>;
using ExpDataWePtr = std::weak_ptr<ExpData>;

struct MatchResult_IMPL{
    /// 可能性系数, 如果为0表示完全不可能
    double possibility;
    /// k = gateMapping2this[j], 则 mergedExpData->gates[j] 与this->gates[k]为同一gate
    std::vector<GateID> gateMapping2this;
    /// k = gateMapping2mergedExpData[j], 则 this->gates[j] 与mergedExpData->gates[k]为同一gate
    std::vector<GateID> gateMapping2mergedExpData;
    /// 两个ExpData的相对位移, 相对于that而言
    TopoVec2 displacement;
    /// 融合后产生的新ExpData, gate编号与that相吻合
    ExpDataPtr mergedExpData;
};

using MatchResult = std::unique_ptr<MatchResult_IMPL>;

/// 代表观测得到的一次地形数据, 比如一个路口, 一个房间的信息
class ExpData
{
    std::string mName;

public:
    /// 用于辅助表示ExpData内部的连接关系, 这种连接关系表示机器人的运动历史, 也表示了里程计准确度的相关性
    struct SubLink{
        SubNode a;
        SubNode b;

        SubLink(const SubNode& a, const SubNode& b) : a(a), b(b)
        {}

        SubLink() : a(SubNodeType::UNSET, 0), b(SubNodeType::UNSET, 0) {}
    };

protected:
    std::vector<GatePtr> mGates;
    std::vector<PLMPtr> mPosLandmarks;

    std::vector<SubLink> mSubLinks;

    void copy2(ExpData* copy2);

    mutable std::unique_ptr<GeoHash> mGeoHash;

    const GeoHash& getHashTable() const;

public:
    static ExpDataPtr madeFromJS(const Jsobj& jdata);

    virtual ExpDataType type() const = 0;

    const std::vector<GatePtr>& getGates() const;

    const std::vector<PLMPtr>& getPLMs() const;

    void addGate(GatePtr pGate);

    void addLandmark(PLMPtr pLandmark);

    GateID findTheCloestGate(const TopoVec2& gatePos);

    /**
     * @brief 计算两个ExpData是否类似
     * @param that 另一个用于比较的ExpData实例
     * @param selfWeight this的比重, 1:1就是1, 1:4就是0.25, 5:1就是5
     * @return 匹配的详细结果, 包含融合后的ExpData
     */
    MatchResult detailedMatch(const ExpData& that, double selfWeight = 1.0) const;

    /**
     * @brief 计算两个ExpData是否类似
     * @param another 另一个用于比较的ExpData实例
     * @param selfWeight this的比重, 1:1就是1, 1:4就是0.25, 5:1就是5
     * @return 匹配的结果, 只包含一个概率评分
     */
    double quickMatch(const ExpData& another, double selfWeight) const;

    size_t nGates() const;

    virtual Json::Value toJS() const;

    const std::string& getName() const;

    void setName(const std::string& name);

    std::string typeStr();

    static std::string typeStr(ExpDataType type);

    /**
     * @brief 产生关于gate的外包洛正方形, ENU
     * @param 额外的扩展长度
     * @return 上下左右, 东南西北 ENU
     */
    virtual std::array<double, 4> getOutBounding(double expandValue) const;

    virtual ExpDataPtr clone() = 0;

    GateID findGateAtPos(const TopoVec2& pos, double threshold = 0.5) const;

    GateID findLmAtPos(const TopoVec2& pos, double threshold = 0.5) const;

    virtual TopoVec2 normalizeSelf();

    GatePtr popBackGate();

    const std::vector<SubLink>& getSubLinks() const;

    void addSubLink(SubNodeType typeA, size_t indexA, SubNodeType typeB, size_t indexB, bool findDup);
};

}


#endif //TMAPPING_EXPDATA_H
