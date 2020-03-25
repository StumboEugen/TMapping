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
    /// 相似性系数, 如果为0表示完全不可能
    double possibility;
    /// k = gateMapping2old[j], 则 mergedExpData->gates[j] 与this->gates[k]为同一gate
    std::vector<GateID> gateMapping2old;
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

    /**
     * @brief 将baseclass的信息拷贝到copy2中
     * @param copy2 拷贝的重点
     * @param accordingSubLinks 是否根据Sublinks来有限地拷贝
     * @return 一张映射表, this的SubNode被映射到了那个copy2的SubNode
     */
    std::vector<SubNode> copy2(ExpData* copy2, bool accordingSubLinks = false) const;

    mutable std::unique_ptr<GeoHash> mGeoHash;

    const GeoHash& getHashTable() const;

    const TopoVec2& getPosOfSubNode(const SubNode& node) const;

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
     * @param that 另一个用于比较的ExpData实例
     * @param selfWeight this的比重, 1:1就是1, 1:4就是0.25, 5:1就是5
     * @return 是否相似
     */
    bool quickMatch(const ExpData& that, double selfWeight) const;

    size_t nGates() const;

    size_t nPoints() const;

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

    virtual ExpDataPtr clone() const = 0 ;

    virtual ExpDataPtr cloneShell() const = 0 ;

    GateID findGateAtPos(const TopoVec2& pos, double threshold = 0.5) const;

    GateID findLmAtPos(const TopoVec2& pos, double threshold = 0.5) const;

    virtual TopoVec2 normalizeSelf();

    GatePtr popBackGate();

    const std::vector<SubLink>& getSubLinks() const;

    void addSubLink(SubNodeType typeA, size_t indexA, SubNodeType typeB, size_t indexB, bool findDup);

    /**
     * @brief 生成一个被缩小的副本
     * @param copyAccordingSubLinks 是否根据this的subNodes来选择性拷贝
     * @param carelessPercentage
     * @return first:被缩小的副本 second:从this到first的gate映射关系
     */
    std::pair<ExpDataPtr, std::vector<SubNode>>
    buildShrinkedCopy(bool copyAccordingSubLinks,
                      const std::vector<SubNode>& whiteList = {},
                      double carelessPercentage = 1.0,
                      size_t nErasedNode = 1) const;

    /**
     * @brief 为节点添加噪声
     * @param maxOdomErrPerM 每米能产生的最大误差
     * @param maxDegreeErr 法向量的最大角度误差
     */
    void addNoise(double maxOdomErrPerM, double maxDegreeErr);

private:
    static std::vector<std::pair<SubNode, SubNode>>
    matchPairs(const ExpData& shape, const ExpData& pattern, bool shapeIsThis);

    double getPbtyOfGivenSubNode(const SubNode& node, bool discrimLM = false) const;

    /**
     * @brief 生成一个所有SubNodes组成的vec
     * @param expData 对应的expData
     * @return
     */
    static std::vector<SubNode> vecOfSubNodes(const ExpData& expData);

    SubNode copySubNode2(ExpData& dest, const SubNode& subNode) const;

    void eraseSubNode(const SubNode& target);
};

}


#endif //TMAPPING_EXPDATA_H
