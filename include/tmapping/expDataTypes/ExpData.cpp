//
// Created by stumbo on 2019/11/15.
//

#include <cfloat>
#include <iostream>
#include <algorithm>
#include <random>
#include <chrono>

#include "ExpData.h"
#include "../tools/TopoParams.h"
#include "../landmarkTypes/LandmarkTypes.h"

#include "Corridor.h"
#include "Intersection.h"
#include "Room.h"

using namespace std;
using namespace tmap;

const std::vector<GatePtr>& tmap::ExpData::getGates() const
{
    return mGates;
}

void ExpData::addGate(GatePtr pGate)
{
    mGates.emplace_back(std::move(pGate));
}

void ExpData::addLandmark(PLMPtr pLandmark)
{
    mPosLandmarks.emplace_back(std::move(pLandmark));
}

GateID ExpData::findTheCloestGate(const TopoVec2& gatePos)
{
    size_t gateSize = mGates.size();
    if (gateSize == 0) {
        cerr << FILE_AND_LINE << " Gate size is ZERO ?!" << endl;
        throw;
    }
    auto lenMax = DBL_MAX;
    GateID res = 0;
    for (size_t i = 0; i < gateSize; ++i) {
        double currentLen = (mGates[i]->getPos() - gatePos).len();
        if (currentLen < lenMax) {
            lenMax = currentLen;
            res = i;
        }
    }

    return res;
}

MatchResult ExpData::detailedMatch(const ExpData& that, double selfWeight) const
{
    /// TODO 完成两个未对齐数据的匹配工作
    /// 要求: MatchResult->gatesMapping是从 [that's gate] = this's gate
    /// MatchResult->mergedExpData 的gate序号和another完全相同
    /// selfWeight指的是this的权重, 比如selfWeight=3, 说明this可能是3次结果融合而成的

    const auto& nGateThis = this->nGates();
    const auto& nGateThat = that.nGates();
    const auto& nPointsThis = nGateThis + this->mPosLandmarks.size();
    const auto& nPointsThat = nGateThat + that.mPosLandmarks.size();

    MatchResult res(new MatchResult_IMPL);
    /// 预分配gate的映射表,在本函数结束前,landmark的映射关系也使用此表
    res->gateMapping2mergedExpData.assign(nPointsThis, GATEID_NO_MAPPING);
    /// +nGateThis是因为可能nPointsMerged > nPointsThat, 而前者的上限是nGateThat + nGateThis
    res->gateMapping2old.assign(nPointsThat + nGateThis, GATEID_NO_MAPPING);

    /// 匹配得到两个点集之间的配对情况
    vector<std::pair<SubNode, SubNode>> pointsMap;
    if (nPointsThat >= nPointsThis) {
        pointsMap = matchPairs(*this, that, true);
    } else {
        pointsMap = matchPairs(that, *this, false);
    }
    if (pointsMap.empty()) {
        res->possibility = 0.0;
        return res;
    }

    /// 开始融合出我们的结果, 首先生成一个没有具体信息的新 ExpData
    res->mergedExpData = that.cloneShell();

    TopoVec2 thatCenter{}, thisCenter{};

    /// 遍历我们获得的最佳匹配
    for (const auto& pair : pointsMap) {
        const auto& thisNode = pair.first;
        const auto& thatNode = pair.second;
        /// 累加后从而认得到匹配点集的中心
        thisCenter += this->getPosOfSubNode(thisNode);
        thatCenter += that.getPosOfSubNode(thatNode);
        /// 记录映射关系, 目前表中存储包括gate和landmark的所有映射,而不只是gate
        auto& idMerged = res->gateMapping2mergedExpData[thisNode.toUIndex(nGateThis)];
        if (idMerged != GATEID_NO_MAPPING) {
            cerr << FILE_AND_LINE << "double trend happened[idMerged]!" << endl;
        }
        auto& idThis = res->gateMapping2old[thatNode.toUIndex(nGateThat)];
        if (idThis != GATEID_NO_MAPPING) {
            cerr << FILE_AND_LINE << "double trend happened[idThis]!" << endl;
        }
        idMerged = thatNode.toUIndex(nGateThat);
        idThis = thisNode.toUIndex(nGateThis);
    }
    thisCenter /= pointsMap.size();
    thatCenter /= pointsMap.size();
    /// from this 2 that
    const auto setsDiff = thatCenter - thisCenter; // TODO rotate?

    /// 用于计算概率的中间变量
    double possDiffSum = 0.0;
    double odomDiffSum = 0.0;
    double singlePtPossSum = 0.0;
    uint32_t nMatches = 0;
    /// 调整后的方差
    double C0 = convErrPerMeter * (1.0 + 1.0 / selfWeight);
//    double C0 = convErrPerMeter * 4 * (1.0 + 1.0 / selfWeight); /// 这里把误差允许范围* 2

    /// @note 这里为了保证index的一致性,我们从that开始遍历
    /// 首先遍历that的所有gate
    for (int idThat = 0; idThat < nGateThat; ++idThat) {
        auto idThis = res->gateMapping2old[idThat];
        const auto& thatGate = that.mGates[idThat];

        if (idThis == GATEID_NO_MAPPING) {
            /// 意味着that的gate没有映射到this的任何gate
            auto clonedGate = thatGate->clone();
            double posblt = clonedGate->getPossibility();
            /// 用于概率减益的计算
            singlePtPossSum += posblt;
            /// 概率需要减半,相当于(0+p)/2
            clonedGate->setPossibility(posblt / 2.0);
            /// 那么我们将that单按照其位置单独加入
            res->mergedExpData->addGate(std::move(clonedGate));
        } else {
            const auto& thisGate = this->mGates[idThis];
            
            nMatches ++;
            const auto& posThat = thatGate->getPos();
            const auto& posThis = setsDiff + thisGate->getPos();
            const auto& posDiff = posThis - posThat;
            const auto& centerErr = posThat - thatCenter;
            /// 位置偏差带来的概率修正
            odomDiffSum += exp(-0.5 * posDiff.len2() / (C0 * centerErr.len2()) );
            /// 存在性的修正
            possDiffSum += abs(thatGate->getPossibility() - thisGate->getPossibility());
            /// 将两者的融合添加到融合的节点中
            res->mergedExpData->addGate(thatGate->newMergedGate(
                    thisGate, posThis, 1.0 / selfWeight));
        }
    }

    /// 遍历that的所有LM
    for (int idThat = 0; idThat < nPointsThat - nGateThat; ++idThat) {
        auto idThis = res->gateMapping2old[idThat + nGateThat];
        const auto& thatlm = that.mPosLandmarks[idThat];
        
        if (idThis == GATEID_NO_MAPPING) {
            auto clonedLM = thatlm->clone();
            double posblt = clonedLM->getPossibility();
            singlePtPossSum += posblt;
            clonedLM->setPossibility(posblt / 2.0);
            res->mergedExpData->addLandmark(std::move(clonedLM));
        } else {
            const auto& thislm = this->mPosLandmarks[idThis];
            
            nMatches ++;
            const auto& posThat = thatlm->getPos();
            const auto& posThis = setsDiff + thislm->getPos();
            const auto& posDiff = posThis - posThat;
            const auto& centerErr = posThat - thatCenter;
            odomDiffSum += exp(-0.5 * posDiff.len2() / (C0 * centerErr.len2()) );

            possDiffSum += abs(thatlm->getPossibility() - thislm->getPossibility());
            
            res->mergedExpData->addLandmark(thatlm->newMergedPLM(
                    thislm, posThis, 1.0 / selfWeight));
        }
    }

    /// 遍历this的所有未匹配Gate
    for (int i = 0; i < nGateThis; ++i) {
        if (res->gateMapping2mergedExpData[i] == GATEID_NO_MAPPING) {
            auto clonedGate = this->mGates[i]->clone();
            auto posblt = clonedGate->getPossibility();
            singlePtPossSum += posblt;
            clonedGate->setPossibility(posblt / 2.0);
            clonedGate->setPos(clonedGate->getPos() + setsDiff);
            res->mergedExpData->addGate(std::move(clonedGate));
            res->gateMapping2mergedExpData[i] = res->mergedExpData->nGates() - 1;
            res->gateMapping2old[res->mergedExpData->nGates() - 1] = i;
        }
    }

    /// 遍历this的所有未匹配LM
    for (int i = 0; i < nPointsThis - nGateThis; ++i) {
        if (res->gateMapping2mergedExpData[i + nGateThis] == GATEID_NO_MAPPING) {
            auto clonedLM = this->mPosLandmarks[i]->clone();
            auto posblt = clonedLM->getPossibility();
            singlePtPossSum += posblt;
            clonedLM->setPossibility(posblt / 2.0);
            clonedLM->setPos(clonedLM->getPos() + setsDiff);
            res->mergedExpData->addLandmark(std::move(clonedLM));
        }
    }

    /// 如果是Corridor, 更新一下endPoint
    if (res->mergedExpData->type() == ExpDataType::Corridor) {
        auto c = (Corridor*)res->mergedExpData.get();
        c->setEndGateA(c->getEndGateA());
        c->setEndGateB(c->getEndGateB());
    }

    const size_t& nGateMerged = res->mergedExpData->nGates();
    res->displacement = -setsDiff;

//    /// 融合节点到老观测的映射表中nGateThat以后的部分必然没有映射关系,
//    /// 这里原本的数据有landemark的映射关系
//    for (int i = nGateThat; i < nGateMerged; ++i) {
//        res->gateMapping2old[i] = GATEID_NO_MAPPING;
//    }

    { /// 从新观测到老的映射表, 从nGateMerged开始erase
        const auto& b = res->gateMapping2old.begin();
        const auto& e = res->gateMapping2old.end();
        res->gateMapping2old.erase(b + nGateMerged, e);
    }

    { /// 从老观测到新观测的映射表, 从nGateThis开始erase
        const auto& b = res->gateMapping2mergedExpData.begin();
        const auto& e = res->gateMapping2mergedExpData.end();
        res->gateMapping2mergedExpData.erase(b + nGateThis,e);
    }

    auto nMergedPoints =
            res->mergedExpData->nGates() + res->mergedExpData->mPosLandmarks.size();
    double possS1 = odomDiffSum / nMatches;
    double possS2 = (1 - possDiffSum / nMatches) * (1 - singlePtPossSum / nMergedPoints);
    res->possibility = possS1 * possS2;

    return res;
}

bool ExpData::quickMatch(const ExpData& that, double selfWeight) const
{
    /// 快速的数据匹配工作
    if (that.type() != this->type()) {
        return false;
    }
    const auto& nGateThis = this->nGates();
    const auto& nGateThat = that.nGates();
    const auto& nPointsThis = nGateThis + this->mPosLandmarks.size();
    const auto& nPointsThat = nGateThat + that.mPosLandmarks.size();

    vector<std::pair<SubNode, SubNode>> pointsMap;
    if (nPointsThat >= nPointsThis) {
        pointsMap = matchPairs(*this, that, true);
    } else {
        pointsMap = matchPairs(that, *this, false);
    }

    return !pointsMap.empty();
}

size_t ExpData::nGates() const
{
    return mGates.size();
}

size_t ExpData::nPoints() const
{
    return mGates.size() + mPosLandmarks.size();
}

Json::Value ExpData::toJS() const
{
    Json::Value res;
    for (const auto& gate : mGates) {
        res["gates"].append(std::move(gate->toJS()));
    }
    for (const auto& landMark : mPosLandmarks) {
        res["landMark"].append(std::move(landMark->toJS()));
    }
    if (!mName.empty()) {
        res["name"] = mName;
    }
    res["type"] = typeStr(type());
    for (const auto& subLink : mSubLinks) {
        Jsobj link;
        link.append(subLink.a.index + static_cast<int32_t>(subLink.a.type) * mGates.size());
        link.append(subLink.b.index + static_cast<int32_t>(subLink.b.type) * mGates.size());
        res["sLink"].append(std::move(link));
    }
    return res;
}

ExpDataPtr ExpData::madeFromJS(const Jsobj& jexp)
{
    ExpDataPtr res;

    try {
        string type = jexp["type"].asString();
        if (type == typeStr(ExpDataType::Corridor)) {
            res = make_shared<Corridor>();
        }
        else if (type == typeStr(ExpDataType::Intersection)) {
            res = make_shared<Intersection>();
        }
        else if (type == typeStr(ExpDataType::Room)) {
            auto room = make_shared<Room>();
            room->setScaling(jexp["scaling"].asDouble());
            res = room;
        }
        else {
            cerr << FILE_AND_LINE << " You input an UNKNOWN expData! typeStr=" << type << endl;
            return res;
        }

        if (jexp.isMember("name")) {
            res->mName = jexp["name"].asString();
        }

        const auto& jgates = jexp["gates"];
        auto nGate = jgates.size();
        auto& gates = res->mGates;
        gates.reserve(nGate);
        for (int i = 0; i < nGate; ++i) {
            gates.emplace_back(Gate::madeFromJS(jgates[i]));
        }

        const auto& jmarks = jexp["landMark"];
        auto nMark = jmarks.size();
        auto& marks = res->mPosLandmarks;
        marks.reserve(nMark);
        for (int i = 0; i < nMark; ++i) {
            marks.emplace_back(PosLandmark::madeFromJS(jmarks[i]));
        }

        auto& subLinks = res->mSubLinks;
        subLinks.reserve(jexp["sLink"].size());
        for (const auto& jSubLink : jexp["sLink"]) {
            size_t l0 = jSubLink[0].asUInt64();
            size_t l1 = jSubLink[1].asUInt64();
            subLinks.emplace_back();
            subLinks.back().a.type = static_cast<SubNodeType>(l0 / nGate);
            subLinks.back().a.index = l0 % nGate;
            subLinks.back().b.type = static_cast<SubNodeType>(l1 / nGate);
            subLinks.back().b.index = l1 % nGate;
        }

        if (auto c = dynamic_cast<Corridor*>(res.get())) {
            c->setEndGateA(jexp["endG_A"].asInt());
            c->setEndGateB(jexp["endG_B"].asInt());
            c->setEndPointA(TopoVec2{jexp["endP_A"]});
            c->setEndPointB(TopoVec2{jexp["endP_B"]});
        }
    } catch (...) {
        res = nullptr;
    }
    return res;
}

const string& ExpData::getName() const
{
    return mName;
}

void ExpData::setName(const string& name)
{
    ExpData::mName = name;
}

std::string ExpData::typeStr(ExpDataType type)
{
    string res;
    switch (type) {
        case ExpDataType::Intersection:
            res = "Intersection";
            break;
        case ExpDataType::Corridor:
            res = "Corridor";
            break;
        case ExpDataType::Stair:
            res = "Stair";
            break;
        case ExpDataType::Room:
            res = "Room";
            break;
    }
    return res;
}

std::array<double, 4> ExpData::getOutBounding(double expandValue) const
{
    if (mGates.empty()) {
        cerr << FILE_AND_LINE << "You try to get the outbounding of a 0 gates ExpData!" << endl;
        return {0., 0., 0., 0.};
    }
    std::array<double, 4> res{-DBL_MAX, DBL_MAX, DBL_MAX, -DBL_MAX};
    for (const auto& gate : mGates) {
        auto& pos = gate->getPos();
        res[0] = max(res[0], pos.py);
        res[1] = min(res[1], pos.py);
        res[2] = min(res[2], pos.px);
        res[3] = max(res[3], pos.px);
    }

    for (const auto& plm : mPosLandmarks) {
        auto& pos = plm->getPos();
        res[0] = max(res[0], pos.py);
        res[1] = min(res[1], pos.py);
        res[2] = min(res[2], pos.px);
        res[3] = max(res[3], pos.px);
    }
    res[0] += expandValue;
    res[1] -= expandValue;
    res[2] -= expandValue;
    res[3] += expandValue;

    return res;
}

std::vector<SubNode> ExpData::copy2(ExpData* copy2, bool accordingSubLinks) const
{
    copy2->mName = this->mName;
    std::vector<SubNode> res;

    if (accordingSubLinks && !this->mSubLinks.empty()) {
        /// 需要部分拷贝
        res.reserve(this->nGates() + this->mPosLandmarks.size());
        const auto& nGate = this->nGates();
        /// 我们希望enterGate始终是enterGate, index为0
        res[0] = this->copySubNode2(*copy2,
                SubNode{SubNodeType::GATE, 0});
        for (const auto& subLink : this->mSubLinks) {
            uint32_t ida = subLink.a.toUIndex(nGate);
            uint32_t idb = subLink.b.toUIndex(nGate);
            /// 这意味着这个subnode还没有被拷贝
            if (res[ida].type == SubNodeType::UNSET) {
                res[ida] = this->copySubNode2(*copy2, subLink.a);
            }
            if (res[ida].type == SubNodeType::UNSET) {
                res[idb] = this->copySubNode2(*copy2, subLink.b);
            }
            copy2->addSubLink(res[ida].type, res[ida].index,
                              res[idb].type, res[idb].index, true);
        }
    } else {
        copy2->mGates.reserve(this->mGates.size());
        for (auto& gate : this->mGates) {
            copy2->mGates.emplace_back(gate->clone());
        }

        copy2->mPosLandmarks.reserve(this->mPosLandmarks.size());
        for (auto& plm : this->mPosLandmarks) {
            copy2->mPosLandmarks.emplace_back(plm->clone());
        }

        copy2->mSubLinks = this->mSubLinks;

        res = vecOfSubNodes(*this);
    }

    return res;
}

GateID ExpData::findGateAtPos(const TopoVec2& pos, double threshold) const
{
    for (int i = 0; i < mGates.size(); ++i) {
        if ((pos - mGates[i]->getPos()).len() < threshold) {
            return i;
        }
    }
    return GATEID_NOT_FOUND;
}

GateID ExpData::findLmAtPos(const TopoVec2& pos, double threshold) const
{
    for (int i = 0; i < mPosLandmarks.size(); ++i) {
        if ((pos - mPosLandmarks[i]->getPos()).len() < threshold) {
            return i;
        }
    }
    return GATEID_NOT_FOUND;
}

TopoVec2 ExpData::normalizeSelf()
{
    TopoVec2 offset = mGates.front()->getPos();
    for (auto& gate : mGates) {
        gate->setPos(gate->getPos() - offset);
    }
    for (auto& plm : mPosLandmarks) {
        plm->setPos(plm->getPos() - offset);
    }
    return offset;
}

GatePtr ExpData::popBackGate()
{
    if (mGates.empty()) {
        return nullptr;
    }
    GatePtr res;
    res = std::move(mGates.back());
    mGates.pop_back();
    return res;
}

std::string ExpData::typeStr()
{
    return typeStr(type());
}

const vector<PLMPtr>& ExpData::getPLMs() const
{
    return mPosLandmarks;
}

const vector<ExpData::SubLink>& ExpData::getSubLinks() const
{
    return mSubLinks;
}

void
ExpData::addSubLink(SubNodeType typeA, size_t indexA, SubNodeType typeB, size_t indexB, bool findDup)
{
    SubNode va(typeA, indexA);
    SubNode vb(typeB, indexB);

    if (findDup) {
        for (const auto& link : mSubLinks) {
            if (link.a == va && link.b == vb) {
                return;
            }
            if (link.a == vb && link.b == va) {
                return;
            }
        }
    }

    mSubLinks.emplace_back(va, vb);
}

const GeoHash& ExpData::getHashTable() const
{
    if (!mGeoHash) {
        mGeoHash.reset(new GeoHash(*this));
    }
    return *mGeoHash;
}

const TopoVec2& ExpData::getPosOfSubNode(const SubNode& node) const
{
    const TopoVec2* res;
    switch (node.type) {
        case SubNodeType::GATE:
            res = &this->getGates()[node.index]->getPos();
            break;
        case SubNodeType::LandMark:
            res = &this->getPLMs()[node.index]->getPos();
            break;
        default:
            cerr << FILE_AND_LINE << " error SubNode!";
            throw;
    }
    return *res;
}

double ExpData::getPbtyOfGivenSubNode(const SubNode& node, bool discrimLM) const
{
    switch (node.type) {
        case SubNodeType::GATE:
            return this->getGates()[node.index]->getPossibility();
        case SubNodeType::LandMark:
            if (discrimLM) {
                return this->getPLMs()[node.index]->getPossibility() * 0.8;
            } else {
                return this->getPLMs()[node.index]->getPossibility();
            }
        default:
            cerr << FILE_AND_LINE << " error SubNode!";
            throw;
    }
}

/**
 * @brief 获得两个ExpData点集之间的匹配
 * @param shape 作为图案的data
 * @param pattern 作为模板的data (生成哈希表)
 * @param shapeIsThis shape是否由this产生, 会影响输出的格式
 * @return 一个匹配表, first是this的点, second是that的点
 */
vector<std::pair<SubNode, SubNode>>
ExpData::matchPairs(const ExpData& shape, const ExpData& pattern, bool shapeIsThis)
{
    const auto& table = pattern.getHashTable();
    auto nPointsPat = pattern.nGates() + pattern.mPosLandmarks.size();
    auto nPointsShape = shape.nGates() + shape.mPosLandmarks.size();

    const auto& gatesOfShape = shape.getGates();
    const auto& lmsOfShape = shape.getPLMs();

    vector<SubNode> baseCandidates = vecOfSubNodes(shape);

    auto comp = [&shape](const SubNode& a, const SubNode&b) {
        return shape.getPbtyOfGivenSubNode(a, true) >
               shape.getPbtyOfGivenSubNode(b, true);
    };

    partial_sort(baseCandidates.begin(), baseCandidates.begin() +
            ceil(nPointsShape / 2.0), baseCandidates.end(), comp);

    ////////////////////// 开始RANSAC, 一次一次地尝试
    for (int nRansac = 0; nRansac < ceil(nPointsShape / 2.0); ++nRansac) {
        /// shape中选取的base的坐标
        const auto& baseOfShape = baseCandidates[nRansac];
        const TopoVec2 basePosOfShape = shape.getPosOfSubNode(baseOfShape);

        /// 开始投票, 投票箱中存储对应base的映射关系
        /// pair<this, that>
        /// 对应int的patternBase下的匹配结果
        vector<vector<pair<SubNode, SubNode>>> votesOfBase(nPointsPat);

        size_t nValidBase = 0;

        /// 先筛选哪些base是合适的
        if (baseOfShape.type == SubNodeType::GATE) {
            auto i = baseOfShape.index;
            const auto& baseGate = shape.mGates[i];
            for (int j = 0; j < pattern.nGates(); ++j) {
                if (baseGate->alike(pattern.mGates[j])) {
                    /// 合适的base所对应的pair添加base的匹配
                    votesOfBase[j].reserve(nPointsPat);
                    nValidBase++;
                    votesOfBase[j].emplace_back(make_pair(
                            SubNode(SubNodeType::GATE, i),
                            SubNode(SubNodeType::GATE, j)));
                }
            }
        }
        else if (baseOfShape.type == SubNodeType::LandMark) {
            auto i = baseOfShape.index;
            const auto& baseLM = shape.mPosLandmarks[i];
            for (int j = 0; j < pattern.mPosLandmarks.size(); ++j) {
                if (baseLM->alike(pattern.mPosLandmarks[j])) {
                    /// 合适的base所对应的pair添加base的匹配
                    votesOfBase[j].reserve(nPointsPat);
                    nValidBase++;
                    votesOfBase[j].emplace_back(make_pair(
                            SubNode(SubNodeType::LandMark, i),
                            SubNode(SubNodeType::LandMark, j)));
                }
            }
        } else {
            cerr << FILE_AND_LINE << "WRONG SUBNODE TYPE" << (int)baseOfShape.type << endl;
            throw;
        }

        if (nValidBase == 0) {
            /// 连base都没有相同的, 下一个
            continue;
        }

        /// 开始投票
        for (int idShapeGate = 0; idShapeGate < gatesOfShape.size(); ++idShapeGate) {
            auto shapeGatePosInBase = gatesOfShape[idShapeGate]->getPos() - basePosOfShape;
            if (auto bins = table.lookUpEntersAtPos(shapeGatePosInBase)) {
                for (const auto& bin : *bins) {
                    /// 找到对应base的投票箱
                    auto& vecVote = votesOfBase[bin.base.index];
                    /// 如果为空,表明base本身就不匹配, 不使用
                    if (!vecVote.empty() &&
                        bin.node.type == SubNodeType::GATE) {
                        auto idPatGate = bin.node.index;
                        const auto& shapesGate = gatesOfShape[idShapeGate];
                        const auto& patternsGate = pattern.getGates()[idPatGate];
                        if (shapesGate->alike(patternsGate)) {
                            auto& lastVote = vecVote.back();
                            if (__glibc_unlikely(lastVote.first.index == idShapeGate)) {
                                /// 如果上一个投票的first也是idShapeGate, 说明idShape在同一个base下
                                /// 命中了两个不同的Gate, 我们要的是一对一关系, 因此通过检查距离, 确定
                                /// 应该用哪个
                                const auto& basePosOfPat = pattern.getPosOfSubNode(bin.base);
                                const auto& oldPatGatePos = 
                                        pattern.getPosOfSubNode(lastVote.second);
                                const auto& newPatGatePos = pattern.mGates[idPatGate]->getPos();
                                auto corrOld = oldPatGatePos - basePosOfPat;
                                auto corrNew = newPatGatePos - basePosOfPat;
                                if ((shapeGatePosInBase - corrOld).len2() >
                                    (shapeGatePosInBase - corrNew).len2()) {
                                    /// 说明新的这个命中距离更近一些, 更改为当前的idPatGate
                                    lastVote.second.index = idPatGate;
                                }
                            } else {
                                vecVote.emplace_back(
                                        SubNode(SubNodeType::GATE, idShapeGate),
                                        SubNode(SubNodeType::GATE, idPatGate));
                            }
                        }
                    }
                }
            }
        }

        for (int idShapeLM = 0; idShapeLM < lmsOfShape.size(); ++idShapeLM) {
            auto shapeLMPosInBase = lmsOfShape[idShapeLM]->getPos() - basePosOfShape;
            if (auto bins = table.lookUpEntersAtPos(shapeLMPosInBase)) {
                for (const auto& bin : *bins) {
                    auto& vecVote = votesOfBase[bin.base.index];
                    if (!vecVote.empty() &&
                        bin.node.type == SubNodeType::LandMark) {
                        auto idPatLM = bin.node.index;
                        const auto& currentLM = lmsOfShape[idShapeLM];
                        const auto& targetLM = pattern.getPLMs()[idPatLM];
                        if (currentLM->alike(targetLM)) {
                            auto& lastVote = vecVote.back();
                            if (__glibc_unlikely(lastVote.first.index == idShapeLM)) {
                                const auto& basePosOfPat = pattern.getPosOfSubNode(bin.base);
                                const auto& oldPatLMPOS =
                                        pattern.getPosOfSubNode(lastVote.second);
                                const auto& newPatLMPOS =
                                        pattern.mPosLandmarks[idPatLM]->getPos();
                                auto corrOld = oldPatLMPOS - basePosOfPat;
                                auto corrNew = newPatLMPOS - basePosOfPat;
                                if ((shapeLMPosInBase - corrOld).len2() >
                                    (shapeLMPosInBase - corrNew).len2()) {
                                    lastVote.second.index = idPatLM;
                                }
                            } else {
                                vecVote.emplace_back(
                                        SubNode(SubNodeType::LandMark, idShapeLM),
                                        SubNode(SubNodeType::LandMark, idPatLM));
                            }
                        }
                    }
                }
            }
        }

        /// 挑选里面最好的一组结果, 以数量为准
        vector<pair<SubNode, SubNode>>* theBestPairs = nullptr;
        {
            size_t nCountMax = 0;
            for (auto& vecPairs : votesOfBase) {
                if (vecPairs.size() > nCountMax) {
                    theBestPairs = &vecPairs;
                    nCountMax = vecPairs.size();
                }
            }
        }

        if (theBestPairs == nullptr || theBestPairs->size() < nPointsShape / 2) {
            /// 如果压根没有pairs, 或者有一半的点都没有匹配, 则换一个点试试
            continue;
        } else {
            if (!shapeIsThis) {
                for (auto& pair : *theBestPairs) {
                    std::swap(pair.first.index, pair.second.index);
                }
            }
            return std::move(*theBestPairs);
        }
    }
    /// 所有的basePoint都失败了!
    return vector<std::pair<SubNode, SubNode>>();
}

vector<SubNode> ExpData::vecOfSubNodes(const ExpData& expData)
{
    auto nPointsShape = expData.nGates() + expData.mPosLandmarks.size();
    vector<SubNode> baseCandidates(nPointsShape);
    for (int i = 0; i < expData.nGates(); ++i) {
        baseCandidates[i].type = SubNodeType::GATE;
        baseCandidates[i].index = i;
    }
    for (int i = 0; i < expData.mPosLandmarks.size(); ++i) {
        baseCandidates[i + expData.nGates()].type = SubNodeType::LandMark;
        baseCandidates[i + expData.nGates()].index = i;
    }
    return baseCandidates;
}

std::pair<ExpDataPtr, std::vector<SubNode>>
ExpData::buildShrinkedCopy(bool copyAccordingSubLinks,
                           const std::vector<SubNode>& whiteList,
                           double carefulPercentage, size_t nErasedNode) const {
    /// @note 这是个工具函数, 所以完全没考虑效率问题
    static default_random_engine engine(
            std::chrono::system_clock::now().time_since_epoch().count());

    /// 根据SubLinks来部分拷贝
    std::pair<ExpDataPtr, std::vector<SubNode>> res;
    res.first = this->cloneShell();
    res.second = this->copy2(res.first.get(), copyAccordingSubLinks);

    /// 看看本次是否需要随机忘记一些节点
    uniform_real_distribution<> r(0.0, 1.0);
    if (r(engine) >= carefulPercentage) {
        /// 是的
        const auto& copy = res.first;
        const auto& nodesMap2Copy = res.second;
        /// 使用这个变量记录copy中可以被抹去的节点
        auto nodesOfCopy = vecOfSubNodes(*copy);
        /// 根据白名单, 从nodesOfCopy中剔除对应的SubNode, 先标记为UNSET
        for (auto guanXiHu : whiteList) {
            /// 将白名单节点转换到copy对应的SubNode编号
            auto xNodeInCopy = nodesMap2Copy[guanXiHu.toUIndex(this->nGates())];
            if (xNodeInCopy.type != SubNodeType::UNSET) {
                /// 如果白名单的确被使用了, 那么将其标记为要删除, 这里用UNSET实现
                nodesOfCopy[xNodeInCopy.toUIndex(copy->nGates())].type = SubNodeType::UNSET;
            }
        }

        /// 如果是走廊, 我们不希望忽略端点, 因为那样就不方便可视化了, 把端点也保留下来
        if (auto pCorridor = dynamic_cast<Corridor*>(copy.get())) {
            if (pCorridor->getEndGateA() >= 0) {
                nodesOfCopy[pCorridor->getEndGateA()].type = SubNodeType::UNSET;
            }
            if (pCorridor->getEndGateB() >= 0) {
                nodesOfCopy[pCorridor->getEndGateB()].type = SubNodeType::UNSET;
            }
        }

        /// 把这些白名单从可能被杀掉的node候选里消掉
        {
            vector<SubNode> temp;
            for (const auto& subNode: nodesOfCopy) {
                if (subNode.type != SubNodeType::UNSET) {
                    temp.emplace_back(subNode);
                }
            }
            nodesOfCopy = std::move(temp);
        }

        /// 主要不要让要"忘记"的节点数量多于可以忘掉的数量
        if (nErasedNode >= nodesOfCopy.size()) {
            nErasedNode = nodesOfCopy.size();
            cout << FILE_AND_LINE << " you erase almost all points in expData!" << endl;
        }

        /// 警告一下SubLinks我们无法处理
        if (nErasedNode >= 0 && !copy->mSubLinks.empty()) {
            cerr << FILE_AND_LINE << " the sublinks will be affected!" << endl;
        }

        /// 随机选一些幸运儿,然后忘掉并删掉
        for (int i = 0; i < nErasedNode; ++i) {
            uniform_int_distribution<> p(0,nodesOfCopy.size());
            int luckyNumber = p(engine);
            const auto luckBoy = nodesOfCopy[luckyNumber];
            bool startMinus = false;
            for (auto& item: res.second) {
                if (!startMinus && item == luckBoy) {
                    /// 找到luckBoy在映射表中的位置, 设置为没有映射 (这里我们不注重效率)
                    startMinus = true;
                    item.type = SubNodeType::UNSET;
                }
                /// 随后把和其相同type的node的index--
                else if (startMinus) {
                    if (item.type == luckBoy.type) {
                        item.index -= 1;
                    } else {
                        /// 相同类型已经结束了, 可以结束循环了
                        break;
                    }
                }
            }
            copy->eraseSubNode(luckBoy);
            nodesOfCopy.erase(nodesOfCopy.begin() + i);
        }
    }
    
    return res;
}

SubNode ExpData::copySubNode2(ExpData& dest, const SubNode& subNode) const
{
    switch (subNode.type) {
        case SubNodeType::GATE:
            dest.addGate(this->mGates[subNode.index]->clone());
            return SubNode{SubNodeType::GATE,
                           static_cast<uint32_t>(dest.mGates.size() - 1)};
        case SubNodeType::LandMark:
            dest.addLandmark(this->mPosLandmarks[subNode.index]->clone());
            return SubNode{SubNodeType::LandMark,
                           static_cast<uint32_t>(dest.mPosLandmarks.size() - 1)};
        default:
            cerr << FILE_AND_LINE << "Unknown SubNode type!" << (int)subNode.type << endl;
            return SubNode();
    }
}

void ExpData::addNoise(double maxOdomErrPerM, double maxDegreeErr)
{
    static default_random_engine engine(
            std::chrono::system_clock::now().time_since_epoch().count());

    /// 寻找最长的两点
    SubNode p0, p1;
    double maxLen = 0.0;
    auto subnodes = vecOfSubNodes(*this);
    for (const auto& node1: subnodes) {
        for (const auto& node2: subnodes) {
            if (node1 == node2) continue;
            const auto& pn1 = this->getPosOfSubNode(node1);
            const auto& pn2 = this->getPosOfSubNode(node2);
            double len = (pn1 - pn2).len();
            if (len > maxLen) {
                maxLen = len;
                p0 = node1;
                p1 = node2;
            }
        }
    }
    /// 其中的一个作为原点, 计算得到单位向量
    TopoVec2 O = this->getPosOfSubNode(p0);
    TopoVec2 X = (this->getPosOfSubNode(p1) - O).unitize();
    TopoVec2 Y = X.rotate(90);

    /// 随机得到两个方向上的噪声 ex ey
    TopoVec2 ex{}, ey{};
    normal_distribution<> g{0, maxOdomErrPerM / 3.5};
    while (true) {
        ex.px = g(engine);
        ex.py = g(engine);
        ey.px = g(engine);
        ey.py = g(engine);
        if (ex.len() + ey.len() > maxOdomErrPerM) {
            continue;
        }
        break;
    }
    
    if (auto thisCorridor = dynamic_cast<Corridor*>(this)) {
        {
            const auto& pA = thisCorridor->getEndPointA();
            const auto& corrA = pA - O;
            const auto& errA = ex * corrA.dotProduct(X) + ey * corrA.dotProduct(Y);
            thisCorridor->setEndPointA(pA + errA);
        }

        {
            const auto& pB = thisCorridor->getEndPointB();
            const auto& corrB = pB - O;
            const auto& errB = ex * corrB.dotProduct(X) + ey * corrB.dotProduct(Y);
            thisCorridor->setEndPointB(pB + errB);
        }
    }

    for (const auto& pGate : this->getGates()) {
        const auto& gatePos = pGate->getPos();
        /// 根据坐标,添加噪声
        const auto& corr = gatePos - O;
        const auto& err = ex * corr.dotProduct(X) + ey * corr.dotProduct(Y);
        pGate->setPos(gatePos + err);

        /// 添加法向量噪声
        while(true) {
            normal_distribution<> dg{0, maxDegreeErr / 2.0};
            double derr = dg(engine);
            if (abs(derr) < maxDegreeErr) {
                pGate->setNormalVec(pGate->getNormalVec().rotate(derr));
                break;
            }
        }
    }

    for (const auto& pLM : this->mPosLandmarks) {
        const auto& gatePos = pLM->getPos();
        const auto& corr = gatePos - O;
        const auto& err = ex * corr.dotProduct(X) + ey * corr.dotProduct(Y);
        pLM->setPos(gatePos + err);
    }
}

void ExpData::eraseSubNode(const SubNode& target)
{
    switch (target.type) {
        case SubNodeType::GATE:
            mGates.erase(mGates.begin() + target.index);
            break;
        case SubNodeType::LandMark:
            mPosLandmarks.erase(mPosLandmarks.begin() + target.index);
            break;
        default:
            cerr << FILE_AND_LINE << " UNKNOWN SubNode Type: " << (int)target.index << endl;
            break;
    }
}
