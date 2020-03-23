//
// Created by stumbo on 2019/11/15.
//

#include <cfloat>
#include <iostream>
#include <algorithm>

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
    res->gateMapping2this.assign(nPointsThat + nGateThis, GATEID_NO_MAPPING);

    /// 匹配得到两个点集之间的配对情况
    vector<std::pair<SubNode, SubNode>> pointsMap;
    if (nPointsThat >= nPointsThis) {
        pointsMap = matchPairs(*this, that, true);
    } else {
        pointsMap = matchPairs(that, *this, false);
    }

    /// 开始融合出我们的结果, 首先生成一个没有具体信息的新 ExpData
    res->mergedExpData = that.cloneShell();

    TopoVec2 thatCenter{}, thisCenter{};

    /// 遍历我们获得的最佳匹配
    for (const auto& pair : pointsMap) {
        const auto& thisNode = pair.first;
        const auto& thatNode = pair.second;
        /// 累加后从而认得到匹配点集的中心
        thisCenter += *this->getPosOfSubNode(thisNode);
        thatCenter += *that.getPosOfSubNode(thatNode);
        /// 记录映射关系, 目前表中存储包括gate和landmark的所有映射,而不只是gate
        auto& idMerged = res->gateMapping2mergedExpData[thisNode.toUIndex(nGateThis)];
        if (idMerged != GATEID_NO_MAPPING) {
            cerr << FILE_AND_LINE << "double trend happened[idMerged]!" << endl;
        }
        auto& idThis = res->gateMapping2this[thatNode.toUIndex(nGateThat)];
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
    double C0 = convEdgePerMeter * (1.0 + 1.0 / selfWeight);

    /// @note 这里为了保证index的一致性,我们从that开始遍历
    /// 首先遍历that的所有gate
    for (int idThat = 0; idThat < nGateThat; ++idThat) {
        auto idThis = res->gateMapping2this[idThat];
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
        auto idThis = res->gateMapping2this[idThat + nGateThat];
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

    const size_t& nGateMerged = res->mergedExpData->nGates();
    res->displacement = -setsDiff;

    /// 融合节点到老观测的映射表中nGateThat以后的部分必然没有映射关系,
    /// 这里原本的数据有landemark的映射关系
    for (int i = nGateThat; i < nGateMerged; ++i) {
        res->gateMapping2this[i] = GATEID_NO_MAPPING;
    }

    { /// 从新观测到老的映射表, 从nGateMerged开始erase
        const auto& b = res->gateMapping2this.begin();
        const auto& e = res->gateMapping2this.end();
        res->gateMapping2this.erase(b + nGateMerged,e);
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
    std::array<double, 4> res{DBL_MIN, DBL_MAX, DBL_MAX, DBL_MIN};
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

void ExpData::copy2(ExpData* copy2)
{
    copy2->mGates.reserve(this->mGates.size());
    for (auto& gate : this->mGates) {
        copy2->mGates.emplace_back(gate->clone());
    }

    copy2->mPosLandmarks.reserve(this->mPosLandmarks.size());
    for (auto& plm : this->mPosLandmarks) {
        copy2->mPosLandmarks.emplace_back(plm->clone());
    }

    copy2->mName = this->mName;
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

const TopoVec2* ExpData::getPosOfSubNode(const SubNode& node) const
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
    return res;
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

    vector<SubNode> baseCandidates(nPointsShape, {SubNodeType::UNSET, 0});
    for (int i = 0; i < shape.nGates(); ++i) {
        baseCandidates[i].type = SubNodeType::GATE;
        baseCandidates[i].index = i;
    }
    for (int i = 0; i < shape.mPosLandmarks.size(); ++i) {
        baseCandidates[i + shape.nGates()].type = SubNodeType::LandMark;
        baseCandidates[i + shape.nGates()].index = i;
    }

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
        const TopoVec2* basePosOfShape = shape.getPosOfSubNode(baseOfShape);

        /// 开始投票, 投票箱中存储对应base的映射关系
        /// pair<this, that>
        /// 对应int的patternBase下的匹配结果
        vector<vector<pair<SubNode, SubNode>>> hitsOfBase(nPointsPat);
        /// TODO 发生歧义匹配时应该怎么办? 1->2 3->2

        size_t nValidBase = 0;

        /// 先筛选哪些base是合适的
        if (baseOfShape.type == SubNodeType::GATE) {
            auto i = baseOfShape.index;
            const auto& baseGate = shape.mGates[i];
            for (int j = 0; j < pattern.nGates(); ++j) {
                if (baseGate->alike(pattern.mGates[j])) {
                    /// 合适的base所对应的pair添加base的匹配
                    hitsOfBase[j].reserve(nPointsPat);
                    nValidBase++;
                    if (shapeIsThis) {
                        hitsOfBase[j].emplace_back(make_pair(
                                SubNode(SubNodeType::GATE, i),
                                SubNode(SubNodeType::GATE, j)));
                    } else {
                        hitsOfBase[j].emplace_back(make_pair(
                                SubNode(SubNodeType::GATE, j),
                                SubNode(SubNodeType::GATE, i)));
                    }
                }
            }
        }
        else if (baseOfShape.type == SubNodeType::LandMark) {
            auto i = baseOfShape.index;
            const auto& baseLM = shape.mPosLandmarks[i];
            for (int j = 0; j < pattern.mPosLandmarks.size(); ++j) {
                if (baseLM->alike(pattern.mPosLandmarks[j])) {
                    /// 合适的base所对应的pair添加base的匹配
                    hitsOfBase[j].reserve(nPointsPat);
                    nValidBase++;
                    if (shapeIsThis) {
                        hitsOfBase[j].emplace_back(make_pair(
                                SubNode(SubNodeType::LandMark, i),
                                SubNode(SubNodeType::LandMark, j)));
                    } else {
                        hitsOfBase[j].emplace_back(make_pair(
                                SubNode(SubNodeType::LandMark, j),
                                SubNode(SubNodeType::LandMark, i)));
                    }
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
            if (auto bins = table.lookUpEntersAtPos(gatesOfShape[idShapeGate]->getPos() - *basePosOfShape)) {
                if (bins != nullptr) {
                    for (const auto& bin : *bins) {
                        auto& hitVec = hitsOfBase[bin.base.index];
                        /// 如果为空,表明base本身就不匹配, 不使用
                        if (!hitVec.empty() &&
                            bin.node.type == SubNodeType::GATE) {
                            auto idPatGate = bin.node.index;
                            const auto& shapesGate = gatesOfShape[idShapeGate];
                            const auto& patternsGate = pattern.getGates()[idPatGate];
                            if (shapesGate->alike(patternsGate)) {
                                if (shapeIsThis) {
                                    hitVec.emplace_back(
                                            SubNode(SubNodeType::GATE, idShapeGate),
                                            SubNode(SubNodeType::GATE, idPatGate));
                                } else {
                                    hitVec.emplace_back(
                                            SubNode(SubNodeType::GATE, idPatGate),
                                            SubNode(SubNodeType::GATE, idShapeGate));
                                }
                            }
                        }
                    }
                }
            }
        }

        for (int idShapeLM = 0; idShapeLM < lmsOfShape.size(); ++idShapeLM) {
            if (auto bins = table.lookUpEntersAtPos(lmsOfShape[idShapeLM]->getPos() - *basePosOfShape)) {
                if (bins != nullptr) {
                    for (const auto& bin : *bins) {
                        auto& hitVec = hitsOfBase[bin.base.index];
                        if (!hitVec.empty() &&
                            bin.node.type == SubNodeType::LandMark) {
                            auto idPatLM = bin.node.index;
                            const auto& currentLM = lmsOfShape[idShapeLM];
                            const auto& targetLM = pattern.getPLMs()[idPatLM];
                            if (currentLM->alike(targetLM)) {
                                if (shapeIsThis) {
                                    hitVec.emplace_back(
                                            SubNode(SubNodeType::LandMark, idShapeLM),
                                            SubNode(SubNodeType::LandMark, idPatLM));
                                } else {
                                    hitVec.emplace_back(
                                            SubNode(SubNodeType::LandMark, idPatLM),
                                            SubNode(SubNodeType::LandMark, idShapeLM));
                                }
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
            for (auto& vecPairs : hitsOfBase) {
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
            return std::move(*theBestPairs);
        }
    }
    /// 所有的basePoint都失败了!
    return vector<std::pair<SubNode, SubNode>>();
}

