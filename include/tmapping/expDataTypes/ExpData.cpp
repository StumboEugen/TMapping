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

    /// TODO 挑选合适的模型
    size_t nPointsThis = this->nGates() + this->mPosLandmarks.size();
    size_t nPointsThat = this->nGates() + this->mPosLandmarks.size();

    const auto& table = that.getHashTable();

    /// 选取最合适的基点 TODO RANSAC
    double maxPoss = 0.0;
    const auto& gates = this->getGates();
    const auto& lms = this->getPLMs();
    SubNode baseNode{SubNodeType::GATE, 0};
    for (int i = 0; i < this->getGates().size(); ++i) {
        if (gates[i]->getPossibility() > maxPoss) {
            maxPoss = gates[i]->getPossibility();
            baseNode.index = i;
        }
    }

    /// 确定数据集合匹配基点的坐标
    const TopoVec2* basePos;
    switch (baseNode.type) {
        case SubNodeType::GATE:
            basePos = &this->getGates()[baseNode.index]->getPos();
            break;
        case SubNodeType::LandMark:
            basePos = &this->getPLMs()[baseNode.index]->getPos();
            break;
        default:
            cerr << FILE_AND_LINE << " error!";
            throw;
    }

    /// 开始投票, 投票箱中存储对应base的映射关系
    unordered_map<int, vector<pair<SubNode, SubNode>>> records;
    records.reserve(max(nPointsThat, nPointsThis));
    for (int i = 0; i < gates.size(); ++i) {
        if(auto res = table.lookUpEntersAtPos(gates[i]->getPos() - *basePos)) {
            for (const auto& bin : *res) {
                if (bin.base.type == baseNode.type &&
                    bin.node.type == SubNodeType::GATE) {
                    auto j = bin.node.index;
                    const auto& currentGate = gates[i];
                    const auto& targetGate = that.getGates()[j];
//                    if (currentGate->alike(targetGate)) { /// TODO ALIKE
                    {
                        records[bin.base.index].push_back(make_pair(
                                SubNode(SubNodeType::GATE, i),
                                SubNode(SubNodeType::GATE, j)));
                    }
                }
            }
        }
    }

    for (int i = 0; i < lms.size(); ++i) {
        /// TODO LMS
    }

    size_t winnerIndex;
    const vector<pair<SubNode, SubNode>>* pairMapping;
    {
        size_t nCountMax = 0;
        for (const auto& record : records) {
            if (record.second.size() > nCountMax) {
                winnerIndex = record.first;
                pairMapping = &record.second;
            }
        }
    }

    /// TODO PLC
}

double ExpData::quickMatch(const ExpData& another, double selfWeight) const
{
    /// 快速的数据匹配工作
    if (another.type() == this->type()) {
        return 1.0;
    } else {
        return 0.0;
    }
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

