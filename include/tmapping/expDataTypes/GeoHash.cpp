//
// Created by stumbo on 2020/3/18.
//

#include "GeoHash.h"
#include "ExpDataTypes.h"

using namespace tmap;

static constexpr double TABLE_RES = 0.25;

/**
 * @brief 计算用于GeoHash::mTable的坐标位置
 */
static int64_t CC(int64_t x, int64_t y) {
    return x + y * 1000;
}

tmap::GeoHash::GeoHash(const ExpData& oriData, double odomErr)
{
    const auto& gates = oriData.getGates();
    const auto& lms = oriData.getPLMs();

    for (uint32_t i = 0; i < gates.size(); ++i) {
        SubNode base{SubNodeType::GATE, i};
        SubNode anotherNode{SubNodeType::GATE, 0};
        const auto& zeroPos = gates[i]->getPos();

        for (uint32_t j = 0; j < gates.size(); ++j) {
            if (i == j) continue;
            const auto& targetPos = gates[j]->getPos();
            TopoVec2 posDiff = targetPos - zeroPos;
            double err = posDiff.len() * odomErr;
            anotherNode.index = j;
            fillEntrances(targetPos, err, base, anotherNode);
        }

        anotherNode.type = SubNodeType::LandMark;
        for (uint32_t j = 0; j < lms.size(); ++j) {
            const auto& targetPos = lms[j]->getPos();
            TopoVec2 posDiff = targetPos - zeroPos;
            double err = posDiff.len() * odomErr;
            anotherNode.index = j;
            fillEntrances(targetPos, err, base, anotherNode);
        }
    }

    for (uint32_t i = 0; i < lms.size(); ++i) {
        SubNode base{SubNodeType::LandMark, i};
        SubNode anotherNode{SubNodeType::GATE, 0};
        const auto& zeroPos = lms[i]->getPos();

        for (uint32_t j = 0; j < gates.size(); ++j) {
            const auto& targetPos = gates[j]->getPos();
            TopoVec2 posDiff = targetPos - zeroPos;
            double err = posDiff.len() * odomErr;
            anotherNode.index = j;
            fillEntrances(targetPos, err, base, anotherNode);
        }

        anotherNode.type = SubNodeType::LandMark;
        for (uint32_t j = 0; j < lms.size(); ++j) {
            if (i == j) continue;
            const auto& targetPos = lms[j]->getPos();
            TopoVec2 posDiff = targetPos - zeroPos;
            double err = posDiff.len() * odomErr;
            anotherNode.index = j;
            fillEntrances(targetPos, err, base, anotherNode);
        }
    }
}

const std::vector<Entrance>* tmap::GeoHash::lookUpEntersAtPos(const tmap::TopoVec2& pos) const
{
    int64_t X = floor(pos.px / TABLE_RES);
    int64_t Y = floor(pos.py / TABLE_RES);
    auto res = mTable.find(CC(X,Y));
    if (res == mTable.end()) {
        return nullptr;
    } else {
        return &res->second;
    }
}

void GeoHash::fillEntrances(TopoVec2 midPos, double err,
        const SubNode& base, const SubNode& anotherNode)
{
    Entrance entrance{base, anotherNode};
    err = std::max(err, TABLE_RES / 2);
    double errInRes = err / TABLE_RES;
    double r2 = errInRes * errInRes; //r^2
    double xmid = midPos.px / TABLE_RES;
    double ymid = midPos.py / TABLE_RES;
    double yu = ymid + errInRes; // up
    double yd = ymid - errInRes; // down
    double xr = xmid + errInRes; // right
    double xl = xmid - errInRes; // left

    const int64_t MIDY = floor(ymid);

    for (int64_t y = floor(yu); y > MIDY; --y) {
        double dy = y - ymid;
        double dx = sqrt(r2 - dy*dy);
        int b = floor(xmid - dx);
        int e = floor(xmid + dx);
        for (int x = b; x <= e; ++x) {
            mTable[CC(x, y)].push_back(entrance);
        }
    }

    for (int x = floor(xl); x <= floor(xr); ++x) {
        mTable[CC(x, MIDY)].push_back(entrance);
    }

    for (int64_t y = floor(yd); y < MIDY; ++y) {
        double dy = ymid - (y + 1);
        double dx = sqrt(r2 - dy*dy);
        int b = floor(xmid - dx);
        int e = floor(xmid + dx);
        for (int x = b; x <= e; ++x) {
            mTable[CC(x, y)].push_back(entrance);
        }
    }
}
