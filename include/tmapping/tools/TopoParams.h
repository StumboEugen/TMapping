//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_TOPOPARAMS_H
#define TMAPPING_TOPOPARAMS_H

#include <cstdint>
#include <list>
#include <string>
#include <vector>
#include <cmath>
#include <memory>

#include "json/json.h"

#define FILE_AND_LINE __FILE__ << ':' << __LINE__

#define TMAPPING_CONFIG_RECORD_POSS
#define TMAPPING_CONFIG_LOG_VERBOSE false
#define TMAPPING_CONFIG_LOG_TIME false
#define TMAPPING_CONFIG_DEBUG_MODE false

namespace tmap
{
static constexpr uint64_t TOPO_JSON_VERSION = 0;
using Jsobj = Json::Value;

/// 表示Gate的序号,使用8位整形, 一个节点不太有可能有128个门
using GateID = int8_t;
static constexpr GateID GATEID_NO_MAPPING = -1;
static constexpr GateID GATEID_BEGINNING_POINT = -2;
static constexpr GateID GATEID_HAVENT_LEFT = -3;
static constexpr GateID GATEID_CORRIDOR_NO_ENDPOINT = -4;
static constexpr GateID GATEID_NOT_FOUND = -5;

/// 几何哈希的噪声冗余填充半径
static constexpr double maxErrForGeoHash = 0.4;
/// 默认的建图过程中每一步保留的地图数量
static constexpr size_t maxMapNumberPerStep = 200;
/// 算法预估的噪声标准差
static constexpr double stdErrPerMeter = 0.1;
static constexpr double convErrPerMeter = stdErrPerMeter * stdErrPerMeter;

static constexpr char TMAP_STD_FILE_SAVE_FLODER_NAME[] = "tmappingMaps/";
static constexpr char TMAP_STD_SERVICE_NAME_NEW_EXP[] = "tmapping/srv/newExp";
static constexpr char TMAP_STD_SERVICE_NAME_GATE_MOVE[] = "tmapping/srv/gateMove";
static constexpr char TMAP_STD_SERVICE_NAME_GET_MAPS[] = "tmapping/srv/getMaps";
static constexpr char TMAP_STD_SERVICE_NAME_SET_SURVIVERS[] = "tmapping/srv/setSurvivers";
static constexpr char TMAP_STD_SERVICE_NAME_RESET[] = "tmapping/srv/reset";
static constexpr char TMAP_STD_SERVICE_GET_CHAMPION_HISTORY[] = "tmapping/srv/championHistory";
static constexpr char TMAP_STD_SERVICE_SAVE_CHAMPION[] = "tmapping/srv/saveChampionMap";

/// 第二轮的时候, 任何概率都接受, 哪些很差的, 交给固定数量去干掉
static constexpr double TOLLERANCE_2ND_MATCH_MERGEDEXP = 0.0;

class MapTwig;

using MapTwigPtr = std::shared_ptr<MapTwig>;
using MapTwigWePtr = std::weak_ptr<MapTwig>;
using MapTwigUnPtr = std::unique_ptr<MapTwig>;

class MergedExp;

using MergedExpWePtr = std::weak_ptr<MergedExp>;
using MergedExpPtr = std::shared_ptr<MergedExp>;

class Exp;

using ExpWePtr = std::weak_ptr<Exp>;
using ExpPtr = std::shared_ptr<Exp>;
using ExpCPtr = std::shared_ptr<const Exp>;

class MapNode;
using MapNodePtr = std::shared_ptr<MapNode>;
using MapNodeWe = std::weak_ptr<MapNode>;

class StructedMapImpl;
using StructedMap = std::shared_ptr<StructedMapImpl>;
}

#endif //TMAPPING_TOPOPARAMS_H
