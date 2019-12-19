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


namespace tmap
{
static constexpr uint64_t TOPO_JSON_VERSION = 0;
using Jsobj = Json::Value;

/// maybe one thousand years later, 128 could run out
using GateID = int8_t;
static constexpr GateID GATEID_NO_MAPPING = -1;
static constexpr GateID GATEID_BEGINNING_POINT = -2;
static constexpr GateID GATEID_HAVENT_LEFT = -3;

static constexpr double convEdgePerMeter = 0.02;
static constexpr double stdDevEdgePerMeter = 0.1 * M_SQRT2;
static constexpr double stdDevEdgePerMeterOneAx = 0.1;
static constexpr double convEdgePerMeterOneAx = 0.01;
/// in RAD
static constexpr double convNodePerGate = 0.04;

static constexpr double convDistPerMeter = 0.01;

static constexpr double piHalf = 3.1415926 / 2.0;
static constexpr double pi = 3.1415926;
static constexpr double piTwo = 3.1415926 * 2.0;

static constexpr double DEG2RAD = pi / 180;
static constexpr double RAD2DEG = 180 / pi;

static constexpr char TMAP_STD_FILE_SAVE_FLODER_NAME[] = "tmappingMaps/";
static constexpr char TMAP_STD_SERVICE_NAME_NEW_EXP[] = "tmapping/srv/newExp";
static constexpr char TMAP_STD_SERVICE_NAME_GATE_MOVE[] = "tmapping/srv/gateMove";
static constexpr char TMAP_STD_SERVICE_NAME_GET_MAPS[] = "tmapping/srv/getMaps";


//static constexpr char TOPO_STD_TOPIC_NAME_NODEINFO[] = "topo/ArriveAtNewNode";
//static constexpr char TOPO_STD_TOPIC_NAME_GATEMOVE[] = "topo/LeaveFromNode";
//static constexpr char TOPO_STD_TOPIC_NAME_CVINFO[] = "topo/cvInfo";
//static constexpr char TOPO_STD_SERVICE_NAME_SAVEMAP[] = "topoSrv/SaveMap";
//static constexpr char TOPO_STD_SERVICE_NAME_GETMAPS[] = "topoSrv/GetMaps";
//static constexpr char TOPO_STD_SERVICE_NAME_PATHPLANNING[] = "topoSrv/PathPlanning";
//static constexpr char TOPO_STD_SERVICE_NAME_ASKINGNEXTSTEP[] = "topoSrv/NextPathStep";

static constexpr double TOLLERANCE_1ST_MATCH_EXP = 0.5;
static constexpr double TOLLERANCE_2ND_MATCH_MERGEDEXP = 0.75;

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
