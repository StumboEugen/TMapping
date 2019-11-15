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

#include "json/json.h"

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

static constexpr char TOPO_STD_FILE_SAVE_FLODER_NAME[] = "topoMaps/";
static constexpr char TOPO_STD_TOPIC_NAME_NODEINFO[] = "topo/ArriveAtNewNode";
static constexpr char TOPO_STD_TOPIC_NAME_GATEMOVE[] = "topo/LeaveFromNode";
static constexpr char TOPO_STD_TOPIC_NAME_CVINFO[] = "topo/cvInfo";
static constexpr char TOPO_STD_SERVICE_NAME_SAVEMAP[] = "topoSrv/SaveMap";
static constexpr char TOPO_STD_SERVICE_NAME_GETMAPS[] = "topoSrv/GetMaps";
static constexpr char TOPO_STD_SERVICE_NAME_PATHPLANNING[] = "topoSrv/PathPlanning";
static constexpr char TOPO_STD_SERVICE_NAME_ASKINGNEXTSTEP[] = "topoSrv/NextPathStep";

#endif //TMAPPING_TOPOPARAMS_H
