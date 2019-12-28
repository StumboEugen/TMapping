//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_CORRIDOR_H
#define TMAPPING_CORRIDOR_H

#include "ExpData.h"

namespace tmap
{

class Corridor : public ExpData
{

    GateID mEndPointA = GATEID_CORRIDOR_NO_ENDPOINT;
    GateID mEndPointB = GATEID_CORRIDOR_NO_ENDPOINT;

public:

    ExpDataType type() const override { return ExpDataType::Corridor; }

    Json::Value toJS() const override;

    ExpDataPtr clone() override;

    GateID getEndPointA() const;

    GateID getEndPointB() const;

    void setEndPointA(GateID endPointA);

    void setEndPointB(GateID endPointB);
};
}


#endif //TMAPPING_CORRIDOR_H
