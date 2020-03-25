//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_SMALLROOM_H
#define TMAPPING_SMALLROOM_H

#include "ExpData.h"

namespace tmap
{

class Room : public ExpData
{
    double mScaling = 1.0;

public:
    ExpDataType type() const override { return ExpDataType::Room; }

    Json::Value toJS() const override;

    ExpDataPtr clone() const override;

    ExpDataPtr cloneShell() const override;

    void setScaling(double scaling);

    double getScaling() const;
};
}


#endif //TMAPPING_SMALLROOM_H
