//
// Created by stumbo on 2020/1/9.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_UI_QROBOT_H
#define TMAPPING_INCLUDE_TMAPPING_UI_QROBOT_H

#include <QGraphicsEllipseItem>
#include "QNode.h"

namespace tmap
{

class QRobot : public QGraphicsEllipseItem
{
    QNodePtr atNode;
public:
    explicit QRobot(QNodePtr at);

    void updatePos();

    virtual ~QRobot();
};

using QRobotPtr = std::unique_ptr<QRobot>;
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_QROBOT_H
