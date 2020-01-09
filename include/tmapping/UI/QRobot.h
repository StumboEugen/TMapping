//
// Created by stumbo on 2020/1/9.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_UI_QROBOT_H
#define TMAPPING_INCLUDE_TMAPPING_UI_QROBOT_H

#include <QGraphicsEllipseItem>
#include "QNode.h"

#include <tmapping/Exp.h>

namespace tmap
{

class QRobot : public QGraphicsEllipseItem
{
    QNodePtr atNode;

    ExpPtr theLastMovedExp;

public:
    explicit QRobot(QNodePtr at);

    void updatePos();

    ~QRobot() override;

    ExpPtr try2move(QPointF scenePos);

};

using QRobotPtr = std::unique_ptr<QRobot>;
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_QROBOT_H
