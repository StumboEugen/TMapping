//
// Created by stumbo on 2019/12/20.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_UI_VICEGVIEW_H
#define TMAPPING_INCLUDE_TMAPPING_UI_VICEGVIEW_H

#include <vector>

#include <QGraphicsView>
#include <QWidget>
#include <QGraphicsItem>

#include "tmapping/tools/TopoParams.h"
#include "tmapping/expDataTypes/ExpDataTypes.h"

namespace tmap
{

class ReferPoint;

class QGate : public QGraphicsItem
{
    GatePtr mData;

public:
    explicit QGate(GatePtr data);

    void changeNormalVec(const QPointF& toPointInScene);

    QRectF boundingRect() const override;

    void
    paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    const GatePtr& getGateData() const;
}; /// end of QGate

class ViceGView : public QGraphicsView
{
    Q_OBJECT

    enum class DisplayStatus{NOTHING, BUILDING_EXP, DRAWING_GATE};

    DisplayStatus mStatus;
    ExpDataPtr mRelatedExpData;
    std::vector<QGate*> mQGates;
    QGraphicsScene mScene;
    GateType mNextGateType = GateType::GateWay;

public:
    explicit ViceGView(QWidget* parent = nullptr);

    void beginExpBuilding(ExpDataType type);

    ExpDataPtr completeExpBuilding();

    void setNextGateType(GateType type);

    void startDrawingGateFromReferPoint(const ReferPoint& rp);

protected:
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_VICEGVIEW_H
