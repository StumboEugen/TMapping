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

    enum class DisplayStatus{NOTHING, BUILDING_EXP, DRAWING_GATE, DISPLAYING_EXP};

    DisplayStatus mStatus = DisplayStatus::NOTHING;
    ExpDataPtr mRelatedExpData;
    std::vector<QGate*> mQGates;
    std::string mNextLM = "";

    QGraphicsScene mScene;
    GateType mNextGateType = GateType::GateWay;

public:
    explicit ViceGView(QWidget* parent = nullptr);

    void beginExpBuilding(ExpDataType type, double size);

    ExpDataPtr completeExpBuilding();

    void setNextGateType(GateType type);

    /// 添加Gate时, 先添加为QGate, 最后complete时再加入到mRelatedExpData
    void startDrawingGateFromReferPoint(const ReferPoint& rp);

    /// 添加Landmark时则直接添加到mRelatedExpData中去, 绘制使用默认圆圈
    void addLandMark(const ReferPoint& rp);

    void displayTheExpData(ExpDataPtr data2show);

protected:
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

public Q_SLOTS:
    void SLOT_NextLandmarkStr(QString str);
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_VICEGVIEW_H
