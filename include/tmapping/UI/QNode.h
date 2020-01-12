//
// Created by stumbo on 2019/12/30.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_UI_QNODE_H
#define TMAPPING_INCLUDE_TMAPPING_UI_QNODE_H

#include "tmapping/StructedMap.h"
#include <QGraphicsItem>

namespace tmap
{

class QNode;
using QNodePtr = std::shared_ptr<QNode>;

class FakeLine_IMPL : public QGraphicsLineItem {
    std::array<QPointF, 2> mPoints;
    QNode* const mOriNode;
    GateID const mFrom;

public:
    void
    paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    QPainterPath shape() const override;

    void setPoint(QNode* who, const QPointF& newP);

    QNode* oriNode() const;

    GateID fromGate() const;

    FakeLine_IMPL(const QPointF& p1, const QPointF& p2, QNode* node1, GateID fromGate);

    void sucide();

    ~FakeLine_IMPL() override;
};

using FakeLine = std::shared_ptr<FakeLine_IMPL>;


enum class MoveStragety
{
    EVERY_NODE = 0,
    ONLY_FIXED_TYPE = 1,
    NO_NODE_FOLLOW = 2
};

class QNode : public QGraphicsItem, public MapNode
{
    mutable QRectF mBoundingRect;
    std::vector<FakeLine> mFakeLines;
    std::vector<bool> mDrawGate;
    GateID mHighLightGate = -1;
    MoveStragety mMoveStragety;

private:
    explicit QNode(MergedExpPtr mergedExp);
    void refreshGateDrawing();
    void removeConnection(size_t gid);

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
    void hoverMoveEvent(QGraphicsSceneHoverEvent* event) override;
    void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override;

public:
    void breakLinks();

    void notifyNeighbours2Move();

    static QNodePtr makeOneFromExpData(const ExpDataPtr& relatedExpData, MoveStragety ms);

    static QNodePtr makeOneFromMergedExp(const MergedExpPtr& relatedMergedExp, MoveStragety ms);

    QRectF boundingRect() const override;

    void
    paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    void notifySizeChange();

    QPainterPath shape() const override;

    void normalizePos();

    ~QNode() override;

    QNodePtr thisQnodePtr();

    QNodePtr qNodeAt(size_t index);

    FakeLine& fakeLineAt(size_t index);

    QPointF gateQPos(size_t index, bool atScene = true) const;

    QPointF plmQPos(size_t index, bool atScene = true) const;

    void mouseHoverAt(const QPointF& at);

    void setMoveStragety(MoveStragety moveStragety);
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_QNODE_H
