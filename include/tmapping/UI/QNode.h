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

class QNode : public QGraphicsItem, public MapNode, public std::enable_shared_from_this<QNode>
{
    mutable QRectF mBoundingRect;

private:
    explicit QNode(MergedExpPtr mergedExp);

protected:
    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;


public:
    void notifyNeighbours2Move() const;

    static QNodePtr makeOneFromExpData(const ExpDataPtr& relatedExpData);

    static QNodePtr makeOneFromMergedExp(const MergedExpPtr& relatedMergedExp);

    QRectF boundingRect() const override;

    void
    paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    void notifySizeChange();

    QPainterPath shape() const override;

    ~QNode() override;
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_QNODE_H
