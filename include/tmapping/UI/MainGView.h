//
// Created by stumbo on 2019/12/20.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H
#define TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H

#include <QGraphicsView>
#include <QWidget>
#include <QGraphicsItem>
#include <set>

#include "tmapping/StructedMap.h"

namespace tmap
{

class QNode;
using QNodePtr = std::shared_ptr<QNode>;

class QNode : public QGraphicsItem, public MapNode
{
    mutable QRectF mBoundingRect;
public:
    QNode(const ExpDataPtr& relatedExpData);

    QRectF boundingRect() const override;

    void
    paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    ~QNode() override;
};

class MainGView : public QGraphicsView
{
    Q_OBJECT
    QGraphicsScene mScene4FakeMap;
    std::set<QNodePtr> mNodesInFakeMap;

protected:
    void wheelEvent(QWheelEvent * event) override ;

public:
    explicit MainGView(QWidget *parent = nullptr);

    void addNode2FakeMap(const ExpDataPtr& usedExpData);

};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H
