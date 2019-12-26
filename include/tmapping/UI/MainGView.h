//
// Created by stumbo on 2019/12/20.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H
#define TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H

#include <QGraphicsView>
#include <QWidget>
#include <QGraphicsItem>

#include "tmapping/StructedMap.h"

namespace tmap
{

class QNode : public QGraphicsItem, public MapNode
{
public:
    QRectF boundingRect() const override;

    void
    paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    ~QNode() override;
};

class MainGView : public QGraphicsView
{
    Q_OBJECT
    QGraphicsScene mScene4FakeMap;

protected:
    void wheelEvent(QWheelEvent * event) override ;

public:
    explicit MainGView(QWidget *parent = nullptr);

};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H
