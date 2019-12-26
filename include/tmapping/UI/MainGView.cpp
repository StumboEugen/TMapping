//
// Created by stumbo on 2019/12/20.
//

#include "MainGView.h"


tmap::QNode::~QNode() {
    if (scene()) {
        scene()->removeItem(this);
    }
}

QRectF tmap::QNode::boundingRect() const
{
    return QRectF();
}

void
tmap::QNode::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{

}

tmap::MainGView::MainGView(QWidget* parent) : QGraphicsView(parent)
{
    setScene(&mScene4FakeMap);
}
