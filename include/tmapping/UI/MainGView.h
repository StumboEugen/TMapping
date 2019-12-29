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

class QNode : public QGraphicsItem, public MapNode, public std::enable_shared_from_this<QNode>
{
    mutable QRectF mBoundingRect;

protected:
    explicit QNode(const ExpDataPtr& relatedExpData);

public:
    static QNodePtr makeOne(const ExpDataPtr& relatedExpData);

    QRectF boundingRect() const override;

    void
    paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

    void changeSize();

    ~QNode() override;
};

class MainGView : public QGraphicsView
{
    Q_OBJECT
    QGraphicsScene mScene4FakeMap;
    std::set<QNodePtr> mNodesInFakeMap;

    QNodePtr mTheDrawingCorridor;

    bool mEnableFakeNodesMoving = true;
    bool mEnableNodeRestriction = false;
    bool mIsDrawingEdge = false;

protected:
    void wheelEvent(QWheelEvent * event) override ;
    void mousePressEvent(QMouseEvent * event) override ;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override ;

public:
    explicit MainGView(QWidget *parent = nullptr);

    void addNode2FakeMap(const ExpDataPtr& usedExpData);

    void restrictQNode(QNode* qNode);

    virtual ~MainGView();

public Q_SLOTS:
    void SLOT_EnableMoving4FakeNodes(bool enableMove);
    void SLOT_EnableGridRestriction(bool enableRes);
    void SLOT_StartDrawingEdge(bool enableDrawing);
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H
