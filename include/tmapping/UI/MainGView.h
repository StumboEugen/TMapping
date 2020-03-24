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
#include "QNode.h"
#include "QRobot.h"

namespace tmap
{

class MainGView : public QGraphicsView
{
    Q_OBJECT
    QGraphicsScene mScene4FakeMap;
    QGraphicsScene mScene4RealMap;
    std::set<QNodePtr> mNodesInFakeMap;

    QNodePtr mTheDrawingCorridor;
    FakeLine mTheDrawingFakeLine;
    QRobotPtr mRobot;

    bool mEnableFakeNodesMoving = true;
    bool mEnableNodeRestriction = false;
    bool mIsDrawingEdge = false;
    bool mAcceptAddingGate2Corridor = false;
    bool mIsDrawingDirectLink = false;

    bool mAtSim = false;

    MoveStragety mMoveStragety = MoveStragety::EVERY_NODE;

private:
    void setQNodeMovability(QNode* node) const;

protected:
    void wheelEvent(QWheelEvent * event) override ;

    void mousePressEvent(QMouseEvent * event) override ;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override ;

    void keyPressEvent(QKeyEvent* event) override;

public:
    explicit MainGView(QWidget *parent = nullptr);

    void addNode2FakeMapFromExpData(const ExpDataPtr& usedExpData);

    void restrictQNode(QNode* qNode);

    void saveFakeMap(const std::string& mapName);

    void loadMap(const std::string& fileName);

    void switch2realMode(bool);
    void switch2simMode(bool);

    bool setRobotInFake(bool directMove);

    ~MainGView() override;

public Q_SLOTS:
    void SLOT_EnableMoving4FakeNodes(bool enableMove);
    void SLOT_EnableGridRestriction(bool enableRes);
    void SLOT_StartDrawingEdge(bool enableDrawing);
    void SLOT_AcceptAddingGates2Corridor(bool acceptAdding);
    void SLOT_RemoveSelectedNodes();
    void SLOT_StartDirectLinking(bool startLink);
    void SLOT_SetMoveStrategy(int strategy);

Q_SIGNALS:
    void SIG_RobotThroughGate(ExpPtr);
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H
