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
    std::set<QNodePtr> mNodesInRealMap;

    QNodePtr mTheDrawingCorridor;
    FakeLine mTheDrawingFakeLine;
    QRobotPtr mRobot;

    bool mEnableFakeNodesMoving = true;
    bool mEnableRealNodesMoving = true;
    bool mEnableNodeRestriction = false;
    bool mIsDrawingEdge = false;
    bool mAcceptAddingGate2Corridor = false;
    bool mIsDrawingDirectLink = false;

    bool mAtSim = false;

    MoveStragety mMoveStragety = MoveStragety::EVERY_NODE;

    StructedMap currentDisplayedRealTimeMap;

    size_t nChampionSucceedSteps = 0;

private:
    void setQNodeMovabilityInFakeMap(QNode* node) const;
    void setQNodeMovabilityInRealMap(QNode* node) const;
    void emitRobotRandomMove();

protected:
    void wheelEvent(QWheelEvent * event) override ;

    void mousePressEvent(QMouseEvent * event) override ;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override ;

    void keyPressEvent(QKeyEvent* event) override;

    void mouseDoubleClickEvent(QMouseEvent *event) override;

public:
    explicit MainGView(QWidget *parent = nullptr);

    void addNode2FakeMapFromExpData(const ExpDataPtr& usedExpData);

    void restrictQNode(QNode* qNode);

    void saveFakeMap(const std::string& mapName);

    void loadMap(const std::string& fileName);

    void switch2realMode(bool);
    void switch2simMode(bool);

    bool setRobotInFake(bool directMove);

    void displayRealMap(const Jsobj& jMap);

    void randomMove(int mSteps, bool untilCover);

    std::string currentPossHistoryStr();

    void setChampionSucceedSteps(size_t steps);

    ~MainGView() override;

public Q_SLOTS:
    void SLOT_EnableMoving4FakeNodes(bool enableMove);
    void SLOT_EnableMoving4RealNodes(bool enableMove);
    void SLOT_EnableGridRestriction(bool enableRes);
    void SLOT_StartDrawingEdge(bool enableDrawing);
    void SLOT_AcceptAddingGates2Corridor(bool acceptAdding);
    void SLOT_RemoveSelectedNodes();
    void SLOT_StartDirectLinking(bool startLink);
    void SLOT_SetMoveStrategy(int strategy);

Q_SIGNALS:
    void SIG_RobotThroughGate(ExpPtr);
    void SIG_ShowStrInInfoView(QString);
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H
