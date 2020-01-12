//
// Created by stumbo on 2019/12/20.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_UI_TMAPUI_H
#define TMAPPING_INCLUDE_TMAPPING_UI_TMAPUI_H

#include <QMainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTextBrowser>
#include <QActionGroup>
#include <QGraphicsScene>
#include <QDockWidget>

#include "tmapping/expDataTypes/ExpData.h"
#include "tmapping/tools/TopoParams.h"

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

namespace Ui
{
class TmapWindow;
class BuildExpDockUI;
class BuildMapDockUI;
class SimulationDockUI;
class RealTimeDockUI;
}

namespace tmap
{
class MainGView;

class ViceGView;

class TmapUI : public QMainWindow
{
    Q_OBJECT

    Ui::TmapWindow* uiMain;

    MainGView* gvMain;
    ViceGView* gvVice;
    QTextBrowser* infoView;
    QHBoxLayout* centerLayout;
    QVBoxLayout* smallWindowLayout;

    QActionGroup* modeGroup;
    QAction* mode_BUILD;
    QAction* mode_SIMULATION;
    QAction* mode_REALTIME;

    QAction * qactConnectToROS;

    Ui::BuildExpDockUI* uiDockExpBuilder;
    QDockWidget * dockExpBuilder;
    Ui::BuildMapDockUI* uiDockMapBuilder;
    QDockWidget * dockMapBuilder;
    Ui::SimulationDockUI* uiDockSimulation;
    QDockWidget * dockSimulation;
    Ui::RealTimeDockUI* uiDockRealtime;
    QDockWidget * dockRealtime;

private: // methods
    void addBuiltExpData(const ExpDataPtr& expData);
    static QString getExpDataLabel(const ExpDataPtr& expData);
    void startEdittingNodes(bool start);
    static bool checkROS();

public:

    explicit TmapUI(QWidget* parent = nullptr);

    ~TmapUI() override;

private Q_SLOTS:
    void SLOT_BuildExp(bool start);
    void SLOT_GateTypeChanged(int index);
    void SLOT_ExpDataSelected(int index);
    void SLOT_EditJsonOfBuiltExpData(bool start);
    void SLOT_SaveExp();
    void SLOT_LoadExp();
    void SLOT_DragMode(bool enableDrag);
    void SLOT_AddFakeNode();
    void SLOT_DrawEdge(bool startDraw);
    void SLOT_SaveMap();
    void SLOT_LoadMap();
    void SLOT_EditJsonOfNodeInFakeMap(bool start);
    void SLOT_AddGate2Corridor(bool start);
    void SLOT_InitROS();
    void SLOT_SwitchMode(QAction*);
    void SLOT_PlaceRobot();
    void SLOT_ROS_ThroughGate(ExpPtr);

};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_TMAPUI_H
