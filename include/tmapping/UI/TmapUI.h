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
#include <QImage>

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

    ros::ServiceClient RSC_newExp;
    ros::ServiceClient RSC_throughGate;
    ros::ServiceClient RSC_getMaps;

    Jsobj realtimeMaps;
    Jsobj sentNodes;

    int stepsMoved = 0;

    std::vector<std::unique_ptr<QImage>> routeImgs;

#ifdef TMAPPING_CONFIG_RECORD_POSS
    std::vector<double> mChampionPoss;
    std::vector<double> mRunnerUpPoss;
#endif

private: // methods
    void addBuiltExpData(const ExpDataPtr& expData);
    static QString getExpDataLabel(const ExpDataPtr& expData);
    void startEdittingNodes(bool start);
    static bool checkROS();

    void saveScenePic(QGraphicsScene* scene,
                      std::string fileName,
                      std::string folder = "");

    std::unique_ptr<QImage> turnScene2Image(QGraphicsScene* scene);

    void saveImg(const std::unique_ptr<QImage>& img, std::string fileName, std::string folder = "");

protected:
    void keyPressEvent(QKeyEvent* event) override;

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
    void SLOT_ROS_ThroughGate(const ExpPtr&);
    void SLOT_GetRealtimeMaps();
    void SLOT_DisplayTheRealMap(int);
    void SLOT_RandomMove();
    void SLOT_ShowPossHistroy();
    void SLOT_SetMapSurvivers(int);
    void SLOT_StartMassiveTrials();

};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_TMAPUI_H
