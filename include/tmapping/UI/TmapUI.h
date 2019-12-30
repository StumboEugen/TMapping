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

namespace Ui
{
class TmapWindow;
class BuildExpDockUI;
class BuildMapDockUI;
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

    Ui::BuildExpDockUI* uiDockExpBuilder;
    QDockWidget * dockExpBuilder;
    Ui::BuildMapDockUI* uiDockMapBuilder;
    QDockWidget * dockMapBuilder;

private: // methods
    void addBuiltExpData(const ExpDataPtr& expData);
    static QString getExpDataLabel(const ExpDataPtr& expData);

public:

    explicit TmapUI(QWidget* parent = nullptr);

    ~TmapUI() override;

private Q_SLOTS:
    void SLOT_BuildExp(bool start);
    void SLOT_GateTypeChanged(int index);
    void SLOT_ExpDataSelected(int index);
    void SLOT_EditJson(bool start);
    void SLOT_SaveExp();
    void SLOT_LoadExp();
    void SLOT_DragMode(bool enableDrag);
    void SLOT_AddFakeNode();
    void SLOT_DrawEdge(bool startDraw);
    void SLOT_SaveMap();
    void SLOT_LoadMap();

};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_TMAPUI_H
