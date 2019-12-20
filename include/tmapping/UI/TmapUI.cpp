//
// Created by stumbo on 2019/12/20.
//

#include "TmapUI.h"
#include "ui_tmapUI.h"
#include "MainGView.h"
#include "ViceGView.h"

using namespace std;

tmap::TmapUI::TmapUI(QWidget* parent) :
        QMainWindow(parent),
        uiMain(new Ui::TmapWindow)
{
    {   /// 添加UI组件
        uiMain->setupUi(this);
        setWindowTitle("Tmapping Viewer");

        centerLayout = new QHBoxLayout(uiMain->centralWidget);
        centerLayout->setSpacing(6);
        centerLayout->setContentsMargins(11, 11, 11, 11);
        centerLayout->setObjectName(QString::fromUtf8("centerLayout"));

        gvMain = new MainGView(uiMain->centralWidget);
        QSizePolicy p;
        p.setHorizontalPolicy(QSizePolicy::Expanding);
        p.setVerticalPolicy(QSizePolicy::Expanding);
//    gvMain->setScene(&this->mapScene);
        gvMain->setSizePolicy(p);
        gvMain->setMinimumSize(601, 401);
        centerLayout->addWidget(gvMain);

        smallWindowLayout = new QVBoxLayout();
        smallWindowLayout->setSpacing(11);
        smallWindowLayout->setContentsMargins(11, 0, 11, 0);

        infoView = new QTextBrowser(uiMain->centralWidget);
        infoView->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
        infoView->setBaseSize(221, 221);
        infoView->setText("Welcome!");
        smallWindowLayout->addWidget(infoView, 0, Qt::AlignTop);

        gvVice = new ViceGView(uiMain->centralWidget);
//    gvVice->setScene(&this->nodeScene);
        gvVice->setFixedSize(221, 221);
        gvVice->setCursor(Qt::CrossCursor);
        gvVice->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
        smallWindowLayout->addWidget(gvVice, 0, Qt::AlignBottom);

        centerLayout->addLayout(smallWindowLayout);
    }

    {   /// 初始化模式切换按钮
        modeGroup = new QActionGroup(uiMain->mainToolBar);

        mode_BUILD = modeGroup->addAction("build map mode");
        mode_SIMULATION = modeGroup->addAction("simulation mode");
        mode_REALTIME = modeGroup->addAction("realtime mode");

        mode_BUILD->setCheckable(true);
        mode_SIMULATION->setCheckable(true);
        mode_REALTIME->setCheckable(true);

        mode_BUILD->setChecked(true);

        uiMain->mainToolBar->addAction(mode_BUILD);
        uiMain->mainToolBar->addAction(mode_SIMULATION);
        uiMain->mainToolBar->addAction(mode_REALTIME);
        uiMain->mainToolBar->addSeparator();
    }
}

tmap::TmapUI::~TmapUI()
{

}
