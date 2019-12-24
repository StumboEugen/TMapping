//
// Created by stumbo on 2019/12/20.
//

#include <iostream>

#include "TmapUI.h"
#include "ui_tmapUI.h"
#include "MainGView.h"
#include "ViceGView.h"

#include "ui_dockBuildExp.h"

using namespace std;

Q_DECLARE_METATYPE(tmap::ExpDataType);
Q_DECLARE_METATYPE(tmap::ExpDataPtr);
Q_DECLARE_METATYPE(tmap::GateType);

tmap::TmapUI::TmapUI(QWidget* parent) :
        QMainWindow(parent),
        uiMain(new Ui::TmapWindow),
        uiDockExpBuilder(new Ui::BuildExpDockUI),
        dockExpBuilder(new QDockWidget(this))
{
    {   /// 添加UI组件
        uiMain->setupUi(this);
        setWindowTitle("Tmapping Viewer");

        centerLayout = new QHBoxLayout(uiMain->centralWidget);
        centerLayout->setSpacing(3);
        centerLayout->setContentsMargins(5, 5, 5, 5);
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
        smallWindowLayout->setSpacing(3);
        smallWindowLayout->setContentsMargins(5, 0, 5, 0);

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

    {
        uiDockExpBuilder->setupUi(dockExpBuilder);
        addDockWidget(Qt::RightDockWidgetArea, dockExpBuilder);
        dockExpBuilder->setShown(true);

        uiDockExpBuilder->cbExpType->setItemData(0,
                QVariant::fromValue(ExpDataType::Intersection));
        uiDockExpBuilder->cbExpType->setItemData(1,
                QVariant::fromValue(ExpDataType::SmallRoom));

        uiDockExpBuilder->cbGateType->setItemData(0,
                QVariant::fromValue(GateType::GateWay));
        uiDockExpBuilder->cbGateType->setItemData(1,
                QVariant::fromValue(GateType::DoorOpened));
        uiDockExpBuilder->cbGateType->setItemData(2,
                QVariant::fromValue(GateType::DoorClosed));

        connect(uiDockExpBuilder->btnBuildExp, SIGNAL(toggled(bool)),
                this, SLOT(slotBuildExp(bool)));

        connect(uiDockExpBuilder->cbGateType, SIGNAL(currentIndexChanged(int)),
                this, SLOT(slotGateTypeChanged(int)));

    }

}

tmap::TmapUI::~TmapUI()
{

}

void tmap::TmapUI::slotBuildExp(bool begin)
{
    if (begin) {
        uiDockExpBuilder->btnBuildExp->setText("Complete Building");
        auto& cbET = uiDockExpBuilder->cbExpType;
        gvVice->beginExpBuilding(cbET->itemData(cbET->currentIndex()).value<ExpDataType>());
    } else {
        uiDockExpBuilder->btnBuildExp->setText("Build an Exp");
        addBuiltExpData(gvVice->completeExpBuilding());
    }
}

void tmap::TmapUI::addBuiltExpData(const ExpDataPtr& expData)
{
    if (!expData) {
        cout << FILE_AND_LINE << " an invaild ExpData! (null)" << endl;
        return;
    }
    QString name(expData->getName().data());
    if (name.isEmpty()) {
        static size_t indexOfExpData = 0;
        name = "[" + QString::number(indexOfExpData++) + "] " +
                QString::number(expData->nGates()) + " gates";
    }
    uiDockExpBuilder->cbBuiltExps->addItem(name, QVariant::fromValue(expData));
}

void tmap::TmapUI::slotGateTypeChanged(int index)
{
    gvVice->setNextGateType(
            uiDockExpBuilder->cbGateType->itemData(index).value<GateType>());
}

