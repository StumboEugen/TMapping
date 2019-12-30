//
// Created by stumbo on 2019/12/20.
//

#include <iostream>

#include "TmapUI.h"
#include "ui_tmapUI.h"
#include "MainGView.h"
#include "ViceGView.h"

#include "tmapping/MergedExp.h"

#include "ui_dockBuildExp.h"
#include "ui_dockBuildMap.h"

using namespace std;

Q_DECLARE_METATYPE(tmap::ExpDataType);
Q_DECLARE_METATYPE(tmap::ExpDataPtr);
Q_DECLARE_METATYPE(tmap::GateType);

tmap::TmapUI::TmapUI(QWidget* parent) :
        QMainWindow(parent),
        uiMain(new Ui::TmapWindow),
        uiDockExpBuilder(new Ui::BuildExpDockUI),
        dockExpBuilder(new QDockWidget(this)),
        uiDockMapBuilder(new Ui::BuildMapDockUI),
        dockMapBuilder(new QDockWidget(this))
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

        auto * dragMode = new QAction("enable Drag", uiMain->mainToolBar);
        dragMode->setCheckable(true);
        uiMain->mainToolBar->addAction(dragMode);
        uiMain->mainToolBar->addSeparator();
        connect(dragMode, SIGNAL(toggled(bool)), this, SLOT(SLOT_DragMode(bool)));
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
                this, SLOT(SLOT_BuildExp(bool)));

        connect(uiDockExpBuilder->btnEditJson, SIGNAL(toggled(bool)),
                this, SLOT(SLOT_EditJsonOfBuiltExpData(bool)));

        connect(uiDockExpBuilder->cbGateType, SIGNAL(currentIndexChanged(int)),
                this, SLOT(SLOT_GateTypeChanged(int)));

        connect(uiDockExpBuilder->cbBuiltExps, SIGNAL(currentIndexChanged(int)),
                this, SLOT(SLOT_ExpDataSelected(int)));
        connect(uiDockExpBuilder->cbBuiltExps, SIGNAL(highlighted(int)),
                this, SLOT(SLOT_ExpDataSelected(int)));

        connect(uiDockExpBuilder->btnLoad, SIGNAL(clicked()),
                this, SLOT(SLOT_LoadExp()));

        connect(uiDockExpBuilder->btnSave, SIGNAL(clicked()),
                this, SLOT(SLOT_SaveExp()));

        connect(uiDockExpBuilder->btnAdd2Map, SIGNAL(clicked()),
                this, SLOT(SLOT_AddFakeNode()));
    }

    {
        uiDockMapBuilder->setupUi(dockMapBuilder);
        addDockWidget(Qt::LeftDockWidgetArea, dockMapBuilder);
        dockExpBuilder->setShown(true);

        connect(uiDockMapBuilder->cbEnableNodesMoving, SIGNAL(toggled(bool)),
                gvMain, SLOT(SLOT_EnableMoving4FakeNodes(bool)));

        connect(uiDockMapBuilder->cbRestrictGrid, SIGNAL(toggled(bool)),
                gvMain, SLOT(SLOT_EnableGridRestriction(bool)));

        connect(uiDockMapBuilder->btnDeleteSelectedNodes, SIGNAL(clicked()),
                gvMain, SLOT(SLOT_RemoveSelectedNodes()));

        connect(uiDockMapBuilder->btnDrawCorridor, SIGNAL(toggled(bool)),
                this, SLOT(SLOT_DrawEdge(bool)));
        connect(uiDockMapBuilder->btnDrawCorridor, SIGNAL(toggled(bool)),
                uiDockMapBuilder->btnAddGate2Corridor, SLOT(setDisabled(bool)));
        connect(uiDockMapBuilder->btnDrawCorridor, SIGNAL(toggled(bool)),
                uiDockMapBuilder->btnEditJson, SLOT(setDisabled(bool)));

        connect(uiDockMapBuilder->btnSave, SIGNAL(clicked()),
                this, SLOT(SLOT_SaveMap()));

        connect(uiDockMapBuilder->btnLoad, SIGNAL(clicked()),
                this, SLOT(SLOT_LoadMap()));

        connect(uiDockMapBuilder->btnEditJson, SIGNAL(toggled(bool)),
                this, SLOT(SLOT_EditJsonOfNodeInFakeMap(bool)));

    }

}

tmap::TmapUI::~TmapUI()
{

}

void tmap::TmapUI::addBuiltExpData(const ExpDataPtr& expData)
{
    if (!expData) {
        cout << FILE_AND_LINE << " an invaild ExpData! (null)" << endl;
        return;
    }

    auto cbBuiltExps = uiDockExpBuilder->cbBuiltExps;
    cbBuiltExps->addItem(getExpDataLabel(expData), QVariant::fromValue(expData));
    cbBuiltExps->setCurrentIndex(cbBuiltExps->count() - 1);
}

void tmap::TmapUI::SLOT_BuildExp(bool start)
{
    if (start) {
        uiDockExpBuilder->btnBuildExp->setText("Complete Building");
        auto& cbET = uiDockExpBuilder->cbExpType;
        gvVice->beginExpBuilding(cbET->itemData(cbET->currentIndex()).value<ExpDataType>());
    } else {
        uiDockExpBuilder->btnBuildExp->setText("Build an Exp");
        auto theBuiltExp = gvVice->completeExpBuilding();
        if (theBuiltExp) {
            addBuiltExpData(theBuiltExp);
            infoView->setText("The built Exp's Json:\n");
            infoView->append(JsonHelper::JS2Str(theBuiltExp->toJS(), false).data());
        }
    }
}

void tmap::TmapUI::SLOT_GateTypeChanged(int index)
{
    gvVice->setNextGateType(
            uiDockExpBuilder->cbGateType->itemData(index).value<GateType>());
}

void tmap::TmapUI::SLOT_ExpDataSelected(int index)
{
    gvVice->displayTheExpData(
            uiDockExpBuilder->cbBuiltExps->itemData(index).value<ExpDataPtr>());
}

void tmap::TmapUI::SLOT_EditJsonOfBuiltExpData(bool start)
{
    static int indexOfEditing;
    auto& exps = uiDockExpBuilder->cbBuiltExps;
    if (start) {
        if (exps->count() == 0) {
            uiDockExpBuilder->btnEditJson->setCheckable(false);
            uiDockExpBuilder->btnEditJson->setCheckable(true);
            return;
        }
        uiDockExpBuilder->btnEditJson->setText("complete Editting");
        indexOfEditing = exps->currentIndex();
        Jsobj jExpData = exps->itemData(indexOfEditing).value<ExpDataPtr>()->toJS();
        infoView->setText(JsonHelper::JS2Str(jExpData, false).data());
        infoView->setTextColor(Qt::darkGray);
        infoView->setReadOnly(false);
    } else {
        uiDockExpBuilder->btnEditJson->setText( "Edit Json");

        Jsobj editedJs = JsonHelper::Str2JS(infoView->toPlainText().toStdString());
        ExpDataPtr madeExp = ExpData::madeFromJS(editedJs);
        if (madeExp) {
            exps->setItemData(indexOfEditing, QVariant::fromValue<ExpDataPtr>(madeExp));
            exps->setItemText(indexOfEditing, getExpDataLabel(madeExp));
            gvVice->displayTheExpData(madeExp);
        } else {
            infoView->append("\n Trans error!");
        }
        infoView->setTextColor(Qt::black);
        infoView->setReadOnly(true);
    }
}

void tmap::TmapUI::SLOT_EditJsonOfNodeInFakeMap(bool start)
{
    const auto& items = gvMain->scene()->selectedItems();
    if (start) {
        if (items.empty()) {
            uiDockMapBuilder->btnEditJson->setCheckable(false);
            uiDockMapBuilder->btnEditJson->setCheckable(true);
            return;
        }
        auto qNode = dynamic_cast<QNode*>(items.front());
        if (!qNode) {
            uiDockMapBuilder->btnEditJson->setCheckable(false);
            uiDockMapBuilder->btnEditJson->setCheckable(true);
            return;
        }
        Jsobj jExpData = qNode->relatedMergedExp->getMergedExpData()->toJS();
        infoView->setText(JsonHelper::JS2Str(jExpData, false).data());
        infoView->setTextColor(Qt::darkGray);
        uiDockMapBuilder->btnEditJson->setText("Complete Editting");
    } else {
        uiDockMapBuilder->btnEditJson->setText("Edit Json of Selected Node");
        Jsobj editedJs = JsonHelper::Str2JS(infoView->toPlainText().toStdString());
        ExpDataPtr madeExpData = ExpData::madeFromJS(editedJs);
        if (madeExpData) {
            auto qNode = dynamic_cast<QNode*>(items.front());
            qNode->relatedMergedExp->exchangeMergedExpData(madeExpData);
            qNode->notifySizeChange();
            qNode->notifyNeighbours2Move();
        } else {
            infoView->append("\n Trans error!");
        }
        infoView->setTextColor(Qt::black);
    }
    startEdittingNodes(start);
    gvMain->setDisabled(start);
    infoView->setReadOnly(!start);
}


QString tmap::TmapUI::getExpDataLabel(const tmap::ExpDataPtr& expData)
{
    QString name(expData->getName().data());
    if (name.isEmpty()) {
        static size_t indexOfAnoymousExpData = 0;
        name = "[" + QString::number(indexOfAnoymousExpData++) + "] " +
               QString::number(expData->nGates()) + "g " +
               QString{ExpData::typeStr(expData->type()).data()};
    }
    return name;
}

void tmap::TmapUI::SLOT_SaveExp()
{
    auto& exps = uiDockExpBuilder->cbBuiltExps;
    if (exps->count() == 0) {
        return;
    }
    const auto& expData2Save = exps->itemData(exps->currentIndex()).value<ExpDataPtr>();
    string fileName = uiDockExpBuilder->expName->text().toStdString();
    if (fileName.empty()) {
        fileName = expData2Save->getName();
    } else {
        expData2Save->setName(fileName);
    }
    if (JsonHelper::saveJson(expData2Save->toJS(), fileName) == 0) {
        cout << "save exp data [name:" + fileName + "] success!" << endl;
    }
}

void tmap::TmapUI::SLOT_LoadExp()
{
    auto& expNameEdit = uiDockExpBuilder->expName;
    const auto& name = expNameEdit->text();
    if (name.isEmpty()) {
        return;
    }
    Jsobj jExpData = JsonHelper::loadJson(name.toStdString());
    ExpDataPtr expRead = ExpData::madeFromJS(jExpData);
    if (expRead) {
        cout << "load exp data [name:" << name.toStdString() << "] success!" << endl;
        addBuiltExpData(expRead);
    }
}

void tmap::TmapUI::SLOT_DragMode(bool isDrag)
{
    if (isDrag) {
        gvMain->setDragMode(QGraphicsView::ScrollHandDrag);
    } else {
        gvMain->setDragMode(QGraphicsView::NoDrag);
    }
}

void tmap::TmapUI::SLOT_AddFakeNode()
{
    auto& exps = uiDockExpBuilder->cbBuiltExps;
    if (exps->count() == 0) {
        return;
    }
    gvMain->addNode2FakeMapFromExpData(exps->itemData(exps->currentIndex()).value<ExpDataPtr>());
}

void tmap::TmapUI::SLOT_DrawEdge(bool startDraw)
{
    startEdittingNodes(startDraw);
    gvMain->SLOT_StartDrawingEdge(startDraw);
}

void tmap::TmapUI::SLOT_SaveMap()
{
    string fileName = uiDockMapBuilder->mapName->text().toStdString();
    gvMain->saveFakeMap(fileName);
}

void tmap::TmapUI::SLOT_LoadMap()
{
    string fileName = uiDockMapBuilder->mapName->text().toStdString();
    gvMain->loadMap(fileName);
}

void tmap::TmapUI::startEdittingNodes(bool start)
{
    uiDockMapBuilder->cbEnableNodesMoving->setDisabled(start);
    uiDockMapBuilder->cbRestrictGrid->setDisabled(start);
    dockExpBuilder->setDisabled(start);

    if (start) {
        uiDockMapBuilder->cbEnableNodesMoving->setChecked(false);
        uiDockMapBuilder->cbRestrictGrid->setChecked(false);
    }
}
