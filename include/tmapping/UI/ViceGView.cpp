//
// Created by stumbo on 2019/12/20.
//

#include "ViceGView.h"
#include "UITools.h"
#include "tmapping/expDataTypes/ExpDataTypes.h"

#include <QGraphicsEllipseItem>
#include <QMouseEvent>
#include <QGraphicsSceneMouseEvent>

#include <iostream>

using namespace std;

namespace tmap {

/// ReferPoint radius
static constexpr double RPR = 5;

class ReferPoint : public QGraphicsEllipseItem {

private:
    ViceGView& parentVGV;
    bool used = false;

public:
    explicit ReferPoint(ViceGView& parentVGV)
            : QGraphicsEllipseItem({
                QPointF{-RPR, -RPR},
                QPointF{RPR, RPR}}),
                parentVGV(parentVGV)
    {}

protected:
    void mousePressEvent(QGraphicsSceneMouseEvent* event) override
    {
        QGraphicsItem::mousePressEvent(event);
        if (!used) {
            used = true;
            if (event->button() == Qt::RightButton) {
                parentVGV.addLandMark(*this);
            } else {
                parentVGV.startDrawingGateFromReferPoint(*this);
            }
        }
    }
}; /// end of class ReferPoint
}

tmap::ViceGView::ViceGView(QWidget* parent) : QGraphicsView(parent)
{
    setScene(&mScene);
}

void tmap::ViceGView::beginExpBuilding(ExpDataType type, double size)
{
    size = max(size, 2.0);
    size /= 2.0;
    mStatus = DisplayStatus::BUILDING_EXP;
    mQGates.clear();

    scene()->clear();

    switch (type) {
        case ExpDataType::Intersection:
            mRelatedExpData.reset(new Intersection());
            break;
        case ExpDataType::Corridor:
            cerr << "Corridor hasn't been impled" << endl;
            break;
        case ExpDataType::Stair:
            cerr << "Stair hasn't been impled" << endl;
            break;
        case ExpDataType::Room: {
            auto room = new Room();
            room->setScaling(size);
            mRelatedExpData.reset(room);
            break;
        }
    }

    for (int i = -2; i < 3; ++i) {
        for (int j = -2; j < 3; ++j) {
            auto* rp = new ReferPoint(*this);
            this->scene()->addItem(rp);
            QPointF pos1{i * UIT::QMeter(0.5), j * UIT::QMeter(0.5)};
            rp->setPos(pos1);
        }
    }
}

tmap::ExpDataPtr tmap::ViceGView::completeExpBuilding()
{
    mStatus = DisplayStatus::NOTHING;

    if (mQGates.empty()) {
        scene()->clear();
        return nullptr;
    }

    for (auto& qGate : mQGates) {
        mRelatedExpData->addGate(qGate->getGateData());
    }
    mQGates.clear();
    return mRelatedExpData;
}

void tmap::ViceGView::setNextGateType(tmap::GateType type)
{
    mNextGateType = type;
}

void tmap::ViceGView::startDrawingGateFromReferPoint(const ReferPoint& rp)
{
    GatePtr newGate;
    const auto& qp = rp.pos();
    TopoVec2 ori = UIT::QPt2TopoVec(qp);
    if (auto room = dynamic_cast<Room*>(mRelatedExpData.get())) {
        ori *= room->getScaling();
    }
    TopoVec2 nor(1, 0);

    switch(mNextGateType) {
        case GateType::GateWay:
            newGate.reset(new GateWay(ori, nor));
            break;
        case GateType::Door:
        case GateType::DoorOpened:
            newGate.reset(new Door(ori, nor, true));
            break;
        case GateType::DoorClosed:
            newGate.reset(new Door(ori, nor, false));
            break;
    }

    auto qGate = new QGate(std::move(newGate));
    scene()->addItem(qGate);
    qGate->setPos(qp);
    mQGates.push_back(qGate);

    mStatus = DisplayStatus::DRAWING_GATE;
}

void tmap::ViceGView::addLandMark(const tmap::ReferPoint& rp)
{
    double scale = 1.0;
    if (auto room = dynamic_cast<Room*>(mRelatedExpData.get())) {
        scale = room->getScaling();
    }
    mRelatedExpData->addLandmark(
            make_shared<StrPLM>(UIT::QPt2TopoVec(rp.pos() * scale), mNextLM));
    double r = UIT::QMeter(0.1);
    auto circle = new QGraphicsEllipseItem({QPointF{-r, -r}, QPointF{r, r}});
    circle->setBrush(Qt::green);
    scene()->addItem(circle);
    circle->setPos(rp.pos());
}

void tmap::ViceGView::mouseMoveEvent(QMouseEvent* event)
{
    QGraphicsView::mouseMoveEvent(event);

    if (mStatus == DisplayStatus::DRAWING_GATE) {
        mQGates.back()->changeNormalVec(mapToScene(event->pos()));
    }
}

void tmap::ViceGView::mouseReleaseEvent(QMouseEvent* event)
{
    QGraphicsView::mouseReleaseEvent(event);
    if (mStatus == DisplayStatus::DRAWING_GATE) {
        mQGates.back()->changeNormalVec(mapToScene(event->pos()));
        mStatus = DisplayStatus::BUILDING_EXP;
    }
}

void tmap::ViceGView::displayTheExpData(tmap::ExpDataPtr data2show)
{
    if (mStatus == DisplayStatus::BUILDING_EXP || mStatus == DisplayStatus::DRAWING_GATE) {
        cerr << FILE_AND_LINE << " Cant display now" << endl;
        return;
    }
    scene()->clear();
    mStatus = DisplayStatus::DISPLAYING_EXP;
    mRelatedExpData = std::move(data2show);
    QRectF rec({-RPR, -RPR}, QPointF{RPR, RPR});
    double scale = 1.0;
    if (mRelatedExpData->type() == ExpDataType::Intersection) {
        scene()->addEllipse(rec, {Qt::gray, 1}, Qt::lightGray);
    }
    else if (mRelatedExpData->type() == ExpDataType::Room) {
        scene()->addRect(rec, {Qt::gray, 1}, Qt::lightGray);
        scale = dynamic_cast<Room*>(mRelatedExpData.get())->getScaling();
    }
    for (const auto& gate : mRelatedExpData->getGates()) {
        auto g = new QGate(gate);
        g->setPos(UIT::TopoVec2QPt(gate->getPos() / scale));
        scene()->addItem(g);
        mQGates.emplace_back(g);
    }
    for (const auto& plm : mRelatedExpData->getPLMs()) {
        double r = UIT::QMeter(0.1);
        auto circle = new QGraphicsEllipseItem({QPointF{-r, -r}, QPointF{r, r}});
        circle->setBrush(Qt::green);
        scene()->addItem(circle);
        circle->setPos(UIT::TopoVec2QPt(plm->getPos() / scale));
    }
}

void tmap::ViceGView::SLOT_NextLandmarkStr(QString str)
{
    mNextLM = str.toStdString();
}

/////////////////////// QGate related

tmap::QGate::QGate(tmap::GatePtr data)
        : QGraphicsItem(),
          mData(std::move(data))
{}

void tmap::QGate::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                        QWidget* widget)
{
    UIT::drawGate(painter, mData.get());
}

void tmap::QGate::changeNormalVec(const QPointF& toPointInScene)
{
    auto diff = toPointInScene - scenePos();
    mData->changeNormalVec2(UIT::QPt2TopoVec(diff).restrictDir());
    update();
}

QRectF tmap::QGate::boundingRect() const
{
    TopoVec2 lt{-0.5, 0.5};
    return QRectF{UIT::TopoVec2QPt(lt), UIT::TopoVec2QPt(lt * -1)};
}

const tmap::GatePtr& tmap::QGate::getGateData() const
{
    return mData;
}
