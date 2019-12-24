//
// Created by stumbo on 2019/12/20.
//

#include "ViceGView.h"
#include "UITools.h"

#include <QGraphicsEllipseItem>
#include <QMouseEvent>

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
            parentVGV.startDrawingGateFromReferPoint(*this);
        }
    }
}; /// end of class ReferPoint
}

tmap::ViceGView::ViceGView(QWidget* parent) : QGraphicsView(parent)
{
    setScene(&mScene);
}

void tmap::ViceGView::beginExpBuilding(tmap::ExpDataType type)
{
    mStatus = DisplayStatus::BUILDING_EXP;

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
        case ExpDataType::BigRoom:
            cerr << "BigRoom hasn't been impled" << endl;
            break;
        case ExpDataType::SmallRoom:
            mRelatedExpData.reset(new SmallRoom());
            break;
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
    for (auto& qGate : mQGates) {
        mRelatedExpData->addGate(qGate->getGateData());
    }
    mQGates.clear();
    scene()->clear();
    return mRelatedExpData;
}

void tmap::ViceGView::setNextGateType(tmap::GateType type)
{
    cout << "change gate type: " << (int)type << endl;
    mNextGateType = type;
}

void tmap::ViceGView::startDrawingGateFromReferPoint(const ReferPoint& rp)
{
    GatePtr newGate;
    const auto& qp = rp.pos();
    TopoVec2 ori = UIT::QPt2TopoVec(qp);
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

    auto qGate = new QGate(newGate);
    scene()->addItem(qGate);
    qGate->setPos(qp);
    mQGates.push_back(qGate);

    mStatus = DisplayStatus::DRAWING_GATE;
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

/////////////////////// QGate related

tmap::QGate::QGate(tmap::GatePtr data)
        : QGraphicsItem(),
          mData(std::move(data))
{}

void tmap::QGate::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                        QWidget* widget)
{
    auto halfNorVec = mData->getNormalVec() / 2;
    auto p2 = UIT::TopoVec2QPt(halfNorVec);
    if (mData->type() == GateType::GateWay) {
        auto pen = painter->pen();
        pen.setWidth(3);
        painter->setPen(pen);
        painter->drawLine({0, 0}, p2);
    }
    if (mData->type() == GateType::Door) {
        painter->drawLine({0, 0}, p2);
        auto halfDoor = halfNorVec.rotate(90).changeLen(0.2);
        auto pen = painter->pen();
        pen.setWidth(3);
        painter->setPen(pen);
        painter->drawLine(UIT::TopoVec2QPt(halfDoor), UIT::TopoVec2QPt(-halfDoor));
    }
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
