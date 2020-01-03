//
// Created by stumbo on 2019/12/30.
//

#include "tmapping/MergedExp.h"
#include "tmapping/Exp.h"
#include "QNode.h"
#include "UITools.h"

#include <QGraphicsScene>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <set>

#include <iostream>

using namespace std;


tmap::QNode::QNode(MergedExpPtr mergedExp)
{
    setZValue(mergedExp->getMergedExpData()->type() == ExpDataType::Corridor ? -1 : 0);
    links.assign(mergedExp->getMergedExpData()->nGates(), Link{});
    relatedMergedExp = std::move(mergedExp);
    setFlag(ItemIsSelectable);
}

tmap::QNode::~QNode() {
    if (scene()) {
        scene()->removeItem(this);
    }
}

QRectF tmap::QNode::boundingRect() const
{
    if (mBoundingRect.isEmpty()) {
        auto bRect = relatedMergedExp->getMergedExpData()->getOutBounding(0.5);
        mBoundingRect.setTopLeft(UIT::TopoVec2QPt({bRect[2], bRect[0]}));
        mBoundingRect.setBottomRight(UIT::TopoVec2QPt({bRect[3], bRect[1]}));
    }
    return mBoundingRect;
}

void
tmap::QNode::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    const auto& centerPoint = boundingRect().center();
    QPointF halfDia{UIT::QMeter(0.3), UIT::QMeter(0.3)};
    QRectF middleHalfSq{centerPoint + halfDia, centerPoint - halfDia};

    auto oriPen = painter->pen();
    auto isSelected = option->state & QStyle::State_Selected;

    auto relatedExpData = relatedMergedExp->getMergedExpData();
    switch (relatedExpData->type()) {
        case ExpDataType::Intersection: {
            painter->setBrush(isSelected ? Qt::lightGray : Qt::yellow);
            painter->drawEllipse(middleHalfSq);
            painter->setPen(oriPen);

            for (const auto& gate : relatedExpData->getGates()) {
                UIT::drawGate(painter, gate.get(), true, false);
                painter->setPen(QPen(Qt::darkGray, 4, Qt::DashDotLine, Qt::FlatCap));
                painter->drawLine(centerPoint, UIT::TopoVec2QPt(gate->getPos()));
                painter->setPen(oriPen);
            }
            break;
        }
        case ExpDataType::Corridor: {
            auto corr = dynamic_cast<Corridor*>(relatedExpData.get());
            auto pA = UIT::TopoVec2QPt(corr->getEndPointA());
            auto pB = UIT::TopoVec2QPt(corr->getEndPointB());
            QPen pen{isSelected ? Qt::green : Qt::darkGreen , 10};
            pen.setCapStyle(Qt::RoundCap);
            painter->setPen(pen);
            painter->drawLine(pA, pB);
            painter->setPen(oriPen);
            ///绘制除了两端之外的其他Gate
            const auto& gates = relatedExpData->getGates();
            for (int i = 0; i < gates.size(); ++i) {
                if (corr->getEndGateA() != i && corr->getEndGateB() != i) {
                    UIT::drawGate(painter, gates[i].get(), true);
                }
            }
            break;
        }
        case ExpDataType::Stair:
            cerr << FILE_AND_LINE << " unimplemented exp type!" << endl;
            break;
        case ExpDataType::BigRoom:
            cerr << FILE_AND_LINE << " unimplemented exp type!" << endl;
            break;
        case ExpDataType::SmallRoom: {
            auto bRect = relatedExpData->getOutBounding(0.);
            auto tl = UIT::TopoVec2QPt({bRect[2], bRect[0]});
            auto br = UIT::TopoVec2QPt({bRect[3], bRect[1]});
            painter->drawRect(QRectF{tl,br});
            painter->setBrush(isSelected ? Qt::lightGray : Qt::yellow);
            painter->drawRect(middleHalfSq);
            painter->setPen(oriPen);
            for (const auto& gate : relatedExpData->getGates()) {
                UIT::drawGate(painter, gate.get(), true, false);
            }
            break;
        }
    }
}

void tmap::QNode::notifySizeChange()
{
    mBoundingRect.setWidth(0.);
    prepareGeometryChange();
    update();
}

tmap::QNodePtr tmap::QNode::makeOneFromExpData(const tmap::ExpDataPtr& relatedExpData)
{
    auto exp = make_shared<Exp>(relatedExpData, 0);
    return QNodePtr(new QNode(MergedExp::singleMergedFromExp(exp)));
}

tmap::QNodePtr tmap::QNode::makeOneFromMergedExp(const MergedExpPtr& relatedMergedExp)
{
    return QNodePtr(new QNode(relatedMergedExp));
}

void tmap::QNode::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
    QGraphicsItem::mouseMoveEvent(event);
    if (flags() & ItemIsMovable) {
        notifyNeighbours2Move();
    }
}

void tmap::QNode::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
    QGraphicsItem::mouseReleaseEvent(event);
    if (flags() & ItemIsMovable) {
        notifyNeighbours2Move();
    }
}

void tmap::QNode::notifyNeighbours2Move() const
{
    vector<const QNode*> stack{this};
    set<const QNode*> searchedNodes;

    while (!stack.empty()) {
        auto current = stack.back();
        stack.pop_back();
        for (int i = 0; i < links.size(); ++i) {
            auto& link = links[i];
            if (!link.to.expired()) {
                const auto& linkedQNode = dynamic_cast<QNode*>(link.to.lock().get());
                if (searchedNodes.find(linkedQNode) != searchedNodes.end()) {
                    /// 要处理的current之前已经处理过了, 不要发生死循环
                    continue;
                }
                GateID linkedGateID = link.at;
                const auto& linkedExpData = linkedQNode->relatedMergedExp->getMergedExpData();
                /// 得到在scene中两个点的位置差距
                auto linkedGatePos = linkedQNode->mapToScene(UIT::TopoVec2QPt
                                                                     (linkedExpData->getGates()[linkedGateID]->getPos()));
                auto currentGatePos = current->mapToScene(UIT::TopoVec2QPt
                                                                  (current->relatedMergedExp->getMergedExpData()->getGates()[i]->getPos()));
                switch (linkedExpData->type()) {
                    case ExpDataType::Intersection:
                    case ExpDataType::SmallRoom:
                        /// Intersection和SmallRoom采用相同的策略, 跟随current移动
                        searchedNodes.insert(linkedQNode);
                        linkedQNode->setPos(
                                linkedQNode->pos() + (currentGatePos - linkedGatePos));
                        /// 移动后相关的Gate也会受影响, 加入修改队列中
                        stack.push_back(linkedQNode);
                        break;
                    case ExpDataType::Corridor: {
                        /// 走廊的话直接移动端点 TODO 考虑是否合理?
                        dynamic_cast<Corridor*>(linkedExpData.get())->moveGatePos
                                (linkedGateID, UIT::QPt2TopoVec(linkedQNode->mapFromScene(currentGatePos)));
                        linkedQNode->notifySizeChange();
                        searchedNodes.insert(linkedQNode);
                        break;
                    }
                    default:
                        cerr << FILE_AND_LINE << " Unimplemented type!" <<
                             (int) linkedExpData->type() << endl;
                        break;
                }
            }
        }
    }
}

QPainterPath tmap::QNode::shape() const
{
    QPainterPath path;
    switch (relatedMergedExp->getMergedExpData()->type()) {

        case ExpDataType::Intersection:
        case ExpDataType::Stair:
        case ExpDataType::BigRoom:
        case ExpDataType::SmallRoom: {
            auto bRect = relatedMergedExp->getMergedExpData()->getOutBounding(0.5);
            QRectF rect;
            rect.setTopLeft(UIT::TopoVec2QPt({bRect[2], bRect[0]}));
            rect.setBottomRight(UIT::TopoVec2QPt({bRect[3], bRect[1]}));
            path.addRect(rect);
            break;
        }
        case ExpDataType::Corridor: {
            QPainterPath p;
            auto corr = dynamic_cast<Corridor*>(relatedMergedExp->getMergedExpData().get());
            auto pA = UIT::TopoVec2QPt(corr->getEndPointA());
            auto pB = UIT::TopoVec2QPt(corr->getEndPointB());
            p.moveTo(pA);
            p.lineTo(pB);
            QPainterPathStroker stroker;
            stroker.setWidth(30);
            path = stroker.createStroke(p);
            break;
        }
    }
    return path;
}

void tmap::QNode::normalizePos()
{
    auto offset = relatedMergedExp->getMergedExpData()->normalizeSelf();
    setPos(pos() + UIT::TopoVec2QPt(offset));
    notifySizeChange();
}
