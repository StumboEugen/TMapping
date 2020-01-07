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


tmap::QNode::QNode(MergedExpPtr mergedExp) : MapNode(std::move(mergedExp), 0)
{
    setFlag(ItemIsSelectable);
    mFakeLines.assign(mRelatedMergedExp->getMergedExpData()->nGates(), nullptr);
}

tmap::QNode::~QNode() {
    if (scene()) {
        scene()->removeItem(this);
    }
}

QRectF tmap::QNode::boundingRect() const
{
    if (mBoundingRect.isEmpty()) {
        auto bRect = expData()->getOutBounding(0.5);
        mBoundingRect.setTopLeft(UIT::TopoVec2QPt({bRect[2], bRect[0]}));
        mBoundingRect.setBottomRight(UIT::TopoVec2QPt({bRect[3], bRect[1]}));
    }
    return mBoundingRect;
}

void
tmap::QNode::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    const auto& centerPoint = boundingRect().center();
    QPointF halfDia{UIT::QMeter(0.2), UIT::QMeter(0.2)};
    QRectF middleHalfSq{centerPoint + halfDia, centerPoint - halfDia};

    auto oriPen = painter->pen();

    auto relatedExpData = expData();
    switch (relatedExpData->type()) {
        case ExpDataType::Intersection: {
            painter->setBrush(isSelected() ? Qt::lightGray : Qt::yellow);
            painter->drawEllipse(middleHalfSq);
            painter->setPen(oriPen);

            for (const auto& gate : relatedExpData->getGates()) {
                UIT::drawGate(painter, gate.get(), true, false);
                painter->setPen(QPen(Qt::darkGray, 2, Qt::DashDotLine, Qt::FlatCap));
                painter->drawLine(centerPoint, UIT::TopoVec2QPt(gate->getPos()));
                painter->setPen(oriPen);
            }
            break;
        }
        case ExpDataType::Corridor: {
            auto corr = dynamic_cast<Corridor*>(relatedExpData.get());

            /// 绘制走廊的斜线
            auto pA = corr->getEndPointA();
            auto pB = corr->getEndPointB();
            auto AB = pA - pB;
            TopoVec2 halfVecPA{}, halfVecPB{};
            GateID GA = corr->getEndGateA();
            GateID GB = corr->getEndGateB();
            halfVecPA = AB.rotate(-90).changeLen(corr->halfWidth());
            halfVecPB = AB.rotate(90).changeLen(corr->halfWidth());
            if (GA >= 0) {
                auto norm = corr->getGates()[GA]->getNormalVec();
                halfVecPA = norm.rotate(-90).changeLen(
                        corr->halfWidth() / AB.unitize().dotProduct(-norm));
            }
            if (GB >= 0) {
                auto norm = corr->getGates()[GB]->getNormalVec();
                halfVecPB = norm.rotate(90).changeLen(
                        corr->halfWidth() / AB.unitize().dotProduct(norm));
            }
            painter->drawLine(UIT::TopoVec2QPt(pA + halfVecPA),
                              UIT::TopoVec2QPt(pB + halfVecPB));
            painter->drawLine(UIT::TopoVec2QPt(pA - halfVecPA),
                              UIT::TopoVec2QPt(pB - halfVecPB));
            painter->setPen(oriPen);

            if (isSelected()) {
                QPainterPath pp;
                pp.moveTo(UIT::TopoVec2QPt(pA + halfVecPA));
                pp.lineTo(UIT::TopoVec2QPt(pB + halfVecPB));
                pp.lineTo(UIT::TopoVec2QPt(pB - halfVecPB));
                pp.lineTo(UIT::TopoVec2QPt(pA - halfVecPA));
                pp.lineTo(UIT::TopoVec2QPt(pA + halfVecPA));
                painter->setPen({Qt::green, 5});
                painter->drawPath(pp);
            }

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
            painter->setBrush(isSelected() ? Qt::lightGray : Qt::yellow);
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
    /// 遍历更新FakeLine的状态
    for (int i = 0; i < expData()->nGates(); ++i) {
        if (const auto& linkedQNode = qNodeAt(i)) {
            const auto& linkedGatePos = linkedQNode->gateQPos(linkedGIDAt(i));
            const auto& currentGatePos = gateQPos(i);
            if (linkedGatePos != currentGatePos) {
                if (auto& fakeLine = fakeLineAt(i)) {
                    fakeLine->setPoint(this, currentGatePos);
                } else {
                    fakeLine = make_shared<FakeLine_IMPL>(currentGatePos, linkedGatePos,
                            this, i);
                    linkedQNode->fakeLineAt(linkedGIDAt(i)) = fakeLine;
                    scene()->addItem(fakeLine.get());
                }
            }
        }
    }

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

void tmap::QNode::notifyNeighbours2Move()
{
    vector<QNode*> stack{this};
    set<QNode*> searchedNodes;

    while (!stack.empty()) {
        auto current = stack.back();
        stack.pop_back();
        /// 检查每一个link, 通知其需要随我运动
        for (int i = 0; i < nLinks(); ++i) {
            if (auto linkedQNode = qNodeAt(i)) {
                auto currentGatePos = current->mapToScene(UIT::TopoVec2QPt
                        (current->expData()->getGates()[i]->getPos()));

                if (auto fakeLine = fakeLineAt(i)) {
                    /// 检查是否为Fake连接, 如果是的话, 只要改动Fake连接即可
                    fakeLine->setPoint(this, currentGatePos);
                    continue;
                }
                if (searchedNodes.find(linkedQNode.get()) != searchedNodes.end()) {
                    /// 要处理的current之前已经处理过了, 不要发生死循环
                    continue;
                }
                GateID linkedGateID = linkAt(i).at;
                const auto& linkedExpData = linkedQNode->expData();
                /// 得到在scene中两个点的位置差距
                auto linkedGatePos = linkedQNode->mapToScene(UIT::TopoVec2QPt
                        (linkedExpData->getGates()[linkedGateID]->getPos()));
                switch (linkedExpData->type()) {
                    case ExpDataType::Intersection:
                    case ExpDataType::SmallRoom:
                        /// Intersection和SmallRoom采用相同的策略, 跟随current移动
                        searchedNodes.insert(linkedQNode.get());
                        linkedQNode->setPos(
                                linkedQNode->pos() + (currentGatePos - linkedGatePos));
                        /// 移动后相关的Gate也会受影响, 加入修改队列中
                        stack.push_back(linkedQNode.get());
                        break;
                    case ExpDataType::Corridor: {
                        /// 走廊的话直接移动端点 TODO 考虑是否合理?
                        dynamic_cast<Corridor*>(linkedExpData.get())->moveGatePos
                                (linkedGateID, UIT::QPt2TopoVec(linkedQNode->mapFromScene(currentGatePos)));
                        linkedQNode->notifySizeChange();
//                        searchedNodes.insert(linkedQNode);
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
    switch (expData()->type()) {

        case ExpDataType::Intersection:
        case ExpDataType::Stair:
        case ExpDataType::BigRoom:
        case ExpDataType::SmallRoom: {
            auto bRect = expData()->getOutBounding(0.5);
            QRectF rect;
            rect.setTopLeft(UIT::TopoVec2QPt({bRect[2], bRect[0]}));
            rect.setBottomRight(UIT::TopoVec2QPt({bRect[3], bRect[1]}));
            path.addRect(rect);
            break;
        }
        case ExpDataType::Corridor: {
            QPainterPath p;
            auto corr = dynamic_cast<Corridor*>(expData().get());
            auto pA = UIT::TopoVec2QPt(corr->getEndPointA());
            auto pB = UIT::TopoVec2QPt(corr->getEndPointB());
            p.moveTo(pA);
            p.lineTo(pB);
            QPainterPathStroker stroker;
            stroker.setWidth(UIT::QMeter(corr->halfWidth() * 2));
            path = stroker.createStroke(p);
            break;
        }
    }
    return path;
}

void tmap::QNode::normalizePos()
{
    auto offset = expData()->normalizeSelf();
    setPos(pos() + UIT::TopoVec2QPt(offset));
    notifySizeChange();
}

tmap::QNodePtr tmap::QNode::thisQnodePtr()
{
    return dynamic_pointer_cast<QNode>(shared_from_this());
}

void tmap::QNode::breakLinks()
{
    for (int i = 0; i < nLinks(); ++i) {
        auto& link = linkAt(i);
        auto linkedQNode = qNodeAt(i);
        if (linkedQNode) {
            linkedQNode->linkAt(link.at).to.reset();
            linkedQNode->linkAt(link.at).at = GATEID_NO_MAPPING;
            linkedQNode->fakeLineAt(i).reset();
        }
    }
}

tmap::QNodePtr tmap::QNode::qNodeAt(size_t index)
{
    MapNodePtr toNode = linkAt(index).to.lock();
    return dynamic_pointer_cast<QNode>(toNode);
}

tmap::FakeLine& tmap::QNode::fakeLineAt(size_t index)
{
    while (mFakeLines.size() <= index) {
        mFakeLines.emplace_back();
    }
    return mFakeLines[index];
}

QPointF tmap::QNode::gateQPos(size_t index, bool atScene) const
{
    QPointF res{0,0};
    if (expData()->nGates() <= index) {
        cerr << FILE_AND_LINE << " you are getting an invalid gate pos!" << endl;
        return res;
    }
    res = UIT::TopoVec2QPt(expData()->getGates()[index]->getPos());
    if (atScene) {
        res = mapToScene(res);
    }
    return res;
}

////////////////// NEXT TO FAKE LINE

tmap::FakeLine_IMPL::~FakeLine_IMPL()
{
    if (scene()) {
        scene()->removeItem(this);
    }
}

tmap::FakeLine_IMPL::FakeLine_IMPL(const QPointF& p1, const QPointF& p2, QNode* node1,
                                   GateID fromGate) :
        mPoints{p1, p2}, mOriNode(node1), mFrom(fromGate)
{
    setFlag(ItemIsSelectable, true);
    setLine({p1, p2});
    setZValue(-3);
    QPen pen{Qt::black, 2, Qt::DashLine, Qt::FlatCap};
    setPen(pen);
}

void tmap::FakeLine_IMPL::setPoint(tmap::QNode* who, const QPointF& newP)
{
    if (who == mOriNode) {
        mPoints[0] = newP;
    } else {
        mPoints[1] = newP;
    }
    setLine({mPoints[0], mPoints[1]});
}

void tmap::FakeLine_IMPL::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                                QWidget* widget)
{
    auto pen = painter->pen();
    if (isSelected()) {
        pen.setWidth(4);
    } else {
        pen.setWidth(2);
    }
    QGraphicsLineItem::paint(painter, option, widget);
}

tmap::QNode* tmap::FakeLine_IMPL::oriNode() const
{
    return mOriNode;
}

tmap::GateID tmap::FakeLine_IMPL::fromGate() const
{
    return mFrom;
}
