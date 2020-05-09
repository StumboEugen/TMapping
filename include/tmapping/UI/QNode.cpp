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
#include <QGraphicsSceneHoverEvent>

#include <set>
#include <iostream>

using namespace std;


tmap::QNode::QNode(MergedExpPtr mergedExp) : MapNode(std::move(mergedExp), 0)
{
    setFlag(ItemIsSelectable);
    setAcceptHoverEvents(true);
    mFakeLines.assign(expData()->nGates(), nullptr);
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

    if (isSelected()) {
        setZValue(-4);
    } else {
        setZValue(0);
    }

    const auto& centerPoint = boundingRect().center();
    QPointF halfDia{UIT::QMeter(0.2), UIT::QMeter(0.2)};
    QRectF middleHalfSq{centerPoint + halfDia, centerPoint - halfDia};

    auto oriPen = painter->pen();

    const auto& relatedExpData = expData();

    switch (relatedExpData->type()) {
        case ExpDataType::Intersection: {

            painter->setPen(QPen{Qt::black, 1, Qt::DashLine});
            auto bRect = relatedExpData->getOutBounding(0.);
            auto tl = UIT::TopoVec2QPt({bRect[2], bRect[0]});
            auto br = UIT::TopoVec2QPt({bRect[3], bRect[1]});
            painter->drawRect(QRectF{tl,br});
            if (isSelected()) {
                painter->setPen(QPen{Qt::green, 5});
                painter->drawRect(QRectF{tl,br});
            }
            painter->setPen(oriPen);

            painter->setBrush(isSelected() ? Qt::green : Qt::yellow);
//            if (isSelected()) {
//                painter->setPen(QPen{Qt::black, 4});
//            }
            painter->drawEllipse(middleHalfSq);
            painter->setPen(oriPen);

            for (const auto& gate : relatedExpData->getGates()) {
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
            break;
        }
        case ExpDataType::Stair:
            cerr << FILE_AND_LINE << " unimplemented exp type!" << endl;
            break;
        case ExpDataType::Room: {
            auto bRect = relatedExpData->getOutBounding(0.);
            auto tl = UIT::TopoVec2QPt({bRect[2], bRect[0]});
            auto br = UIT::TopoVec2QPt({bRect[3], bRect[1]});
            painter->drawRect(QRectF{tl,br});
            if (isSelected()) {
                painter->setPen(QPen{Qt::green, 5});
                painter->drawRect(QRectF{tl,br});
            }
            painter->setPen(oriPen);
            painter->setBrush(isSelected() ? Qt::green : Qt::yellow);
            painter->drawRect(middleHalfSq);
            painter->setPen(oriPen);
            break;
        }
    }

    for (const auto& plm : relatedExpData->getPLMs()) {
        UIT::drawLandMark(painter, plm.get(), true);
    }

    const auto& gates = relatedExpData->getGates();
    refreshGateDrawing();
    for (int i = 0; i < gates.size(); ++i) {
        if (mDrawGate[i]) {
            UIT::drawGate(painter, gates[i].get(), true, true);
        }
    }

    if (mHighLightGate >= 0) {
        painter->setPen(Qt::red);
        painter->setBrush(Qt::NoBrush);
        QPointF diag{7.5, 7.5};
        auto center = gateQPos(mHighLightGate, false);
        painter->drawEllipse({center - diag, center + diag});
    }
}

void tmap::QNode::notifySizeChange()
{
    /// 遍历更新FakeLine的状态
    for (int i = 0; i < expData()->nGates(); ++i) {
        if (const auto& linkedQNode = qNodeAt(i)) {
            const auto& linkedGatePos = linkedQNode->gateQPos(linkedGIDAt(i));
            const auto& currentGatePos = gateQPos(i);
            auto& fakeLine = fakeLineAt(i);
            if (fakeLine) {
                /// 如果fakeLine存在的话直接重新设置位置点
                fakeLine->setPoint(this, currentGatePos);
            }
            else if (linkedGatePos != currentGatePos) {
                /// 判断一下位置是否有问题, 有的话制造一个位置点出来
                fakeLine = make_shared<FakeLine_IMPL>(currentGatePos, linkedGatePos,
                        this, i);
                linkedQNode->fakeLineAt(linkedGIDAt(i)) = fakeLine;
                scene()->addItem(fakeLine.get());
            }
        }
    }

    mBoundingRect.setWidth(0.);
    prepareGeometryChange();
    update();
}

tmap::QNodePtr tmap::QNode::makeOneFromExpData(const tmap::ExpDataPtr& relatedExpData, MoveStragety ms)
{
    auto exp = make_shared<Exp>(relatedExpData, 0);
    auto pNode = new QNode(MergedExp::singleMergedFromExp(exp));
    pNode->setMoveStragety(ms);
    return QNodePtr(pNode);
}

tmap::QNodePtr tmap::QNode::makeOneFromMergedExp(const MergedExpPtr& relatedMergedExp, MoveStragety ms)
{
    auto pNode = new QNode(relatedMergedExp);
    pNode->setMoveStragety(ms);
    return QNodePtr(pNode);
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
    if (mMoveStragety == MoveStragety::NO_NODE_FOLLOW) {
        this->notifySizeChange();
        return;
    }

    vector<QNode*> stack{this};
    set<QNode*> movedNodes;
    movedNodes.insert(this);
    vector<QNode*> touchedNodes{this};

    while (!stack.empty()) {
        auto current = stack.back();
        stack.pop_back();
        /// 检查每一个link, 通知其需要随我运动
        for (int i = 0; i < current->nLinks(); ++i) {
            if (auto linkedQNode = current->qNodeAt(i)) {

                if (movedNodes.find(linkedQNode.get()) != movedNodes.end()) {
                    /// 要处理的linkedQNode之前已经处理过了(移动过了, 所以对于Corridor可能会多次遇到)
                    /// 跳过, 不要发生死循环
                    continue;
                }

                /// 当前Gate的全局Pos(在Scene中)
                auto currentGatePos = current->gateQPos(i);

                if (auto fakeLine = current->fakeLineAt(i)) {
                    /// 检查是否为Fake连接, 如果是的话, 只要改动Fake连接即可
                    fakeLine->setPoint(current, currentGatePos);
                    continue;
                }
                /// 除了moved ones, 接触到的都可能有尺寸改变
                touchedNodes.push_back(linkedQNode.get());

                GateID linkedGID = current->linkedGIDAt(i);
                /// linkedGate相对于linkedQNode的原点的位置
                auto linkedGatePos = linkedQNode->gateQPos(linkedGID, false);

                const auto& linkedExpData = linkedQNode->expData();
                switch (linkedExpData->type()) {
                    case ExpDataType::Intersection:
                    case ExpDataType::Room:
                        /// Intersection和SmallRoom采用相同的策略, 跟随current移动
                        movedNodes.insert(linkedQNode.get());
                        linkedQNode->setPos(currentGatePos - linkedGatePos);
                        /// 移动后相关的Gate也会受影响, 加入修改队列中
                        stack.push_back(linkedQNode.get());
                        break;
                    case ExpDataType::Corridor: {
                        if (mMoveStragety != MoveStragety::ONLY_FIXED_TYPE) {
                            /// 走廊的话直接移动端点
                            dynamic_cast<Corridor*>(linkedExpData.get())->moveGatePos
                                    (linkedGID, UIT::QPt2TopoVec(linkedQNode->mapFromScene(currentGatePos)));
//                        searchedNodes.insert(linkedQNode);
                        }
                        break;
                    }
                    default:
                        cerr << FILE_AND_LINE << " Unimplemented type! " <<
                             linkedExpData->typeStr() << endl;
                        break;
                }
            }
        }
    }

    for (const auto& node : touchedNodes) {
        node->notifySizeChange();
    }
}

QPainterPath tmap::QNode::shape() const
{
    QPainterPath path;
    switch (expData()->type()) {

        case ExpDataType::Intersection:
        case ExpDataType::Stair:
        case ExpDataType::Room: {
            auto bRect = expData()->getOutBounding(0.1);
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
        breakLinkAt(i);
//        auto& link = linkAt(i);
//        auto linkedQNode = qNodeAt(i);
//        if (linkedQNode) {
//            linkedQNode->linkAt(link.at).to.reset();
//            linkedQNode->linkAt(link.at).at = GATEID_NO_MAPPING;
//            linkedQNode->fakeLineAt(link.at).reset();
//            this->fakeLineAt(i).reset();
//        }
    }
}

void tmap::QNode::breakLinkAt(int i)
{
    auto& link = linkAt(i);
    auto linkedQNode = qNodeAt(i);
    if (linkedQNode) {
        linkedQNode->linkAt(link.at).to.reset();
        linkedQNode->linkAt(link.at).at = GATEID_NO_MAPPING;
        linkedQNode->fakeLineAt(link.at).reset();
        this->fakeLineAt(i).reset();

        link.to.reset();
        link.at = GATEID_NO_MAPPING;
        fakeLineAt(i).reset();
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

QPointF tmap::QNode::plmQPos(size_t index, bool atScene) const
{
    QPointF res{0,0};
    if (expData()->getPLMs().size() <= index) {
        cerr << FILE_AND_LINE << " you are getting an invalid PLM pos!" << index << endl;
        return res;
    }
    res = UIT::TopoVec2QPt(expData()->getPLMs()[index]->getPos());
    if (atScene) {
        res = mapToScene(res);
    }
    return res;
}

void tmap::QNode::hoverMoveEvent(QGraphicsSceneHoverEvent* event)
{
    QGraphicsItem::hoverMoveEvent(event);
    mouseHoverAt(mapFromScene(event->scenePos()));
}

void tmap::QNode::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
    QGraphicsItem::hoverLeaveEvent(event);
    mHighLightGate = GATEID_NO_MAPPING;
    update();
}

void tmap::QNode::mouseHoverAt(const QPointF& at)
{
    mHighLightGate = expData()->findGateAtPos(UIT::QPt2TopoVec(at), UIT::pix2meter(15));
    update();
}

/**
 * @brief 更新每个gate另一端的绘制情况, 从而确认this的gate要不要绘制
 */
void tmap::QNode::refreshGateDrawing()
{
    while (mDrawGate.size() <= expData()->nGates()) {
        mDrawGate.push_back(false);
    }
    for (int gid = 0; gid < expData()->nGates(); ++gid) {
        if (auto connectedQNode = qNodeAt(gid)) {
            if (!fakeLineAt(gid)) {
                auto thatGID = linkedGIDAt(gid);
                auto& thatDrawVec = connectedQNode->mDrawGate;
                while (thatDrawVec.size() <= thatGID) {
                    thatDrawVec.push_back(false);
                }
                mDrawGate[gid] = !thatDrawVec[thatGID];
                continue;
            }
        }
        mDrawGate[gid] = true;
    }
}

void tmap::QNode::setMoveStragety(tmap::MoveStragety moveStragety)
{
    if (expData()->type() == ExpDataType::Corridor) {
        setFlag(ItemIsMovable, moveStragety == MoveStragety::NO_NODE_FOLLOW);
    }
    QNode::mMoveStragety = moveStragety;
}

void tmap::QNode::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
    QGraphicsItem::mousePressEvent(event);
    if (event->modifiers() & Qt::CTRL && event->button() == Qt::MidButton) {
        auto gid = expData()->findGateAtPos(UIT::QPt2TopoVec(event->pos()), UIT::pix2meter(15));
        if (gid >= 0) {
            removeConnection(gid);
        }
    }
}

void tmap::QNode::removeConnection(size_t gid)
{
    auto& linkThis = linkAt(gid);
    if (auto thatNode = linkThis.to.lock()) {
        if (auto fakeLine = fakeLineAt(gid)) {
            fakeLine->sucide();
        }
        auto thatGID = linkedGIDAt(gid);
        auto& linkThat = thatNode->linkAt(thatGID);
        linkThat.to.reset();
        linkThat.at = GATEID_NO_MAPPING;
        linkThis.to.reset();
        linkThis.at = GATEID_NO_MAPPING;
    }
}

bool tmap::QNode::isPassed() const
{
    return passed;
}

void tmap::QNode::setPassed(bool pass)
{
    passed = pass;
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
    if (isSelected()) {
        QLineF line2Draw{mapFromScene(mPoints[0]), mapFromScene(mPoints[1])};
        line2Draw.setLength(line2Draw.length() / 4);
        painter->setPen({Qt::red, 4});
        painter->drawLine(line2Draw);
    }
}

tmap::QNode* tmap::FakeLine_IMPL::oriNode() const
{
    return mOriNode;
}

tmap::GateID tmap::FakeLine_IMPL::fromGate() const
{
    return mFrom;
}

QPainterPath tmap::FakeLine_IMPL::shape() const
{
    QPainterPath path;
    path.moveTo(line().p1());
    path.lineTo(line().p2());
    QPainterPathStroker stroker;
    stroker.setWidth(5);
    return stroker.createStroke(path);
}

void tmap::FakeLine_IMPL::sucide()
{
    if (mOriNode) {
        mOriNode->fakeLineAt(mFrom).reset();
        mOriNode->qNodeAt(mFrom)->fakeLineAt(mOriNode->linkedGIDAt(mFrom)).reset();
    }
}
