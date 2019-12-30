//
// Created by stumbo on 2019/12/20.
//

#include <iostream>
#include <queue>

#include <QWheelEvent>
#include <QStyleOptionGraphicsItem>

#include "MainGView.h"
#include "tmapping/MergedExp.h"
#include "tmapping/Exp.h"
#include "UITools.h"

using namespace std;

tmap::MainGView::MainGView(QWidget* parent) : QGraphicsView(parent)
{
    setScene(&mScene4FakeMap);
    /// 设置默认的缩放系数, 缩小至0.75倍
    setMatrix(transform().scale(0.75, 0.75).toAffine());
}

static constexpr double SCALE_MAX = 10.0;
static constexpr double SCALE_TIME = 1.2;

void tmap::MainGView::wheelEvent(QWheelEvent* event)
{
    if (event->modifiers() & Qt::ControlModifier) {
        auto curTrans = transform();
        auto curScale = curTrans.m11();
        if (event->delta() > 0 && curScale <= SCALE_MAX) {
            curTrans.scale(SCALE_TIME, SCALE_TIME);
        }
        else if (event->delta() < 0 && curScale >= 1 / SCALE_MAX) {
            curTrans.scale(1 / SCALE_TIME, 1 / SCALE_TIME);
        }
        setMatrix(curTrans.toAffine());
    } else {
        QGraphicsView::wheelEvent(event);
    }
}

void tmap::MainGView::addNode2FakeMapFromExpData(const tmap::ExpDataPtr& usedExpData)
{
    auto qNode = QNode::makeOneFromExpData(usedExpData);
    mNodesInFakeMap.insert(qNode);
    mScene4FakeMap.addItem(qNode.get());
    qNode->setFlag(QGraphicsItem::ItemIsMovable, mEnableFakeNodesMoving);
    qNode->setPos(sceneRect().center());
}

void tmap::MainGView::SLOT_EnableMoving4FakeNodes(bool enableMove)
{
    mEnableFakeNodesMoving = enableMove;
    for (auto& item : mScene4FakeMap.items()) {
        if (auto qNode = dynamic_cast<QNode*>(item)) {
            if (qNode->relatedMergedExp->getMergedExpData()->type() != ExpDataType::Corridor) {
                qNode->setFlag(QGraphicsItem::ItemIsMovable, enableMove);
            }
        }
    }
}

void tmap::MainGView::SLOT_EnableGridRestriction(bool enableRes)
{
    mEnableNodeRestriction = enableRes;
}

void tmap::MainGView::restrictQNode(tmap::QNode* qNode)
{
    if (qNode &&
        qNode->scene() == &mScene4FakeMap &&
        qNode->relatedMergedExp->getMergedExpData()->type() != ExpDataType::Corridor) {
        auto pos = qNode->pos();
        auto roundedP = UIT::QPt2TopoVec(pos).round2();
        qNode->setPos(UIT::TopoVec2QPt(roundedP));
        qNode->notifyNeighbours2Move();
    }
}

void tmap::MainGView::SLOT_StartDrawingEdge(bool enableDrawing)
{
    mIsDrawingEdge = enableDrawing;

    if (enableDrawing) {
        setCursor(Qt::CrossCursor);
    } else {
        setCursor(Qt::ArrowCursor);
    }
}

void tmap::MainGView::SLOT_RemoveSelectedNodes()
{
    for (auto& item : mScene4FakeMap.selectedItems()) {
        auto qNode = dynamic_cast<QNode*>(item);
        if (qNode) {
            for (auto& link : qNode->links) {
                auto linkedNode = link.to.lock();
                if (linkedNode) {
                    linkedNode->links[link.at].to.reset();
                    linkedNode->links[link.at].at = GATEID_NO_MAPPING;
                }
            }
            mNodesInFakeMap.erase(qNode->shared_from_this());
        }
    }
}

void tmap::MainGView::mousePressEvent(QMouseEvent* event)
{
    QGraphicsView::mousePressEvent(event);
    const auto & clickPosInView = event->pos();
    const auto & clickPosInScene = mapToScene(clickPosInView);
    auto item = scene()->itemAt(clickPosInScene);

    if (item) {
        if (auto clickedQNode = dynamic_cast<QNode*>(item)) {

            if (mIsDrawingEdge && event->button() == Qt::LeftButton) {
                /// 被点中的QNode的坐标系中点击的位置
                const auto& clickPosInItem = clickedQNode->mapFromScene(clickPosInScene);
                /// 被点中的QNode对应的ExpData数据
                const auto& clickedExpData = clickedQNode->relatedMergedExp->getMergedExpData();
                /// 查找被点中的GateID
                int clickedGateID = clickedExpData->
                        findGateAtPos(UIT::QPt2TopoVec(clickPosInItem),UIT::pix2meter(15));
                if (clickedGateID >= 0) {
                    auto& toNewQNode = clickedQNode->links[clickedGateID].to;
                    if (toNewQNode.expired()) {
                        /// 这个Gate还没有连接任何的地方, 开始连接

                        /// 被点中的Gate
                        const auto& clickedGate = clickedExpData->getGates()[clickedGateID];
                        /// 新Gate按照这个Gate制作
                        auto newGate = clickedGate->clone();
                        newGate->setPos({0,0});
                        newGate->setNormalVec(clickedGate->getNormalVec().rotate(180));
                        /// 开始制作新的Corridor
                        auto newCorridor = make_shared<Corridor>();
                        newCorridor->addGate(std::move(newGate));
                        newCorridor->setEndGateA(0);
                        /// 制作对应的QNode
                        auto newQNode = QNode::makeOneFromExpData(newCorridor);
                        /// 添加连接关系
                        clickedQNode->links[clickedGateID].to = newQNode;
                        clickedQNode->links[clickedGateID].at = 0;
                        MapNode::Link l;
                        newQNode->links[0].to = clickedQNode->shared_from_this();
                        newQNode->links[0].at = clickedGateID;
                        /// 添加刚刚制作好的QNode
                        mScene4FakeMap.addItem(newQNode.get());
                        newQNode->setPos(clickedQNode->mapToScene(
                                UIT::TopoVec2QPt(clickedGate->getPos())));
                        mNodesInFakeMap.insert(newQNode);
                        mTheDrawingCorridor = std::move(newQNode);
                    }
                }
            }
        }
    }
}

void tmap::MainGView::mouseMoveEvent(QMouseEvent* event)
{
    QGraphicsView::mouseMoveEvent(event);
    if (mIsDrawingEdge && mTheDrawingCorridor) {
        const auto& clickPosInScene = mapToScene(event->pos());
        const auto& clickPosInQNode = mTheDrawingCorridor->mapFromScene(clickPosInScene);
        dynamic_cast<Corridor*>(mTheDrawingCorridor->relatedMergedExp->getMergedExpData().get())
                ->setEndPointB(UIT::QPt2TopoVec(clickPosInQNode));
        mTheDrawingCorridor->notifySizeChange();
    }
}

void tmap::MainGView::mouseReleaseEvent(QMouseEvent* event)
{
    QGraphicsView::mouseReleaseEvent(event);
    const auto & clickPosInView = event->pos();
    const auto & clickPosInScene = mapToScene(clickPosInView);
    auto item = scene()->itemAt(clickPosInScene);

    if (item) {
        if (auto clickedQNode = dynamic_cast<QNode*>(item)) {
            if (mEnableNodeRestriction) {
                if (clickedQNode->relatedMergedExp->getMergedExpData()->type() != ExpDataType::Corridor) {
                    restrictQNode(clickedQNode);
                }
            }

            if (mIsDrawingEdge && mTheDrawingCorridor) {
                if (clickedQNode != mTheDrawingCorridor.get()) {
                    /// 被点中的QNode的坐标系中点击的位置
                    const auto& clickPosInItem = clickedQNode->mapFromScene(clickPosInScene);
                    /// 被点中的QNode对应的ExpData数据
                    const auto& clickedExpData = clickedQNode->relatedMergedExp->getMergedExpData();
                    /// 查找被点中的GateID
                    int clickedGateID = clickedExpData->
                            findGateAtPos(UIT::QPt2TopoVec(clickPosInItem),UIT::pix2meter(15));
                    if (clickedGateID >= 0) {
                        auto& toNewQNode = clickedQNode->links[clickedGateID].to;
                        if (toNewQNode.expired()) {
                            /// 这个Gate还没有连接任何的地方, 那就连接到这个Gate

                            /// 被点中的Gate
                            const auto& clickedGate = clickedExpData->getGates()[clickedGateID];
                            /// 新Gate按照这个Gate制作
                            auto newGate = clickedGate->clone();
                            auto gatePosInClickedNode = UIT::TopoVec2QPt(clickedGate->getPos());
                            auto gatePosInScene = clickedQNode->mapToScene(gatePosInClickedNode);
                            auto gatePosInThis = mTheDrawingCorridor->mapFromScene(gatePosInScene);
                            newGate->setPos(UIT::QPt2TopoVec(gatePosInThis));
                            newGate->setNormalVec(clickedGate->getNormalVec().rotate(180));
                            /// 添加新的Gate
                            auto theDrawingCorridor = dynamic_cast<Corridor*>
                            (mTheDrawingCorridor->relatedMergedExp->getMergedExpData().get());
                            theDrawingCorridor->addGate(std::move(newGate));
                            theDrawingCorridor->setEndGateB(1);
                            if (theDrawingCorridor->getGates().size() != 2) {
                                cerr << FILE_AND_LINE << " You add a gate before corridor complete!"
                                     << endl;
                            }
                            /// 添加连接关系
                            clickedQNode->links[clickedGateID].to = mTheDrawingCorridor;
                            clickedQNode->links[clickedGateID].at = 1;
                            MapNode::Link l;
                            l.to = clickedQNode->shared_from_this();
                            l.at = clickedGateID;
                            mTheDrawingCorridor->links.push_back(std::move(l));
                            /// 完成制作, 确定图形
                            mTheDrawingCorridor->notifySizeChange();
                            mTheDrawingCorridor.reset();
                            return;
                        }
                    }
                }
                const auto& clickPosInQNode = mTheDrawingCorridor->mapFromScene(clickPosInScene);
                dynamic_cast<Corridor*>(mTheDrawingCorridor->relatedMergedExp->getMergedExpData().get())
                        ->setEndPointB(UIT::QPt2TopoVec(clickPosInQNode));
                mTheDrawingCorridor->notifySizeChange();
                mTheDrawingCorridor.reset();
            }
        }
    }

}

void tmap::MainGView::saveFakeMap(const std::string& mapName)
{
    vector<MapNodePtr> nodes{mNodesInFakeMap.begin(), mNodesInFakeMap.end()};
    int res = JsonHelper::saveJson(
            StructedMapImpl(nodes, nullptr, 1.0).toJS(), mapName, false);
    if (res < 0) {
        cerr << FILE_AND_LINE << "Save Json ERROR!" << endl;
    } else {
        cout << "map save success! [name:" << mapName << "]" << endl;
    }
}

tmap::MainGView::~MainGView()
{
    /// QNode是智能指针管理的, 不要交给scene校徽
    mNodesInFakeMap.clear();
}

void tmap::MainGView::loadMap(const std::string& fileName)
{
    Jsobj jMap = JsonHelper::loadJson(fileName);
    if (jMap.isNull()) {
        cerr << FILE_AND_LINE << " load Map json FAIL " << endl;
        return;
    }
    /// 读取Json文件成功

    /// 为了维护正确的依赖关系, 构造出只含有MapNode的Map, 再从这些MapNode生成QNode
    /// 由于底层大量数据是复用的, 主要是指针的传送
    StructedMapImpl map(jMap);
    const auto& nodes = map.getNodes();
    vector<QNodePtr> qNodes(nodes.size());
    for (int i = 0; i < nodes.size(); ++i) {
        qNodes[i] = QNode::makeOneFromMergedExp(nodes[i]->relatedMergedExp);
    }
    /// links的内容需要被更新, 连接对象的指针需要被更改
    for (int i = 0; i < nodes.size(); ++i) {
        auto& newLinks = qNodes[i]->links;
        const auto& oldLinks = nodes[i]->links;
        newLinks = oldLinks;
        for (int j = 0; j < newLinks.size(); ++j) {
            const auto& toNode = oldLinks[j].to.lock();
            if (toNode) {
                newLinks[j].to = qNodes[toNode->serial];
            }
        }
    }

    /// 开始BFS构造地图
    mNodesInFakeMap.clear();
    bool once = false;
    for (auto& qNode : qNodes) {
        /// 为了处理拓扑不相连的情况, 对qNodes中所有成员进行BFS, 理论上其实下面这个if只发生一次
        if (mNodesInFakeMap.find(qNode) == mNodesInFakeMap.end()) {
            if (once) {
                /// 发生了两次, 说明存在不相连的情况
                cout << FILE_AND_LINE << " this map doesnt fully connected!" << endl;
            } else {
                once = true;
            }
            /// BFS使用的队列
            queue<QNode*> lookupQueue;
            lookupQueue.push(qNode.get());
            /// BFS中的元素是已经添加过的QNode
            qNode->setPos(0., 0.);
            mNodesInFakeMap.insert(qNode);

            while (!lookupQueue.empty()) {
                auto& currentQnode = lookupQueue.front();
                lookupQueue.pop();
                auto& currentLinks = currentQnode->links;
                /// 查找所有的连接
                for (int i = 0; i < currentLinks.size(); ++i) {
                    auto& currentLink = currentLinks[i];
                    auto link2 = currentLink.to.lock();
                    /// 是否与实际的QNode相连?
                    if (link2) {
                        auto linkedQNode = dynamic_pointer_cast<QNode>(link2);
                        /// 是否已经被遍历过?
                        if (mNodesInFakeMap.find(linkedQNode)
                            == mNodesInFakeMap.end()) {
                            /// 计算QNode应该放置的位置
                            auto currentLinkGatePos = currentQnode->mapToScene(UIT::TopoVec2QPt(
                                    currentQnode->relatedMergedExp->getMergedExpData()->getGates()[i]->getPos()));
                            auto anotherGatePosInNode = UIT::TopoVec2QPt(
                                    linkedQNode->relatedMergedExp->getMergedExpData()
                                    ->getGates()[currentLink.at]->getPos());
                            linkedQNode->setPos(currentLinkGatePos - anotherGatePosInNode);
                            mNodesInFakeMap.insert(linkedQNode);
                            lookupQueue.push(linkedQNode.get());
                        }
                    }
                }
            }
            mNodesInFakeMap.insert(std::move(qNode));
        }
    }
    for (const auto& qNode : mNodesInFakeMap) {
        if (qNode->relatedMergedExp->getMergedExpData()->type() != ExpDataType::Corridor) {
            qNode->setFlag(QGraphicsItem::ItemIsMovable, mEnableFakeNodesMoving);
        }
        mScene4FakeMap.addItem(qNode.get());
    }
}
