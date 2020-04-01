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
    auto qNode = QNode::makeOneFromExpData(usedExpData, mMoveStragety);
    mNodesInFakeMap.insert(qNode);
    mScene4FakeMap.addItem(qNode.get());
    setQNodeMovabilityInFakeMap(qNode.get());
    qNode->setPos(sceneRect().center());
}

void tmap::MainGView::SLOT_EnableMoving4FakeNodes(bool enableMove)
{
    mEnableFakeNodesMoving = enableMove;
    for (auto& item : mScene4FakeMap.items()) {
        if (auto qNode = dynamic_cast<QNode*>(item)) {
            setQNodeMovabilityInFakeMap(qNode);
        }
    }
}

void tmap::MainGView::SLOT_EnableMoving4RealNodes(bool enableMove)
{
    mEnableRealNodesMoving = enableMove;
    for (auto& item : mScene4RealMap.items()) {
        if (auto qNode = dynamic_cast<QNode*>(item)) {
            setQNodeMovabilityInRealMap(qNode);
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
        qNode->expData()->type() != ExpDataType::Corridor) {
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
        if (auto qNode = dynamic_cast<QNode*>(item)) {
            qNode->breakLinks();
            mNodesInFakeMap.erase(qNode->thisQnodePtr());
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

            /// 被点中的QNode的坐标系中点击的位置
            const auto& clickPosInItem = clickedQNode->mapFromScene(clickPosInScene);
            /// 被点中的QNode对应的ExpData数据
            const auto& clickedExpData = clickedQNode->expData();

            if (mIsDrawingEdge && event->button() == Qt::LeftButton) {
                /**
                 * @brief 画边模式
                 */

                /// 查找被点中的GateID
                int clickedGateID = clickedExpData->
                        findGateAtPos(UIT::QPt2TopoVec(clickPosInItem),UIT::pix2meter(15));

                if (clickedGateID >= 0) {
                    if (!clickedQNode->qNodeAt(clickedGateID)) {
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
                        auto newQNode = QNode::makeOneFromExpData(newCorridor, mMoveStragety);
                        /// 确保能够链接到正确的QNode上(其他的表层QNode)
                        newQNode->setZValue(-2);
                        /// 添加连接关系
                        newQNode->setLinkAtIndex(0, clickedQNode->thisQnodePtr(),
                                                 clickedGateID);
                        /// 添加刚刚制作好的QNode
                        mScene4FakeMap.addItem(newQNode.get());
                        newQNode->setPos(clickedQNode->mapToScene(
                                UIT::TopoVec2QPt(clickedGate->getPos())));
                        mNodesInFakeMap.insert(newQNode);
                        mTheDrawingCorridor = std::move(newQNode);
                    }
                }
            }
            else if (clickedQNode->expData()->type() == ExpDataType::Corridor
                     && mAcceptAddingGate2Corridor
                     && event->button() == Qt::LeftButton) {
                /**
                 * @brief 在Corridor上画Gate的模式
                 */

                auto clickedCorridor = dynamic_cast<Corridor*>(clickedExpData.get());
                /// 通过点击位置计算得到对应的Gate位置参数
                auto res = clickedCorridor->calPosAmdNvFromPointC(
                        UIT::QPt2TopoVec(clickPosInItem));
                clickedExpData->addGate(GatePtr{
                    new Door(res.first, res.second, true)});
                mTheDrawingCorridor = clickedQNode->thisQnodePtr();
                mTheDrawingCorridor->notifySizeChange();
            }
            else if (mIsDrawingDirectLink && event->button() == Qt::LeftButton) {
                /**
                 * @brief 直接画Fake Line连接的模式
                 */
                /// 查找被点中的GateID
                int clickedGateID = clickedExpData->
                        findGateAtPos(UIT::QPt2TopoVec(clickPosInItem),UIT::pix2meter(15));
                if (clickedGateID >= 0 && !clickedQNode->qNodeAt(clickedGateID)) {
                    /// click在了一个Gate上, 而且这个Gate没有已经被连接
                    const auto& gatePos = clickedQNode->gateQPos(clickedGateID, true);
                    /// 创建FakeLine, 但是先不添加到Gate上
                    auto fakeLine = make_shared<FakeLine_IMPL>
                            (gatePos, gatePos, clickedQNode, clickedGateID);
                    mScene4FakeMap.addItem(fakeLine.get());
                    mTheDrawingFakeLine = std::move(fakeLine);
                }
            }
        }
    }

    if (mAtSim && mRobot && event->button() & Qt::RightButton) {
        if (mRobot->try2move(clickPosInScene)) {
            if (event->modifiers() & Qt::CTRL) {
                if (auto oldExp = mRobot->try2ThroughGate()) {
                    Q_EMIT SIG_RobotThroughGate(oldExp);
                }
            }
            mRobot->atNode()->setSelected(true);
        }
    }
}

void tmap::MainGView::mouseMoveEvent(QMouseEvent* event)
{
    QGraphicsView::mouseMoveEvent(event);
    const auto& clickPosInScene = mapToScene(event->pos());
    if (mIsDrawingEdge && mTheDrawingCorridor) {
        const auto& clickPosInQNode = mTheDrawingCorridor->mapFromScene(clickPosInScene);
        dynamic_cast<Corridor*>(mTheDrawingCorridor->expData().get())
                ->setEndPointB(UIT::QPt2TopoVec(clickPosInQNode));
        mTheDrawingCorridor->notifySizeChange();
    }
    else if (mAcceptAddingGate2Corridor && mTheDrawingCorridor) {
        const auto& clickPosInQNode = mTheDrawingCorridor->mapFromScene(clickPosInScene);
        auto clickedCorridor = dynamic_cast<Corridor*>(mTheDrawingCorridor->expData().get());
        auto res = clickedCorridor->calPosAmdNvFromPointC(UIT::QPt2TopoVec(clickPosInQNode));
        auto theDrawingGate = mTheDrawingCorridor->expData()->getGates().back();
        if (theDrawingGate->getNormalVec() != res.second) {
            /// gate的方向改变了, 调整法向量并根据width调整位置
            theDrawingGate->setNormalVec(res.second);
            theDrawingGate->setPos(
                    theDrawingGate->getPos() + res.second * 2 *clickedCorridor->halfWidth());
            mTheDrawingCorridor->notifySizeChange();
        }
    }
    else if (mIsDrawingDirectLink && mTheDrawingFakeLine) {
        mTheDrawingFakeLine->setPoint(nullptr, clickPosInScene);
    }
    if (auto qNode = dynamic_cast<QNode*>(scene()->itemAt(clickPosInScene))) {
        qNode->mouseHoverAt(qNode->mapFromScene(clickPosInScene));
    }
}

void tmap::MainGView::mouseReleaseEvent(QMouseEvent* event)
{
    QGraphicsView::mouseReleaseEvent(event);
    const auto & clickPosInView = event->pos();
    const auto & clickPosInScene = mapToScene(clickPosInView);
    auto item = scene()->itemAt(clickPosInScene);

    /// FakeLine不在QNode的继承体系中, 可能会在空白处松开, 因此单独考虑
    if (mIsDrawingDirectLink && mTheDrawingFakeLine) {
        /**
         * 画fakeLine的模式
         */
        if (auto clickedQNode = dynamic_cast<QNode*>(item)) {
            if (clickedQNode != mTheDrawingCorridor.get()) {
                /// 被点中的QNode的坐标系中的点击的位置
                const auto& clickPosInItem = clickedQNode->mapFromScene(clickPosInScene);
                /// 被点中的QNode对应的ExpData数据
                const auto& clickedExpData = clickedQNode->expData();
                /// 查找被点中的GateID
                int clickedGateID = clickedExpData->findGateAtPos(
                        UIT::QPt2TopoVec(clickPosInItem), UIT::pix2meter(15));
                if (clickedGateID >= 0 && !clickedQNode->qNodeAt(clickedGateID)) {
                    mTheDrawingFakeLine->setPoint(clickedQNode,
                            clickedQNode->gateQPos(clickedGateID, true));
                    auto theOriQNode = mTheDrawingFakeLine->oriNode();
                    clickedQNode->setLinkAtIndex(clickedGateID,
                                                 theOriQNode->shared_from_this(),
                                                 mTheDrawingFakeLine->fromGate());
                    clickedQNode->fakeLineAt(clickedGateID) = mTheDrawingFakeLine;
                    theOriQNode->fakeLineAt(mTheDrawingFakeLine->fromGate()) =
                            std::move(mTheDrawingFakeLine);
                }
            }
        }
        /// 没有点在正确的Gate上, 或者其他什么原因, 导致FakeLine没用, 直接删除
        mTheDrawingFakeLine.reset();
    }
    else if (item) {
        if (auto clickedQNode = dynamic_cast<QNode*>(item)) {
            if (mEnableNodeRestriction) {
                /**
                 * 位置限定模式
                 */
                if (clickedQNode->expData()->type() != ExpDataType::Corridor) {
                    restrictQNode(clickedQNode);
                }
            }
            else if (mIsDrawingEdge && mTheDrawingCorridor) {
                /**
                 * Corridor链接模式
                 */
                if (clickedQNode != mTheDrawingCorridor.get()) {
                    /// 被点中的QNode的坐标系中点击的位置
                    const auto& clickPosInItem = clickedQNode->mapFromScene(clickPosInScene);
                    /// 被点中的QNode对应的ExpData数据
                    const auto& clickedExpData = clickedQNode->expData();
                    /// 查找被点中的GateID
                    int clickedGateID = clickedExpData->
                            findGateAtPos(UIT::QPt2TopoVec(clickPosInItem),UIT::pix2meter(15));
                    if (clickedGateID >= 0) {
                        if (!clickedQNode->qNodeAt(clickedGateID)) {
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
                            (mTheDrawingCorridor->expData().get());
                            theDrawingCorridor->addGate(std::move(newGate));
                            theDrawingCorridor->setEndGateB(1);
                            if (theDrawingCorridor->getGates().size() != 2) {
                                cerr << FILE_AND_LINE << " You add a gate before corridor complete!"
                                     << endl;
                            }
                            /// 添加连接关系
                            mTheDrawingCorridor->addNewLink(
                                    clickedQNode->shared_from_this(), clickedGateID);
                            /// 完成制作, 确定图形
                            mTheDrawingCorridor->notifySizeChange();
                            mTheDrawingCorridor->setZValue(0);
                            mTheDrawingCorridor.reset();
                            return;
                        }
                    }
                    /// 松开的位置不在clickedNode的Gate上
                }
                /// 没有松在Qnode上
                const auto& clickPosInQNode = mTheDrawingCorridor->mapFromScene(clickPosInScene);
                const auto& clickTopoPos = UIT::QPt2TopoVec(clickPosInQNode);
                Corridor* corr = dynamic_cast<Corridor*>(
                        mTheDrawingCorridor->expData().get());
                auto newDoor = make_shared<Door>(
                        clickTopoPos, clickTopoPos - corr->getEndPointA(), false);
                corr->addGate(std::move(newDoor));
                corr->setEndGateB(corr->nGates() - 1);
                mTheDrawingCorridor->addNewDanglingLink();
                mTheDrawingCorridor->notifySizeChange();
                mTheDrawingCorridor->setZValue(0);
                mTheDrawingCorridor.reset();
            }
        }
    }
    else if (mAcceptAddingGate2Corridor && mTheDrawingCorridor) {
        /// Corridor上画Gate的模式
        const auto& clickPosInQNode = mTheDrawingCorridor->mapFromScene(clickPosInScene);
        auto clickedCorridor = dynamic_cast<Corridor*>(mTheDrawingCorridor->expData().get());
        auto res = clickedCorridor->calPosAmdNvFromPointC(UIT::QPt2TopoVec(clickPosInQNode));
        auto theDrawingGate = mTheDrawingCorridor->expData()
                ->getGates().back();
        if (theDrawingGate->getNormalVec() != res.second) {
            theDrawingGate->setNormalVec(res.second);
            theDrawingGate->setPos(
                    theDrawingGate->getPos() + res.second * 2 *clickedCorridor->halfWidth());
        }

        auto theAddedGate = clickedCorridor->popBackGate();
        /// 检查添加的位置是否有太近的Gate
        int clickedGateID = clickedCorridor->findGateAtPos(theAddedGate->getPos(), 0.5);
        if (clickedGateID >= 0) {
            cout << FILE_AND_LINE << " The clicked Pos is too close to an exist "
                                     "Gate" << endl;
        } else {
            clickedCorridor->addGate(std::move(theAddedGate));
            mTheDrawingCorridor->addNewDanglingLink();
        }
        mTheDrawingCorridor->notifySizeChange();
        mTheDrawingCorridor.reset();
    }

}

void tmap::MainGView::mouseDoubleClickEvent(QMouseEvent* event)
{
    QGraphicsView::mouseDoubleClickEvent(event);
    const auto & clickPosInView = event->pos();
    const auto & clickPosInScene = mapToScene(clickPosInView);
    auto item = scene()->itemAt(clickPosInScene);

    if (auto qNode = dynamic_cast<QNode*>(item)) {
        auto str = JsonHelper::JS2Str(qNode->expData()->toJS(), false);
        SIG_ShowStrInInfoView(str.data());
    }
}

void tmap::MainGView::saveFakeMap(const std::string& mapName)
{
    vector<MapNodePtr> nodes{mNodesInFakeMap.begin(), mNodesInFakeMap.end()};
    for (auto& node : mNodesInFakeMap) {
        node->normalizePos();
    }
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
    /// 由于底层大量数据是复用的, 这里基本只有指针的复制
    StructedMapImpl map(jMap);
    const auto& nodes = map.getNodes();
    vector<QNodePtr> qNodes(nodes.size());

    for (int i = 0; i < nodes.size(); ++i) {
        qNodes[i] = QNode::makeOneFromMergedExp(nodes[i]->getRelatedMergedExp(), mMoveStragety);
    }

    /// links的内容需要被更新, 连接对象的指针需要被更改
    for (int i = 0; i < nodes.size(); ++i) {
        for (int j = 0; j < nodes[i]->nLinks(); ++j) {
            qNodes[i]->linkAt(j).at = nodes[i]->linkAt(j).at;
            const auto& toNode = nodes[i]->linkAt(j).to.lock();
            if (toNode) {
                qNodes[i]->linkAt(j).to = qNodes[toNode->getSerial()];
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
            /// BFS中的元素是已经添加过的QNode
            lookupQueue.push(qNode.get());
            qNode->setPos(0., 0.);
            mScene4FakeMap.addItem(qNode.get());
            mNodesInFakeMap.insert(std::move(qNode));

            while (!lookupQueue.empty()) {
                auto& currentQnode = lookupQueue.front();
                lookupQueue.pop();
                /// 查找所有的连接
                for (int currentGID = 0; currentGID < currentQnode->nLinks(); ++currentGID) {
                    auto linkedGID = currentQnode->linkedGIDAt(currentGID);
                    auto linkedQNode = currentQnode->qNodeAt(currentGID);
                    /// 是否与实际的QNode相连?
                    if (linkedQNode) {
                        /// 计算QNode应该放置的位置
                        auto currentLinkGatePos = currentQnode->mapToScene(UIT::TopoVec2QPt(
                                currentQnode->expData()->getGates()[currentGID]->getPos()));
                        auto anotherGatePosInNode = UIT::TopoVec2QPt(
                                linkedQNode->expData()->getGates()[linkedGID]->getPos());
                        /// 是否已经被遍历过? (有没有被添加进scene?)
                        if (linkedQNode->scene()) {
                            auto anotherGatePosInScene =
                                    linkedQNode->mapToScene(anotherGatePosInNode);
                            if (anotherGatePosInScene != currentLinkGatePos) {
                                /// 另外一个相连的Node已经被遍历过, 但是位置没有匹配, 建立FakeLine
                                auto fakeLine = make_shared<FakeLine_IMPL>(currentLinkGatePos,
                                                                           anotherGatePosInScene,
                                                                           currentQnode,
                                                                           currentGID);
                                mScene4FakeMap.addItem(fakeLine.get());
                                currentQnode->fakeLineAt(currentGID) = fakeLine;
                                linkedQNode->fakeLineAt(linkedGID) = std::move(fakeLine);
                            }
                        } else {
                            /// 没有被遍历过, 则加入scene以及collection
                            linkedQNode->setPos(currentLinkGatePos - anotherGatePosInNode);
                            mNodesInFakeMap.insert(linkedQNode);
                            mScene4FakeMap.addItem(linkedQNode.get());
                            lookupQueue.push(linkedQNode.get());
                        }
                    }
                }
            }
        }
    }

    for (auto& node : mNodesInFakeMap) {
        setQNodeMovabilityInFakeMap(node.get());
    }
}

void tmap::MainGView::SLOT_AcceptAddingGates2Corridor(bool acceptAdding)
{
    mAcceptAddingGate2Corridor = acceptAdding;
    if (acceptAdding) {
        setCursor(Qt::CrossCursor);
    } else {
        setCursor(Qt::ArrowCursor);
    }
}

void tmap::MainGView::keyPressEvent(QKeyEvent* event)
{
    QGraphicsView::keyReleaseEvent(event);
    switch (event->key()) {
        case Qt::Key_D:
            if (auto fakeLine = dynamic_cast<FakeLine_IMPL*>(scene()->selectedItems().first())){
                auto socket = fakeLine->oriNode();
                auto socketGID = fakeLine->fromGate();
                auto plug = socket->qNodeAt(socketGID).get();
                auto plugGID = socket->linkedGIDAt(socketGID);
                if (event->modifiers() & Qt::CTRL) {
                    std::swap(socket, plug);
                    std::swap(socketGID, plugGID);
                }
                auto socketPos = socket->gateQPos(socketGID);
                auto plugPos = plug->gateQPos(socket->linkedGIDAt(socketGID), false);
                plug->setPos(socketPos - plugPos);
                plug->notifyNeighbours2Move();
                socket->fakeLineAt(socketGID).reset();
                plug->fakeLineAt(plugGID).reset();
            }
        case Qt::Key_Delete:
            SLOT_RemoveSelectedNodes();
            break;
        case Qt::Key_M: {
            QNode* a = nullptr;
            QNode* b = nullptr;
            for(const auto& item : mScene4FakeMap.selectedItems()) {
                if (auto node = dynamic_cast<QNode*>(item)) {
                    if (a == nullptr) {
                        a = node;
                        continue;
                    } else {
                        b = node;
                        break;
                    }
                }
            }
            if (a && b) {
                auto res = a->expData()->detailedMatch(*b->expData());
                this->addNode2FakeMapFromExpData(res->mergedExpData);
            }
        }
            break;
        default:
            break;
    }
}

void tmap::MainGView::SLOT_StartDirectLinking(bool startLink)
{
    mIsDrawingDirectLink = startLink;
    if (startLink) {
        setCursor(Qt::CrossCursor);
    } else {
        setCursor(Qt::ArrowCursor);
    }
}

void tmap::MainGView::SLOT_SetMoveStrategy(int strategy)
{
    auto moveStragety = static_cast<MoveStragety>(strategy);
    for (auto& item : this->items()) {
        if (auto qnode = dynamic_cast<QNode*>(item)) {
            qnode->setMoveStragety(moveStragety);
        }
    }
}

void tmap::MainGView::switch2realMode(bool toReal)
{
    if (toReal) {
        setScene(&mScene4RealMap);
    } else {
        setScene(&mScene4FakeMap);
    }
}

void tmap::MainGView::switch2simMode(bool toSim)
{
    mAtSim = toSim;

    if (toSim) {
        for (auto& item : mScene4FakeMap.items()) {
            item->setFlag(QGraphicsItem::ItemIsMovable, false);
        }
        if (mRobot) {
            mRobot->setVisible(true);
            mRobot->updatePos();
        }
    } else {
        for (auto& item : mScene4FakeMap.items()) {
            if (auto qNode = dynamic_cast<QNode*>(item)) {
                setQNodeMovabilityInFakeMap(qNode);
            }
        }

        if (mRobot) {
            mRobot->setVisible(false);
        }
    }
}

bool tmap::MainGView::setRobotInFake(bool directMove)
{
    if (!mScene4FakeMap.selectedItems().empty()) {
        if (auto qNode = dynamic_cast<QNode*>(mScene4FakeMap.selectedItems().front())) {
            auto ptr = qNode->thisQnodePtr();
            mRobot.reset(new QRobot(ptr, directMove));
            return true;
        }
    }
    return false;
}

void tmap::MainGView::setQNodeMovabilityInFakeMap(tmap::QNode* node) const
{
    if (mEnableFakeNodesMoving) {
        if (node->expData()->type() != ExpDataType::Corridor) {
            node->setFlag(QGraphicsItem::ItemIsMovable);
        } else {
            if (mMoveStragety == MoveStragety::NO_NODE_FOLLOW) {
                node->setFlag(QGraphicsItem::ItemIsMovable);
            }
        }
    } else {
        node->setFlag(QGraphicsItem::ItemIsMovable, false);
    }
}

void tmap::MainGView::setQNodeMovabilityInRealMap(tmap::QNode* node) const
{
    node->setFlag(QGraphicsItem::ItemIsMovable, mEnableRealNodesMoving);
}

void tmap::MainGView::displayRealMap(const Jsobj& jMap)
{
    StructedMapImpl map(jMap);
    const auto& nodes = map.getNodes();
    vector<QNodePtr> qNodes(nodes.size());

    /// 复制一份QNode
    for (int i = 0; i < nodes.size(); ++i) {
        qNodes[i] = QNode::makeOneFromMergedExp(nodes[i]->getRelatedMergedExp(),
                MoveStragety::ONLY_FIXED_TYPE);
    }

    /// links的内容需要被更新, 连接对象的指针需要被更改
    for (int i = 0; i < nodes.size(); ++i) {
        for (int j = 0; j < nodes[i]->nLinks(); ++j) {
            qNodes[i]->linkAt(j).at = nodes[i]->linkAt(j).at;
            const auto& toNode = nodes[i]->linkAt(j).to.lock();
            if (toNode) {
                qNodes[i]->linkAt(j).to = qNodes[toNode->getSerial()];
            }
        }
    }

    /// 开始BFS构造地图
    mNodesInRealMap.clear();
    bool once = false;
    for (auto& qNode : qNodes) {
        /// 为了处理拓扑不相连的情况, 对qNodes中所有成员进行BFS, 理论上其实下面这个if只发生一次
        if (mNodesInRealMap.find(qNode) == mNodesInRealMap.end()) {
            if (once) {
                /// 发生了两次, 说明存在不相连的情况
                cout << FILE_AND_LINE << " this map doesnt fully connected!" << endl;
            } else {
                once = true;
            }
            /// BFS使用的队列
            queue<QNode*> lookupQueue;
            /// BFS中的元素是已经添加过的QNode
            lookupQueue.push(qNode.get());
            qNode->setPos(0., 0.);
            mScene4RealMap.addItem(qNode.get());
            mNodesInRealMap.insert(std::move(qNode));

            while (!lookupQueue.empty()) {
                auto& currentQnode = lookupQueue.front();
                lookupQueue.pop();
                /// 查找所有的连接
                for (int currentGID = 0; currentGID < currentQnode->nLinks(); ++currentGID) {
                    auto linkedGID = currentQnode->linkedGIDAt(currentGID);
                    auto linkedQNode = currentQnode->qNodeAt(currentGID);
                    /// 是否与实际的QNode相连?
                    if (linkedQNode) {
                        /// 计算QNode应该放置的位置
                        auto currentLinkGatePos = currentQnode->mapToScene(UIT::TopoVec2QPt(
                                currentQnode->expData()->getGates()[currentGID]->getPos()));
                        auto anotherGatePosInNode = UIT::TopoVec2QPt(
                                linkedQNode->expData()->getGates()[linkedGID]->getPos());
                        /// 是否已经被遍历过? (有没有被添加进scene?)
                        if (linkedQNode->scene()) {
                            auto anotherGatePosInScene =
                                    linkedQNode->mapToScene(anotherGatePosInNode);
                            if (anotherGatePosInScene != currentLinkGatePos) {
                                /// 另外一个相连的Node已经被遍历过, 但是位置没有匹配, 建立FakeLine
                                auto fakeLine = make_shared<FakeLine_IMPL>(currentLinkGatePos,
                                                                           anotherGatePosInScene,
                                                                           currentQnode,
                                                                           currentGID);
                                mScene4RealMap.addItem(fakeLine.get());
                                currentQnode->fakeLineAt(currentGID) = fakeLine;
                                linkedQNode->fakeLineAt(linkedGID) = std::move(fakeLine);
                            }
                        } else {
                            /// 没有被遍历过, 则加入scene以及collection
                            linkedQNode->setPos(currentLinkGatePos - anotherGatePosInNode);
                            mNodesInRealMap.insert(linkedQNode);
                            mScene4RealMap.addItem(linkedQNode.get());
                            lookupQueue.push(linkedQNode.get());
                        }
                    }
                }
            }
        }
    }

    for (auto& node : mNodesInRealMap) {
        setQNodeMovabilityInRealMap(node.get());
    }
}

