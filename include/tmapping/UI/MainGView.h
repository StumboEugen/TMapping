//
// Created by stumbo on 2019/12/20.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H
#define TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H

#include <QGraphicsView>
#include <QWidget>

namespace tmap
{

class MainGView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit MainGView(QWidget *parent = nullptr) : QGraphicsView(parent) {}

};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_MAINGVIEW_H
