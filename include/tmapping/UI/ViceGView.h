//
// Created by stumbo on 2019/12/20.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_UI_VICEGVIEW_H
#define TMAPPING_INCLUDE_TMAPPING_UI_VICEGVIEW_H

#include <QGraphicsView>
#include <QWidget>

namespace tmap
{

class ViceGView : public QGraphicsView
{
    Q_OBJECT

public:
    explicit ViceGView(QWidget *parent = nullptr) : QGraphicsView(parent) {}

};
}


#endif //TMAPPING_INCLUDE_TMAPPING_UI_VICEGVIEW_H
