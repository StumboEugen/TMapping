//
// Created by stumbo on 2019/12/20.
//

#include "tmapping/UI/ui.h"
#include <QApplication>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    tmap::TmapUI theUi;
    theUi.show();
    app.exec();
}