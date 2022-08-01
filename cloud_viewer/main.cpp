#include "cloud_viewer.h"
#include <QtWidgets/QApplication>

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    cloud_viewer w;
    w.show();
    return a.exec();
}