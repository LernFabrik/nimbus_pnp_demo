#include "accept_pose.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QWidget widget;
    QDialog dialog;
    Ui::Dialog ui;
    ui.setupUi(&dialog);

    dialog.show();
    return app.exec();
}