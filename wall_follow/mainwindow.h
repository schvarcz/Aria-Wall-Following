#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <robot.h>
#include <iostream>
#include <QThread>

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
private slots:
    void startWallFollow();
private:
    void connectActions();
    Ui::MainWindow *ui;
    Robot *mRobot;
};

#endif // MAINWINDOW_H
