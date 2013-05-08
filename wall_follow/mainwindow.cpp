#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    this->connectActions();
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::startWallFollow()
{
    if(ui->PBStart->isChecked())
    {
        int argc = 0;
        char *argv;
        mRobot = new Robot(&argc,&argv);
        mRobot->startWallFollowing();
    }
    else
    {
        mRobot->stopWallFollowing();
        delete mRobot;
    }
}

void MainWindow::connectActions()
{
    this->connect(ui->PBStart,SIGNAL(clicked()),this,SLOT(startWallFollow()));
}
