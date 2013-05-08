#ifndef ROBOT_H
#define ROBOT_H

#include <Aria.h>
#include <vector>
#include <QObject>
#include <QThread>
#include <pidcorrection.h>

using namespace std;

namespace RobotBehavior
{
    enum Behavior
    {
        WANDER,
        WALL_LEFT,
        WALL_RIGHT
    };
}

class Robot : public QObject, public ArRobot
{
    Q_OBJECT

public:
    explicit Robot(int *argc = 0, char **argv = 0, const char * name = NULL, bool ignored = true,
          bool doSigHandle=true,
          bool normalInit = true, bool addAriaExitCallback = true);
    ~Robot();
    void move(int distanceMM);
    void rotate(int degrees);
    int getLaserRange(int angle);
    int getSonarRange(int id_sonar);
    double getNorth();
    void readingSensors();
    bool shutdown();
    bool start();
    bool stop();

    void startWallFollowing();
    bool isWallFollowingRunnig();
    void stopWallFollowing();

private:
    void walkWithPIDCorrection(double velocity,double correction);
    ArSick sick;
    ArRobotConnector robotConnector;
    ArLaserConnector laserConnector;
    ArArgumentParser parser;
    vector<ArSensorReading> *lasers = NULL;
    QThread *thread;
    bool run = true;
    RobotBehavior::Behavior currentBehavior;


signals:

public slots:
    void doWallFollowing();

};

#endif // ROBOT_H
