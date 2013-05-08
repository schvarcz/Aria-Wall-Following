#include "robot.h"

Robot::Robot(int *argc, char **argv, const char * name, bool ignored,
             bool doSigHandle,
             bool normalInit, bool addAriaExitCallback):
    parser(argc,argv),
    robotConnector(&parser,this),
    laserConnector(&parser,this,&robotConnector),
    ArRobot(name, ignored, doSigHandle, normalInit, addAriaExitCallback),
    QObject()
{

    currentBehavior = RobotBehavior::WANDER;
    thread = new QThread();
    this->moveToThread(thread);

    connect(thread,SIGNAL(started()),this,SLOT(doWallFollowing()));
    connect(thread,SIGNAL(finished()),this,SLOT(stopWallFollowing()));
}

bool Robot::start()
{
    Aria::init();
    //parser.addDefaultArgument("-rh 192.168.1.11 -remoteLaserTcpPort 10002");
    this->addRangeDevice(&sick);
    robotConnector.parseArgs();
    if(!robotConnector.connectRobot())
    {
        ArLog::log(ArLog::Terse,"Ops... falha ao conectar ao servidor do robo.");
        return false;
    }

    laserConnector.setupLaser(&sick);
    if(!laserConnector.connectLaser(&sick))
    {
        ArLog::log(ArLog::Terse,"Ops... falha ao conectar os lasers do robo.");
        return false;
    }

    ArLog::log(ArLog::Normal,"Robot connected");
    sick.runAsync();
    this->runAsync(true);
    ArUtil::sleep(500);
    this->lock();
    this->enableMotors();
    this->unlock();
    ArUtil::sleep(500);
    return true;
}

bool Robot::stop()
{
    ArRobot::stop();
    this->disableMotors();
}

bool Robot::shutdown()
{
    this->stop();
    this->stopRunning();
    this->waitForRunExit();
    Aria::shutdown();

    return true;
}

Robot::~Robot()
{
    this->stopWallFollowing();
    thread->exit();
    this->shutdown();
}

void Robot::readingSensors()
{
    if(this->isConnected() && sick.isConnected())
    {
            sick.lockDevice();
            lasers = sick.getRawReadingsAsVector();
            sick.unlockDevice();
    }
}

int Robot::getLaserRange(int angle)
{
    if( !this->lasers || (this->lasers->size() <= angle))
        return 0;

    return this->lasers->at(angle).getRange();
}

int Robot::getSonarRange(int id_sonar)
{
    if(id_sonar > 8)
        return 0;
    return ArRobot::getSonarRange(id_sonar);
}

double Robot::getNorth()
{
    return ArRobot::getTh();
}

void Robot::move(int distanceMM)
{
    ArLog::log(ArLog::Normal,"Movendo: %d",distanceMM);
    this->lock();
    ArRobot::move(distanceMM);
    this->unlock();
}

void Robot::rotate(int degrees)
{
    ArLog::log(ArLog::Normal,"Rotacionando: %d",degrees);
    this->lock();
    ArRobot::setDeltaHeading(degrees);
    this->unlock();
}


void Robot::startWallFollowing()
{
    run = true;
    this->moveToThread(thread);
    thread->start();
}

bool Robot::isWallFollowingRunnig()
{
    return thread->isRunning();
}

void Robot::doWallFollowing()
{
    if(!this->start())
    {
        return;
    }

    double waitTime = 0.5, v = 300;
    int iteration = 0;

    PIDCorrection *left = new PIDCorrection(&sick,30,60,1000);
    PIDCorrection *right = new PIDCorrection(&sick,-30,-60,1000);
    PIDCorrection *front = new PIDCorrection(&sick,-30,30,1000);

    srand(time(NULL));
    while((this->isConnected()) && (sick.isConnected()) && (run))
    {
        this->readingSensors();

        if(currentBehavior == RobotBehavior::WANDER)
        {
            if(left->getDistance() < left->desiredValue*1.5)
            {
                currentBehavior = RobotBehavior::WALL_LEFT;
            }
            else if(right->getDistance() < right->desiredValue*1.5)
            {
                if(currentBehavior == RobotBehavior::WANDER || (right->getDistance() < left->getDistance()))
                {
                    currentBehavior = RobotBehavior::WALL_RIGHT;
                }
            }

            //Se ainda continuar vagando... vague!
            if(currentBehavior == RobotBehavior::WANDER)
            {
                if(iteration % 20 == 0)
                {
                    this->rotate((rand()%90) - 45);
                }
                this->move(500);
            }
        }

        if(currentBehavior == RobotBehavior::WALL_LEFT)
        {
            int cf = front->getCorrection(waitTime),
                cl = left->getCorrection(waitTime);
            /*if(fabs(cf) < fabs(cl))
                this->walkWithPIDCorrection(v,cf);
            else
            */
                this->walkWithPIDCorrection(v,cl);
        }
        else if(currentBehavior == RobotBehavior::WALL_RIGHT)
        {
            int cf = front->getCorrection(waitTime),
                cr = right->getCorrection(waitTime);
            /*if(fabs(cf) < fabs(cr))
                this->walkWithPIDCorrection(v,cf);
            else*/
                this->walkWithPIDCorrection(v,cr);
        }
        iteration++;
        iteration %= (int)(60/waitTime);
        ArUtil::sleep(1000*waitTime);
    }
    this->stop();
}

void Robot::walkWithPIDCorrection(double velocity, double correction)
{
    double e1 = 0, e2 = 0;
    double v1 = velocity, v2 = velocity;

    if(correction > 0)
    {
        e1 = correction;
    }
    else
    {
        e2 = -correction;
    }

    if(correction > v1)
    {
        e1 = v1;
    }
    else if(correction < -v1)
    {
        e2 = v2;
    }
    this->setVel2(v1-e1,v2-e2);

    ArLog::log(ArLog::Terse,"Menor distancia: %f",sick.currentReadingPolar(30,60));
    ArLog::log(ArLog::Terse,"Coeficiente: %f",correction);
}

void Robot::stopWallFollowing()
{
    run = false;
}
