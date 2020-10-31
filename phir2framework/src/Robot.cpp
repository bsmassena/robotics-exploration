#include "Robot.h"

#include <unistd.h>
#include <GL/glut.h>
#include <cmath>
#include <iostream>


//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

Robot::Robot()
{
    ready_ = false;
    running_ = true;

    grid = new Grid();

    plan = new Planning();
    plan->setGrid(grid);
    plan->setMaxUpdateRange(base.getMaxLaserRange());

    // variables used for navigation
    isFollowingLeftWall_=false;

    // variables used for visualization
    viewMode=0;
    numViewModes=5;
    motionMode_=MANUAL_SIMPLE;

}

Robot::~Robot()
{
    base.closeARIAConnection();
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void Robot::initialize(ConnectionMode cmode, LogMode lmode, std::string fname)
{
    logMode_ = lmode;
//    logFile_ = new LogFile(logMode_,fname);
    ready_ = true;

    // initialize ARIA
    if(logMode_!=PLAYBACK){
        bool success = base.initialize(cmode,lmode,fname);
        if(!success){
            printf("Could not connect to robot... exiting\n");
            exit(0);
        }
    }

    ready_ = true;
    controlTimer.startLap();
}

void Robot::run()
{
    controlTimer.waitTime(0.2);

    if(logMode_==PLAYBACK){
        bool hasEnded = base.readFromLog();
        if(hasEnded){
            std::cout << "PROCESS COMPLETE. CLOSING PROGRAM." << std::endl;
            exit(0);
        }
    }else{
        bool success = base.readOdometryAndSensors();
        if(!success){
            usleep(50000);
            return;
        }

        if(logMode_==RECORDING)
            base.writeOnLog();
    }

    currentPose_ = base.getOdometry();

    pthread_mutex_lock(grid->mutex);

    // Mapping
    mappingWithHIMMUsingLaser();
    mappingWithLogOddsUsingLaser();
    mappingUsingSonar();

    pthread_mutex_unlock(grid->mutex);

    plan->setNewRobotPose(currentPose_);

    // Save path traversed by the robot
    if(base.isMoving() || logMode_==PLAYBACK){
        path_.push_back(base.getOdometry());
    }

    // Navigation
    switch(motionMode_){
        case WANDER:
            wanderAvoidingCollisions();
            break;
        case WALLFOLLOW:
            wallFollow();
            break;
        case POTFIELD_0:
            followPotentialField(0);
            break;
        case POTFIELD_1:
            followPotentialField(1);
            break;
        case POTFIELD_2:
            followPotentialField(2);
            break;
        case ENDING:
            running_=false;
            break;
        default:
            break;
    }

    base.resumeMovement();

    usleep(50000);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void Robot::move(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            std::cout << "moving front" << std::endl;
            break;
        case BACK:
            std::cout << "moving back" << std::endl;
            break;
        case LEFT:
            std::cout << "turning left" << std::endl;
            break;
        case RIGHT:
            std::cout << "turning right" << std::endl;
            break;
        case STOP:
            std::cout << "stopping robot" << std::endl;
    }

    if(motionMode_==MANUAL_SIMPLE)
        base.setMovementSimple(dir);
    else if(motionMode_==MANUAL_VEL)
        base.setMovementVel(dir);
    else if(motionMode_==WALLFOLLOW)
        if(dir==LEFT)
            isFollowingLeftWall_=true;
        else if(dir==RIGHT)
            isFollowingLeftWall_=false;



}

void Robot::wanderAvoidingCollisions()
{
    float linVel=0;
    float angVel=0;

    //TODO - implement obstacle avoidance




    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::wallFollow()
{
    float linVel=0;
    float angVel=0;

    if(isFollowingLeftWall_)
        std::cout << "Following LEFT wall" << std::endl;
    else
        std::cout << "Following RIGHT wall" << std::endl;

    //TODO - implementar wall following usando PID



    base.setWheelsVelocity_fromLinAngVelocity(linVel, angVel);
}

void Robot::followPotentialField(int t)
{
    int scale = grid->getMapScale();
    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    // how to access the grid cell associated to the robot position
    Cell* c=grid->getCell(robotX,robotY);

    float linVel, angVel;

    // TODO: define the robot velocities using a control strategy
    //       based on the direction of the gradient of c given by c->dirX[t] and c->dirY[t]




    base.setWheelsVelocity_fromLinAngVelocity(linVel,angVel);
}


///////////////////////////
///// MAPPING METHODS /////
///////////////////////////

float Robot::getOccupancyFromLogOdds(float logodds)
{
    return 1.0 - 1.0/(1.0+exp(logodds));
}

float Robot::getLogOddsFromOccupancy(float occupancy)
{
    return log(occupancy/(1.0-occupancy));
}

double Robot::inverseSensorModel(int xCell, int yCell, int xRobot, int yRobot, float robotAngle) {
    float lambda_r = 0.1;   //  10 cm
    float lambda_phi = 1.0; // 1 degree
    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange * scale;
    float r = sqrt(pow(xCell - xRobot, 2) + pow(yCell - yRobot, 2)) / scale;
    float phi = normalizeAngleDEG(RAD2DEG(atan2(yCell - yRobot, xCell - xRobot)) - robotAngle);
    int k = base.getNearestLaserBeam(phi);

    if ((fabs(phi - base.getAngleOfLaserBeam(k)) > lambda_phi / 2) ||
        (r > std::min(maxRange, base.getKthLaserReading(k)))) {
        return 0.5;
    }

    if ((base.getKthLaserReading(k) < maxRange) &&
        (fabs(r - base.getKthLaserReading(k)) < lambda_r / 2)) {
        return 0.9;
    }

    if (r <= base.getKthLaserReading(k)) {
        return 0.1;
    }
}

void Robot::mappingWithLogOddsUsingLaser()
{
    float lambda_r = 0.1;   //  10 cm
    float lambda_phi = 1.0; // 1 degree

    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange * scale;

    int robotX = currentPose_.x * scale;
    int robotY = currentPose_.y * scale;
    float robotAngle = currentPose_.theta;

    // how to access a grid cell:
    //    Cell* c=grid->getCell(robotX,robotY);

    // access log-odds value of variable in c->logodds
    // how to convert logodds to occupancy values:
    //    c->occupancy = getOccupancyFromLogOdds(c->logodds);
    float locc, lfree;

    for (int cellX = robotX - maxRangeInt; cellX <= robotX + maxRangeInt; cellX++) {
        for (int cellY = robotY - maxRangeInt; cellY <= robotY + maxRangeInt; cellY++) {
            Cell *cell = grid->getCell(cellX, cellY);
            double r = sqrt(pow(cell->x - robotX, 2) + pow(cell->y - robotY, 2));
            if (r < maxRangeInt) {
                float occupancyUpdate = inverseSensorModel(cell->x, cell->y, robotX, robotY, robotAngle);
                cell->logodds += getLogOddsFromOccupancy(occupancyUpdate);
                cell->occupancy = getOccupancyFromLogOdds(cell->logodds);
            }
        }
    }
}

void Robot::mappingUsingSonar()
{
    float lambda_r = 0.5; //  50 cm
    float lambda_phi = 30.0;  // 30 degrees

    // TODO: update cells in the sensors' field-of-view
    // Follow the example in mappingWithLogOddsUsingLaser()

    int scale = grid->getMapScale();
    float maxRange = base.getMaxSonarRange();
    int maxRangeInt = maxRange * scale;

    int robotX = currentPose_.x * scale;
    int robotY = currentPose_.y * scale;
    float robotAngle = currentPose_.theta;

    for (int cellX = robotX - maxRangeInt; cellX <= robotX + maxRangeInt; cellX++) {
        for (int cellY = robotY - maxRangeInt; cellY <= robotY + maxRangeInt; cellY++) {
            Cell *cell = grid->getCell(cellX, cellY);
            float r = sqrt(pow(cellX - robotX, 2) + pow(cellY - robotY, 2)) / scale;
            float phi = normalizeAngleDEG(RAD2DEG(atan2(cellY - robotY, cellX - robotX)) - robotAngle);
            int k = base.getNearestSonarBeam(phi);
            float occUpdate;
            float R = maxRange;
            float alpha = fabs(phi - base.getAngleOfSonarBeam(k));
            float beta = lambda_phi / 2;
            float occUpdateMainTerm = (((R-r)/R) + ((beta-alpha)/beta))/2;

            // If sonar not in direction
            if (fabs(phi - base.getAngleOfSonarBeam(k)) > lambda_phi / 2) {
                continue;
            }

            // if in region 1
            if ((base.getKthSonarReading(k) < maxRange) &&
                (fabs(r - base.getKthSonarReading(k)) < lambda_r / 2)) {
                occUpdate = 0.5 * occUpdateMainTerm + 0.5;
            } else if (r <= base.getKthSonarReading(k)) { // if in region 2
                occUpdate = 0.5 * (1 - occUpdateMainTerm);
            }
            else {
                continue;
            }

            cell->occupancySonar = (occUpdate * cell->occupancySonar) /
                                    ((occUpdate * cell->occupancySonar) + ((1.0 - occUpdate) * (1.0 - cell->occupancySonar)));

            if(cell->occupancySonar > 0.99) cell->occupancySonar = 0.99;
            if(cell->occupancySonar < 0.01) cell->occupancySonar = 0.01;
        }
    }
}

void Robot::mappingWithHIMMUsingLaser()
{
    float lambda_r = 0.2; //  20 cm
    float lambda_phi = 1.0;  // 1 degree

    int scale = grid->getMapScale();
    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    float robotAngle = currentPose_.theta;

    for(int cellX = robotX - maxRangeInt; cellX <= robotX + maxRangeInt; cellX++) {
        for(int cellY = robotY - maxRangeInt; cellY <= robotY + maxRangeInt; cellY++) {
            Cell* cell = grid->getCell(cellX, cellY);

            float r = sqrt(pow(cellX - robotX, 2) + pow(cellY - robotY, 2)) / scale;
            float phi = normalizeAngleDEG(RAD2DEG(atan2(cellY - robotY, cellX - robotX)) - robotAngle);
            int k = base.getNearestLaserBeam(phi);

            if((fabs(phi - base.getAngleOfLaserBeam(k)) > lambda_phi / 2) ||
            (r > std::min(maxRange, base.getKthLaserReading(k)))) {
                continue;
            }

            if((base.getKthLaserReading(k) < maxRange) &&
                (fabs(r - base.getKthLaserReading(k)) < lambda_r / 2)) {
                cell->himm += 3;
                cell->himm = std::min(cell->himm, 15);
                continue;
            }

            if(r <= base.getKthLaserReading(k)) {
                cell->himm -= 1;
                cell->himm = std::max(cell->himm, 0);
                continue;
            }
        }
    }
}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void Robot::writeOnLog()
{
    logFile_->writePose("Odometry",currentPose_);
    logFile_->writeSensors("Sonar",base.getSonarReadings());
    logFile_->writeSensors("Laser",base.getLaserReadings());
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool Robot::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    base.setOdometry(logFile_->readPose("Odometry"));
    base.setSonarReadings(logFile_->readSensors("Sonar"));
    base.setLaserReadings(logFile_->readSensors("Laser"));

    return false;
}

////////////////////////
///// DRAW METHODS /////
////////////////////////

void Robot::draw(float xRobot, float yRobot, float angRobot)
{
    float scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);
    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/scale,1.0/scale,1.0/scale);

    // sonars and lasers draw in cm
    if(viewMode==1)
        base.drawSonars(true);
    else if(viewMode==2)
        base.drawSonars(false);
    else if(viewMode==3)
        base.drawLasers(true);
    else if(viewMode==4)
        base.drawLasers(false);

    // robot draw in cm
    base.drawBase();

    glScalef(scale,scale,scale);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);
}

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::isReady()
{
    return ready_;
}

bool Robot::isRunning()
{
    return running_;
}

const Pose& Robot::getCurrentPose()
{
    return currentPose_;
}

void Robot::drawPath()
{
    float scale = grid->getMapScale();

    if(path_.size() > 1){
        glScalef(scale,scale,scale);
        glLineWidth(3);
        glBegin( GL_LINE_STRIP );
        {
            for(unsigned int i=0;i<path_.size()-1; i++){
                glColor3f(1.0,0.0,1.0);

                glVertex2f(path_[i].x, path_[i].y);
                glVertex2f(path_[i+1].x, path_[i+1].y);
            }
        }
        glEnd();
        glLineWidth(1);
        glScalef(1.0/scale,1.0/scale,1.0/scale);

    }
}

void Robot::waitTime(float t){
    float l;
    do{
        usleep(1000);
        l = controlTimer.getLapTime();
    }while(l < t);
    controlTimer.startLap();
}
