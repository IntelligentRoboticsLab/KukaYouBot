

#include "youbot/youbot.cpp"
#include "fabrik.cpp"

// Some positions for the youbot arm (IN RADIAN)
//double POSITION_HOME[]  = { 2.8186,     0.9632,     -1.9736,    3.2648,     2.9539 };
//double POSITION_GRASP[] = { 3.0417,     2.04427,    -1.5189,    2.5434,     2.8761 };

double POSITION_IK[]    = { 3.0417,   1.470528515855737,   -2.08832111972146,   0.24996022140276616,     2.8761 };



int main(int argc, char** argv) {

    ros::init( argc, argv, "fabrik" );

    YouBot theYouBot;
    theYouBot.initArm();

    Fabrik theFabrikEngine;

    return 1;

}
