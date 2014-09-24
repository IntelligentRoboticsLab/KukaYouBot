/********************************************************************
 *
 * Author:
 *      Sébastien Negrijn
 *
 * Contact:
 *      sebastien.negrijn@student.uva.nl
 *
 * Description:
 *      Demo application for the Kuka youBot using the ros interface.
 *
 * How to:
 *      -Place the "hello_world" folder in "catkin_ws/src"
 *      -Execute the "catkin_make" command from the catkin_ws folder
 *      -Run with "rosrun hello_world hello_world_node"
 *
 ********************************************************************/


#include <ros/ros.h>
#include <brics_actuator/JointPositions.h>
#include <geometry_msgs/Twist.h>

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>


// Some positions for the youbot arm (IN RADIAN)
//double POSITION_HOME[]  = { 2.8186,     0.9632,     -1.9736,    3.2648,     2.9539 };
//double POSITION_GRASP[] = { 3.0417,     2.04427,    -1.5189,    2.5434,     2.8761 };

double POSITION_IK[]    = { 3.0417,   1.470528515855737,   -2.08832111972146,   0.24996022140276616,     2.8761 };

// Class that represents the youbot
class YouBot {

    // Create a handle to talk to the ROS server
    ros::NodeHandle n_;

    // Create three publishers that will send the messages to the driver of the youbot
    ros::Publisher publisher_base_;
    ros::Publisher publisher_arm_;
	ros::Publisher publisher_gripper_;

    // Arm
    // Arm message
    brics_actuator::JointPositions armJointActuator;
    // Arm position variables
    std::vector <brics_actuator::JointValue> armJointPositions;

    // Base
    // Base message
    geometry_msgs::Twist baseVel;

	// Gripper
	// Gripper message
	brics_actuator::JointPositions gripperActuator;
	// Gripper positions
    std::vector <brics_actuator::JointValue> gripperPositions;


public:
    // Initialise the youbot
    YouBot() {

        ROS_INFO( "Initiating YouBot" );

        // Tell the ROS server what kind of messages we will be sending and on what topics
        publisher_arm_ = n_.advertise<brics_actuator::JointPositions>( "/arm_1/arm_controller/position_command", 1 );
        publisher_base_  = n_.advertise<geometry_msgs::Twist>( "/cmd_vel",1 );

		publisher_gripper_ = n_.advertise<brics_actuator::JointPositions>( "/arm_1/gripper_controller/position_command", 1 );

        // Wait for a second so the publishers can be initialised
        sleep( 1 );

    }

    // Be sure to return to the home position on shutdown
    // and to set the speed of the base to 0
    ~YouBot()
    {

        //setArmPosition( POSITION_HOME );
        setBaseSpeed( 0, 0, 0 );

    }

    // Prepare all variables in the arm message
    void initArm() {

        // set the number of joints in the arm
        armJointPositions.resize( 5 );

        std::stringstream jointName;
        for(int i = 0; i < 5; i++)
        {

            jointName.str("");
            jointName << "arm_joint_" << (i + 1);

            // set joint name
            armJointPositions[i].joint_uri = jointName.str();

            // set begin value to 0
            armJointPositions[i].value = 0;

            // set unit of joint to radians
            armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);

        }
    }

    // Prepare the base message
    void initBase() {

        // Set all begin speeds to 0
        baseVel.linear.x = baseVel.linear.y = baseVel.angular.z = 0;

    }

	// Initialise the grippermessage for th youBot
	void initGripper() {

		gripperPositions.resize(2);
		gripperPositions[0].joint_uri = "gripper_finger_joint_l";
        gripperPositions[0].value = 0;
        gripperPositions[0].unit = boost::units::to_string(boost::units::si::meter);

        gripperPositions[1].joint_uri = "gripper_finger_joint_r";
        gripperPositions[1].value = 0;
        gripperPositions[1].unit = boost::units::to_string(boost::units::si::meter);

        gripperActuator.positions = gripperPositions;


	}

    // Move the arm of the youbot to a new position
    void setArmPosition( double positions[] ) {

        ROS_INFO( "Moving arm to new position" );

        // Set all positions in the ros message
        armJointPositions[0].value = positions[0];
        armJointPositions[1].value = positions[1];
        armJointPositions[2].value = positions[2];
        armJointPositions[3].value = positions[3];
        armJointPositions[4].value = positions[4];

        armJointActuator.positions = armJointPositions;

        // Send the message to the youbot driver
        publisher_arm_.publish( armJointActuator );
		ros::spinOnce();

    }

    // Set a new speed for the youbot base
    void setBaseSpeed( double x, double y, double rot ) {

        ROS_INFO( "Set new speed for base" );

        // Set the new speed
        baseVel.linear.x  = x;
        baseVel.linear.y  = y;
        baseVel.angular.z = rot;

        // Send the message to the youbot base
        publisher_base_.publish( baseVel );
		ros::spinOnce();

    }

	// Set a new position for the youBot gripper
	void setGripperPosition( double joint1, double joint2 ) {

		gripperPositions[0].value = joint1;
		gripperPositions[1].value = joint2;

		gripperActuator.positions = gripperPositions;

		publisher_gripper_.publish( gripperActuator );

		
	}
};

// Main funtion, called on startup
int main(int argc, char** argv)
{

    // Initialise the new ros node with name "hello world" (will also show up in the rqt_graph)
	ros::init( argc, argv, "hello_world" );

    ROS_INFO( "Initiating the youbot");

    // Create a new youbot instance
    YouBot theYouBot;

    // Initialise the youbot arm and base
    theYouBot.initArm();
    //theYouBot.initBase();
	theYouBot.initGripper();

    // Move the youbot arm
    theYouBot.setArmPosition( POSITION_IK );
    sleep( 3 );

    /*
    // Turn the youbot for a bit
    theYouBot.setBaseSpeed( 0, 0, 0.02 );
    sleep( 3 );

    // Stop the youbot from turning
    theYouBot.setBaseSpeed( 0, 0, 0 );

	theYouBot.setGripperPosition( 0, 0 );
    */
    ROS_INFO( "Shutdown" );

    return 1;

}
