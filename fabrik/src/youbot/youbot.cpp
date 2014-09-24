#include <ros/ros.h>
#include <brics_actuator/JointPositions.h>
#include <geometry_msgs/Twist.h>

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

// Class that represents the youbot
class YouBot {

    // Create a handle to talk to the ROS server
    ros::NodeHandle n_;

    // Create publisher that will send the messages to the driver of the youbot
    ros::Publisher publisher_arm_;

    // Arm
    // Arm message
    brics_actuator::JointPositions armJointActuator;
    // Arm position variables
    std::vector <brics_actuator::JointValue> armJointPositions;


public:
    // Initialise the youbot
    YouBot() {

        ROS_INFO( "Initiating YouBot" );

        // Tell the ROS server what kind of messages we will be sending and on what topics
        publisher_arm_ = n_.advertise<brics_actuator::JointPositions>( "/arm_1/arm_controller/position_command", 1 );

        // Wait for a second so the publisher can be initialised
        sleep( 1 );

    }

    // Be sure to return to the home position on shutdown
    ~YouBot()
    {

        //setArmPosition( POSITION_HOME );

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
};
