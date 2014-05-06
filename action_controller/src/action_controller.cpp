#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <brics_actuator/JointPositions.h>
#include <control_msgs/JointTrajectoryControllerState.h>


#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>
//#include <boost/units/systems/angle/degrees.hpp>
//#include <boost/units/conversion.hpp>

class YouBotArm
{

    brics_actuator::JointPositions jointPositionsActuator;
    std::vector <brics_actuator::JointValue> armJointPositions;

    int numberOfArmJoints;

public:

    YouBotArm()
    {
        numberOfArmJoints = 5;
        armJointPositions.resize( numberOfArmJoints );

        this->init();

    }

    void parseROSMsg( const control_msgs::JointTrajectoryControllerState &msg )
    {

        double pos;
        for( int i = 0; i < msg.actual.positions.size(); i++ )
        {
            pos = msg.actual.positions.at( i );
            armJointPositions[i].value = pos;

            std::cout << pos << " ";
        }

        std::cout << std::endl;

        jointPositionsActuator.positions = armJointPositions;

    }

    const brics_actuator::JointPositions toROSMsg()
    {

        return jointPositionsActuator;

    }

    double getJointPosition( int joint )
    {

        return armJointPositions[joint].value;

    }

    void setJointPosition( int joint, double jointPosition )
    {

        armJointPositions[joint].value = jointPosition;

        jointPositionsActuator.positions = armJointPositions;

    }

    void setJointPositions( double joint0, double joint1, double joint2, double joint3, double joint4 )
    {


        armJointPositions[0].value = joint0;
        armJointPositions[1].value = joint1;
        armJointPositions[2].value = joint2;
        armJointPositions[3].value = joint3;
        armJointPositions[4].value = joint4;

        jointPositionsActuator.positions = armJointPositions;

    }

    void init()
    {

        std::stringstream jointName;
        for(int i = 0; i < numberOfArmJoints; i++)
        {

            jointName.str("");
            jointName << "arm_joint_" << (i + 1);

            armJointPositions[i].joint_uri = jointName.str();

            armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);

        }
    }

};

class YouBotGripper
{

    void parseROSMsg( const brics_actuator::JointPositions &msg )
    {

    }

    const brics_actuator::JointPositions toROSMsg()
    {
        brics_actuator::JointPositions positions;

        return positions;
    }

};

class ActionController
{

    ros::NodeHandle n_;
    ros::Subscriber arm_subscriber_;
    ros::Publisher arm_publisher_;

    YouBotArm currentYouBotArm;
    YouBotGripper currentYouBotGripper;

public:

    ActionController()
    {

        arm_subscriber_ = n_.subscribe( "/arm_1/arm_controller/state", 1,
                                        &ActionController::updateCurrentArm, this);

        arm_publisher_  = n_.advertise<brics_actuator::JointPositions>( "/arm_1/arm_controller/position_command", 1 );

        test();
    }

    void test()
    {

        ros::spinOnce();

        for( int i = 0; i < 10; i ++ )
        {
            ros::spinOnce();

            double joint1 = currentYouBotArm.getJointPosition( 0 )+0.1;
            double joint2 = currentYouBotArm.getJointPosition( 1 );
            double joint3 = currentYouBotArm.getJointPosition( 2 );
            double joint4 = currentYouBotArm.getJointPosition( 3 );
            double joint5 = currentYouBotArm.getJointPosition( 4 );

            YouBotArm newArm;

            newArm.setJointPositions( joint1, joint2, joint3, joint4, joint5 );

            std::cout << joint1 << std::endl;
            std::cout << newArm.getJointPosition( 0 ) << std::endl;

            arm_publisher_.publish( newArm.toROSMsg() );

            sleep( 1 );

        }


    }

    void updateCurrentArm( const control_msgs::JointTrajectoryControllerState &msg )
    {
        currentYouBotArm.parseROSMsg( msg );
    }

};

int main( int argc, char** argv )
{
    ros::init( argc, argv, "action_controller" );

    ActionController theActionController;

    ros::spin();

}
