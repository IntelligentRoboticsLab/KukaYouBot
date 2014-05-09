#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <brics_actuator/JointPositions.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Twist.h>


#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>


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

    void parseROSMsg( const sensor_msgs::JointState &msg )
    {

        double pos;
        for( int i = 0; i < msg.position.size(); i++ )
        {
            pos = msg.position.at(i);

            armJointPositions[i].value = pos;

        }

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

    void setJointPositions( double joints[] )
    {

        this->setJointPositions( joints[0], joints[1], joints[2], joints[3], joints[4] );

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

class YouBotBase
{

    geometry_msgs::Twist velocityMsg;

    double minForwardSpeed;
    double maxForwardSpeed;

    double minRotateSpeed;
    double maxRotateSpeed;

    double forwardSpeed;
    double rotateSpeed;




public:

    YouBotBase()
    {

        velocityMsg.linear.x = velocityMsg.linear.y = velocityMsg.angular.z = 0;

    }

    const geometry_msgs::Twist toROSMsg()
    {

        return velocityMsg;

    }

    void setForwardSpeed( double speed )
    {
        velocityMsg.linear.x = speed;
    }

    void setRotateSpeed( double speed )
    {
        velocityMsg.angular.z = speed;
    }

};

class YouBotGripper
{

    brics_actuator::JointPositions gripperPositionsActuator;
    std::vector <brics_actuator::JointValue> gripperPositions;

    int numberOfArmJoints;


public:

    YouBotGripper()
    {

        numberOfArmJoints = 5;
        gripperPositions.resize( 2 );

        init();

    }

    void parseROSMsg( const sensor_msgs::JointState &msg )
    {

        double pos;
        for( int i = 0; i < 2; i++ )
        {
            pos = msg.position[numberOfArmJoints + i];

            gripperPositions[i].value = pos;
        }

    }

    void setJointPositions( double joint0, double joint1 )
    {


        gripperPositions[0].value = joint0;
        gripperPositions[1].value = joint1;


        gripperPositionsActuator.positions = gripperPositions;

    }

    void setJointPositions( double joints[] )
    {

        this->setJointPositions( joints[0], joints[1] );

    }

    double getJointPosition( int joint )
    {

        return gripperPositions[joint].value;

    }

    const brics_actuator::JointPositions toROSMsg()
    {

        return gripperPositionsActuator;
    }

    void init()
    {

        gripperPositions[0].joint_uri = "gripper_finger_joint_l";
        gripperPositions[0].value = 0;
        gripperPositions[0].unit = boost::units::to_string(boost::units::si::meter);

        gripperPositions[1].joint_uri = "gripper_finger_joint_r";
        gripperPositions[1].value = 0;
        gripperPositions[1].unit = boost::units::to_string(boost::units::si::meter);

        gripperPositionsActuator.positions = gripperPositions;

    }

};

class ActionController
{

    ros::NodeHandle n_;
    ros::Subscriber arm_subscriber_;
    ros::Publisher arm_publisher_;
    ros::Publisher gripper_publisher_;
    ros::Subscriber path_subscriber_;
    ros::Publisher base_publisher_;

    ros::Subscriber feature_subscriber_;

    YouBotArm currentYouBotArm;
    YouBotGripper currentYouBotGripper;


    // PARAMS
    bool ACTION_MOVE_ARM_TO_CENTER_X;
    bool ACTION_MOVE_ARM_TO_CENTER_Y;
    bool ACTION_GRIPPER_POSITION;
    bool ACTION_MOVE_BASE;
    bool ACTION_MOVE_BASE_LEFT;
    bool ACTION_MOVE_ARM_DOWN;

    int GOAL_X;
    int GOAL_Y;

    bool initial;

public:



    ActionController()
    {

        initial = true;

        GOAL_X = 320;
        GOAL_Y = 320;

        ROS_INFO( "Starting ActionController.." );
        arm_subscriber_ = n_.subscribe( "/joint_states", 1,
                                        &ActionController::updateCurrentModel, this);

        arm_publisher_  = n_.advertise<brics_actuator::JointPositions>( "/arm_1/arm_controller/position_command", 1 );

        gripper_publisher_ = n_.advertise<brics_actuator::JointPositions>( "/arm_1/gripper_controller/position_command", 1 );

        feature_subscriber_ = n_.subscribe( "/detector/coords", 1, &ActionController::initiateAction, this );

        path_subscriber_ = n_.subscribe( "path/path", 1, &ActionController::moveBaseFromPath, this );

        base_publisher_ = n_.advertise<geometry_msgs::Twist>( "/cmd_vel",1 );

        ROS_INFO( "Started ActionController" );




    }

    void initiate()
    {


        double poseGrasp[] = {3.04171,2.04427,-1.5189129,2.5434289757,2.8761944};

        double poseHome2[] = {2.8186, 0.9632, -1.9736, 3.2648, 2.9539};

        double poseBack[] = {2.8186, 0.1, -2.736, 3.2648, 2.9539};

        double poseStore[] = {2.8186, 0.1, -2.736, 3.2648, 2.9539};

        double poseOverview[] = {3.04171, 1.4, -1.1, 3.0, 2.9539};

        // 5 cm above ground above object
        double poseClosePickup[] = {2.9882, 1.9828, -1.1004, 2.5464, 2.9538};

        double poseGroundPickup[] = {2.9882, 2.25, -1.25, 2.5464, 2.9538};

        double gripperClosed[] = {0.0, 0.0 };

        double gripperOpen[] = {0.0115, 0.0115 };


        YouBotArm newArmPosition;
        ros::spinOnce();

        sleep( 1 );

        newArmPosition.setJointPositions( poseOverview );
        arm_publisher_.publish( newArmPosition.toROSMsg() );
        ros::spinOnce();
        sleep( 5 );

    }

    void initiateAction( const std_msgs::Float32MultiArray &msg )
    {

        if( msg.data.size() > 0 )
        {
            n_.setParam( "FEEDBACK_FEATURE", true );
        }
        else
        {
            n_.setParam( "FEEDBACK_FEATURE", false );
        }

        updateParams();

        YouBotArm newYouBotArmPosition;
        YouBotGripper newYouBotGripperPosition;
        YouBotBase newYouBotBaseSpeed;


        // TODO: clone function
        newYouBotArmPosition.setJointPosition( 0, currentYouBotArm.getJointPosition( 0 ));
        newYouBotArmPosition.setJointPosition( 1, currentYouBotArm.getJointPosition( 1 ));
        newYouBotArmPosition.setJointPosition( 2, currentYouBotArm.getJointPosition( 2 ));
        newYouBotArmPosition.setJointPosition( 3, currentYouBotArm.getJointPosition( 3 ));
        newYouBotArmPosition.setJointPosition( 4, currentYouBotArm.getJointPosition( 4 ));

        if( ACTION_MOVE_ARM_TO_CENTER_X )
        {
            moveArmToCenterX( msg, &newYouBotArmPosition );
        }

        if( ACTION_MOVE_ARM_TO_CENTER_Y )
        {
            moveArmToCenterY( msg, &newYouBotArmPosition );
        }

        if( ACTION_MOVE_ARM_DOWN )
        {
            moveArmDown( &newYouBotArmPosition );
        }

        if( ACTION_MOVE_BASE )
        {
            moveBase( msg, &newYouBotBaseSpeed );
        }

        if( ACTION_MOVE_BASE_LEFT )
        {
            moveBaseLeft( msg, &newYouBotBaseSpeed );
        }



        moveGripper( msg, &newYouBotGripperPosition );


        arm_publisher_.publish( newYouBotArmPosition.toROSMsg() );
        base_publisher_.publish( newYouBotBaseSpeed.toROSMsg());
        gripper_publisher_.publish( newYouBotGripperPosition.toROSMsg() );


    }

    bool moveArmDown( YouBotArm *newArmPosition )
    {

        double joint0StepSize = 0.005;
        double newJoint1Position = currentYouBotArm.getJointPosition(1) + joint0StepSize;

        newArmPosition->setJointPosition( 1, newJoint1Position );

    }

    bool moveArmToCenterX( const std_msgs::Float32MultiArray msg, YouBotArm *newArmPosition )
    {
        if( msg.data.size() == 0 )
        {
            return false;
        }

        int x = static_cast<int> ( msg.data[0] );
        int y = static_cast<int> ( msg.data[1] );

        //Correct x
        int maxRange = 0;
        double joint0StepSize = 0.0001;

        int diffX = GOAL_X - x;


        double newJoint0Pos = currentYouBotArm.getJointPosition( 0 );

        if( newJoint0Pos == 0 )
            return false;

        if( diffX > maxRange )
        {
            std::cout << "left ";

            newJoint0Pos = newJoint0Pos - (joint0StepSize * diffX);
            n_.setParam( "FEEDBACK_DIFF_X", false );

        }
        else if( diffX < -maxRange )
        {
            std::cout << "right ";

            newJoint0Pos = newJoint0Pos - joint0StepSize * diffX;
            n_.setParam( "FEEDBACK_DIFF_X", false );

        }
        else
        {

            n_.setParam( "FEEDBACK_DIFF_X", true );

        }

        newArmPosition->setJointPosition( 0, newJoint0Pos );

    }

    bool moveArmToCenterY( const std_msgs::Float32MultiArray msg, YouBotArm *newArmPosition )
    {
        if( msg.data.size() == 0 )
        {
            return false;
        }

        int x = static_cast<int> ( msg.data[0] );
        int y = static_cast<int> ( msg.data[1] );


        int maxRange = 0;

        double joint3StepSize = 0.0004;

        int diffY = GOAL_Y - y;

        double newJoint3Pos = currentYouBotArm.getJointPosition( 3 );

        if( diffY > maxRange )
        {
            std::cout << "up ";

            newJoint3Pos = newJoint3Pos - joint3StepSize * diffY;
            n_.setParam( "FEEDBACK_DIFF_Y", false);
        }
        else if( diffY < -maxRange )
        {
            std::cout << "down " << std::endl;

            newJoint3Pos = newJoint3Pos - joint3StepSize * diffY;
            n_.setParam( "FEEDBACK_DIFF_Y", false);
        }
        else
        {
            n_.setParam( "FEEDBACK_DIFF_Y", true);
        }



        newArmPosition->setJointPosition( 3, newJoint3Pos );

        std::cout << x << " " << y;

        std::cout << std::endl;


        return true;


    }

    bool moveBase( const std_msgs::Float32MultiArray &msg, YouBotBase *newBase )
    {

        int baseGoalX = 320;
        int baseGoalY = 425;

        int minLongSpeed = 0.03;
        int maxLongSpeed = 0.1;

        int minRotSpeed = 0.02;
        int maxRotSpeed = 0.2;

        int maxRange = 15;

        double rotSpeed, longSpeed;
        rotSpeed = longSpeed = 0;

        if( msg.data.size() == 0 )
            return false;

        int x = static_cast<int>( msg.data[0] );
        int y = static_cast<int>( msg.data[1] );


        // Direct in x direction;
        int diffX = baseGoalX - x;

        if( diffX > maxRange )
        {

            rotSpeed = ((double)(diffX-maxRange) / ( 2 * maxRange ) * maxRotSpeed) + minRotSpeed;
        }
        if( diffX < -maxRange )
        {

            rotSpeed = ((double)(diffX+maxRange) / ( 2 * maxRange ) * maxRotSpeed) - minRotSpeed;
        }
        if( rotSpeed > maxRotSpeed )
            rotSpeed = maxRotSpeed;

        if( rotSpeed < -maxRotSpeed )
            rotSpeed = -maxRotSpeed;

        // Direct in y direction;
        int diffY = baseGoalY - y;

        if( diffY > maxRange )
        {

            longSpeed = ((double)(diffY-maxRange) / ( 2 * maxRange ) * maxLongSpeed) + minLongSpeed;
        }
        if( diffY < -maxRange )
        {

            longSpeed = ((double)(diffY+maxRange) / ( 2 * maxRange ) * maxLongSpeed) - minLongSpeed;
        }
        if( longSpeed > maxLongSpeed )
            longSpeed = maxLongSpeed;

        if( longSpeed < -maxLongSpeed )
            longSpeed = -maxLongSpeed;

        newBase->setForwardSpeed( longSpeed );
        newBase->setRotateSpeed( rotSpeed );

    }

    void moveBaseLeft( const std_msgs::Float32MultiArray &msg, YouBotBase *newBase )
    {

        double rotSpeed = 0.1;

        newBase->setRotateSpeed( rotSpeed );

    }

    void moveBaseFromPath( const tf2_msgs::TFMessageConstPtr &msg )
    {

    }

    void moveGripper( const std_msgs::Float32MultiArray msg, YouBotGripper *newGripper )
    {

        double gripperClosed[] = {0.0, 0.0 };

        double gripperOpen[] = {0.0115, 0.0115 };

        if( ACTION_GRIPPER_POSITION )
        {
            newGripper->setJointPositions( gripperOpen );
        }
        else
        {
            newGripper->setJointPositions( gripperClosed );
        }

    }

    void updateCurrentModel( const sensor_msgs::JointState &msg )
    {

        if( msg.name[0] == "arm_joint_1" )
        {

            currentYouBotArm.parseROSMsg( msg );
            currentYouBotGripper.parseROSMsg( msg );

        }
        else if( msg.name[0] == "wheel_joint_fl" )
        {
            // update current base moddel?
        }
    }


    void updateParams()
    {

        n_.param( "ACTION_MOVE_ARM_TO_CENTER_X", ACTION_MOVE_ARM_TO_CENTER_X, false );
        n_.param( "ACTION_MOVE_ARM_DOWN", ACTION_MOVE_ARM_DOWN, false );
        n_.param( "ACTION_MOVE_ARM_TO_CENTER_Y", ACTION_MOVE_ARM_TO_CENTER_Y, false );
        n_.param( "ACTION_GRIPPER_POSITION", ACTION_GRIPPER_POSITION, false );
        n_.param( "ACTION_MOVE_BASE", ACTION_MOVE_BASE, false );
        n_.param( "ACTION_MOVE_BASE_LEFT", ACTION_MOVE_BASE_LEFT, false );

    }

};

int main( int argc, char** argv )
{
    ros::init( argc, argv, "action_controller" );

    ActionController theActionController;

    ros::spinOnce();

    theActionController.initiate();


    ros::spin();

}
