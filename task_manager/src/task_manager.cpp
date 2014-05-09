#include <ros/ros.h>

class TaskManager
{

    ros::NodeHandle n_;

    // PARAMS
    bool ACTION_MOVE_ARM_TO_CENTER_X;
    bool ACTION_MOVE_ARM_TO_CENTER_Y;
    bool ACTION_GRIPPER_POSITION;
    bool ACTION_MOVE_BASE;
    bool ACTION_MOVE_BASE_LEFT;
    bool ACTION_MOVE_ARM_DOWN;

    bool FEEDBACK_FEATURE;
    bool FEEDBACK_DIFF_X;
    bool FEEDBACK_DIFF_Y;

    enum { STATE_LOOK_FOR_TARGET1, STATE_DRIVE_TO_TARGET1,
          STATE_PICKUP, STATE_PUT_AWAY,
          STATE_LOOK_FOR_TARGET2, STATE_DRIVE_TO_TARGET2,
          STATE_FROM_BACK, STATE_DROP };

    int GOAL_X;
    int GOAL_Y;

    int STATE;

public:

    TaskManager()
    {

        ACTION_MOVE_ARM_TO_CENTER_X = false;
        ACTION_MOVE_ARM_TO_CENTER_Y = false;
        ACTION_GRIPPER_POSITION     = false;
        ACTION_MOVE_BASE            = false;
        ACTION_MOVE_BASE_LEFT       = false;
        ACTION_MOVE_ARM_DOWN        = false;

        STATE = 0;

    }

    void progress()
    {

        switch( STATE )
        {
            case STATE_LOOK_FOR_TARGET1:
            {
                n_.setParam( "ACTION_MOVE_BASE_LEFT", true );

                if( FEEDBACK_FEATURE )
                {
                    ROS_INFO( "FOUND TARGET" );
                    n_.setParam( "ACTION_MOVE_BASE_LEFT", false );
                    STATE++;
                }

                break;
            }

            case STATE_DRIVE_TO_TARGET1:
            {
                n_.setParam( "image_controller/topic_name", "rgb/image" );
                n_.setParam( "ACTION_MOVE_BASE", true );
                n_.setParam( "GOAL_X", 320 );
                n_.setParam( "GOAL_Y", 320 );

                if( FEEDBACK_DIFF_X && FEEDBACK_DIFF_Y )
                {
                    ROS_INFO( "AT TARGET" );
                    n_.setParam( "ACTION_MOVE_BASE", false );
                    STATE++;
                }

                break;
            }

            case STATE_PICKUP:
            {



                break;
            }

            case STATE_PUT_AWAY:
            {


                break;
            }

            case STATE_LOOK_FOR_TARGET2:
            {


                break;
            }

            case STATE_DRIVE_TO_TARGET2:
            {


                break;
            }

            case STATE_FROM_BACK:
            {


                break;
            }

            case STATE_DROP:
            {


                break;
            }
        }

    }

    void updateParams()
    {

        n_.param( "FEEDBACK_FEATURE", FEEDBACK_FEATURE, false );
        n_.param( "FEEDBACK_FEATURE", FEEDBACK_DIFF_X, false );
        n_.param( "FEEDBACK_FEATURE", FEEDBACK_DIFF_Y, false );

    }


};


int main( int argc, char** argv )
{
    ros::init( argc, argv, "task_controller" );

    TaskManager theTaskManager;
    theTaskManager.updateParams();

    ros::spinOnce();

    while( ros::ok() )
    {

        theTaskManager.progress();
        ros::spinOnce();
    }
}
