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
    bool FEEDBACK_ARM_IS_DOWN;

    enum { STATE_LOOK_FOR_TARGET1, STATE_DRIVE_TO_TARGET1,
          STATE_OVERVIEW, STATE_PICKUP, STATE_PUT_AWAY,
          STATE_LOOK_FOR_TARGET2, STATE_DRIVE_TO_TARGET2,
          STATE_FROM_BACK, STATE_DROP, STATE_DONE };

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
        FEEDBACK_FEATURE            = false;
        FEEDBACK_ARM_IS_DOWN        = false;

        n_.setParam( "ACTION_MOVE_ARM_TO_CENTER_X", ACTION_MOVE_ARM_TO_CENTER_X );
        n_.setParam( "ACTION_MOVE_ARM_TO_CENTER_Y", ACTION_MOVE_ARM_TO_CENTER_Y );
        n_.setParam( "ACTION_MOVE_BASE",            ACTION_MOVE_BASE );
        n_.setParam( "ACTION_MOVE_BASE_LEFT",       ACTION_MOVE_BASE_LEFT );
        n_.setParam( "ACTION_MOVE_ARM_DOWN",        ACTION_MOVE_ARM_DOWN );
        n_.setParam( "FEEDBACK_FEATURE",            FEEDBACK_FEATURE );
        n_.setParam( "FEEDBACK_ARM_IS_DOWN",        FEEDBACK_ARM_IS_DOWN );

        STATE = STATE_LOOK_FOR_TARGET1;

        sleep( 2 );

        ROS_INFO( "Rotating left" );

    }



    void progress()
    {

        switch( STATE )
        {
            case STATE_LOOK_FOR_TARGET1:
            {
                n_.setParam( "image_controller/topic_name", "rgb/image" );
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

                n_.setParam( "ACTION_MOVE_BASE", true );
                n_.setParam( "image_controller/topic_name", "rgb/image" );
                n_.setParam( "GOAL_X", 340 );
                n_.setParam( "GOAL_Y", 430 );

                if( FEEDBACK_DIFF_X && FEEDBACK_DIFF_Y )
                {

                    ROS_INFO( "AT TARGET" );
                    n_.setParam( "ACTION_MOVE_BASE", false );
                    n_.setParam( "FEEDBACK_DIFF_X", false );
                    n_.setParam( "FEEDBACK_DIFF_Y", false );
                    STATE++;
                }

                break;
            }

            case STATE_OVERVIEW:
            {

                n_.setParam( "GOAL_X", 320 );
                n_.setParam( "GOAL_Y", 320 );
                n_.setParam( "ACTION_OVERVIEW", true );
                n_.setParam( "image_controller/topic_name", "rgb/webcam" );
                n_.setParam( "ACTION_GRIPPER_POSITION", true );

                sleep( 1 );
                ros::spinOnce();
                sleep( 4 );
                n_.setParam( "ACTION_OVERVIEW", false );

                ROS_INFO( "IN OVERVIEW POSITION" );

                STATE++;



            }

            case STATE_PICKUP:
            {

                n_.setParam( "image_thresholder/h_lower", 94 );
                n_.setParam( "image_thresholder/h_upper", 122 );

                n_.setParam( "image_thresholder/s_lower", 132 );
                n_.setParam( "image_thresholder/s_upper", 229 );

                n_.setParam( "image_thresholder/v_lower", 55 );
                n_.setParam( "image_thresholder/v_upper", 174 );

                n_.setParam( "image_thresholder/gaussian", 51 );

                n_.setParam( "ACTION_MOVE_ARM_TO_CENTER_X", true );
                n_.setParam( "ACTION_MOVE_ARM_TO_CENTER_Y", true );




                n_.setParam( "ACTION_MOVE_ARM_DOWN", true );



                if( FEEDBACK_ARM_IS_DOWN )
                {
                    ROS_INFO( "ARM IS DOWN" );
                    n_.setParam( "ACTION_MOVE_ARM_TO_CENTER_X", false );
                    n_.setParam( "ACTION_MOVE_ARM_TO_CENTER_Y", false );
                    ACTION_GRIPPER_POSITION = false;
                    n_.setParam( "ACTION_GRIPPER_POSITION", ACTION_GRIPPER_POSITION );
                    sleep( 4 );
                    STATE++;
                }



                break;
            }

            case STATE_PUT_AWAY:
            {

                n_.setParam( "ACTION_OVERVIEW", true );
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

            case STATE_DONE:
                break;
        }

    }

    void updateParams()
    {

        n_.param( "FEEDBACK_FEATURE", FEEDBACK_FEATURE, false );
        n_.param( "FEEDBACK_DIFF_X", FEEDBACK_DIFF_X, false );
        n_.param( "FEEDBACK_DIFF_Y", FEEDBACK_DIFF_Y, false );
        n_.param( "FEEDBACK_ARM_IS_DOWN", FEEDBACK_ARM_IS_DOWN, false );


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
        theTaskManager.updateParams();
        ros::spinOnce();
    }
}
