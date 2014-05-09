#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <tf2_msgs/TFMessage.h>

class A_Star
{

    ros::NodeHandle n_;
    ros::Publisher tf_publisher_;
    ros::Subscriber tf_subscriber_;


    std::string topic_name;

public:

    A_Star()
    {

        tf_publisher_ = n_.advertise<tf2_msgs::TFMessage>( "path/path", 1 );
        tf_subscriber_ = n_.subscribe( "/tf", 1, &A_Star::callBack, this );

    }



    void callBack( const tf2_msgs::TFMessageConstPtr &msg )
    {

    }
};


int main( int argc, char** argv )
{

    ros::init( argc, argv, "a_star_pathfinding" );

    A_Star theAStar;

    ros::spin();

}
