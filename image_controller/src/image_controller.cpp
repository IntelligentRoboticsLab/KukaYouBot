#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>

#include <sensor_msgs/Image.h>

class ImageController
{

    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher  image_pub_;

    std::string topic_name;

public:

    ImageController()
        : it_(n_)
    {

        image_pub_ = it_.advertise( "image/active", 1 );

        updateParameters();

    }

    void updateParameters()
    {

        std::string param;
        n_.getParam( "image_controller/topic_name", param );

        if( param != topic_name )
        {
            ROS_INFO( "Changing active webcam" );
            topic_name = param;

            image_sub_ = it_.subscribe( topic_name, 1, &ImageController::callBack, this );
        }

    }

    void callBack( const sensor_msgs::ImageConstPtr &msg )
    {

        updateParameters();

        image_pub_.publish( msg );

    }
};


int main( int argc, char** argv )
{

    ros::init( argc, argv, "image_controller" );

    ImageController theImageController;

    ros::spin();

}
