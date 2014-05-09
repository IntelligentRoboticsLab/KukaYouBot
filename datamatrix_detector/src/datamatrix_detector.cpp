#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Float32MultiArray.h>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class Datamatrix_Detector
{
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher feature_pub_;





public:


    Datamatrix_Detector()
        : it_( n_ )
    {

        image_sub_ = it_.subscribe( "/image/active", 1, &Datamatrix_Detector::callBack, this );
        feature_pub_ = n_.advertise<std_msgs::Float32MultiArray>( "/detector/coords", 1 );



    }

    void callBack( const sensor_msgs::ImageConstPtr &msg )
    {



    }

};

int main( int argc, char** argv )
{

    ros::init( argc, argv, "blob_detector" );

    Datamatrix_Detector theDatamatrixDetector;

    ros::spin();

}
