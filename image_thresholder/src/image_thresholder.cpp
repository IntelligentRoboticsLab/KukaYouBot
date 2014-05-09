#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <image_transport/publisher.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

class Image_Thresholder
{
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    int h_lower;
    int h_upper;

    int s_lower;
    int s_upper;

    int v_upper;
    int v_lower;

    int gaussian;

public:


    Image_Thresholder()
        : it_( n_ )
    {
        ROS_INFO("Starting image_thresholder" );

        image_sub_ = it_.subscribe( "image/active", 1, &Image_Thresholder::callBack, this );
        image_pub_ = it_.advertise( "thresh/image", 1 );

        if( paramsSet() )
        {
            setParams();
            ROS_INFO( "All params are set" );

            ROS_INFO( "Started image_thresholder" );
        }
        else
        {
            ROS_ERROR( "Set all params for this node to work" );
        }

        setParams();

    }

    void callBack( const sensor_msgs::ImageConstPtr &msg )
    {

        setParams();

        cv_bridge::CvImageConstPtr cv_img;
        cv_img = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );

        cv_bridge::CvImage output;

        //output.header = cv_img->header;
        output.encoding = cv_img->encoding;

        cv::Mat hsv_image;
        cv::Mat thres_image;
        cv::Mat smooth_image;
        cv::Mat output_image;

        cv::Size size;
        size.height = gaussian;
        size.width = gaussian;

        cv::GaussianBlur( cv_img->image, smooth_image, size, 0, 0 );

        cv::cvtColor( smooth_image, hsv_image, CV_BGR2HSV );

        cv::inRange( hsv_image, cv::Scalar( h_lower, s_lower, v_lower), cv::Scalar( h_upper, s_upper, v_upper ), thres_image );

        cv::cvtColor( thres_image, output.image, CV_GRAY2BGR );

        //for( int i = 0; i < smooth_image.cols; i++ )

        image_pub_.publish( output.toImageMsg() );
    }

    bool paramsSet()
    {

        bool allSet = true;

        if( !n_.hasParam( "image_thresholder/h_lower" ))
        {
            ROS_ERROR( "PARAM image_thresholder/h_lower not set" );
            allSet = false;
        }
        if( !n_.hasParam( "image_thresholder/h_upper" ))
        {
            ROS_ERROR( "PARAM image_thresholder/h_upper not set" );
            allSet = false;
        }

        if( !n_.hasParam( "image_thresholder/s_lower" ))
        {
            ROS_ERROR( "PARAM image_thresholder/s_lower not set" );
            allSet = false;
        }
        if( !n_.hasParam( "image_thresholder/s_upper" ))
        {
            ROS_ERROR( "PARAM image_thresholder/s_upper not set" );
            allSet = false;
        }

        if( !n_.hasParam( "image_thresholder/v_lower" ))
        {
            ROS_ERROR( "PARAM image_thresholder/v_lower not set" );
            allSet = false;
        }
        if( !n_.hasParam( "image_thresholder/v_upper" ))
        {
            ROS_ERROR( "PARAM image_thresholder/v_upper not set" );
            allSet = false;
        }

        if( !n_.hasParam( "image_thresholder/gaussian" ))
        {
            ROS_ERROR( "PARAM image_thresholder/gaussian not set" );
            allSet = false;
        }

        return allSet;

    }

    void setParams()
    {
        n_.getParam( "image_thresholder/h_lower", h_lower );
        n_.getParam( "image_thresholder/h_upper", h_upper );

        n_.getParam( "image_thresholder/s_lower", s_lower );
        n_.getParam( "image_thresholder/s_upper", s_upper );

        n_.getParam( "image_thresholder/v_lower", v_lower );
        n_.getParam( "image_thresholder/v_upper", v_upper );

        n_.getParam( "image_thresholder/gaussian", gaussian );
    }
};


int main( int argc, char** argv )
{

    ros::init( argc, argv, "image_thresholder" );

    Image_Thresholder theImageThresholder;

    ros::spin();

}
