#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>


class HSV_Calibrator {

    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    const char* window_name;

    int h_lower;
    int h_upper;

    int s_lower;
    int s_upper;

    int v_lower;
    int v_upper;



public:

    HSV_Calibrator()
        : it_( n_ )
    {

        ROS_INFO( "Starting HSV Calibrator.." );

        const char* topic = "/rgb/webcam";


        // HSV upper and lower bounds
        h_lower = 0;
        h_upper = 180;

        s_lower = 0;
        s_upper = 255;

        v_lower = 0;
        v_upper = 255;

        // Create threshold window
        window_name = "calibrate_window";
        cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );


        // Add slider bars to preview window for thresholds
        cv::createTrackbar( "h_lower", window_name, &h_lower, 180, NULL );
        cv::createTrackbar( "h_upper", window_name, &h_upper, 180, NULL );

        cv::createTrackbar( "s_lower", window_name, &s_lower, 255, NULL );
        cv::createTrackbar( "s_upper", window_name, &s_upper, 255, NULL );

        cv::createTrackbar( "v_lower", window_name, &v_lower, 255, NULL );
        cv::createTrackbar( "v_upper", window_name, &v_upper, 255, NULL );

        // Subscribe to topic
        image_sub_ = it_.subscribe(topic, 1, &HSV_Calibrator::callBack, this );

        ROS_INFO( "Started HSV Calibrator.." );

    }

    ~HSV_Calibrator()
    {
        cvDestroyWindow( window_name );
    }

    void callBack( const sensor_msgs::ImageConstPtr& msg )
    {
        ROS_INFO( "Received image" );
        cv_bridge::CvImageConstPtr cv_img_ptr_;
        cv_img_ptr_= cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
        cv::Mat image = cv_img_ptr_->image;
        cv::Mat hsv_image;
        cv::Mat thres_image;

        cv::cvtColor( image, hsv_image, CV_BGR2HSV );

        cv::inRange( hsv_image, cv::Scalar( h_lower, s_lower, v_lower), cv::Scalar( h_upper, s_upper, v_upper ), thres_image );



        cv::imshow( window_name, thres_image );

        cv::waitKey(10);

        if( h_lower > h_upper )
            h_lower = h_upper;

        if( s_lower > s_upper )
            s_lower = s_upper;

        if( v_lower > v_upper )
            v_lower = v_upper;


    }
};



int main( int argc, char** argv )
{
    ros::init( argc, argv, "HSV_Calibrator");

    HSV_Calibrator theHSVCalibrator;

    ros::spin();
}
