/*******************************
 *
 * Sébastien Negrijn
 * Uva@Work
 * University of Amsterdam
 *
 * Publishes the webcam stream of an opencv device
 * to the rostopic "rgb/webcam"
 *
 * Uses the webcam id as set in rosparam: webcam/webcamID
 * Defaults to -1 (any cam).
 *
 *******************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>


class Webcam
{
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub;
    CvCapture* capture;

    int webcamID;
    int frameID;

    IplImage* frame;
    cv::Mat image, imageCopy;


public:
    Webcam()
        : it_( n_ )
    {


        image_pub = it_.advertise("/rgb/webcam", 1 );

        capture = 0;
        frameID = 0;

        n_.param( "webcam/webcamID", webcamID, -1 );


        capture = cvCaptureFromCAM( webcamID );

        cvNamedWindow( "result", CV_WINDOW_AUTOSIZE );

        if(!capture )
        {
            ROS_ERROR( "Could not lock onto webcam" );

        }


        ROS_INFO( "Publishing webcam stream.." );
        //std::cout << "Publishing webcam stream.." << std:: endl;
        while( ros::ok() )
        {

            this->publishImage();

            frameID++;
        }
        std::cout << "Error with ROS (ros not ok)." << std::endl;
    }

    ~Webcam()
    {
        std::cout << "Exiting webcam stream" << std:: endl;
    }

    // Grabs and publishes an image from the webcam stream
    bool publishImage()
    {

        // Grab a frame
        frame = cvQueryFrame( capture );

        // Convert to cv::Mat
        image = frame;

        // Check if the image is set
        if( image.empty() )
        {
            std::cout << "Empty frame" << std::endl;
            return false;
        }


        if( frame->origin == IPL_ORIGIN_TL )
        {
            image.copyTo( imageCopy );
        }
        else
        {
            flip( image, imageCopy, 0 );
        };

        // Create ros convertible opencv image
        cv_bridge::CvImage cv_image;

        // Set image
        cv_image.image = image;

        // Set encoding
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;

        // Set header (partially)
        cv_image.header.frame_id = frameID;

        // Publisch the image
        image_pub.publish( cv_image.toImageMsg() );

        return true;
    }

};

int main(int argc, char** argv)
{
    ros::init( argc, argv, "webcam" );

    Webcam webcam;

    ros::spin();

}
