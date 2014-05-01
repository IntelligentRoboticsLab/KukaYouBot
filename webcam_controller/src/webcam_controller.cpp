/**********************************************************
 *
 * Sébastien Negrijn
 * Uva@Work
 * University of Amsterdam
 *
 * Publishes the webcam stream of an opencv device
 * to the rostopic "rgb/webcam"
 *
 * Uses the webcam id as set in rosparam: webcam/webcamID
 * Switch off:    -2
 * Defaultwebcam: -1
 * WebcamID:      0-99
 **********************************************************/

#include <iostream>
#include <fstream>
#include <string.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>




class WebcamController
{
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub;
    CvCapture* capture;

    int webcamID;
    int frameID;

    IplImage* frame;
    cv::Mat image, imageCopy;

    const char* webcamIDParamName;


public:
    WebcamController()
        : it_( n_ )
    {


        image_pub = it_.advertise("/rgb/webcam", 1 );

        capture = 0;
        frameID = 0;

        webcamIDParamName = "webcam/webcamID";


        // Set value that is never met
        webcamID = -3;

        ROS_INFO( "Starting webcam controller" );

        // Loop for new frames
        while( ros::ok() )
        {

            this->broadCast();

            frameID++;

            ros::spinOnce();
        }

        cvReleaseCapture( &capture );
        ROS_INFO( "Error with ROS (ros not ok)" );
    }

    ~WebcamController()
    {
        cvReleaseCapture( &capture );
        ROS_INFO( "Exiting webcam stream" );
    }


    // Switch webcam if the rosparam has changed
    bool broadCast()
    {

        // Get the rosparam
        int webcamIDParam;
        n_.getParam( webcamIDParamName, webcamIDParam );


        if( webcamID != webcamIDParam )
        {

            // Release previous webcam stream
            cvReleaseCapture( &capture );

            // Change current webcamID
            webcamID = webcamIDParam;

            if( webcamID == -2 ) {
                ROS_INFO( "Switching off webcamstream" );

                ROS_INFO( "Switched off webcamstream" );

            }
            else
            {

                ROS_INFO( "Switching webcamstream" );
                webcamID = webcamIDParam;

                // Grab new webcam stream
                capture = cvCaptureFromCAM( webcamID );

                // Check if the webcamstream is active
                if( !capture )
                {
                    ROS_ERROR( "Invalid webcamID" );
                }
                else
                {
                    ROS_INFO( "Switched webcamstream" );
                }
            }
        }


        // Check if the webcam stream is still active
        if( capture )
        {
            return publishImage();
        }

        return false;
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

    WebcamController theWebcamController;

}
