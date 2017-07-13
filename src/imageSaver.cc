#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../include/calib_image_saver/chessboard/Chessboard.h"

ros::Subscriber image_sub_0;

std::string image_path;
bool is_use_OpenCV = false;
bool is_show       = false;
cv::Size boardSize;

int image_count   = 0;
bool is_first_run = true;

cv::Mat DistributedImage;
// std::vector< std::vector< cv::Point2f > > total_image_points;

void
drawChessBoard( cv::Mat& image_input, cv::Mat& _DistributedImage, const std::vector< cv::Point2f >& imagePoints )
{
    int drawShiftBits  = 4;
    int drawMultiplier = 1 << drawShiftBits;

    cv::Scalar yellow( 0, 255, 255 );
    cv::Scalar green( 0, 255, 0 );

    cv::Mat& image = image_input;

    if ( image.channels( ) == 1 )
    {
        cv::cvtColor( image, image, CV_GRAY2RGB );
    }

    for ( size_t j = 0; j < imagePoints.size( ); ++j )
    {
        cv::Point2f pObs = imagePoints.at( j );

        // green points is the observed points
        cv::circle( image, cv::Point( cvRound( pObs.x * drawMultiplier ), cvRound( pObs.y * drawMultiplier ) ),
                    5, green, 2, CV_AA, drawShiftBits );

        // yellow points is the observed points
        cv::circle( _DistributedImage,
                    cv::Point( cvRound( pObs.x * drawMultiplier ), cvRound( pObs.y * drawMultiplier ) ), 5,
                    yellow, 2, CV_AA, drawShiftBits );
    }
}

void
callback_0( const sensor_msgs::Image::ConstPtr& img )
{
    cv::Mat image_input = cv_bridge::toCvCopy( img, "mono8" )->image;

    camera_model::Chessboard chessboard( boardSize, image_input );

    chessboard.findCorners( is_use_OpenCV );

    if ( chessboard.cornersFound( ) )
    {
        std::stringstream ss_num;
        ss_num << image_count;
        std::string image_name = image_path + "/IMG_" + ss_num.str( ) + ".jpg";
        std::cout << "#[INFO] Get chessboard image: " << image_name << std::endl;

        cv::imwrite( image_name, image_input );
        ++image_count;

        if ( is_show )
        {
            if ( is_first_run )
            {
                cv::Mat image_tmp( image_input.rows, image_input.cols, CV_8UC3 );
                DistributedImage = image_tmp;
                cv::namedWindow( "DistributedImage", CV_WINDOW_NORMAL );
            }

            cv::namedWindow( image_name, CV_WINDOW_NORMAL );
            drawChessBoard( image_input, DistributedImage, chessboard.getCorners( ) );
            cv::imshow( image_name, image_input );
            cv::waitKey( 10 );
            cv::destroyWindow( image_name );
        }
    }
    else
    {
        std::cout << "#[ERROR] Get no chessboard image." << std::endl;
    }
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "test_pub" );
    ros::NodeHandle n( "~" );

    n.getParam( "image_path", image_path );
    n.getParam( "board_width", boardSize.width );
    n.getParam( "board_height", boardSize.height );
    n.getParam( "is_use_OpenCV", is_use_OpenCV );
    n.getParam( "is_show", is_show );

    if ( boardSize.height <= 1 || boardSize.width <= 1 )
    {
        std::cout << "#[ERROR] Error with input chessbopard Size." << std::endl;
        return 0;
    }

    image_sub_0
    = n.subscribe< sensor_msgs::Image >( "/image_input", 3, callback_0, ros::TransportHints( ).tcpNoDelay( ) );

    while ( ros::ok( ) )
    {
        ros::spinOnce( );
    }

    cv::imwrite( image_path + "/IMG_" + "Distributed.jpg", DistributedImage );
    std::cout << "#[INFO] Get chessboard iamges: " << image_count << std::endl;

    return 0;
}
