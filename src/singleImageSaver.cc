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

ros::Subscriber image_sub;

std::string image_path;
bool is_use_OpenCV = false;
bool is_show       = false;
bool is_save_data  = false;
std::string data_path;
std::string image_name = "IMG_";
cv::Size boardSize;
cv::Mat iamge_dst;

cv::Size image_size;
int image_count   = 0;
bool is_first_run = true;

cv::Mat DistributedImage;
std::vector< std::vector< cv::Point2f > > total_image_points;

void
save_chessboard_data( const std::string file_name,
                      const std::vector< std::vector< cv::Point2f > > _image_points )
{
    cv::FileStorage fs( file_name, cv::FileStorage::WRITE );

    fs << "model_type"
       << "Chessboard";

    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;

    for ( int image_index = 0; image_index < total_image_points.size( ); ++image_index )
        for ( int point_index = 0; point_index < total_image_points.at( image_index ).size( ); ++point_index )
        {
        }
}

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
        cv::circle( _DistributedImage, cv::Point( cvRound( pObs.x * drawMultiplier ),
                                                  cvRound( pObs.y * drawMultiplier ) ),
                    5, yellow, 2, CV_AA, drawShiftBits );
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
        std::string image_file = image_path + "/" + image_name + ss_num.str( ) + ".jpg";
        std::cout << "#[INFO] Get chessboard image: " << image_name << std::endl;

        cv::imwrite( image_file, image_input );

        ++image_count;
        total_image_points.push_back( chessboard.getCorners( ) );

        if ( is_first_run )
        {
            image_size.height = image_input.rows;
            image_size.width  = image_input.cols;

            DistributedImage.create( image_size, CV_8UC3 );

            is_first_run = false;
        }
        if ( is_show )
        {
            cv::namedWindow( "DistributedImage", CV_WINDOW_NORMAL );
            cv::namedWindow( image_file, CV_WINDOW_NORMAL );
            drawChessBoard( image_input, DistributedImage, total_image_points.back( ) );
            cv::imshow( image_file, image_input );
            cv::imshow( "DistributedImage", DistributedImage );
            cv::waitKey( 10 );
            cv::destroyWindow( image_file );
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
    ros::init( argc, argv, "singleImageSaver" );
    ros::NodeHandle n( "~" );

    n.getParam( "image_path", image_path );
    n.getParam( "board_width", boardSize.width );
    n.getParam( "board_height", boardSize.height );
    n.getParam( "is_use_OpenCV", is_use_OpenCV );
    n.getParam( "is_show", is_show );
    n.getParam( "is_save_data", is_save_data );
    n.getParam( "data_path", data_path );
    n.getParam( "image_name", image_name );

    if ( boardSize.height <= 1 || boardSize.width <= 1 )
    {
        std::cout << "#[ERROR] Error with input chessbopard Size." << std::endl;
        return 0;
    }

    std::string data_file_name;
    if ( is_save_data )
    {
        if ( data_path.empty( ) )
            data_path = image_path;

        data_file_name = data_path + "/data.ymal";
    }

    image_sub = n.subscribe< sensor_msgs::Image >( "/image_input", 3, callback_0,
                                                   ros::TransportHints( ).tcpNoDelay( ) );

    while ( ros::ok( ) )
    {
        ros::spinOnce( );
    }

    cv::imwrite( image_path + "/IMG_" + "Distributed.jpg", DistributedImage );
    std::cout << "#[INFO] Get chessboard iamges: " << image_count << std::endl;

    return 0;
}
