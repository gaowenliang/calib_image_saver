#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

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
std::string image_name_left  = "left_";
std::string image_name_right = "right_";
cv::Size boardSize;
cv::Mat iamge_dst;

cv::Size image_size;
int image_count   = 0;
bool is_first_run = true;

cv::Mat DistributedImage_left;
cv::Mat DistributedImage_right;
std::vector< std::vector< cv::Point2f > > total_image_points_left;
std::vector< std::vector< cv::Point2f > > total_image_points_right;

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

    for ( int image_index = 0; image_index < _image_points.size( ); ++image_index )
        for ( int point_index = 0; point_index < _image_points.at( image_index ).size( ); ++point_index )
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
imageProcessCallback( const sensor_msgs::ImageConstPtr& left_image_msg,
                      const sensor_msgs::ImageConstPtr& right_image_msg )
{
    cv::Mat image_left  = cv_bridge::toCvCopy( left_image_msg, "mono8" )->image;
    cv::Mat image_right = cv_bridge::toCvCopy( right_image_msg, "mono8" )->image;

    camera_model::Chessboard chessboard_left( boardSize, image_left );
    camera_model::Chessboard chessboard_right( boardSize, image_right );

    chessboard_left.findCorners( is_use_OpenCV );
    chessboard_right.findCorners( is_use_OpenCV );

    if ( chessboard_left.cornersFound( ) && chessboard_right.cornersFound( ) )
    {
        std::stringstream ss_num;
        ss_num << image_count;
        std::string image_file_left
        = image_path + "/" + image_name_left + ss_num.str( ) + ".jpg";
        std::string image_file_right
        = image_path + "/" + image_name_right + ss_num.str( ) + ".jpg";

        std::cout << "#[INFO] Get chessboard image, left: " << image_name_left << std::endl;
        std::cout << "                             right: " << image_name_right << std::endl;

        cv::imwrite( image_file_left, image_left );
        cv::imwrite( image_file_right, image_right );

        ++image_count;
        total_image_points_left.push_back( chessboard_left.getCorners( ) );
        total_image_points_right.push_back( chessboard_right.getCorners( ) );

        if ( is_first_run )
        {
            image_size.height = image_left.rows;
            image_size.width  = image_right.cols;

            DistributedImage_left.create( image_size, CV_8UC3 );
            DistributedImage_right.create( image_size, CV_8UC3 );

            is_first_run = false;
        }
        if ( is_show )
        {
            cv::namedWindow( "DistributedImage_left", CV_WINDOW_NORMAL );
            cv::namedWindow( "DistributedImage_right", CV_WINDOW_NORMAL );
            cv::namedWindow( image_file_left, CV_WINDOW_NORMAL );
            cv::namedWindow( image_file_right, CV_WINDOW_NORMAL );
            drawChessBoard( image_left, DistributedImage_left, total_image_points_left.back( ) );
            drawChessBoard( image_right, DistributedImage_right, total_image_points_right.back( ) );
            cv::imshow( image_file_left, image_left );
            cv::imshow( image_file_right, image_right );
            cv::imshow( "DistributedImage_left", DistributedImage_left );
            cv::imshow( "DistributedImage_right", DistributedImage_right );
            cv::waitKey( 10 );
            cv::destroyWindow( image_file_left );
            cv::destroyWindow( image_file_right );
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
    ros::init( argc, argv, "stereoImageSaver" );
    ros::NodeHandle n( "~" );

    n.getParam( "image_path", image_path );
    n.getParam( "board_width", boardSize.width );
    n.getParam( "board_height", boardSize.height );
    n.getParam( "is_use_OpenCV", is_use_OpenCV );
    n.getParam( "is_show", is_show );
    n.getParam( "is_save_data", is_save_data );
    n.getParam( "data_path", data_path );
    n.getParam( "image_name_left", image_name_left );
    n.getParam( "image_name_right", image_name_right );

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

    message_filters::Subscriber< sensor_msgs::Image > sub_imgL( n, "/left_image", 2 );
    message_filters::Subscriber< sensor_msgs::Image > sub_imgR( n, "/right_image", 2 );

    typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image > SyncPolicy;
    message_filters::Synchronizer< SyncPolicy > sync( SyncPolicy( 3 ), sub_imgL, sub_imgR );

    sync.registerCallback( boost::bind( &imageProcessCallback, _1, _2 ) );

    while ( ros::ok( ) )
    {
        ros::spinOnce( );
    }

    cv::imwrite( image_path + "/left_" + "Distributed.jpg", DistributedImage_left );
    cv::imwrite( image_path + "/right_" + "Distributed.jpg", DistributedImage_right );
    std::cout << "#[INFO] Get chessboard iamges: " << image_count << std::endl;

    return 0;
}
