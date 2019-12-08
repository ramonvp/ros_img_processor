#include "ros_img_processor_node.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

//constants
static const int DEFAULT_GAUSSIAN_BLUR_SIZE = 7;
static const double DEFAULT_GAUSSIAN_BLUR_SIGMA = 2;
static const double DEFAULT_CANNY_EDGE_TH = 150;
static const double DEFAULT_HOUGH_ACCUM_RESOLUTION = 2;
static const double DEFAULT_MIN_CIRCLE_DIST = 40;
static const double DEFAULT_HOUGH_ACCUM_TH = 70;
static const int DEFAULT_MIN_RADIUS = 20;
static const int DEFAULT_MAX_RADIUS = 100;


RosImgProcessorNode::RosImgProcessorNode() :
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
    //loop rate [hz], Could be set from a yaml file
    rate_=10;

    //sets publishers
    image_pub_ = img_tp_.advertise("image_out", 100);
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>( "arrow_marker", 1 );

    //sets subscribers
    image_subs_ = img_tp_.subscribe("image_in", 1, &RosImgProcessorNode::imageCallback, this);
    camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &RosImgProcessorNode::cameraInfoCallback, this);

    // load detection params
    ros::NodeHandle private_nh("~");

    private_nh.param("gaussian_blur_size", params_.gaussian_blur_size, DEFAULT_GAUSSIAN_BLUR_SIZE);
    private_nh.param<double>("gaussian_blur_sigma", params_.gaussian_blur_sigma, DEFAULT_GAUSSIAN_BLUR_SIGMA);

    private_nh.param<double>("hough_accum_resolution", params_.hough_accum_resolution, DEFAULT_HOUGH_ACCUM_RESOLUTION);
    private_nh.param<double>("min_circle_dist", params_.min_circle_dist, DEFAULT_MIN_CIRCLE_DIST);
    private_nh.param<double>("canny_edge_th", params_.canny_edge_th, DEFAULT_CANNY_EDGE_TH);
    private_nh.param<double>("hough_accum_th", params_.hough_accum_th, DEFAULT_HOUGH_ACCUM_TH);
    private_nh.param("min_radius", params_.min_radius, DEFAULT_MIN_RADIUS);
    private_nh.param("max_radius", params_.max_radius, DEFAULT_MAX_RADIUS);

    first_reconfig_ = true;
    dynamic_reconfigure::Server<ros_img_processor::CircleDetectorConfig>::CallbackType f;
    f = boost::bind(&RosImgProcessorNode::reconfigure_callback, this, _1, _2);
    config_server_.setCallback(f);

}

void RosImgProcessorNode::reconfigure_callback(ros_img_processor::CircleDetectorConfig& config, uint32_t level)
{
    if (first_reconfig_)
    {
        config = params_;
        first_reconfig_ = false;
        return;  // Ignore the first call to reconfigure which happens at startup
    }

    params_ = config;
}

RosImgProcessorNode::~RosImgProcessorNode()
{
    //
}

void RosImgProcessorNode::process()
{
    //check if new image is there
    if ( cv_img_ptr_in_ != nullptr )
    {
        // copy the input image to the out one
        cv_img_out_.image = cv_img_ptr_in_->image;

        // find the ball
        cv::Mat gray_image;
        std::vector<cv::Vec3f> circles;

        // If input image is RGB, convert it to gray
        cv::cvtColor(cv_img_out_.image, gray_image, CV_BGR2GRAY);

        //Reduce the noise so we avoid false circle detection
        cv::GaussianBlur( gray_image, gray_image, cv::Size(params_.gaussian_blur_size, params_.gaussian_blur_size), params_.gaussian_blur_sigma );

        //Apply the Hough Transform to find the circles
        cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, params_.hough_accum_resolution, params_.min_circle_dist, params_.canny_edge_th, params_.hough_accum_th, params_.min_radius, params_.max_radius );

        //draw circles on the image, if more than 1, display only the first one
        if ( circles.size() )
        {
            cv::Point center = cv::Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
            int radius = cvRound(circles[0][2]);

            std::cout << "Ball found at (" << center.x << ", " << center.y << ") radius = " << radius << std::endl;

            cv::circle(cv_img_out_.image, center, 5, cv::Scalar(0,0,255), -1, 8, 0 );// circle center in green
            cv::circle(cv_img_out_.image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );// circle perimeter in red

            // Find the direction of the circle
            Eigen::Vector3d point;
            point[0] = center.x;
            point[1] = center.y;
            point[2] = 1;

            direction_ = matrixK_.inverse() * point;
        }
    }

    //reset input image
    cv_img_ptr_in_ = nullptr;
}

void RosImgProcessorNode::publishImage()
{
	if( !cv_img_out_.image.data ) return;

    cv_img_out_.header.seq ++;
    cv_img_out_.header.stamp = ros::Time::now();
    cv_img_out_.header.frame_id = "camera";
    cv_img_out_.encoding = img_encoding_;
    image_pub_.publish(cv_img_out_.toImageMsg());
}

void RosImgProcessorNode::publishMarker()
{
	if( !cv_img_out_.image.data) return; //to assure that we enter here when an image is available

	visualization_msgs::Marker marker_msg;
	std::ptrdiff_t idx;
	Eigen::Matrix3d rotation;

	//from vector direction to quaternion
	rotation.block<3,1>(0,0) = direction_;
	direction_.minCoeff(&idx);
	switch(idx)
	{
		case 0:
			rotation.block<3,1>(0,1) << 0,direction_(2),-direction_(1);
			break;
		case 1:
			rotation.block<3,1>(0,1) << -direction_(2),0,direction_(0);
			break;
		case 2:
			rotation.block<3,1>(0,1) << direction_(1),-direction_(0),0;
			break;
		default:
			break;
	}
	rotation.block<3,1>(0,2) = rotation.block<3,1>(0,0).cross(rotation.block<3,1>(0,1));
	rotation.block<3,1>(0,0) = rotation.block<3,1>(0,0)/rotation.block<3,1>(0,0).norm();
	rotation.block<3,1>(0,1) = rotation.block<3,1>(0,1)/rotation.block<3,1>(0,1).norm();
	rotation.block<3,1>(0,2) = rotation.block<3,1>(0,2)/rotation.block<3,1>(0,2).norm();
	//std::cout << "rotation: " << std::endl << rotation << std::endl;
	Eigen::Quaterniond quaternion(rotation);

	// fill the arrow message
	marker_msg.header.stamp = ros::Time::now();
	marker_msg.header.frame_id = "camera";
	marker_msg.ns = "direction_marker";
	marker_msg.id = 1;
	marker_msg.action = visualization_msgs::Marker::ADD;
	marker_msg.type = visualization_msgs::Marker::ARROW;
	marker_msg.pose.position.x = 0;
	marker_msg.pose.position.y = 0;
	marker_msg.pose.position.z = 0;
	marker_msg.pose.orientation.x = quaternion.x();
	marker_msg.pose.orientation.y = quaternion.y();
	marker_msg.pose.orientation.z = quaternion.z();
	marker_msg.pose.orientation.w = quaternion.w();
	marker_msg.scale.x = 0.8;
	marker_msg.scale.y = 0.02;
	marker_msg.scale.z = 0.02;
	marker_msg.color.r = 1.0;
	marker_msg.color.g = 0.0;
	marker_msg.color.b = 1.0;
	marker_msg.color.a = 1.0;
	marker_msg.lifetime = ros::Duration(0.2);

	//publish marker
	marker_publisher_.publish(marker_msg);
}

double RosImgProcessorNode::getRate() const
{
    return rate_;
}

void RosImgProcessorNode::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try
    {
        img_encoding_ = _msg->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
        return;
    }
}

void RosImgProcessorNode::cameraInfoCallback(const sensor_msgs::CameraInfo & _msg)
{
	matrixK_  << _msg.K[0],_msg.K[1],_msg.K[2],
                 _msg.K[3],_msg.K[4],_msg.K[5],
                 _msg.K[6],_msg.K[7],_msg.K[8];
	//std::cout << "matrixK: " << std::endl << matrixK_ << std::endl;
}
