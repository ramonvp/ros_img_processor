#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "ros_img_processor/CircleDetectionStamped.h"

#define S2(X) (X*X)

// A Kalman Filter implementation for tracking a circle inside an image
class Tracker
{
public:

  enum StateMembers {
      X = 0, // x position of the circle [pixels]
      Y,     // y position of the circle [pixels]
      Vx,    // x speed [pixels/s]
      Vy,    // y speed [pixels/s]
      R      // radius of the detected circle [pixels]
  };

  enum MeasMembers {
      Zx = 0, // x position of the circle [pixels]
      Zy,     // y position of the circle [pixels]
      Zr      // radius of the detected circle [pixels]
  };

  enum Status {
    pending,
    predicting,
    normal
  };

  Tracker() :
    dt_(0.01)
  , x_(5)
  , z_(3)
  , Cx_(5,5)
  , Cnx_(5,5)
  , Cnz_(3,3)
  , F_(5,5)
  , H_(3,5)
  , waiting_(true)
  , nh_(ros::this_node::getName())
  , img_tp_(nh_)
  {

      F_ << 1,  0, dt_,  0,  0,
            0,  1,  0 , dt_, 0,
            0,  0,  1,   0,  0,
            0,  0,  0,   1,  0,
            0,  0,  0,   0,  1;

      H_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 0, 0, 1;

      Cnz_ << 1,  0,  0,
              0,  1,  0,
              0,  0,  4;

      Cnx_ << S2(10), 0,    0,    0,      0,
              0,    S2(10), 0,    0,      0,
              0,      0, S2(20),  0,      0,
              0,      0,     0,  S2(20),  0,
              0,      0,     0,   0,    S2(10);

      image_subs_ = img_tp_.subscribe("image_in", 1, &Tracker::imageCallback, this);
      image_pub_ = img_tp_.advertise("image_out", 100);

      detector_output_subs_ = nh_.subscribe("detector_in", 1, &Tracker::detectorCallback, this);
  }

  void reset()
  {
      waiting_ = true;
      x_ << 0,0,0,0,0;
      Cx_ << S2(1), 0,    0,    0,     0,
              0,   S2(1), 0,    0,     0,
              0,    0,  S2(10), 0,     0,
              0,    0,    0,   S2(10), 0,
              0,    0,    0,    0,   S2(5);
  }

  void predict()
  {
      double elapsed = (ros::Time::now() - last_output_).toSec();

      F_ << 1,  0, elapsed,  0,  0,
            0,  1,  0 , elapsed, 0,
            0,  0,  1,    0,     0,
            0,  0,  0,    1,     0,
            0,  0,  0,    0,     1;

      x_ = F_*x_;
      Cx_ = F_*Cx_*F_.transpose() + Cnx_;
  }

  void update()
  {
      // compute the expected measurement
      Eigen::VectorXd z_exp = H_*x_;

      // compute the K gain
      Eigen::MatrixXd K = Cx_*H_.transpose()*(H_*Cx_*H_.transpose()+Cnz_).inverse();

      Eigen::MatrixXd I = Eigen::Matrix<double, 5, 5>::Identity();
      // update state mean and covariance
      x_ = x_ + K*(z_-z_exp);
      Cx_ = (I-K*H_)*Cx_*(I-K*H_).transpose() + K*Cnz_*K.transpose();
  }

  void publishImage()
  {
    if( !cv_img_out_.image.data ) return;

      cv_img_out_.header.seq ++;
      cv_img_out_.header.stamp = ros::Time::now();
      cv_img_out_.header.frame_id = "camera";
      cv_img_out_.encoding = img_encoding_;
      image_pub_.publish(cv_img_out_.toImageMsg());
  }

  void drawResult()
  {
      if ( cv_img_ptr_in_ != nullptr )
      {
          // copy the input image to the out one
          cv_img_out_.image = cv_img_ptr_in_->image;

          if(!waiting_) {
              int x = static_cast<int>(x_[StateMembers::X]);
              int y = static_cast<int>(x_[StateMembers::Y]);
              int radius = static_cast<int>(x_[StateMembers::R]);

              cv::Point center = cv::Point(cvRound(x), cvRound(y));

              cv::circle(cv_img_out_.image, center, 5, cv::Scalar(255,255,0), -1, 8, 0 );
              cv::circle(cv_img_out_.image, center, radius+8, cv::Scalar(255,255,0), 3, 4, 0 );
          }
      }
  }

  double loop_time() const { return dt_; }

  bool is_valid() const { return !waiting_; }


  void try_predict()
  {
      if( !is_valid() )
          return;

      ros::Time currentTime = ros::Time::now();
      double elapsed = (currentTime - last_output_).toSec();
      if( elapsed > dt_ )
      {
          predict();
          last_output_ = currentTime;
      }
  }

private:

  void detectorCallback(const ros_img_processor::CircleDetectionStampedConstPtr& msg)
  {
      if(waiting_)
      {
          x_[StateMembers::X] = msg->circle.x;
          x_[StateMembers::Y] = msg->circle.y;
          x_[StateMembers::Vx] = 0;
          x_[StateMembers::Vy] = 0;
          x_[StateMembers::R] = msg->circle.radius;
          last_output_ = msg->header.stamp;
          waiting_ = false;
          return;
      }

      z_[Zx] = msg->circle.x;
      z_[Zy] = msg->circle.y;
      z_[Zr] = msg->circle.radius;
      predict();
      update();
      last_output_ = msg->header.stamp;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
      try
      {
          img_encoding_ = msg->encoding;
          cv_img_ptr_in_ = cv_bridge::toCvCopy(msg, msg->encoding);//get image
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("Tracker::image_callback(): cv_bridge exception: %s", e.what());
          return;
      }
  }


  double dt_;          // loop frequency of the filter
  Eigen::VectorXd x_;  // state variable
  Eigen::VectorXd z_;  // measurement variable

  Eigen::MatrixXd Cx_;  // state covariance
  Eigen::MatrixXd Cnx_; // covariance of prediction
  Eigen::MatrixXd Cnz_; // covariance of measurement

  Eigen::MatrixXd F_;   // prediction model
  Eigen::MatrixXd H_;   // estimation model

  bool waiting_;  // waiting for first measurement to arrive

  //pointer to received (in) and published (out) images
  cv_bridge::CvImagePtr cv_img_ptr_in_;
  cv_bridge::CvImage cv_img_out_;

  //ros node handle
  ros::NodeHandle nh_;

  //image encoding label
  std::string img_encoding_;

  //image transport
  image_transport::ImageTransport img_tp_;

  // subscribers
  image_transport::Subscriber image_subs_;
  ros::Subscriber detector_output_subs_;

  image_transport::Publisher image_pub_;

  ros::Time last_output_;
};

int main(int argc, char * argv[])
{
  //init ros
  ros::init(argc, argv, "kf_tracker");

  Tracker tracker;

  //set node loop rate
  ros::Rate loopRate(1.0/tracker.loop_time());

  ROS_INFO("Kalman Filter tracker ready");

  //node loop
  while ( ros::ok() )
  {
      //execute pending callbacks
      ros::spinOnce();

      //do prediction
      tracker.try_predict();
      tracker.drawResult();
      tracker.publishImage();

      //relax to fit output rate
      loopRate.sleep();
  }

  return 0;
}
