/**
 * Calculate velocity from wheel encoders.
 */

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <svea_msgs/lli_encoder.h>

/** Constants */

double PI = 3.141592;
double TAU = 2 * PI;
double MILLI = 1e-3;
double MICRO = 1e-6;

/** Parameters */
//! TF frame id of the encoders (default 'base_link').
std::string encoder_frame;
//! Topic that encoder messages should be read from (default 'lli/encoder').
std::string encoder_topic;
//! Topic that  velocity will be advertised to (default 'sensor/encoder/velocity').
std::string velocity_topic;
//! Topic with twist messages used for calculating the direction (default '').
std::string actuation_topic;
//! Width between the wheels in mm (default 199.0).
double axle_track;
//! Radius of the wheels in mm (default 60.0).
double wheel_radius;
//! Encoder PPR (pulses per revolution) of a wheel (default 60).
int encoder_ppr;
//! Covariance of the linear velocity in the published twist messages (default 0.2).
double linear_covariance;
//! Covariance of the angular velocity in the published twist messages (default 0.4).
double angular_covariance;

/** Globals */

//! Advertises the velocity based on encoder resolution
ros::Publisher velocity_pub;
//! Listens to actuation in order to determine direction
ros::Subscriber actuation_sub;
//! Listens to encoder ppr
ros::Subscriber encoder_sub;
//! Current velocity state
geometry_msgs::TwistWithCovarianceStamped vel;
//! Current direction
int direction = 1;


/**
 * Callback to low-level encoder topic.
 */
void encoderCallback(const svea_msgs::lli_encoder::ConstPtr& msg)
{
  // diam [m/rev] / ppr [tick/rev] * x [tick] / time [s]
  // v [m/s] = [m/rev] * [rev/tick] * [tick] * [1/s]
  //         = [m/s]
  auto convert = [=](double x, double time){ return (TAU * wheel_radius) / encoder_ppr * x / time; };

  // Only one encoder working for now...
  // direction * (convert(msg.right_ticks) + convert(msg.left_ticks)) / 2;
  vel.twist.twist.linear.x = direction * convert(msg->right_ticks, msg->right_time_delta);
}

/**
 * Callback to actuation topic.
 */
void actuationCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
  direction = msg->twist.twist.linear.x > 0 ? 1 : -1;
}

int main(int argc, char** argv)
{
  // Initialize node, create handles and other resources

  ros::init(argc, argv, "wheel_encoder");

  ros::NodeHandle node;
  ros::NodeHandle priv("~");

  ros::TransportHints transportHints;
  transportHints.tcpNoDelay();

  // Get parameters

  encoder_frame = priv.param<std::string>("encoder_frame", "base_link");
  encoder_topic = priv.param<std::string>("encoder_topic", "lli/encoder");
  velocity_topic = priv.param<std::string>("velocity_topic", "sensor/encoder/velocity");
  actuation_topic = priv.param<std::string>("actuation_topic", "actuation_twist");
  axle_track = priv.param<double>("axle_track", 199.0);
  wheel_radius = priv.param<double>("wheel_radius", 60.0);
  encoder_ppr = priv.param<int>("encoder_ppr", 60);
  linear_covariance = priv.param<double>("linear_covariance", 0.2);
  angular_covariance = priv.param<double>("angular_covariance", 0.4);

  // Initialize velocity state

  vel.twist.covariance[0 + 0 * 6] = linear_covariance;
  vel.twist.covariance[1 + 1 * 6] = linear_covariance;
  vel.twist.covariance[2 + 2 * 6] = linear_covariance;
  vel.twist.covariance[3 + 3 * 6] = angular_covariance;
  vel.twist.covariance[4 + 4 * 6] = angular_covariance;
  vel.twist.covariance[5 + 5 * 6] = angular_covariance;

  vel.header.frame_id = encoder_frame;

  // Create publishers and subscribers

  velocity_pub = node.advertise<geometry_msgs::TwistWithCovarianceStamped>(velocity_topic, 1);
  actuation_sub = node.subscribe<geometry_msgs::TwistWithCovarianceStamped>(actuation_topic, 1, actuationCallback, transportHints);
  encoder_sub = node.subscribe<svea_msgs::lli_encoder>(encoder_topic, 1, encoderCallback, transportHints);

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
};

