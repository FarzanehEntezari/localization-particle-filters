#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core.hpp>
#include <random>
#include <math.h>
#include <stdio.h>
#include "opencv2/calib3d/calib3d.hpp"


#define _USE_MATH_DEFINES

#define N 3000
#define METRE_TO_PIXEL_SCALE 50
#define FORWARD_SWIM_SPEED_SCALING 0.1
#define POSITION_GRAPHIC_RADIUS 20.0
#define HEADING_GRAPHIC_LENGTH 50.0

using namespace cv;
using namespace cv::xfeatures2d;


struct particle{

int x;
int y;
double o;
double w;

};




struct position{

int x;
int y;
double o;

};


double gaussian(double mean, double stddev, double x) 
{ 
    double variance2 = stddev*stddev*2.0; 
    double term = x-mean; 
    return exp(-(term*term)/variance2)/sqrt(M_PI*variance2); 
}




double correct(double i, double r)   
{
  double c;
  c = 2 * M_PI - i;
  if (abs(r - c) > abs(r - i))
  {return i;  }
  else
  {return c;  }
}



struct particle match_view(const sensor_msgs::ImageConstPtr &robot_img, Mat map_image) 
{
  struct particle result;

  cv::Mat robot_sensed = cv_bridge::toCvCopy(robot_img, sensor_msgs::image_encodings::BGR8)->image;
 
  // from opencv documnetation: 
  // https://docs.opencv.org/3.1.0/d5/d6f/tutorial_feature_flann_matcher.html
  //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
  int minHessian = 400;
  Ptr<SURF> detector = SURF::create(minHessian);

  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  Mat descriptors1, descriptors2;

  detector->detectAndCompute(robot_sensed, Mat(), keypoints1, descriptors1);
  detector->detectAndCompute(map_image, Mat(), keypoints2, descriptors2);

  //-- Step 2: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector<DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for (int i = 0; i < descriptors1.rows; i++)
  {
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  //-- Draw only "good" matches (i.e. whose distance is less than 4*min_dist )
  std::vector<cv::DMatch> good_matches;

  for (int i = 0; i < descriptors1.rows; i++)
  {
    if (matches[i].distance <= max(4*min_dist,0.02) )
    {good_matches.push_back(matches[i]);    }
  }

  std::vector<cv::Point2f> rob_pts;
  std::vector<cv::Point2f> map_pts;


  for (int i = 0; i < good_matches.size(); i++)
  {
    //-- Get the keypoints from the good matches
    rob_pts.push_back(keypoints1[good_matches[i].queryIdx].pt);
    map_pts.push_back(keypoints2[good_matches[i].trainIdx].pt);
  }

  Mat H = findHomography(rob_pts, map_pts, CV_RANSAC);

  if (H.empty())
  {
    result.x = -1;
    result.y = -1;
    result.o = 0;
    return result;
  }

  //-- Get the corners from the image_1 
  std::vector<cv::Point2f> obj_corners(4);
  corners[0] = cvPoint(0, 0);
  corners[1] = cvPoint(robot_sensed.cols, 0);
  corners[2] = cvPoint(robot_sensed.cols, robot_sensed.rows);
  corners[3] = cvPoint(0, robot_sensed.rows);
  std::vector<cv::Point2f> scene_corners(4);

  perspectiveTransform(corners, match_corners, H);

  result.x = match_corners[0].x + match_corners[1].x + match_corners[2].x + match_corners[3].x;
  result.x /= 4;
  result.y = match_corners[0].y + match_corners[1].y + match_corners[2].y + match_corners[3].y;
  result.y /= 4;                                                                                                                       

  double a, b;
  a = H.at<double>(0, 0);
  b = H.at<double>(0, 1);

  result.o = fmod(atan2(a, b) + M_PI , 2*M_PI);
  result.w = 1;

  return result;
}




class Localizer {

public:
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  image_transport::Subscriber gt_img_sub;
  image_transport::Subscriber robot_img_sub;
  ros::Publisher estimate_pub;

  ros::Subscriber motion_command_sub;

  geometry_msgs::PoseStamped estimated_location;

  cv::Mat map_image;
  cv::Mat localization_result_image;

  std::vector<struct particle> all_particles;

  Localizer( int argc, char** argv ){

    image_transport::ImageTransport it(nh);
    pub = it.advertise("/assign1/localization_debug_image", 1);
    estimate_pub = nh.advertise<geometry_msgs::PoseStamped>( "/assign1/localization_estimate",1);
    std::string ag_path = ros::package::getPath("aqua_gazebo");
    map_image = cv::imread((ag_path+"/materials/fishermans_small.png").c_str(), CV_LOAD_IMAGE_COLOR);

    estimated_location.pose.position.x = 0;
    estimated_location.pose.position.y = 0;

    localization_result_image = map_image.clone();

    robot_img_sub = it.subscribe("/aqua/back_down/image_raw", 1, &Localizer::robotImageCallback, this);
    motion_command_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aqua/target_pose", 1, &Localizer::motionCommandCallback, this);

    ROS_INFO( "localizer node constructed and subscribed." );



    // initial particles uniformly distributed all over the image 
    for(int i=0; i<N; i++){
    
      particle init={
         rand() % map_image.size().width ,
         rand() % map_image.size().height ,
         ( (double)rand() / RAND_MAX ) * 2 * M_PI ,
         1.0f / N       
      };

    all_particles.push_back(init);

    }


  }

  void robotImageCallback( const sensor_msgs::ImageConstPtr& robot_img ){


    struct particle pose;
    pose = match_view(robot_img, map_image);

    ROS_INFO("\n pose from sensor: x= %d  y=%d   o=%f" , pose.x , pose.y , pose.o);
    //fprintf(stderr,"\n stderr ouput pose from sensor: x= %d  y=%d   o=%f" , pose.x , pose.y , pose.o);

    if (pose.x != -1){ // robot view did not match any part of the map

    // update weights x
    for (int i=0; i<N ; i++){
       all_particles[i].w = gaussian(pose.x , 1 , all_particles[i].x);
    }

    long double sum_w = 0.0;
    for (int i=0; i<N ; i++){
       sum_w += all_particles[i].w;
    }

    for(int i=0; i<N ; i++){
       all_particles[i].w = all_particles[i].w/sum_w;
    }

    //update weights y
    for (int i=0; i<N ; i++){
       all_particles[i].w *= gaussian(pose.y , 1 , all_particles[i].y);
    }    

    sum_w = 0.0;
    for (int i=0; i<N ; i++){
       sum_w += all_particles[i].w;
    }

    for(int i=0; i<N ; i++){
       all_particles[i].w = all_particles[i].w/sum_w;
    }

    ///update weights orient

    double circularOrient;
    for (int i=0; i<N ; i++){
       circularOrient = correct(all_particles[i].o, pose.o);
       all_particles[i].w *= gaussian(pose.o , 0.1 , circularOrient);
    }    

    sum_w = 0.0;
    for (int i=0; i<N ; i++){
       sum_w += all_particles[i].w;
    }

    for(int i=0; i<N ; i++){
       all_particles[i].w = all_particles[i].w/sum_w;
    }

    // resample particles
    // based on the first method described in https://www.youtube.com/watch?v=DhxRxG5bSrg
    std::vector<struct particle> resampled_particles;
    double w_i,w_sum;
    int j;

    for (int i ; i<N ; i++){
       w_i = ( (double)rand() / RAND_MAX ) ;
       w_sum =0 ;
       j=0;
       while(w_i > w_sum){
          w_sum += all_particles[j].w ;
          j ++ ;
       }
       resampled_particles.push_back(all_particles[j-1]);
    }
    all_particles = resampled_particles; 
    
    }   //// if( != -1) 



  }

  // Function motionCommandCallback is a example of how to work with Aqua's motion commands (your view on the odometry).
  // The initial version simply integrates the commands over time to make a very rough location estimate.
  // TODO: You must improve it to work with the localizer you implement.
  //
  // Note the somewhat unique meaning of fields in motion_command
  //    motion_command
  //      pose
  //        position
  //          x - requested forward swim effort in a unitless number ranging from 0.0 to 1.0. You must translate this into
  //              a velocity estimate in some way. Currently a simple constant is used.
  //          y - requested up/down swim effort. Not used in this assignment
  //          z - unused
  //        orientation - A quaternion that represents the desired body orientation w.r.t. global frame. Note that
  //                      Aqua's controller will not achieve this orientation perfectly, so you must consider that
  //                      this is only a noisy estimate of the robot's orientation (correct for it with your filter!)
  //
  // Note that we use negative angles because the geometry of the map image is formed with its Z axis pointing downwards
  // (the camera is looking at the ocean floor)
  //
  void motionCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& motion_command ){

    geometry_msgs::PoseStamped command = *motion_command;
    double target_roll, target_pitch, target_yaw;
    tf::Quaternion target_orientation;
    tf::quaternionMsgToTF(command.pose.orientation, target_orientation);
    tf::Matrix3x3(target_orientation).getEulerYPR( target_yaw, target_pitch, target_roll );

    std::default_random_engine generator;
    std::normal_distribution<double> noise_x(0.0, 1);
    std::normal_distribution<double> noise_y(0.0, 1);
    std::normal_distribution<double> noise_o(0.0, 0.1);

    
    // move particles ahead and add noise 
    for (int i = 0; i < N ; i++)
    {
      all_particles[i].o = fmod(-target_yaw + noise_o(generator) , 2* M_PI);  
      all_particles[i].x = all_particles[i].x + METRE_TO_PIXEL_SCALE * FORWARD_SWIM_SPEED_SCALING * command.pose.position.x * cos(all_particles[i].o) + noise_x(generator)  ;
      all_particles[i].y =  all_particles[i].y + METRE_TO_PIXEL_SCALE * FORWARD_SWIM_SPEED_SCALING * command.pose.position.x * sin(all_particles[i].o) + noise_y(generator) ; 

    } 
    ROS_INFO("METRE_TO_PIXEL_SCALE * FORWARD_SWIM_SPEED_SCALING * command.pose.position.x = %f" , METRE_TO_PIXEL_SCALE * FORWARD_SWIM_SPEED_SCALING * command.pose.position.x);

    double w=-10;  
    int index_best ; //all_particles[index_best] is the most probable location!
    for (int i=0; i<N ; i++){

       if(w< all_particles[i].w)
       index_best = i;
       w= all_particles[i].w;
    }  

    // The following three lines implement the basic motion model example
    //estimated_location.pose.position.x = estimated_location.pose.position.x + FORWARD_SWIM_SPEED_SCALING * command.pose.position.x * cos( -target_yaw );
    //estimated_location.pose.position.y = estimated_location.pose.position.y + FORWARD_SWIM_SPEED_SCALING * command.pose.position.x * sin( -target_yaw );
    //estimated_location.pose.orientation = command.pose.orientation;

    estimated_location.pose.position.x = (all_particles[index_best].x - localization_result_image.size().width / 2) * (double(1) / METRE_TO_PIXEL_SCALE);  
    estimated_location.pose.position.y = (all_particles[index_best].y - localization_result_image.size().height / 2) * (double(1) / METRE_TO_PIXEL_SCALE);
    estimated_location.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0f, 0.0f, -all_particles[index_best].o);




    // This line resets the image to the original map so you can start drawing fresh each time.
    localization_result_image = map_image.clone();

    int estimated_robo_image_x = localization_result_image.size().width/2 + METRE_TO_PIXEL_SCALE * estimated_location.pose.position.x;
    int estimated_robo_image_y = localization_result_image.size().height/2 + METRE_TO_PIXEL_SCALE * estimated_location.pose.position.y;

    int estimated_heading_image_x = estimated_robo_image_x + HEADING_GRAPHIC_LENGTH * cos(-target_yaw);
    int estimated_heading_image_y = estimated_robo_image_y + HEADING_GRAPHIC_LENGTH * sin(-target_yaw);

    cv::circle( localization_result_image, cv::Point(estimated_robo_image_x, estimated_robo_image_y), POSITION_GRAPHIC_RADIUS, CV_RGB(250,0,0), -1);
    cv::line( localization_result_image, cv::Point(estimated_robo_image_x, estimated_robo_image_y), cv::Point(estimated_heading_image_x, estimated_heading_image_y), CV_RGB(250,0,0), 10);

    estimate_pub.publish( estimated_location );
    //printf(estimated_location);
    
  }

  // This function publishes your localization result image and spins ROS to execute its callbacks
  void spin(){

    ros::Rate loop_rate(30);
    while (nh.ok()) {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", localization_result_image).toImageMsg();
      pub.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char** argv){

  ros::init(argc, argv, "localizer");
  Localizer my_loc(argc, argv);
  my_loc.spin();
}
