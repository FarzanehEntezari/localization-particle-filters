#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>

#define METRE_TO_PIXEL_SCALE 50
#define FORWARD_SWIM_SPEED_SCALING 0.2
#define POSITION_GRAPHIC_RADIUS 20.0
#define HEADING_GRAPHIC_LENGTH 50.0

class GroundTruthPublisher {
public:
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  ros::Subscriber ground_truth_sub;
  ros::Subscriber estimate_sub;
  geometry_msgs::PoseStamped ground_truth_location;

  cv::Mat map_image;
  cv::Mat drawing_image;

  std::vector<geometry_msgs::PoseStamped> gt_traj;

  ros::Publisher gt_array_pub;
  geometry_msgs::PoseArray gt_pose_array_msg;
  ros::Publisher estimate_array_pub;
  geometry_msgs::PoseArray estimate_pose_array_msg;


  double total_error;


  GroundTruthPublisher( int argc, char** argv ) : total_error(0.0){

    image_transport::ImageTransport it(nh);
    pub = it.advertise("/assign1/result_image", 1);

    std::string ag_path = ros::package::getPath("aqua_gazebo");
    map_image = cv::imread((ag_path+"/materials/fishermans_small.png").c_str(), CV_LOAD_IMAGE_COLOR);
    drawing_image = map_image.clone();

    ground_truth_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &GroundTruthPublisher::groundTruthCallback, this);
    estimate_sub = nh.subscribe<geometry_msgs::PoseStamped>("/assign1/localization_estimate", 1, &GroundTruthPublisher::locationEstimateCallback, this);

    gt_array_pub = nh.advertise<geometry_msgs::PoseArray>( "/assign1/gt_trajectory",1);
    estimate_array_pub = nh.advertise<geometry_msgs::PoseArray>( "/assign1/estimate_trajectory",1);
   
    ROS_INFO( "Ground truth publisher constructed. Waiting for model state information." );
  }

  // Function groundTruthCallback is executed automatically each time gazebo updates the robot's pose
  void groundTruthCallback( const gazebo_msgs::ModelStates::ConstPtr& ground_truth_state ){

    std::string robot_name = "aqua";

    int index = -1;
    for( unsigned int i=0; i<ground_truth_state->name.size(); i++ ){
      if( ground_truth_state->name[i] == robot_name ){
        index = i;
        break;
      }
    }

    if( index < 0 ){
      ROS_ERROR( "Unable to find aqua model information in gazebo message. I hope you never see this.");
      return;
    }

    ground_truth_location.header.stamp = ros::Time::now();
    ground_truth_location.pose = ground_truth_state->pose[index];

    gt_traj.push_back( ground_truth_location );

    //ground truth pose array
    double gt_yaw, gt_pitch, gt_roll;
    tf::Quaternion gt_orientation;
    tf::quaternionMsgToTF(ground_truth_location.pose.orientation, gt_orientation);
    tf::Matrix3x3(gt_orientation).getEulerYPR( gt_yaw, gt_pitch, gt_roll );


    geometry_msgs::Pose pose = ground_truth_location.pose;
    pose.position.z = 0;

    int gt_poses_size = gt_pose_array_msg.poses.size();
    if (gt_poses_size > 0)
    {
      geometry_msgs::Point current_position = pose.position;
      geometry_msgs::Pose last_pose = gt_pose_array_msg.poses[gt_pose_array_msg.poses.size() - 1];
      geometry_msgs::Point last_position = last_pose.position;
      float dist = sqrt(pow(current_position.x-last_position.x, 2) + pow(current_position.y-last_position.y, 2) + pow(current_position.z-last_position.z, 2));
      if (dist > 0.1)
      {
          gt_pose_array_msg.poses.push_back(pose);
      }
    }
    else
    {
      gt_pose_array_msg.poses.push_back(pose);
    }
  }

  // Function locationEstimateCallback is executed automatically when poses are published by your assignment code,
  // It computes the error between your pose and the ground truth and plots your estimate on the output image.
  void locationEstimateCallback( const geometry_msgs::PoseStamped::ConstPtr& estimated_state ){

	  // It can be useful to un-comment this if you have connectivity problems with the localizer
    //ROS_INFO( "Got location estimate callback." );

	  double junk_r, junk_p,estimated_yaw, gt_yaw;
    tf::Quaternion tf_q;

    tf::quaternionMsgToTF(ground_truth_location.pose.orientation, tf_q );
	  tf::Matrix3x3(tf_q).getEulerYPR( gt_yaw, junk_p, junk_r );

	  tf::quaternionMsgToTF(estimated_state->pose.orientation, tf_q );
    tf::Matrix3x3(tf_q).getEulerYPR( estimated_yaw, junk_p, junk_r );

    double current_error = pow(estimated_state->pose.position.x-ground_truth_location.pose.position.x,2) +
    		               pow(estimated_state->pose.position.y+ground_truth_location.pose.position.y,2);

    total_error += current_error;

    printf( "\nESTIMATE: [x, y, yaw]=[%f %f %f]\nGR TRUTH: [x, y, yaw]=[%f %f %f]\nCUR ERROR: %f\nTOT ERROR: %f\n\n",
    		estimated_state->pose.position.x, estimated_state->pose.position.y, estimated_yaw,
			ground_truth_location.pose.position.x, -ground_truth_location.pose.position.y, gt_yaw,
			current_error, total_error);

    int estimated_robo_image_x = drawing_image.size().width/2 + METRE_TO_PIXEL_SCALE * estimated_state->pose.position.x;
    int estimated_robo_image_y = drawing_image.size().height/2 + METRE_TO_PIXEL_SCALE * estimated_state->pose.position.y;

    int estimated_heading_image_x = estimated_robo_image_x + HEADING_GRAPHIC_LENGTH * cos(-estimated_yaw);
    int estimated_heading_image_y = estimated_robo_image_y + HEADING_GRAPHIC_LENGTH * sin(-estimated_yaw);

    cv::circle( drawing_image, cv::Point(estimated_robo_image_x, estimated_robo_image_y), POSITION_GRAPHIC_RADIUS, CV_RGB(250,0,0), -1);
    cv::line( drawing_image, cv::Point(estimated_robo_image_x, estimated_robo_image_y), cv::Point(estimated_heading_image_x, estimated_heading_image_y), CV_RGB(250,0,0), 10);

    //estimate pose array
    geometry_msgs::Pose pose = estimated_state->pose;
    pose.position.z = 0;
    pose.position.y = -pose.position.y;

    int estimate_poses_size = estimate_pose_array_msg.poses.size();
    if (estimate_poses_size > 0)
    {
      geometry_msgs::Point current_position = pose.position;
      geometry_msgs::Pose last_pose = estimate_pose_array_msg.poses[estimate_pose_array_msg.poses.size() - 1];
      geometry_msgs::Point last_position = last_pose.position;
      float dist = sqrt(pow(current_position.x-last_position.x, 2) + pow(current_position.y-last_position.y, 2) + pow(current_position.z-last_position.z, 2));
      if (dist > 0.1)
      {
          estimate_pose_array_msg.poses.push_back(pose);
      }
    }
    else
    {
      estimate_pose_array_msg.poses.push_back(pose);
    }
  }

  // Spin as long as the process exists drawing the ground truth trajectory as the top layer on the
  // output image and publishing it to ROS for viewing.
  void spin(){

    ros::Rate loop_rate(5);
    while (nh.ok()) {
      for( std::vector<geometry_msgs::PoseStamped>::const_iterator gt_it = gt_traj.begin(); gt_it != gt_traj.end(); gt_it++ ){
    	  ground_truth_location = *gt_it;
		  int ground_truth_robo_image_x = drawing_image.size().width/2 + METRE_TO_PIXEL_SCALE * ground_truth_location.pose.position.x;
		  int ground_truth_robo_image_y = drawing_image.size().height/2 - METRE_TO_PIXEL_SCALE * ground_truth_location.pose.position.y;

		  double gt_yaw, gt_pitch, gt_roll;
		  tf::Quaternion gt_orientation;
		  tf::quaternionMsgToTF(ground_truth_location.pose.orientation, gt_orientation);
		  tf::Matrix3x3(gt_orientation).getEulerYPR( gt_yaw, gt_pitch, gt_roll );

		  int ground_truth_heading_image_x = ground_truth_robo_image_x + HEADING_GRAPHIC_LENGTH * cos(-gt_yaw);
		  int ground_truth_heading_image_y = ground_truth_robo_image_y + HEADING_GRAPHIC_LENGTH * sin(-gt_yaw);

		  cv::circle( drawing_image, cv::Point(ground_truth_robo_image_x, ground_truth_robo_image_y), POSITION_GRAPHIC_RADIUS, CV_RGB(0,0,250), -1);
		  cv::line( drawing_image, cv::Point(ground_truth_robo_image_x, ground_truth_robo_image_y), cv::Point(ground_truth_heading_image_x, ground_truth_heading_image_y), CV_RGB(0,0,250), 10);
      }

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing_image).toImageMsg();
	    pub.publish(msg);

      gt_pose_array_msg.header.frame_id = "/aqua_base";
      gt_pose_array_msg.header.stamp = ros::Time::now();
      gt_array_pub.publish(gt_pose_array_msg);

      estimate_pose_array_msg.header.frame_id = "/aqua_base";
      estimate_pose_array_msg.header.stamp = ros::Time::now();
      estimate_array_pub.publish(estimate_pose_array_msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char** argv){

  ros::init(argc, argv, "ground_truth_publisher");
  GroundTruthPublisher gtp(argc, argv);
  gtp.spin();
}
