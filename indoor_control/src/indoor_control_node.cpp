#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //for px4's external pose estimate
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::PoseStamped  zed_pose;
geometry_msgs::TransformStamped  fakegps_pose;

ros::Publisher px4_external_pose_estimate;
ros::Publisher fakegps_pose_estimate;

double roll_cam = 0, pitch_cam = 0, yaw_cam = 1.5707963, gamma_world = -1.5707963;

void zedposeHandler(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  static tf::Vector3 position_orig, position_body;
  tf::StampedTransform transform;
  tf::Quaternion q(pose->pose.orientation.x,pose->pose.orientation.y,pose->pose.orientation.z,pose->pose.orientation.w);
  tf::Vector3 origin(pose->pose.position.x,pose->pose.position.y,pose->pose.position.z);
  transform.setOrigin(origin); 
  transform.setRotation(q);
  
  static tf::Quaternion quat_cam, quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z, quat_rot_z, quat_body;

  // 1) Rotation from original world frame to world frame with y forward.
  // See the full rotation matrix at https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
  position_orig = transform.getOrigin();

  position_body.setX( cos(gamma_world) * position_orig.getX() + sin(gamma_world) * position_orig.getY());
  position_body.setY(-sin(gamma_world) * position_orig.getX() + cos(gamma_world) * position_orig.getY());
  position_body.setZ(position_orig.getZ());

  // 2) Rotation from camera to body frame.
  quat_cam = transform.getRotation();

  quat_cam_to_body_x = tf::createQuaternionFromRPY(roll_cam, 0, 0);
  quat_cam_to_body_y = tf::createQuaternionFromRPY(0, pitch_cam, 0);
  quat_cam_to_body_z = tf::createQuaternionFromRPY(0, 0, yaw_cam);

  // 3) Rotate body frame 90 degree (align body x with world y at launch)
  quat_rot_z = tf::createQuaternionFromRPY(0, 0, -gamma_world);

  quat_body = quat_rot_z * quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
  quat_body.normalize();

//   fakegps_pose.transform.translation.x = position_body.getX();
//   fakegps_pose.transform.translation.y = position_body.getY();
//   fakegps_pose.transform.translation.z = position_body.getZ();
//   fakegps_pose.transform.rotation.x = quat_body.getX();
//   fakegps_pose.transform.rotation.y = quat_body.getY();
//   fakegps_pose.transform.rotation.z = quat_body.getZ();
//   fakegps_pose.transform.rotation.w = quat_body.getW();
  

  fakegps_pose.transform.translation.x = -pose->pose.position.x;
  fakegps_pose.transform.translation.y = -pose->pose.position.y;
  fakegps_pose.transform.translation.z = pose->pose.position.z;
  fakegps_pose.transform.rotation.x = pose->pose.orientation.x;
  fakegps_pose.transform.rotation.y = pose->pose.orientation.y;
  fakegps_pose.transform.rotation.z = pose->pose.orientation.z;
  fakegps_pose.transform.rotation.w = pose->pose.orientation.w;

// Publish pose of body frame in world frame
  fakegps_pose_estimate.publish(fakegps_pose);
  
  
}


int main(int argc, char **argv) {
    

    ros::init(argc, argv, "indoor_control_node");
    ros::NodeHandle nh;

    //NOTE For px4's external pose estimate
    px4_external_pose_estimate = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",100);
    fakegps_pose_estimate = nh.advertise<geometry_msgs::TransformStamped>("/mavros/fake_gps/mocap/tf",100);

    ros::Subscriber zed_pose_sub = nh.subscribe<geometry_msgs::PoseStamped> ("/zed/zed_node/pose", 1000, zedposeHandler);
    
    
    ros::spin();
    
    ros::Rate r(30); // 10 hz
    while (ros::ok())
    {

      r.sleep();
      ros::spinOnce();
    }

    
    return 0;
}
