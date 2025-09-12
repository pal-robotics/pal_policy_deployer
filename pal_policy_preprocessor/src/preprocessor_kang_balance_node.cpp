#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <algorithm> 
#include <array>

// ---------------------------------------------------------------------------------------------------
// -------------------- full body obs ---------------------------------------

static const std::vector<std::string> left_leg_order {
    "leg_left_1_joint","leg_left_2_joint","leg_left_3_joint",
    "leg_left_length_joint",
    "leg_left_4_joint","leg_left_5_joint"
};

static const std::vector<std::string> legs_only_order {
    "leg_left_1_joint", "leg_right_1_joint", 
    "leg_left_2_joint", "leg_right_2_joint", 
    "leg_left_3_joint", "leg_right_3_joint", 
    "leg_left_length_joint", "leg_right_length_joint",
    "leg_left_4_joint", "leg_right_4_joint",
    "leg_left_5_joint", "leg_right_5_joint"
};

static const std::vector<std::string> full_body_order {
  "pelvis_1_joint", 
  "leg_left_1_joint", "leg_right_1_joint", 
  "pelvis_2_joint", 
  "leg_left_2_joint", "leg_right_2_joint", 
  "arm_left_1_joint", "arm_right_1_joint", 
  "leg_left_3_joint", "leg_right_3_joint", 
  "arm_left_2_joint", "arm_right_2_joint", 
  "leg_left_length_joint", "leg_left_femur_joint", 
  "leg_right_length_joint", "leg_right_femur_joint", 
  "arm_left_3_joint", "arm_right_3_joint", 
  "leg_left_4_joint", "leg_left_knee_joint", 
  "leg_right_4_joint", "leg_right_knee_joint", 
  "arm_left_4_joint", "arm_right_4_joint", 
  "leg_left_5_joint", "leg_right_5_joint"
};

// offset in the order of joint_states joint names
static const std::vector<double> joint_pos_offset {
  0.0, 
  0.0, 0.0, 
  0.0, 
  0.05, 0.05, 
  0.24, -0.24, 
  0.0, 0.0, 
  1.32, 1.32, 
  0.6832, 1.1172, 
  0.6832, 1.1172, 
  1.57, -1.57, 
  -0.05, 2.2345, 
  -0.05, 2.2345, 
  0.8, 0.8, 
  0.0, 0.0
};

std::vector<double> obs(72, 0.0); // work on the size to be dynamic

std::vector<std::string> J;


// compute joint pos and joint vel
// (12 + 12 = 24) or 26 + 26 = 52
void jsCB(const sensor_msgs::JointState::ConstPtr& msg)
{
    // if (nanani) J = ...
    J = full_body_order;

    // for (size_t k = 0; k < J.size(); ++k)
    //     for (size_t i = 0; i < msg->name.size(); ++i)
    //         if (msg->name[i] == J[k]) {
    //             if (i < msg->position.size()) obs[k] = msg->position[i] - joint_pos_offset[i];
    //             ROS_INFO("at %zu it's %s, and its %zu in js", k, msg->name[i].c_str(), i);
    //             if (i < msg->velocity.size()) obs[J.size() + k] = msg->velocity[i];
    //         }


    // Override first 52 entries with default observations
    
    // left leg joints pos
    obs[1] = msg->position[8] - 0.0;      // leg left 1
    obs[4] = msg->position[9] - 0.05;     // leg left 2
    obs[8] = msg->position[10] - 0.0;     // leg left 3
    obs[12] = msg->position[15] - 0.6832; //leg left length
    obs[18] = msg->position[11] + 0.05;   // leg left 4
    obs[24] = msg->position[12] - 0.0;    //leg left 5

    obs[13] = msg->position[13] - 1.1172; // leg left femur
    obs[19] = msg->position[14] - 2.2345; // leg left knee

    // right leg joints pos
    obs[2] = msg->position[16] - 0.0;
    obs[5] = msg->position[17] - 0.05;
    obs[9] = msg->position[18] - 0.0;
    obs[14] = msg->position[23] - 0.6832; // tricky position in obs
    obs[20] = msg->position[19] + 0.05; // tricky position in obs
    obs[25] = msg->position[20] - 0.0;

    obs[15] = msg->position[21] - 1.1172; // leg right femur
    obs[21] = msg->position[22] - 2.2345; // leg right knee

    // pelvis joints pos
    obs[0] = msg->position[24]; // pelvis 1
    obs[3] = msg->position[25]; // pelvis 2




    // left leg joints pos
    // obs[1] = msg->position[8] - 0.0;      // leg left 1
    // ROS_INFO("obs[1] -> %s | msg->name[8] -> %s", full_body_order[1].c_str(), msg->name[8].c_str());

    // obs[4] = msg->position[9] - 0.05;     // leg left 2
    // ROS_INFO("obs[4] -> %s | msg->name[9] -> %s", full_body_order[4].c_str(), msg->name[9].c_str());

    // obs[8] = msg->position[10] - 0.0;     // leg left 3
    // ROS_INFO("obs[8] -> %s | msg->name[10] -> %s", full_body_order[8].c_str(), msg->name[10].c_str());

    // obs[12] = msg->position[15] - 0.6832; // leg left length
    // ROS_INFO("obs[12] -> %s | msg->name[15] -> %s", full_body_order[12].c_str(), msg->name[15].c_str());

    // obs[18] = msg->position[11] + 0.05;   // leg left 4
    // ROS_INFO("obs[18] -> %s | msg->name[11] -> %s", full_body_order[18].c_str(), msg->name[11].c_str());

    // obs[24] = msg->position[12] - 0.0;    // leg left 5
    // ROS_INFO("obs[24] -> %s | msg->name[12] -> %s", full_body_order[24].c_str(), msg->name[12].c_str());

    // obs[13] = msg->position[13] - 1.1172; // leg left femur
    // ROS_INFO("obs[13] -> %s | msg->name[13] -> %s", full_body_order[13].c_str(), msg->name[13].c_str());

    // obs[19] = msg->position[14] - 2.2345; // leg left knee
    // ROS_INFO("obs[19] -> %s | msg->name[14] -> %s", full_body_order[19].c_str(), msg->name[14].c_str());

    // // right leg joints pos
    // obs[2] = msg->position[16] - 0.0;
    // ROS_INFO("obs[2] -> %s | msg->name[16] -> %s", full_body_order[2].c_str(), msg->name[16].c_str());

    // obs[5] = msg->position[17] - 0.05;
    // ROS_INFO("obs[5] -> %s | msg->name[17] -> %s", full_body_order[5].c_str(), msg->name[17].c_str());

    // obs[9] = msg->position[18] - 0.0;
    // ROS_INFO("obs[9] -> %s | msg->name[18] -> %s", full_body_order[9].c_str(), msg->name[18].c_str());

    // obs[14] = msg->position[23] - 0.6832; // tricky position in obs
    // ROS_INFO("obs[14] -> %s | msg->name[23] -> %s", full_body_order[14].c_str(), msg->name[23].c_str());

    // obs[20] = msg->position[19] + 0.05; // tricky position in obs
    // ROS_INFO("obs[20] -> %s | msg->name[19] -> %s", full_body_order[20].c_str(), msg->name[19].c_str());

    // obs[25] = msg->position[20] - 0.0;
    // ROS_INFO("obs[25] -> %s | msg->name[20] -> %s", full_body_order[25].c_str(), msg->name[20].c_str());

    // obs[15] = msg->position[21] - 1.1172; // leg right femur
    // ROS_INFO("obs[15] -> %s | msg->name[21] -> %s", full_body_order[15].c_str(), msg->name[21].c_str());

    // obs[21] = msg->position[22] - 2.2345; // leg right knee
    // ROS_INFO("obs[21] -> %s | msg->name[22] -> %s", full_body_order[21].c_str(), msg->name[22].c_str());

    // // pelvis joints pos
    // obs[0] = msg->position[24]; // pelvis 1
    // ROS_INFO("obs[0] -> %s | msg->name[24] -> %s", full_body_order[0].c_str(), msg->name[24].c_str());

    // obs[3] = msg->position[25]; // pelvis 2
    // ROS_INFO("obs[3] -> %s | msg->name[25] -> %s", full_body_order[3].c_str(), msg->name[25].c_str());

    // arm
    // obs[6] = 0.24; 
    // obs[7] = -0.24; 
    // obs[10] = 1.32; 
    // obs[11] = 1.32; 
    // obs[16] = 1.57; 
    // obs[17] = -1.57; 
    // obs[22] = 0.8; 
    // obs[23] = 0.8; 

    // left leg joints vel
    obs[1 + 26] = msg->velocity[8];
    obs[4 + 26] = msg->velocity[9];
    obs[8 + 26] = msg->velocity[10];
    obs[12 + 26] = msg->velocity[15];
    obs[18 + 26] = msg->velocity[11];
    obs[24 + 26] = msg->velocity[12];

    obs[13 + 26] = msg->velocity[13]; // leg left femur
    obs[19 + 26] = msg->velocity[14]; // leg left knee

    // right leg joints vel
    obs[2 + 26] = msg->velocity[16];
    obs[5 + 26] = msg->velocity[17];
    obs[9 + 26] = msg->velocity[18];
    obs[14 + 26] = msg->velocity[23]; // tricky position in obs
    obs[20 + 26] = msg->velocity[19]; // tricky position in obs
    obs[25 + 26] = msg->velocity[20];

    obs[15 + 26] = msg->velocity[21]; // leg right femur
    obs[21 + 26] = msg->velocity[22]; // leg right knee

    // pelvis joints vel
    obs[0 + 26] = msg->position[24]; // pelvis 1
    obs[3 + 26] = msg->position[25]; // pelvis 2


}


// 14 joints positions
// (14)
void lastCB(const std_msgs::Float64MultiArray::ConstPtr& a)
{
  for (size_t i = 0; i < 14 && i < a->data.size(); ++i)
    obs[52 + i] = a->data[i];

}

// compute imu data
// (6)
void imuCB(const sensor_msgs::Imu::ConstPtr& msg)
{
    // ATTENTION: EQUIVALENCE ISAAC TO PAL PS

    // Y = X
    // X = Y
    // Z = -Z


    // orientation = msg->orientation; // x, y, z, w
    // angular_velocity = msg->angular_velocity; // x, y, z
    // linear_acceleration = msg->linear_acceleration; // x, y, z

    // base_ang_vel
    obs[66] = msg->angular_velocity.x;
    obs[67] = msg->angular_velocity.y;
    obs[68] = msg->angular_velocity.z;
    printf("Base angular velocity: x: %f, y: %f, z: %f\n", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // obs[66] = 1.2292e-01;
    // obs[67] = 8.3318e-02;
    // obs[68] = -7.0993e-02;


    // base_lin_acc
    obs[69] = msg->linear_acceleration.x;
    obs[70] = msg->linear_acceleration.y;
    obs[71] = msg->linear_acceleration.z;
    printf("Base linear acceleration: x: %f, y: %f, z: %f\n", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    // obs[69] = 3.7145e-01;
    // obs[70] = -1.6914e-01;
    // obs[71] = 1.0298e+01;

    // // projected gravity
    // tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // tf2::Matrix3x3 R(q);

    // // gravity in world frame
    // tf2::Vector3 g_world(0.0, 0.0, -9.81);

    // // projected gravity = R^T * g_world (i.e., gravity in base frame)
    // tf2::Vector3 g_proj = R.transpose() * g_world;

    // obs[70] = g_proj.x();
    // obs[71] = g_proj.y();
    // obs[72] = g_proj.z();
    // printf("Projected gravity: x: %f, y: %f, z: %f\n", g_proj.x(), g_proj.y(), g_proj.z());

    // obs[70] = 2.0341e-02;
    // obs[71] = -1.3970e-02;
    // obs[72] = -1.0497e+00;
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"onnx_input_pub"); ros::NodeHandle nh;

  ros::Subscriber s1 = nh.subscribe("/joint_states", 1, jsCB);
  ros::Subscriber s2 = nh.subscribe("/rl_policy/last_action", 1, lastCB);
  ros::Subscriber s3 = nh.subscribe("/base_imu", 1, imuCB);

  ros::Publisher  pub= nh.advertise<std_msgs::Float64MultiArray>("/rl_policy/obs", 1);

  ros::Rate r(50);
  while(ros::ok()){
    std_msgs::Float64MultiArray m; m.data = obs; pub.publish(m);
    ros::spinOnce(); r.sleep();
  }
}








// -1.4696e-02, -9.9247e-02, -2.4337e-01,
// -3.8004e-01,  3.9041e-02,  4.4693e-02, 
// -1.4259e-01, -1.0743e-01, 2.9879e-02,  
// 1.2574e-01, -4.3608e-02,  1.1433e-02,  


// 1.2292e-01, 8.3318e-02, -7.0993e-02,  

// 3.7145e-01, -1.6914e-01,  1.0298e+01
