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

std::vector<double> obs(77, 0.0); // work on the size to be dynamic

std::vector<std::string> J;

// Default first 52 observations
static const std::array<double, 52> default_obs = {{
  -4.8093e-03, -1.1073e-01,  1.8107e-02, -4.5884e-03,  2.6222e-01,
  5.1888e-01, -5.1660e-03, -5.9444e-03, -1.2422e-02,  1.0425e-02,
  -2.4786e-03,  2.8573e-02, -1.7757e-01, -3.7888e-01, -3.5333e-01,
  -6.5049e-01, -5.5669e-03, -2.1682e-03, -1.7941e-01, -7.5274e-01,
  2.8351e-01, -1.3054e+00,  7.8863e-04, -7.4411e-03, -1.7731e-01,
  -4.1880e-01,  

  5.7644e-01,  1.2059e+00,  3.6580e-01,  4.5978e-01, -1.0608e+00, 
  -1.1098e+00,  1.1494e+00, -9.2570e-02, -9.9428e-01, 9.6201e-01, 
  -1.0529e+00, -1.4043e+00,  2.7441e-01,  5.0558e-01, 8.2466e-01, 
  -1.4077e+00,  7.3865e-02, -1.2732e-01,  9.6142e-01, -4.0938e-01,  
  2.5792e-01, -6.3345e-01,  3.6401e-01, -1.3355e+00, 8.2380e-01,  
  4.2572e-01, 

}};


// compute joint pos and joint vel
// (12 + 12 = 24) or 26 + 26 = 52
void jsCB(const sensor_msgs::JointState::ConstPtr& msg)
{
    // J = full_body_order;

    // for (size_t k = 0; k < J.size(); ++k)
    //     for (size_t i = 0; i < msg->name.size(); ++i)
    //         if (msg->name[i] == J[k]) {
    //             if (i < msg->position.size()) obs[k] = msg->position[i] - joint_pos_offset[i];
    //             ROS_INFO("at %zu it's %s, and its %zu in js", k, msg->name[i].c_str(), i);
    //             if (i < msg->velocity.size()) obs[J.size() + k] = msg->velocity[i];
    //         }


    // Override first 52 entries with default observations
    // std::copy_n(default_obs.begin(), default_obs.size(), obs.begin());

    /*------ JOINT POS ------*/

    // left leg joints pos
    obs[1] = msg->position[8]   - joint_pos_offset[1];      // leg left 1
    obs[4] = msg->position[9]   - joint_pos_offset[4];      // leg left 2
    obs[8] = msg->position[10]  - joint_pos_offset[8];      // leg left 3
    obs[12] = msg->position[15] - joint_pos_offset[12];     //leg left length
    obs[18] = msg->position[11] - joint_pos_offset[18];     // leg left 4
    obs[24] = msg->position[12] - joint_pos_offset[24];     //leg left 5

    obs[13] = msg->position[13] - joint_pos_offset[13];     // leg left femur
    obs[19] = msg->position[14] - joint_pos_offset[19];     // leg left knee

    // right leg joints pos
    obs[2] = msg->position[16]  - joint_pos_offset[2];
    obs[5] = msg->position[17]  - joint_pos_offset[5];
    obs[9] = msg->position[18]  - joint_pos_offset[9];
    obs[14] = msg->position[23] - joint_pos_offset[14];     // tricky position in obs
    obs[20] = msg->position[19] - joint_pos_offset[20];     // tricky position in obs
    obs[25] = msg->position[20] - joint_pos_offset[25];

    obs[15] = msg->position[21] - joint_pos_offset[15];     // leg right femur
    obs[21] = msg->position[22] - joint_pos_offset[21];     // leg right knee

    // pelvis joints pos
    obs[0] = msg->position[24]  - joint_pos_offset[0];      // pelvis 1
    obs[3] = msg->position[25]  - joint_pos_offset[3];      // pelvis 2



    /*------ JOINT VEL ------*/

    // left leg joints vel
    obs[1 + 26] = 0.05 * msg->velocity[8];
    obs[4 + 26] = 0.05 * msg->velocity[9];
    obs[8 + 26] = 0.05 * msg->velocity[10];
    obs[12 + 26] = 0.05 * msg->velocity[15];
    obs[18 + 26] = 0.05 * msg->velocity[11];
    obs[24 + 26] = 0.05 * msg->velocity[12];

    obs[13 + 26] = 0.05 * msg->velocity[13];     // leg left femur
    obs[19 + 26] = 0.05 * msg->velocity[14];     // leg left knee

    // right leg joints vel
    obs[2 + 26] = 0.05 * msg->velocity[16];
    obs[5 + 26] = 0.05 * msg->velocity[17];
    obs[9 + 26] = 0.05 * msg->velocity[18];
    obs[14 + 26] = 0.05 * msg->velocity[23];     // tricky position in obs
    obs[20 + 26] = 0.05 * msg->velocity[19];     // tricky position in obs
    obs[25 + 26] = 0.05 * msg->velocity[20];

    obs[15 + 26] = 0.05 * msg->velocity[21];     // leg right femur
    obs[21 + 26] = 0.05 * msg->velocity[22];     // leg right knee

    // pelvis joints vel
    obs[0 + 26] = 0.05 * msg->position[24];      // pelvis 1
    obs[3 + 26] = 0.05 * msg->position[25];      // pelvis 2
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
    // base_ang_vel
    obs[66] = msg->angular_velocity.x;
    obs[67] = msg->angular_velocity.y;
    obs[68] = msg->angular_velocity.z;
    // printf("Base angular velocity: x: %f, y: %f, z: %f\n", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    // base_lin_acc
    obs[69] = msg->linear_acceleration.x;
    obs[70] = msg->linear_acceleration.y;
    obs[71] = msg->linear_acceleration.z;
    // printf("Base linear acceleration: x: %f, y: %f, z: %f\n", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}

// rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
// (3)
void velCmdCB(const geometry_msgs::Twist::ConstPtr& msg)
{
    obs[72] = msg->linear.x;
    obs[73] = msg->linear.y;
    obs[74] = msg->angular.z;

    printf("Velocity command: Vx: %f, Vy: %f, wz: %f\n", msg->linear.x, msg->linear.y, msg->angular.z);
}



int main(int argc,char** argv)
{
  ros::init(argc,argv,"onnx_input_pub"); ros::NodeHandle nh;

  obs[72] = 0.0; // Vx
  obs[73] = 0.0; // Vx
  obs[74] = 0.0; // Vxz;

  ros::Subscriber s1 = nh.subscribe("/joint_states", 1, jsCB);
  ros::Subscriber s2 = nh.subscribe("/rl_policy/last_action", 1, lastCB);
  ros::Subscriber s3 = nh.subscribe("/base_imu", 1, imuCB);
  ros::Subscriber s4 = nh.subscribe("/cmd_vel", 1, velCmdCB);

  ros::Publisher  pub= nh.advertise<std_msgs::Float64MultiArray>("/rl_policy/obs", 1);

  ros::Rate r(50);

  const double period = 0.8; //0.6;
  double global_phase = 0.0;
  ros::Time last_time = ros::Time::now();

  while (ros::ok()) {
    ros::Time now = ros::Time::now();
    double dt = (now - last_time).toSec();
    last_time = now;
    if (dt <= 0.0) dt = 0.0;

    if (dt > 0.0) {
      double freq = 1.0 / dt;
      printf("Callback frequency: %f Hz\n", freq);
    }

    global_phase += dt / period;
    global_phase = std::fmod(global_phase, 1.0);

    obs[75] = std::sin(global_phase * 2.0 * M_PI);
    obs[76] = std::cos(global_phase * 2.0 * M_PI);


    std_msgs::Float64MultiArray m; m.data = obs; pub.publish(m);
    ros::spinOnce(); r.sleep();
  }
}
