#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
  1.24, -1.24, 
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

std::vector<double> obs(65, 0.0); // work on the size to be dynamic

std::vector<std::string> J;

// Default first 52 observations
static const std::array<double, 52> default_obs = {{
    -5.6957e-03,  
     4.0456e-02,  -9.9550e-03, 
    -4.7219e-03,  
     2.2692e-02,   4.5535e-03, 
    -2.4187e-03, -1.3660e-03, 
    -2.0534e-01,  4.5023e-02, 
     7.2025e-03, 2.0674e-02, 
    -2.0041e-01, -4.2530e-01,
     1.9200e-02, 4.0204e-02, 
    -2.4761e-03,  1.2973e-03, 
    -2.4998e-02, -8.3431e-01,
     6.2749e-03,  5.7671e-02, 
    -1.3029e-02, -5.2135e-04,
    -2.4133e-01,  5.8092e-04,


     8.3491e-03, -4.0760e-03, -3.4956e-02,  9.5593e-03, 
     7.4607e-03,  5.2576e-02, 1.0561e-02, -1.0560e-02, 
    -3.1431e-02, -5.2157e-02,-1.4686e-02, -1.2233e-02, 
    -4.1484e-02, -7.6170e-02, -2.0217e-02, -4.3525e-02, 
     1.6276e-03, -7.8563e-03, -2.7567e-02, -1.5098e-01,
    -1.2132e-02, -8.7075e-02,  2.0031e-02, 
     3.2831e-03, -3.5677e-02, -7.3319e-03
}};


// compute joint pos and joint vel
// (12 + 12 = 24) or 26 + 26 = 52
void jsCB(const sensor_msgs::JointState::ConstPtr& msg)
{
    J = full_body_order;


    // for (size_t k = 0; k < J.size(); ++k)
    //     for (size_t i = 0; i < msg->name.size(); ++i)
    //         if (msg->name[i] == J[k]) {
    //             if (i < msg->position.size()) obs[k] = msg->position[i] - joint_pos_offset[i];
    //             ROS_INFO("at %zu it's %s, and its %zu in js", k, msg->name[i].c_str(), i);
    //             if (i < msg->velocity.size()) obs[J.size() + k] = msg->velocity[i];
    //         }
    // Override first 52 entries with default observations
    std::copy_n(default_obs.begin(), default_obs.size(), obs.begin());

    obs[1] = msg->position[8] - 0.0;
    obs[4] = msg->position[9] - 0.05;
    obs[8] = msg->position[10] - 0.0;
    obs[12] = msg->position[15] - 0.6832;
    obs[18] = msg->position[11] + 0.05;
    obs[24] = msg->position[12] - 0.0;

    obs[15] = msg->position[21] - 1.1172; // leg right femur
    obs[21] = msg->position[22] - 2.2345; // leg right knee

    obs[27] = msg->velocity[8];
    obs[30] = msg->velocity[9];
    obs[34] = msg->velocity[10];
    obs[38] = msg->velocity[15];
    obs[44] = msg->velocity[11];
    obs[50] = msg->velocity[12];

    obs[13 + 26] = msg->velocity[13]; // leg left femur
    obs[19 + 26] = msg->velocity[14]; // leg left knee
    
}

// ------------------------------------------------------------------------------------

void poseCB(const geometry_msgs::Point::ConstPtr& msg)
{
  obs[52] = msg->x;
  obs[53] = msg->y;
  obs[54] = msg->z;
  printf("Target Position: x: %f, y: %f, z: %f\n", obs[52], obs[53], obs[54]);
}

void lastCB(const std_msgs::Float64MultiArray::ConstPtr& a)
{
  for (size_t i = 0; i < 6 && i < a->data.size(); ++i)
    obs[59 + i] = a->data[i];
}

int main(int argc,char** argv)
{
  obs[52] = 0.025;  
  obs[53] = 0.141; 
  obs[54] = -0.898;  

  obs[55] = 9.8558e-01;
  obs[56] = 0.0000;  
  obs[57] = 1.6918e-01;
  obs[58] = 0.0000;  

  ros::init(argc,argv,"onnx_input_pub"); ros::NodeHandle nh;

  ros::Subscriber s1 = nh.subscribe("/joint_states",1,jsCB);
  ros::Subscriber s2 = nh.subscribe("/target_position",1,poseCB);
  ros::Subscriber s3 = nh.subscribe("/reach_policy/last_action",1,lastCB);

  ros::Publisher  pub= nh.advertise<std_msgs::Float64MultiArray>("/reach_policy/obs",1);

  ros::Rate r(50);
  while(ros::ok()){
    std_msgs::Float64MultiArray m; m.data = obs; pub.publish(m);
    ros::spinOnce(); r.sleep();
  }
}