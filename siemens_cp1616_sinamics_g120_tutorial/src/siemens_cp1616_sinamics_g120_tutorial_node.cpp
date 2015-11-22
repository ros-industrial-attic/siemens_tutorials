#include <ros/ros.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>

const int DRIVE_TELEGRAM_SIZE = 4;

//declarations
void driveTelegramCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
void setDriveTelegram(unsigned char drive_telegram[], int velocity);

//Main function
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cp1616_sinamics_tutorial_node");
  ros::NodeHandle nh;

  //Subscribe to drive topic defined in yaml config file
  ros::Subscriber sub_drive = nh.subscribe("/drive_1_input_topic", 1,
					   &driveTelegramCallback);

  //Create publihser for drive topic defined in yaml config file
  ros::Publisher pub_drive = nh.advertise<std_msgs::UInt8MultiArray>
                                         ("/drive_1_output_topic", 1);

  //Telegram 111 
  unsigned char drive_telegram_1[DRIVE_TELEGRAM_SIZE];

  //std_msgs::MultiArray variables
  std_msgs::MultiArrayDimension msg_dim;
  std_msgs::UInt8MultiArray msg;

  //Wait until communication starts properly
  ros::Duration(10).sleep();
  
  msg_dim.label = "Drive_1_output";
  msg_dim.size = DRIVE_TELEGRAM_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);

  msg.data.clear();

  drive_telegram_1[0] =  0b00000100;   //STW1 high
  drive_telegram_1[1] =  0b00000000;   //STW1 low
  drive_telegram_1[2] =  0b00000000;   //NSOLL_A high
  drive_telegram_1[3] =  0b00000000;   //NSOLL_A low
  
  msg.data.clear();
  for(size_t i = 0;  i < DRIVE_TELEGRAM_SIZE; i++)
    msg.data.push_back(drive_telegram_1[i]);

  pub_drive.publish(msg);
  
  //Wait 2 seconds 
  ros::Duration(2).sleep();
 
  //-------------------------------------------
  //Send initialization DRIVE telegram
  // - set all necessary bytes except ON/OFF1 to true
  //-------------------------------------------
  msg_dim.label = "Drive_1_output";
  msg_dim.size = DRIVE_TELEGRAM_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);

  msg.data.clear();

  drive_telegram_1[0] =  0b00000100;   //STW1 high
  drive_telegram_1[1] =  0b11111110;   //STW1 low
  drive_telegram_1[2] =  0b00000000;   //NSOLL_A high
  drive_telegram_1[3] =  0b00000000;   //NSOLL_A low
  
  msg.data.clear();
  for(size_t i = 0;  i < DRIVE_TELEGRAM_SIZE; i++)
    msg.data.push_back(drive_telegram_1[i]);

  pub_drive.publish(msg);
    
  //Wait 2 seconds before entering loop
  ros::Duration(2).sleep();
  
  //-------------------------------------------
  // Main loop
  //-------------------------------------------
  while(ros::ok())
  {
       
    ROS_INFO("Goal speed: 500");
    //message header
    msg_dim.label = "Drive_1_output";
    msg_dim.size = DRIVE_TELEGRAM_SIZE;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    
    setDriveTelegram(drive_telegram_1, 5000);
        
    //copy telegram data
    msg.data.clear();
    for(size_t i = 0;  i < DRIVE_TELEGRAM_SIZE; i++)
        msg.data.push_back(drive_telegram_1[i]);

    //publish
    pub_drive.publish(msg);
    
    //wait for 5 seconds
    ros::Duration(5).sleep();
    ros::spinOnce();
           
    //======================================================
    
    ROS_INFO("Goal speed: 1000");
    //message header
    msg_dim.label = "Drive_1_output";
    msg_dim.size = DRIVE_TELEGRAM_SIZE;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
        
    setDriveTelegram(drive_telegram_1, 10000);
    
    //copy telegram data
    msg.data.clear();
    for(size_t i = 0;  i < DRIVE_TELEGRAM_SIZE; i++)
        msg.data.push_back(drive_telegram_1[i]);
    
    //publish
    pub_drive.publish(msg);
        
    //wait for 5 seconds
    ros::Duration(5).sleep();
    ros::spinOnce();
  }     
  
  return (EXIT_SUCCESS);
}    
    
void setDriveTelegram(unsigned char drive_telegram[], int velocity)   
{ 
     
  //Decompose velocity from int to 4*unsigned char
  unsigned char velocity_bytes[2]; 
  
  velocity_bytes[0] = (velocity >> 8)  & 0xFF;
  velocity_bytes[1] = velocity & 0xFF;   
  
  //Set telegram data
  drive_telegram[0] =  0b00000100;           //STW1 high
  drive_telegram[1] =  0b11111111;           //STW1 low
  drive_telegram[2] =  velocity_bytes[0];    //NSOLL_A high 
  drive_telegram[3] =  velocity_bytes[1];    //NSOLL_A low
}

void driveTelegramCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
  unsigned int received_byte_array[DRIVE_TELEGRAM_SIZE];

  for(unsigned int i = 0; i < DRIVE_TELEGRAM_SIZE; i++)
    received_byte_array[i] = msg->data[i];
}
