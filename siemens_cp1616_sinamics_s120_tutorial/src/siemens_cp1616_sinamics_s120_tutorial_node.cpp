#include <ros/ros.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>

const int DRIVE_TELEGRAM_SIZE = 24;

//declarations
void driveTelegramCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
void setDriveTelegram(unsigned char drive_telegram[], int possition, int velocity);

//Main function
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cp1616_sinamics_tutorial_node");
  ros::NodeHandle nh;

  //Subscribe to drive topic defined in yaml config file
  ros::Subscriber sub_drive = nh.subscribe("/drive_1_input_topic", 1, &driveTelegramCallback);

  //Create publihser for drive topic defined in yaml config file
  ros::Publisher pub_drive = nh.advertise<std_msgs::UInt8MultiArray>("/drive_1_output_topic", 1);

  //Telegram 111 
  unsigned char drive_telegram_111[DRIVE_TELEGRAM_SIZE];

  //std_msgs::MultiArray variables
  std_msgs::MultiArrayDimension msg_dim;
  std_msgs::UInt8MultiArray msg;

  //Wait until communication starts properly
  ros::Duration(5).sleep();
  
  //-------------------------------------------
  //Send initialization DRIVE telegram
  // - set all necessary bytes except ON/OFF1 to true
  // - set reference point to true
  //-------------------------------------------
  msg_dim.label = "Drive_1_output";
  msg_dim.size = DRIVE_TELEGRAM_SIZE;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);

  msg.data.clear();

  drive_telegram_111[0] =  0b00000100;   //STW1 high
  drive_telegram_111[1] =  0b10111110;   //STW1 low
  drive_telegram_111[2] =  0b00000000;   //POS_STW1 high
  drive_telegram_111[3] =  0b00000000;   //POS_STW1 low
  drive_telegram_111[4] =  0b00000000;   //POS_STW2 high 
  drive_telegram_111[5] =  0b00000010;   //POS_STW2 low
  drive_telegram_111[6] =  0b00000000;   //STW2 high
  drive_telegram_111[7] =  0b00000000;   //STW2 low

  drive_telegram_111[8] =   0;  //OVERRIDE high
  drive_telegram_111[9] =   0;  //OVERRIDE low
  drive_telegram_111[10] =  0;  //MDI_TARPOS_6 high
  drive_telegram_111[11] =  0;  //MDI_TARPOS_6 low
  drive_telegram_111[12] =  0;  //MDI_TARPOS_7 high
  drive_telegram_111[13] =  0;  //MDI_TARPOS_7 low
  drive_telegram_111[14] =  0;  //MDI_VELOCITY_8 high
  drive_telegram_111[15] =  0;  //MDI_VELOCITY_8 low

  drive_telegram_111[16] =  0;  //MDI_VELOCITY_9 high
  drive_telegram_111[17] =  0;  //MDI_VELOCITY_9 low
  drive_telegram_111[18] =  0;  //MDI_ACC high
  drive_telegram_111[19] =  0;  //MDI_ACC low
  drive_telegram_111[20] =  0;  //MDI_DEC high
  drive_telegram_111[21] =  0;  //MDI_DEC low
  drive_telegram_111[22] =  0;  //Reserved
  drive_telegram_111[23] =  0;  //Reserved

  msg.data.clear();
  for(size_t i = 0;  i < DRIVE_TELEGRAM_SIZE; i++)
    msg.data.push_back(drive_telegram_111[i]);

  pub_drive.publish(msg);
  
  //Wait for 5 seconds
  ros::Duration(5).sleep();
  
  //-------------------------------------------
  // Main loop
  //-------------------------------------------
  while(ros::ok())
  {
       
    ROS_INFO("Moving to position 1");
    //message header
    msg_dim.label = "Drive_1_output";
    msg_dim.size = DRIVE_TELEGRAM_SIZE;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
    
    //fill telegram data with position and velocity values
    setDriveTelegram(drive_telegram_111, 1000, 100);
    
    //copy telegram data
    msg.data.clear();
    for(size_t i = 0;  i < DRIVE_TELEGRAM_SIZE; i++)
        msg.data.push_back(drive_telegram_111[i]);

    //publish
    pub_drive.publish(msg);
    
    //wait for 5 seconds
    ros::Duration(5).sleep();
    ros::spinOnce();
           
    //======================================================
    
    ROS_INFO("Moving to position 2");
    //message header
    msg_dim.label = "Drive_1_output";
    msg_dim.size = DRIVE_TELEGRAM_SIZE;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);
        
    //fill telegram data with position and velocity values
    setDriveTelegram(drive_telegram_111,-1000, 100);
    
    //copy telegram data
    msg.data.clear();
    for(size_t i = 0;  i < DRIVE_TELEGRAM_SIZE; i++)
        msg.data.push_back(drive_telegram_111[i]);
    
    //publish
    pub_drive.publish(msg);
        
    //wait for 5 seconds
    ros::Duration(5).sleep();
    ros::spinOnce();
  }     
  
  return (EXIT_SUCCESS);
}
    
    
void setDriveTelegram(unsigned char drive_telegram[], int position, int velocity)   
{ 
  //Decompose position from int to 4*unsigned char
  unsigned char position_bytes[4]; 
  
  position_bytes[0] = (position >> 24) & 0xFF;
  position_bytes[1] = (position >> 16) & 0xFF;
  position_bytes[2] = (position >> 8)  & 0xFF;
  position_bytes[3] = position & 0xFF;
   
  //Decompose velocity from int to 4*unsigned char
  unsigned char velocity_bytes[4]; 
  
  velocity_bytes[0] = (velocity >> 24) & 0xFF;
  velocity_bytes[1] = (velocity >> 16) & 0xFF;
  velocity_bytes[2] = (velocity >> 8)  & 0xFF;
  velocity_bytes[3] = velocity & 0xFF;   
  
  //Set telegram data
  drive_telegram[0] =  0b00000100;   //STW1 low
  drive_telegram[1] =  0b10111111;   //STW1 high
  drive_telegram[2] =  0b10010001;   //POS_STW1 low
  drive_telegram[3] =  0b00000000;   //POS_STW1 high
  drive_telegram[4] =  0b00000000;   //POS_STW2 low 
  drive_telegram[5] =  0b00000000;   //POS_STW2 high
  drive_telegram[6] =  0b00000000;   //STW2 low
  drive_telegram[7] =  0b00000000;   //STW2 high
  
  drive_telegram[8] =   0x40;               //OVERRIDE high = 0x4000 = 100%
  drive_telegram[9] =   0x00;               //OVERRIDE low
  drive_telegram[10] =  position_bytes[0];  //MDI_TARPOS_6 high
  drive_telegram[11] =  position_bytes[1];  //MDI_TARPOS_6 low
  drive_telegram[12] =  position_bytes[2];  //MDI_TARPOS_7 high
  drive_telegram[13] =  position_bytes[3];  //MDI_TARPOS_7 low
  drive_telegram[14] =  velocity_bytes[0];  //MDI_VELOCITY_8 high
  drive_telegram[15] =  velocity_bytes[1];  //MDI_VELOCITY_8 low

  drive_telegram[16] =  velocity_bytes[2];  //MDI_VELOCITY_9 high
  drive_telegram[17] =  velocity_bytes[3];  //MDI_VELOCITY_9 low
  drive_telegram[18] =  0x40;               //MDI_ACC high = 0x4000 = 100%
  drive_telegram[19] =  0;                  //MDI_ACC low
  drive_telegram[20] =  0x40;               //MDI_DEC high = 0x4000 = 100%
  drive_telegram[21] =  0;                  //MDI_DEC low
  drive_telegram[22] =  0;                  //Reserved
  drive_telegram[23] =  0;                  //Reserved

}

void driveTelegramCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
  unsigned int received_byte_array[DRIVE_TELEGRAM_SIZE];

  for(unsigned int i = 0; i < DRIVE_TELEGRAM_SIZE; i++)
    received_byte_array[i] = msg->data[i];
}
