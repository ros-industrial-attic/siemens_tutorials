#include <ros/ros.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt8MultiArray.h>

//declare callback functions 
void subByteCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
void subRealCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);
void subByteArrayCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cp1616_io_controller_tutorial_node");
  ros::NodeHandle nh;
  
  //Subscribe to topics defined in yaml config file
  ros::Subscriber sub_byte = nh.subscribe("/cp_input_byte_topic",
                                          1, &subByteCallback);
  ros::Subscriber sub_real = nh.subscribe("/cp_input_real_topic",
                                          1, &subRealCallback);
  ros::Subscriber sub_byte_array = nh.subscribe("/cp_input_byte_array_topic",
                                          1, &subByteArrayCallback);
  
  //Create publihsers for topics defined in yaml config file
  ros::Publisher pub_byte = nh.advertise<std_msgs::UInt8MultiArray>
                            ("/cp_output_byte_topic", 1);
  ros::Publisher pub_real = nh.advertise<std_msgs::UInt8MultiArray>
                            ("/cp_output_real_topic", 1);
  ros::Publisher pub_byte_array = nh.advertise<std_msgs::UInt8MultiArray>
                            ("/cp_output_byte_array_topic", 1);
  
  //Define and initialize output variables
  unsigned char output_byte = 0;
  float output_real = 0;
  unsigned char output_byte_array[] = {0, 0, 0, 0, 0, 0, 0, 0};
  
  //std_msgs::MultiArray variables
  std_msgs::MultiArrayDimension msg_dim;
  std_msgs::UInt8MultiArray msg;
  
  while(ros::ok())
  {
     //-------------------------------------------
    //Increment and publish CP Output byte
    //-------------------------------------------
    output_byte++;                         
    msg_dim.label = "CP_output_byte";       
    msg_dim.size = 1;                     
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);    
    
    msg.data.clear();                      
    msg.data.push_back(output_byte);       
    pub_byte.publish(msg);
   
    //-------------------------------------------
    //Increment and publish CP Output real
    //-------------------------------------------
    output_real += 0.1;                     
    msg_dim.label = "CP_output_real";       
    msg_dim.size = 4;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);    
    
    msg.data.clear();
    
    //Decompose float to 4 bytes
    unsigned char *p_byte;
    unsigned int output_array[sizeof(float)];
    
    p_byte = (unsigned char*)(&output_real);
    for(size_t i = 0; i < sizeof(float); i++)
    {
      output_array[i] = (static_cast<unsigned int>(p_byte[i]));
      msg.data.push_back(output_array[3-i]);
    }
    pub_real.publish(msg);
       
    //-------------------------------------------
    //Increment and publish CP Output byte array
    //-------------------------------------------
    for(size_t i = 0; i < 8; i++)
      output_byte_array[i] += 1;            
    
    msg_dim.label = "CP_output_byte_array"; 
    msg_dim.size = 8;
    msg.layout.dim.clear();
    msg.layout.dim.push_back(msg_dim);    
    
    msg.data.clear();        
    for(size_t i = 0; i < 8; i++)
      msg.data.push_back(output_byte_array[i]);
    
    pub_byte_array.publish(msg);
     
    //-------------------------------------------
    //Sleep and SpinOnce
    //-------------------------------------------
    ros::Duration(0.5).sleep();  
    ros::spinOnce();
  }  
  
  return (EXIT_SUCCESS);
}

// cp_input_byte_topic message recieved
void subByteCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
  unsigned int recieved_byte = msg->data[0];    
}

// cp_input_real_topic message recieved
void subRealCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
  unsigned int raw[4];
  
  raw[0] = msg->data[0];
  raw[1] = msg->data[1];
  raw[2] = msg->data[2];
  raw[3] = msg->data[3];
    
  //RAW bytes to float conversion
  int sign = (raw[0] >> 7) ? -1 : 1;
  int8_t exponent = (raw[0] << 1) + (raw[1] >> 7) - 126;
  uint32_t fraction_bits = ((raw[1] & 0x7F) << 16) + (raw[2] << 8) + raw[3];

  float fraction = 0.5f;
  for (uint8_t ii = 0; ii < 24; ++ii)
    fraction += ldexpf((fraction_bits >> (23 - ii)) & 1, -(ii + 1));

  float significand = sign * fraction;
  float recieved_real = ldexpf(significand, exponent);  
}

// cp_input_byte_array_topic message recieved
void subByteArrayCallback(const std_msgs::UInt8MultiArray::ConstPtr &msg)
{
  unsigned int recieved_byte_array[8];
  
  for(unsigned int i = 0; i < 8; i++)
    recieved_byte_array[i] = msg->data[i];  
}