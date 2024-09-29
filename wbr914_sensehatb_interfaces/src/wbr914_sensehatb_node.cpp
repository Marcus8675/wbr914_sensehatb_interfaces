#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
#include <SHTC3.h>
#include <ICM20948.h>
#include <LPS22HB.h>
#include <wbr914_sensehatb_interfaces/msg/ad.hpp>
#include <wbr914_sensehatb_interfaces/msg/baro_pressure.hpp>
#include <wbr914_sensehatb_interfaces/msg/imu.hpp>
#include <wbr914_sensehatb_interfaces/msg/temperature.hpp>

float TH_Value, RH_Value;  //Temperature and Relitive Humidity
float PRESS_DATA=0;  //Barometric Pressure
float TEMP_DATA=0;  //Temperature

//IMU
IMU_EN_SENSOR_TYPE enMotionSensorType;
IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;


using namespace std::chrono_literals;

//  A ROS publisher posting Twist messages from the sensehat B.

class SensehatbPublisher : public rclcpp::Node
{
public:
  SensehatbPublisher()
  : Node("sensehatb_publisher")
    {
      temperature_publisher_ =
      this->create_publisher<wbr914_sensehatb_interfaces::msg::Temperature>("temperature", 10);

      imu_publisher_ =
      this->create_publisher<wbr914_sensehatb_interfaces::msg::Imu>("imu", 10);

      baropressure_publisher_ = 
      this->create_publisher<wbr914_sensehatb_interfaces::msg::BaroPressure>("baropressure", 10);
      
      timer_ = this->create_wall_timer(1s, std::bind(&SensehatbPublisher::timer_callback, this));
    };


  private:
  void timer_callback()
  {
    //Read and report Temperature and Humidity data
    auto message1 = wbr914_sensehatb_interfaces::msg::Temperature();

    lgI2cOpen(1, SHTC3_I2C_ADDRESS, 0);
    SHTC3_Read_DATA(&TH_Value, &RH_Value);

    message1.temperature = TH_Value;  //test value
    message1.humidity = RH_Value;  //test value

    this->temperature_publisher_->publish(message1);


    //Read and report IMU data
    auto message2 = wbr914_sensehatb_interfaces::msg::Imu();

    imuInit(&enMotionSensorType);
    imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

    //Angles
    message2.yaw = stAngles.fYaw;  
    message2.pitch = stAngles.fPitch;
    message2.roll = stAngles.fRoll;  

    //Accelorometer
    message2.acc_x = stAccelRawData.s16X;
    message2.acc_y = stAccelRawData.s16Y;
    message2.acc_z = stAccelRawData.s16Z;

    //Gyroscope
    message2.gyro_x = stGyroRawData.s16X;
    message2.gyro_y = stGyroRawData.s16Y;
    message2.gyro_z = stGyroRawData.s16Z;

    //Magnetometer
    message2.mag_x = stMagnRawData.s16X;
    message2.mag_y = stMagnRawData.s16Y;
    message2.mag_z = stMagnRawData.s16Z;

    this->imu_publisher_->publish(message2);


    //Read and report Barometric pressure data
    auto message3 = wbr914_sensehatb_interfaces::msg::BaroPressure();

    unsigned char u8Buf[3];
    LPS22HB_INIT();
    LPS22HB_START_ONESHOT();        //Trigger one shot data acquisition
    if((I2C_readByte(LPS_STATUS)&0x01)==0x01)   //a new pressure data is generated
    {
        u8Buf[0]=I2C_readByte(LPS_PRESS_OUT_XL);
        u8Buf[1]=I2C_readByte(LPS_PRESS_OUT_L);
        u8Buf[2]=I2C_readByte(LPS_PRESS_OUT_H);
        PRESS_DATA=(float)((u8Buf[2]<<16)+(u8Buf[1]<<8)+u8Buf[0])/4096.0f;
    }
    else PRESS_DATA = 0;
    if((I2C_readByte(LPS_STATUS)&0x02)==0x02)   // a new temperature data is generated
    {
        u8Buf[0]=I2C_readByte(LPS_TEMP_OUT_L);
        u8Buf[1]=I2C_readByte(LPS_TEMP_OUT_H);
        TEMP_DATA=(float)((u8Buf[1]<<8)+u8Buf[0])/100.0f;
    }
    else TEMP_DATA = 0;

    message3.pressure = PRESS_DATA;  //hPa
    message3.temperature = TEMP_DATA;  //Deg C

    this->baropressure_publisher_->publish(message3);

  }

  rclcpp::Publisher<wbr914_sensehatb_interfaces::msg::Temperature>::SharedPtr temperature_publisher_;
  rclcpp::Publisher<wbr914_sensehatb_interfaces::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<wbr914_sensehatb_interfaces::msg::BaroPressure>::SharedPtr baropressure_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
}; 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensehatbPublisher>());
  rclcpp::shutdown();
  return 0;
}