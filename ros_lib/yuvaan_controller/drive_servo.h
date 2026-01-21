#ifndef _ROS_yuvaan_controller_drive_servo_h
#define _ROS_yuvaan_controller_drive_servo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace yuvaan_controller
{

  class drive_servo : public ros::Msg
  {
    public:
      typedef int32_t _vel_linear_x_type;
      _vel_linear_x_type vel_linear_x;
      typedef int32_t _vel_angular_z_type;
      _vel_angular_z_type vel_angular_z;
      typedef int32_t _mode_type;
      _mode_type mode;
      typedef int32_t _servo_speed_type;
      _servo_speed_type servo_speed;

    drive_servo():
      vel_linear_x(0),
      vel_angular_z(0),
      mode(0),
      servo_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_vel_linear_x;
      u_vel_linear_x.real = this->vel_linear_x;
      *(outbuffer + offset + 0) = (u_vel_linear_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_linear_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_linear_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_linear_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_linear_x);
      union {
        int32_t real;
        uint32_t base;
      } u_vel_angular_z;
      u_vel_angular_z.real = this->vel_angular_z;
      *(outbuffer + offset + 0) = (u_vel_angular_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel_angular_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel_angular_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel_angular_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vel_angular_z);
      union {
        int32_t real;
        uint32_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_speed;
      u_servo_speed.real = this->servo_speed;
      *(outbuffer + offset + 0) = (u_servo_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_vel_linear_x;
      u_vel_linear_x.base = 0;
      u_vel_linear_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel_linear_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel_linear_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel_linear_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel_linear_x = u_vel_linear_x.real;
      offset += sizeof(this->vel_linear_x);
      union {
        int32_t real;
        uint32_t base;
      } u_vel_angular_z;
      u_vel_angular_z.base = 0;
      u_vel_angular_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel_angular_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel_angular_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel_angular_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vel_angular_z = u_vel_angular_z.real;
      offset += sizeof(this->vel_angular_z);
      union {
        int32_t real;
        uint32_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
      union {
        int32_t real;
        uint32_t base;
      } u_servo_speed;
      u_servo_speed.base = 0;
      u_servo_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo_speed = u_servo_speed.real;
      offset += sizeof(this->servo_speed);
     return offset;
    }

    virtual const char * getType() override { return "yuvaan_controller/drive_servo"; };
    virtual const char * getMD5() override { return "f038d4ff1f38bef575d3a34d6c844c6d"; };

  };

}
#endif
