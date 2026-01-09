#ifndef _ROS_yuvaan_controller_yuvaan_h
#define _ROS_yuvaan_controller_yuvaan_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace yuvaan_controller
{

  class yuvaan : public ros::Msg
  {
    public:
      typedef int32_t _vel_linear_x_type;
      _vel_linear_x_type vel_linear_x;
      typedef int32_t _vel_angular_z_type;
      _vel_angular_z_type vel_angular_z;
      typedef int32_t _mode_type;
      _mode_type mode;
      typedef int32_t _yaw_mode_type;
      _yaw_mode_type yaw_mode;
      typedef int32_t _roll_mode_type;
      _roll_mode_type roll_mode;
      typedef int32_t _ra_1_type;
      _ra_1_type ra_1;
      typedef int32_t _ra_2_type;
      _ra_2_type ra_2;
      typedef int32_t _ra_3_type;
      _ra_3_type ra_3;
      typedef int32_t _ra_4_type;
      _ra_4_type ra_4;
      typedef int32_t _ra_5_type;
      _ra_5_type ra_5;
      typedef int32_t _ra_6_type;
      _ra_6_type ra_6;
      typedef int32_t _ba_1_type;
      _ba_1_type ba_1;
      typedef int32_t _ba_2_type;
      _ba_2_type ba_2;
      typedef int32_t _ba_3_type;
      _ba_3_type ba_3;
      typedef int32_t _ba_4_type;
      _ba_4_type ba_4;
      typedef int32_t _ba_5_type;
      _ba_5_type ba_5;
      typedef int32_t _ba_6_type;
      _ba_6_type ba_6;

    yuvaan():
      vel_linear_x(0),
      vel_angular_z(0),
      mode(0),
      yaw_mode(0),
      roll_mode(0),
      ra_1(0),
      ra_2(0),
      ra_3(0),
      ra_4(0),
      ra_5(0),
      ra_6(0),
      ba_1(0),
      ba_2(0),
      ba_3(0),
      ba_4(0),
      ba_5(0),
      ba_6(0)
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
      } u_yaw_mode;
      u_yaw_mode.real = this->yaw_mode;
      *(outbuffer + offset + 0) = (u_yaw_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_roll_mode;
      u_roll_mode.real = this->roll_mode;
      *(outbuffer + offset + 0) = (u_roll_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_1;
      u_ra_1.real = this->ra_1;
      *(outbuffer + offset + 0) = (u_ra_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ra_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ra_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ra_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ra_1);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_2;
      u_ra_2.real = this->ra_2;
      *(outbuffer + offset + 0) = (u_ra_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ra_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ra_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ra_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ra_2);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_3;
      u_ra_3.real = this->ra_3;
      *(outbuffer + offset + 0) = (u_ra_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ra_3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ra_3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ra_3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ra_3);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_4;
      u_ra_4.real = this->ra_4;
      *(outbuffer + offset + 0) = (u_ra_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ra_4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ra_4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ra_4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ra_4);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_5;
      u_ra_5.real = this->ra_5;
      *(outbuffer + offset + 0) = (u_ra_5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ra_5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ra_5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ra_5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ra_5);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_6;
      u_ra_6.real = this->ra_6;
      *(outbuffer + offset + 0) = (u_ra_6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ra_6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ra_6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ra_6.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ra_6);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_1;
      u_ba_1.real = this->ba_1;
      *(outbuffer + offset + 0) = (u_ba_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_1);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_2;
      u_ba_2.real = this->ba_2;
      *(outbuffer + offset + 0) = (u_ba_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_2);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_3;
      u_ba_3.real = this->ba_3;
      *(outbuffer + offset + 0) = (u_ba_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_3);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_4;
      u_ba_4.real = this->ba_4;
      *(outbuffer + offset + 0) = (u_ba_4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_4);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_5;
      u_ba_5.real = this->ba_5;
      *(outbuffer + offset + 0) = (u_ba_5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_5.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_5.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_5.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_5);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_6;
      u_ba_6.real = this->ba_6;
      *(outbuffer + offset + 0) = (u_ba_6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ba_6.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ba_6.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ba_6.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ba_6);
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
      } u_yaw_mode;
      u_yaw_mode.base = 0;
      u_yaw_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw_mode = u_yaw_mode.real;
      offset += sizeof(this->yaw_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_roll_mode;
      u_roll_mode.base = 0;
      u_roll_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_mode = u_roll_mode.real;
      offset += sizeof(this->roll_mode);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_1;
      u_ra_1.base = 0;
      u_ra_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ra_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ra_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ra_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ra_1 = u_ra_1.real;
      offset += sizeof(this->ra_1);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_2;
      u_ra_2.base = 0;
      u_ra_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ra_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ra_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ra_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ra_2 = u_ra_2.real;
      offset += sizeof(this->ra_2);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_3;
      u_ra_3.base = 0;
      u_ra_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ra_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ra_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ra_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ra_3 = u_ra_3.real;
      offset += sizeof(this->ra_3);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_4;
      u_ra_4.base = 0;
      u_ra_4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ra_4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ra_4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ra_4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ra_4 = u_ra_4.real;
      offset += sizeof(this->ra_4);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_5;
      u_ra_5.base = 0;
      u_ra_5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ra_5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ra_5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ra_5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ra_5 = u_ra_5.real;
      offset += sizeof(this->ra_5);
      union {
        int32_t real;
        uint32_t base;
      } u_ra_6;
      u_ra_6.base = 0;
      u_ra_6.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ra_6.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ra_6.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ra_6.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ra_6 = u_ra_6.real;
      offset += sizeof(this->ra_6);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_1;
      u_ba_1.base = 0;
      u_ba_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_1 = u_ba_1.real;
      offset += sizeof(this->ba_1);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_2;
      u_ba_2.base = 0;
      u_ba_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_2 = u_ba_2.real;
      offset += sizeof(this->ba_2);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_3;
      u_ba_3.base = 0;
      u_ba_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_3 = u_ba_3.real;
      offset += sizeof(this->ba_3);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_4;
      u_ba_4.base = 0;
      u_ba_4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_4 = u_ba_4.real;
      offset += sizeof(this->ba_4);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_5;
      u_ba_5.base = 0;
      u_ba_5.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_5.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_5.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_5.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_5 = u_ba_5.real;
      offset += sizeof(this->ba_5);
      union {
        int32_t real;
        uint32_t base;
      } u_ba_6;
      u_ba_6.base = 0;
      u_ba_6.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ba_6.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ba_6.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ba_6.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ba_6 = u_ba_6.real;
      offset += sizeof(this->ba_6);
     return offset;
    }

    virtual const char * getType() override { return "yuvaan_controller/yuvaan"; };
    virtual const char * getMD5() override { return "f9d9c81a204467304bb82823914c9c9d"; };

  };

}
#endif
