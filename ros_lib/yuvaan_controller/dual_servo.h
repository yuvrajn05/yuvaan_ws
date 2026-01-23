#ifndef _ROS_yuvaan_controller_dual_servo_h
#define _ROS_yuvaan_controller_dual_servo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace yuvaan_controller
{

  class dual_servo : public ros::Msg
  {
    public:
      typedef int32_t _servo1_speed_type;
      _servo1_speed_type servo1_speed;
      typedef int32_t _servo2_speed_type;
      _servo2_speed_type servo2_speed;

    dual_servo():
      servo1_speed(0),
      servo2_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_servo1_speed;
      u_servo1_speed.real = this->servo1_speed;
      *(outbuffer + offset + 0) = (u_servo1_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo1_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo1_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo1_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo1_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_servo2_speed;
      u_servo2_speed.real = this->servo2_speed;
      *(outbuffer + offset + 0) = (u_servo2_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_servo2_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_servo2_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_servo2_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->servo2_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_servo1_speed;
      u_servo1_speed.base = 0;
      u_servo1_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo1_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo1_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo1_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo1_speed = u_servo1_speed.real;
      offset += sizeof(this->servo1_speed);
      union {
        int32_t real;
        uint32_t base;
      } u_servo2_speed;
      u_servo2_speed.base = 0;
      u_servo2_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_servo2_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_servo2_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_servo2_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->servo2_speed = u_servo2_speed.real;
      offset += sizeof(this->servo2_speed);
     return offset;
    }

    virtual const char * getType() override { return "yuvaan_controller/dual_servo"; };
    virtual const char * getMD5() override { return "af2b50820dddbd7e357854376715f6d2"; };

  };

}
#endif
