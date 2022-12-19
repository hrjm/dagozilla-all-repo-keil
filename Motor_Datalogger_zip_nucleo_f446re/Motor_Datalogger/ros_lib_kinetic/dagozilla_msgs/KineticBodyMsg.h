#ifndef _ROS_dagozilla_msgs_KineticBodyMsg_h
#define _ROS_dagozilla_msgs_KineticBodyMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dagozilla_msgs/VectorMsg.h"
#include "dagozilla_msgs/AngleMsg.h"

namespace dagozilla_msgs
{

  class KineticBodyMsg : public ros::Msg
  {
    public:
      typedef dagozilla_msgs::VectorMsg _position_type;
      _position_type position;
      typedef dagozilla_msgs::AngleMsg _orientation_type;
      _orientation_type orientation;
      typedef dagozilla_msgs::VectorMsg _velocity_type;
      _velocity_type velocity;
      typedef dagozilla_msgs::AngleMsg _rotation_type;
      _rotation_type rotation;

    KineticBodyMsg():
      position(),
      orientation(),
      velocity(),
      rotation()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      offset += this->velocity.serialize(outbuffer + offset);
      offset += this->rotation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      offset += this->velocity.deserialize(inbuffer + offset);
      offset += this->rotation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/KineticBodyMsg"; };
    const char * getMD5(){ return "0ff0dd1f852a6ab65a0b1f6982201ca3"; };

  };

}
#endif