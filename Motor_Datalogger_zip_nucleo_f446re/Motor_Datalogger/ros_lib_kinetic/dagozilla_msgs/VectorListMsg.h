#ifndef _ROS_dagozilla_msgs_VectorListMsg_h
#define _ROS_dagozilla_msgs_VectorListMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dagozilla_msgs/VectorMsg.h"

namespace dagozilla_msgs
{

  class VectorListMsg : public ros::Msg
  {
    public:
      uint32_t data_length;
      typedef dagozilla_msgs::VectorMsg _data_type;
      _data_type st_data;
      _data_type * data;

    VectorListMsg():
      data_length(0), data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->data[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (dagozilla_msgs::VectorMsg*)realloc(this->data, data_lengthT * sizeof(dagozilla_msgs::VectorMsg));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      offset += this->st_data.deserialize(inbuffer + offset);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(dagozilla_msgs::VectorMsg));
      }
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/VectorListMsg"; };
    const char * getMD5(){ return "f88ab6e515853fd3715748d3598553b1"; };

  };

}
#endif