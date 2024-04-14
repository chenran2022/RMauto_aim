// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0xAA;  //0
  float yaw_angle;        //123
  float pitch_angle;      //45
  int aim_mode;           //6
  int detect_color;       //6
  int bullet_speed;       //7

  int unrecognized_num;   //8-不击打的数字  五位数
  uint8_t tail;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xAA;
  float yaw_angle;
  float pitch_angle;
  int distance;
  int shoot_flag = 0;
  int detect_flag;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & r_data)
{
  ReceivePacket packet;

  packet.yaw_angle = r_data[1] * 100 + r_data[2] + (float)r_data[3] / 100;
  if (r_data[4] - 90 < 0) {
    packet.pitch_angle = int(r_data[4] - 90) + 1.0 - (float)(r_data[5] / 100.0);
  } else {
    packet.pitch_angle = int(r_data[4] - 90) + (float)(r_data[5] / 100.0);
  }
  packet.aim_mode = r_data[6] / 10;
  packet.detect_color = int(r_data[6] % 10);
  packet.bullet_speed = double(r_data[7]) * 2 / 10;
  if (r_data[7] == 0) {
    packet.bullet_speed = 27;
  }
  packet.aim_mode = (packet.aim_mode == 1 || packet.aim_mode >= 4) ? 0 : packet.aim_mode;


  //不击打的数字
  uint8_t num[5]={0};

  num[0]=r_data[8]>>4 & 1;  
  num[1]=r_data[8]>>3 & 1;  
  num[2]=r_data[8]>>2 & 1;  
  num[3]=r_data[8]>>1 & 1;  
  num[4]=r_data[8] & 1;  

  packet.unrecognized_num=num[0]*10000+num[1]*3000+num[2]*400+num[3]*50+num[4]*7;              


  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & send_data)
{
  std::vector<uint8_t> packet(sizeof(uint8_t) * 9);

  packet[0] = send_data.header;
  packet[1] = (unsigned char)((int)send_data.yaw_angle / 100);    // integer part
  packet[2] = (unsigned char)((int)(send_data.yaw_angle) % 100);  // fractional part
  packet[3] = (unsigned char)((int)(send_data.yaw_angle * 100) % 100);

  packet[4] = (unsigned char)((int)send_data.pitch_angle % 256);
  packet[5] = (unsigned char)(((int)(send_data.pitch_angle * 100) % 100));
  packet[6] = (unsigned char)(int(send_data.distance / 100));  // High eight
  packet[7] =
    (unsigned char)(send_data.detect_flag + 100 * send_data.shoot_flag);  // detection marker
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
