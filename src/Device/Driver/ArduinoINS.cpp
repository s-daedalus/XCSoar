/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Device/Driver/ArduinoINS.hpp"
#include "Device/Driver.hpp"
#include "Device/Port/Port.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/MoreData.hpp"
#include "NMEA/Derived.hpp"
#include "Units/System.hpp"
#include "LogFile.hpp"
#include "Time/Cast.hxx"

class ArduinoINSDevice : public AbstractDevice {
  Port &port;

public:
  ArduinoINSDevice(Port &_port):port(_port) {};

  /* virtual methods from class Device */
  bool DataReceived(const void *data, size_t length, struct NMEAInfo &info) override;
  void OnCalculatedUpdate(const MoreData &basic,
                  const DerivedInfo &calculated) override;
private:
    float x_kal_x=0,x_kal_y=0, x_kal_z=0;
    double last_altitude;
    Validity last_altitude_validity;
   //Validity lastStep;
};


void
ArduinoINSDevice::OnCalculatedUpdate(const MoreData &basic, 
    const DerivedInfo &calculated)
{
  if (basic.airspeed_available.IsValid()) {
    int16_t gs_10 = (int) (10 * basic.true_airspeed);
    //LogFormat("GS int to send: %i", gs_10);
    char buff[4];
    buff[0] = '$';
    buff[1] = 'V';
    buff[2] = *((char*)&gs_10);
    buff[3] = *((char*)&gs_10 + 1);
    port.Write(buff, 4);
    }


  // TODO GPS update.
  float K_kal = 3e-2f;
  float vx_ned = 0;
  float vy_ned = 0;
  float vz_ned = 0;
  if (basic.ground_speed_available.IsValid()){
    
    vx_ned = basic.ground_speed * basic.track.fastcosine();
    vy_ned = basic.ground_speed * basic.track.fastsine();
    
  }else{
    return;
  }
  
  if(basic.pressure_altitude_available){
    if(last_altitude_validity){

      vz_ned = (last_altitude - basic.pressure_altitude) / 
      ToFloatSeconds(basic.pressure_altitude_available.GetTimeDifference(last_altitude_validity));
    }

    last_altitude = basic.pressure_altitude;
    last_altitude_validity = basic.pressure_altitude_available;

  }else if(basic.baro_altitude_available){
    if(last_altitude_validity){
      vz_ned = (last_altitude - basic.baro_altitude) / 
      ToFloatSeconds(basic.baro_altitude_available.GetTimeDifference(last_altitude_validity));
    }
    
    last_altitude = basic.baro_altitude;
    last_altitude_validity = basic.baro_altitude_available;
    
  }else if(basic.gps_altitude_available){
    if(last_altitude_validity){
      vz_ned = (last_altitude - basic.gps_altitude) / 
      ToFloatSeconds(basic.gps_altitude_available.GetTimeDifference(last_altitude_validity));
    }
    last_altitude = basic.gps_altitude;
    last_altitude_validity = basic.gps_altitude_available;
    
  }else{
    return;
  }
  x_kal_x += K_kal * (vx_ned - x_kal_x);
  x_kal_y += K_kal * (vy_ned - x_kal_y);
  x_kal_z += K_kal * (vz_ned - x_kal_z);
  //lastStep.Update(basic.clock);
  // todo: transmit airspeed and not groundspeed
  int16_t i_vx100 = (x_kal_x * 100);
  int16_t i_vy100 = (x_kal_y * 100);
  int16_t i_vz100 = (x_kal_z * 100);
  char buff[8];
  buff[0] = '$';
  buff[1] = 'G';
  // todo: rotate from body to NED frame before transmitting
  buff[2] = *((char*)&i_vx100);
  buff[3] = *((char*)&i_vx100 + 1);
  buff[4] = *((char*)&i_vy100);
  buff[5] = *((char*)&i_vy100 + 1);
  buff[6] = *((char*)&i_vz100);
  buff[7] = *((char*)&i_vz100 + 1);
  port.Write(buff, 8);
}

bool
ArduinoINSDevice::DataReceived(const void *data, size_t length,
                            struct NMEAInfo &info)
{
  const char* _line = (const char*)data;
  if(length != 14){
    return true;
  }
  if(_line[0] == '$' && _line[1] == 'A'){
    int16_t i_roll, i_pitch, i_yaw, i_vx, i_vy, i_vz;
    *((char*)&i_roll) = _line[2];
    *((char*)&i_roll + 1) = _line[3];
    *((char*)&i_pitch) = _line[4];
    *((char*)&i_pitch + 1) = _line[5];
    *((char*)&i_yaw) = _line[6];
    *((char*)&i_yaw + 1) = _line[7];

    *((char*)&i_vx) = _line[8];
    *((char*)&i_vx + 1) = _line[9];
    *((char*)&i_vy) = _line[10];
    *((char*)&i_vy + 1) = _line[11];
    *((char*)&i_vz) = _line[12];
    *((char*)&i_vz + 1) = _line[13];

    info.attitude.bank_angle_available.Update(info.clock);
    info.attitude.bank_angle = Angle::Degrees(0.1f * i_roll);

    info.attitude.pitch_angle_available.Update(info.clock);
    info.attitude.pitch_angle = Angle::Degrees(0.1f *i_pitch);

    info.attitude.heading_available.Update(info.clock);
    info.attitude.heading = Angle::Degrees(0.1f * i_yaw);
    
    x_kal_x = 0.01f * i_vx;
    x_kal_y = 0.01f * i_vy;
    x_kal_z = 0.01f * i_vz;

    info.ProvideNoncompVario(-x_kal_z);
    info.ground_speed = sqrtf(x_kal_x * x_kal_x + x_kal_y * x_kal_y);
    info.ground_speed_available.Update(info.clock);
    info.alive.Update(info.clock);
    return true;
  }
  return true;
}


static Device *
ArduinoINSCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new ArduinoINSDevice(com_port);
}

const struct DeviceRegister arduino_ins_driver = {
  _T("ArduinoINS"),
  _T("ArduinoINS"),
    DeviceRegister::NO_TIMEOUT |
  DeviceRegister::RAW_GPS_DATA,
  ArduinoINSCreateOnPort,
};
