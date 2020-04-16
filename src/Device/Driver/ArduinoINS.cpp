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


class ArduinoINSDevice : public AbstractDevice {
  Port &port;

public:
  ArduinoINSDevice(Port &_port):port(_port) {};

  /* virtual methods from class Device */
  bool DataReceived(const void *data, size_t length, struct NMEAInfo &info) override;
  void OnCalculatedUpdate(const MoreData &basic,
                  const DerivedInfo &calculated) override;
private:
    float P_kal=1;
    float x_kal_x=0, x_kal_z=0;
   //Validity lastStep;
};


void
ArduinoINSDevice::OnCalculatedUpdate(const MoreData &basic, 
    const DerivedInfo &calculated)
{
  if (basic.ground_speed_available.IsValid()) {
    int16_t gs_10 = (int) 10 * basic.ground_speed;
    //LogFormat("GS int to send: %i", gs_10);
    char buff[4];
    buff[0] = '$';
    buff[1] = 'V';
    buff[2] = *((char*)&gs_10);
    buff[3] = *((char*)&gs_10 + 1);
    port.Write(buff, 4);
    }
  if (basic.ground_speed_available.IsValid() && basic.gps_vario_available.IsValid()){
    // TODO GPS update;
    P_kal +=  1e-3*0.1;
    float K_kal = P_kal / (P_kal + 1.0f); // 1.0f := noise of gps velocity
    P_kal = (1 - K_kal) * P_kal;
    x_kal_x += K_kal * (basic.ground_speed - x_kal_x);
    x_kal_z += K_kal * (-basic.gps_vario - x_kal_z);
    //lastStep.Update(basic.clock);
    int16_t i_vx100 = (int) x_kal_x * 100;
    int16_t i_vz100 = (int) x_kal_z * 100;
    char buff[6];
    buff[0] = '$';
    buff[1] = 'G';
    buff[2] = *((char*)&i_vx100);
    buff[3] = *((char*)&i_vx100 + 1);
    buff[4] = *((char*)&i_vz100);
    buff[5] = *((char*)&i_vz100 + 1);
    port.Write(buff, 6);


  }
}

bool
ArduinoINSDevice::DataReceived(const void *data, size_t length,
                            struct NMEAInfo &info)
{
  const char* _line = (const char*)data;
  if(length != 12){
    //LogFormat("length: %i", (int)length);
    return true;
  }
  if(_line[0] == '$' && _line[1] == 'A'){
    int16_t i_roll, i_pitch, i_yaw, i_vx, i_vz;
    *((char*)&i_roll) = _line[2];
    *((char*)&i_roll + 1) = _line[3];
    *((char*)&i_pitch) = _line[4];
    *((char*)&i_pitch + 1) = _line[5];
    *((char*)&i_yaw) = _line[6];
    *((char*)&i_yaw + 1) = _line[7];

    *((char*)&i_vx) = _line[8];
    *((char*)&i_vx + 1) = _line[9];
    *((char*)&i_vz) = _line[10];
    *((char*)&i_vz + 1) = _line[11];
    //LogFormat("R: %i, P: %i, Y: %i", i_roll, i_pitch, i_yaw);
    info.attitude.bank_angle_available.Update(info.clock);
    info.attitude.bank_angle = Angle::Degrees(0.1f * i_roll);

    info.attitude.pitch_angle_available.Update(info.clock);
    info.attitude.pitch_angle = Angle::Degrees(0.1f *i_pitch);

    info.attitude.heading_available.Update(info.clock);
    info.attitude.heading = Angle::Degrees(0.1f * i_yaw);
    x_kal_x = 0.01f * i_vx;
    x_kal_z = 0.01f * i_vz;
    LogFormat("Vz: %f", x_kal_z);
    //info.ProvideNoncompVario(-x_kal_z);
    info.ProvideTotalEnergyVario(-x_kal_z);
    //info.ground_speed(0.01 * i_vx);
    //info.ground_speed_available.Update(info.clock)
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
