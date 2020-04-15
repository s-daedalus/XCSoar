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
  ArduinoINSDevice(Port &_port):port(_port) {}

  /* virtual methods from class Device */
  bool ParseNMEA(const char *line, NMEAInfo &info) override;
  void OnCalculatedUpdate(const MoreData &basic,
                  const DerivedInfo &calculated) override;
};

void
ArduinoINSDevice::OnCalculatedUpdate(const MoreData &basic, 
    const DerivedInfo &calculated)
{
  if (basic.ground_speed_available.IsValid()) {
    int16_t gs_10 = (int) 10 * basic.ground_speed;
    char buff[4];
    buff[0] = '$';
    buff[1] = 'V';
    buff[2] = (gs_10 & 0x00ff);
    buff[3] = (gs_10 & 0xff00) >> 8;
    port.Write(buff, 4);
    }
}

bool
ArduinoINSDevice::ParseNMEA(const char *_line, NMEAInfo &info)
{
  if(_line[0] == '$' && _line[1] == 'A'){
    float roll = (int)(_line[2] | (_line[3] << 8)) / 10;
    float pitch = (int)(_line[4] | (_line[5] << 8)) / 10;
    float yaw = (int)(_line[6] | (_line[7] << 8)) / 10;
    
    info.attitude.bank_angle_available.Update(info.clock);
    info.attitude.bank_angle = Angle::Degrees(roll / 10.);

    info.attitude.pitch_angle_available.Update(info.clock);
    info.attitude.pitch_angle = Angle::Degrees(pitch / 10.);

    info.attitude.heading_available.Update(info.clock);
    info.attitude.heading = Angle::Degrees(yaw / 10.);
    return true;
  }
  return false;
}


static Device *
ArduinoINSCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new ArduinoINSDevice(com_port);
}

const struct DeviceRegister arduino_ins_driver = {
  _T("ArduinoINS"),
  _T("ArduinoINS"),
  0,
  ArduinoINSCreateOnPort,
};
