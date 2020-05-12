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

#ifndef XCSOAR_AUDIO_TONE_SYNTHESISER_CONT_HPP
#define XCSOAR_AUDIO_TONE_SYNTHESISER_CONT_HPP

#include "ToneSynthesiser.hpp"
#include "Util/Compiler.h"

/**
 * This class generates sine wave tones with smooth pitch transitions.
 */
class ToneSynthesiserCont : public ToneSynthesiser {
  unsigned volume = 100, angle = 0, increment = 0;

public:
  explicit ToneSynthesiserCont(unsigned _sample_rate);

  void SetTone(unsigned tone_hz);

  /* methods from class PCMSynthesiser */
  virtual void Synthesise(int16_t *buffer, size_t n);

private:
// Low pass filter transposed time constant := 1 / (T_1 * f_sample + 1)
const float lpf_param;
float freq, freq_tgt; 
};

#endif
