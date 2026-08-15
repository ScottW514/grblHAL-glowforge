/*
  grbl_eeprom_extensions.h - checksummed block copy over the NVS byte interface

  Part of grblHAL-glowforge (derived from the Grbl Simulator)

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL.  If not, see <http://www.gnu.org/licenses/>.

  SPDX-License-Identifier: GPL-3.0-or-later
*/
#include "grbl/nvs.h"

bool memcpy_to_eeprom(uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum);
bool memcpy_from_eeprom(uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum);
