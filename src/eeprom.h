/*
  eeprom.h - file-backed NVS for the Linux build

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
#include <stdint.h>
#include <stdbool.h>

void eeprom_close (void);
bool set_eeprom_name (const char *name);
uint8_t eeprom_get_char (uint32_t addr );
void eeprom_put_char (uint32_t addr, uint8_t new_value );