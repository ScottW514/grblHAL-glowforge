/*
  glowforge_switches.h - safety switch inputs (lid, interlock, e-stop sense)

  Part of grblHAL-glowforge

  Copyright (c) 2026 Scott Wiederhold

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

#ifndef _GLOWFORGE_SWITCHES_H_
#define _GLOWFORGE_SWITCHES_H_

#include "grbl/hal.h"

/* Opens the switch device and reads the initial state. Safe to call with no
   hardware present (null-sink mode), where every signal reads clear. */
void gfsw_init (void);

/* Current control-signal state; the hal.control.get_state backend. */
control_signals_t gfsw_get_state (void);

/* Re-reads the switches and reports changes to the core. Called from the
   protocol thread's realtime hook. */
void gfsw_poll (void);

#endif
