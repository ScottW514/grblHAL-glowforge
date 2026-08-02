/*
  my_machine.h - build-time machine configuration

  Part of grblHAL-glowforge. Force-included into every translation unit
  (driver and grblHAL core alike) by CMake, so defines here override the
  core's #ifndef-guarded defaults in grbl/config.h.

  Copyright (c) 2026 Scott Wiederhold <s.e.wiederhold@gmail.com>
  SPDX-License-Identifier: GPL-3.0-or-later
*/
#pragma once

#include "boards/glowforge.h"
