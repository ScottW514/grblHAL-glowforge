/*
  eeprom.c - replacement for the avr library of the same name to provide
  replacement functionality - write to "EEPROM.dat" in working directory

  Part of Grbl Simulator

  Copyright (c) 2012 Jens Geisler
  Copyright (c) 2014 Adam Shelly

  2020 - modified for grblHAL by Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "fflog.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>

#define MAX_EEPROM_SIZE 4096   // 4KB EEPROM

static char eeprom_file[128];

bool set_eeprom_name (const char *name)
{
    if (!name || strlen(name) >= sizeof(eeprom_file))
        return false;
    strcpy(eeprom_file, name);
    return true;
}

static FILE *eeprom_create_empty_file (void)
{
    int i;
    FILE* fp = fopen(eeprom_file, "w+b");

    if (fp) {
        for(i = 0; i < MAX_EEPROM_SIZE; i++)
            fputc(0xFF, fp);

        fseek(fp, 0, SEEK_SET);
    }

    return fp;
}

static FILE* EEPROM_FP = NULL;

// RAM-backed fallback when the settings file can neither be opened nor
// created (read-only or full filesystem): the controller keeps running on
// defaults for the life of the process instead of faulting on the first
// settings access. Reads as erased until written.
static uint8_t eeprom_ram[MAX_EEPROM_SIZE];
static bool eeprom_volatile = false;

static FILE *eeprom_fp (void)
{
    static int tried = 0;

    if (!EEPROM_FP && !tried) {
        tried = 1;
        EEPROM_FP = fopen(eeprom_file, "r+b");
        if (!EEPROM_FP) {
            EEPROM_FP = eeprom_create_empty_file();
        }
        if (!EEPROM_FP) {
            fflog(LOG_ERR, "eeprom: cannot open or create %s: %s - "
                           "settings are volatile for this run",
                  eeprom_file, strerror(errno));
            memset(eeprom_ram, 0xFF, sizeof(eeprom_ram));
            eeprom_volatile = true;
        }
    }

    return EEPROM_FP;
}

void eeprom_close (void)
{
    // NOTE: may run from the exit handler while the grbl thread never got as
    // far as opening the file - do not open (or close) it on its behalf here.
    if (EEPROM_FP) {
        fclose(EEPROM_FP);
        EEPROM_FP = NULL;
    }
}

uint8_t eeprom_get_char(uint32_t addr )
{
    FILE* fp = eeprom_fp();

    if (addr >= MAX_EEPROM_SIZE)
        return 0xFF; //no such address: reads as erased

    if (!fp)
        return eeprom_volatile ? eeprom_ram[addr] : 0xFF;

    if (fseek(fp, addr, SEEK_SET))
        return 0xFF; //no such address

    return fgetc(fp);
}

void eeprom_put_char(uint32_t addr, uint8_t new_value )
{
    FILE* fp = eeprom_fp();

    if (addr >= MAX_EEPROM_SIZE)
        return; //no such address

    if (!fp) {
        if (eeprom_volatile)
            eeprom_ram[addr] = new_value;
        return;
    }

    if (fseek(fp, addr, SEEK_SET))
        return; //no such address

    fputc(new_value, fp);
    fflush(fp);
}

// end of file
