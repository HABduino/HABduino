
/* From Project Swift - High altitude balloon flight software                 */
/*=======================================================================*/
/* Copyright 2010-2012 Philip Heron <phil@sanslogic.co.uk>               */
/*                     Nigel Smart <nigel@projectswift.co.uk>            */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program.  If not, see <http://www.gnu.org/licenses/>. */

//#define APRS // Uncomment to use APRS.

#ifndef __AX25MODEM_H
#define __AX25MODEM_H
#define APRS_TX_INTERVAL 1  // APRS TX Interval in minutes
#define APRS_CALLSIGN "CHANGEME"
#define APRS_SSID     (11)

extern void ax25_init(void);
extern void ax25_frame(char *scallsign, char sssid, char *dcallsign, char dssid,
	char *path1, char ttl1, char *path2, char ttl2, char *data, ...);
extern char *ax25_base91enc(char *s, uint8_t n, uint32_t v);

#endif

