/*
 * nmea.h
 *
 *  Created on: 17 февр. 2017 г.
 *      Author: Lockdog
 */

#ifndef NMEA_H_
#define NMEA_H_

#include "qm_common.h"
#include "qm_uart.h"
#include "string.h"
#include "inttypes.h"
#include "debug.h"
#include "stdlib.h"

#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"
#define PMTK_SET_SLEEP "$PMTK161,1*29\r\n"

#define TIMEOUT 100

typedef struct {
	uint8_t header[7];
	uint8_t time[12];
	uint8_t warning;
	uint8_t latitude[10];
	uint8_t ns[2];
	uint8_t longtitude[11];
	uint8_t ew[2];
	uint8_t speed;
	uint8_t course;
	uint8_t date;
	uint8_t magvar;
	uint8_t magdir;
	uint8_t checksum[6];
	uint8_t ret;
} gnss_gprmc_t;

typedef enum {
	NMEA_GPRMC
} nmea_header_t;

gnss_gprmc_t read_gprmc(void);

#endif /* NMEA_H_ */
