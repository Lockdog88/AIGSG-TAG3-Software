/*
 *  Copyright (c) 2016, Intel Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  3. Neither the name of the Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE INTEL CORPORATION OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "cli_parser.h"
#include "qm_common.h"
#include "qm_uart.h"
#include "qm_spi.h"
#include "qm_pinmux.h"
#include "qm_interrupt.h"
#include "qm_isr.h"
#include "clk.h"
#include "cli_config.h"
#include "cli_microrl.h"
#include "sx1272.h"
#include "string.h"
#include "nmea.h"
#include "rtc.h"
#include "lis3de.h"


#define TX (1)

//#define GATEWAY (1)
/* the SPI clock divider is calculated in reference to a 32MHz system clock */
#define SPI_CLOCK_125KHZ_DIV (256)
#define SPI_CLOCK_1MHZ_DIV (32)
#define SPI_CLOCK_DIV SPI_CLOCK_125KHZ_DIV
#define QM_SPI_BMODE QM_SPI_BMODE_0
#define BUFFERSIZE (128)
/* wait time expressed in usecs */
#define WAIT_4SEC 4000000

uint8_t show = 2;

//#define SERIAL "AiGsGSeR0\r\n"
//uint8_t serial[] = "AiGsGSeR0\r\n";

microrl_t rl;
microrl_t * prl = &rl;

gnss_gprmc_t nmea;

uint8_t msg[11];

void testing(void);
void showing(void);

/*  QMSI SPI app example */
int main(void)
{
	qm_pmux_select(QM_PIN_ID_12, QM_PMUX_FN_2); /* configure UART_A_TXD */
	qm_pmux_select(QM_PIN_ID_13, QM_PMUX_FN_2); /* configure UART_A_RXD */
	qm_pmux_input_en(QM_PIN_ID_13, true); /* UART_A_RXD is an input */
	microrl_init (prl, print);
	microrl_set_execute_callback (prl, execute);

	#ifdef _USE_COMPLETE
	// set callback for completion
		microrl_set_complete_callback (prl, complet);
	#endif
	// set callback for Ctrl+C
		microrl_set_sigint_callback (prl, sigint);
		debug_str("Starting. . .");

	qm_uart_status_t uart_status __attribute__((unused)) = 0;
	uint8_t msgBase __attribute__((unused)) = 1;
	debug_str("DEBUG START");
	rtc_hal_init();
	Radio_Init();
	setMode(RF92_MODE_SLEEP);
	lis3de_init();
	//uint8_t lis3de_sf, data_x;
	//lis3de_sf = lis3de_selftest();
	//QM_PRINTF("LIS3DE status: 0x%X\r\n", lis3de_sf);
	//while (1)
	//{
	//	i2c_read_register(LIS3DE_ADDR, LIS3DE_OUT_X, &data_x, 1);
	//	QM_PRINTF("X DATA: 0x%X\r\n", data_x);
	//	rtc_wait_sec(ALARM_SEC*2);
	//}
	led_blink(5, DELAY50);
	rtc_wait(ALARM_SEC*5);
	QM_PUTS(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	//QM_PUTS(PMTK_SET_SLEEP);
	#ifdef RX
	startReceiving();
	while(1) {
		receiveMessage(msg);
		qm_uart_get_status(QM_UART_0, &uart_status);
		if (uart_status & QM_UART_RX_BUSY)
		{
			uint8_t data;
			qm_uart_read(QM_UART_0, &data, &uart_status);
			microrl_insert_char (prl, data);
		}
	}
	#endif

	#ifdef GATEWAY
	startReceiving();
	while(1) {
		startReceiving();
		receiveMessage(msg, show);

		qm_uart_get_status(QM_UART_0, &uart_status);
		if (uart_status & QM_UART_RX_BUSY)
		{
			uint8_t data;
			qm_uart_read(QM_UART_0, &data, &uart_status);
			microrl_insert_char (prl, data);
		}
	}
	#endif
	#ifdef TX
	while(1)
	{
		if (show) showing(); else testing();
	}
    #endif
	while (1)
	{
		qm_uart_get_status(QM_UART_0, &uart_status);
		if (uart_status & QM_UART_RX_BUSY)
		{
			uint8_t data;
			qm_uart_read(QM_UART_0, &data, &uart_status);
			microrl_insert_char (prl, data);
		}
	}
}

void testing(void)
{
	nmea = read_gprmc();
	if (nmea.ret == 3) {
		uint8_t gnss_data[100];
		sprintf((char*)gnss_data, "[t: %s] | [crc: %s] | [h: %s]", nmea.time, nmea.checksum, nmea.header);
		setMode(RF92_MODE_STANDBY);
		sendData(gnss_data, sizeof(gnss_data));

		startReceiving();
		while(receiveMessage(msg, 0));
		//debug_str("SLEEP");
		//setMode(RF92_MODE_SLEEP);
		//QM_PUTS(PMTK_SET_SLEEP);
		rtc_wait(ALARM_SEC*5);

	} else if (nmea.ret == 0) {
		uint8_t gnss_data[100];
		sprintf((char*)gnss_data, "[t: %s] | [lat: %s] | [long: %s]", nmea.time, nmea.latitude, nmea.longtitude);
		setMode(RF92_MODE_STANDBY);
		sendData(gnss_data, sizeof(gnss_data));

		startReceiving();
		while(receiveMessage(msg, 0));
		//debug_str("SLEEP");
		//setMode(RF92_MODE_SLEEP);
		//QM_PUTS(PMTK_SET_SLEEP);
		rtc_wait(ALARM_SEC*5);
	} else debug_str("FAILED");
}

void showing(void)
{
	nmea = read_gprmc();
	if (nmea.ret == 3) {
		uint8_t gnss_data[100];
		sprintf((char*)gnss_data, "V,%s,%s", nmea.time, nmea.header);
		setMode(RF92_MODE_STANDBY);
		sendData(gnss_data, sizeof(gnss_data));
		rtc_wait(ALARM_SEC*5);

	} else if (nmea.ret == 0) {
		uint8_t gnss_data[100];
		sprintf((char*)gnss_data, "A,%s,%s,%s,%s,%s", nmea.time, nmea.latitude, nmea.longtitude, &nmea.ns[0], &nmea.ew[0]);
		setMode(RF92_MODE_STANDBY);
		sendData(gnss_data, sizeof(gnss_data));
		rtc_wait(ALARM_SEC*5);
	} else debug_str("FAILED");
}

