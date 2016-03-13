/* --------------------------------------------------------------------------                                 
    Template project main
    File name: template.c
    Author: Etienne Le Sueur
    Description: The template main file. It provides a simple example of using
                 some standard scandal functions such as the UART library, the
                 CAN library, timers, LEDs, GPIOs.
                 It is designed to compile and work for the 3 micros we use on
                 the car currently, the MSP430F149 and MCP2515 combination and
                 the LPC11C14 and LPC1768 which have built in CAN controllers.

                 UART_printf is provided by the Scandal stdio library and 
                 should work well on all 3 micros.

                 If you are using this as the base for a new project, you
                 should first change the name of this file to the name of
                 your project, and then in the top level makefile, you should
                 change the CHIP and MAIN_NAME variables to correspond to
                 your hardware.

                 Don't be scared of Scandal. Dig into the code and try to
                 understand what's going on. If you think of an improvement
                 to any of the functions, then go ahead and implement it.
                 However, before committing the changes to the Scandal repo,
                 you should discuss with someone else to ensure that what 
                 you've done is a good thing ;-)

                 Keep in mind that this code is live to the public on
                 Google Code. No profanity in comments please!

    Copyright (C) Etienne Le Sueur, 2011

    Created: 07/09/2011
   -------------------------------------------------------------------------- */

/* 
 * This file is part of the Sunswift Template project
 * 
 * This tempalte is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with the project.  If not, see <http://www.gnu.org/licenses/>.
 */

/* We have two types of GPS modules. Their default baud rates are
 * different, so we have to separate them unfortunately.
 * The LOCOSYS is the newer one that can acquire like lightning (Node 31). */
#define SANJOSE
#undef LOCOSYS

#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>

#include <project/bmp085.h>

#include <string.h>

#if defined(lpc11c14) || defined(lpc1768)
#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/can.h>
#include <arch/uart.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/i2c.h>
#else
#ifdef msp430f149
#include <msp430x14x.h>
#include <signal.h>
#include <project/hardware.h>

uint8_t gps_lock = 0;

/* Set up the clocks on the MSP430f149. Use XTAL2, which is externally attached */
void init_clock(void) {
	volatile unsigned int i;

	/* XTAL = LF crystal, ACLK = LFXT1/1, DCO Rset = 4, XT2 = ON */
	BCSCTL1 = 0x04;

	/* Clear OSCOFF flag - start oscillator */
	_BIC_SR( OSCOFF );

	do {
		/* Clear OSCFault flag */
		IFG1 &= ~OFIFG; 
		/* Wait for flag to set */
		for( i = 255; i > 0; i-- )
			;
	} while(( IFG1 & OFIFG ) != 0);

	/* Set MCLK to XT2CLK and SMCLK to XT2CLK */
	BCSCTL2 = 0x88; 
} // init_clock

/* The interrupt line on the MCP2515 CAN controller can be connected to
 * any interrupt enabled GPIO, hence we allow the developer to handle these
 * functions */
void enable_can_interrupt(){
	P2IE = CAN_INT;
}

void disable_can_interrupt(){
	P2IE = 0x00;
}

interrupt (PORT2_VECTOR) port2int(void) {
	can_interrupt();
	P2IFG = 0x00;
}

#endif // msp430f149
#endif // lpc11c14 || lpc1768

#include <project/nmea.h>
#include <project/rtc_mcp79410.h>

/* Do some general setup for clocks, LEDs and interrupts
 * and UART stuff on the MSP430 */


// resync timer every second
void gps_timer_handler() {
    sc_init_timer_1();
    GPIO_IntClear(2, 10);
}

void setup(void) {
#if defined(lpc11c14) || defined(lpc1768)
    
	GPIO_Init();
	GPIO_SetDir(RED_LED_PORT, RED_LED_BIT, 1);
	GPIO_SetDir(YELLOW_LED_PORT, YELLOW_LED_BIT, 1);
    // Register the GPS timer sync
    GPIO_SetFunction(2, 10, GPIO_PIO, GPIO_MODE_NONE);
	GPIO_SetDir(2, 10, 0);
    GPIO_RegisterInterruptHandler(2, 10, 0, 0, 1, &gps_timer_handler);

	InitRTC();
    SetTime("000000"); // FOR DEBUG PURPOSE, REMOVE LATER
	scandal_naive_delay(100000);
	
	// StartOsc();
	   
#else
#ifdef msp43f149
	init_clock();

	P1OUT = 0x00;
	P1SEL = 0x00;
	P1DIR = 0x00;
	P1IES = 0x00;
	P1IE  = 0x00;
	
	P2OUT = 0x00;
	P2SEL = 0x00;
	P2DIR = 0x00;
	P2IES = CAN_INT;
	P2IE  = 0x00;

	P3OUT = 0x00;
	P3SEL = TX | RX;
	P3DIR = TX;

	P4OUT = 0x00;
	P4SEL = 0x00;
	P4DIR = 0x00;

	P5OUT = CAN_CS;
	P5SEL = SIMO1 | SOMI1 | UCLK1;
	P5DIR = CAN_CS | SIMO1 | UCLK1 | YELLOW_LED_BIT | RED_LED_BIT;

	P6SEL = MEAS_12V_PIN;
    
#endif // msp430f149
#endif // lpc1768 || lpc11c14
} // setup

/* Send a command to the GPS
 * See:
 *  http://www.sparkfun.com/datasheets/GPS/Modules/PMTK_Protocol 
 * and
 *  http://www.sparkfun.com/datasheets/GPS/EB-230-Data-Sheet-V1.2.pdf
 */ 



void mtk_send_command(char* command){
	uint8_t  checksum = 0; 
	int i; 

	/* Send what we have first, and then calculate checksums, etc while
	 its sending */ 
	UART_printf("%s", command);

	for(i=0; command[i] != '$'; i++)
		;

	/* Get to the character after the $ */ 
	i++;

	/* Calculate the checksum */ 
	for(; (command[i] != '*') && (command[i] != '\0'); i++)
		checksum ^= command[i];

	if(command[i] != '*')
		UART_putchar('*');

	UART_putchar(checksum);

	UART_printf("\r\n");
}

/* This is your main function! You should have an infinite loop in here that
 * does all the important stuff your node was designed for */
int main(void) {
    int initialGPSLock = 1;
	char nmea_buf_1[NMEA_BUFFER_SIZE];
	char nmea_buf_2[NMEA_BUFFER_SIZE];
	char *nmea_current_buf;
	struct UART_buffer_descriptor nmea_buf_desc_1, nmea_buf_desc_2;
	gps_point cur_point; 
	gps_speed cur_speed; 
	sc_time_t cur_point_stamp = 0; 
	sc_time_t last_timesync_time = 0; 
	uint32_t gga_parse_errors = 0;
	uint32_t next_baro_read=0;
	uint32_t next_rtc_time=0;

	/* We allow some time for the GPS to come up before we continue here */
	scandal_naive_delay(100000);
	setup();
	scandal_naive_delay(100000);

	scandal_init();
	scandal_delay(1000);

	/* Initialise the UART to the correct GPS baud rate */
#if defined(LOCOSYS)
	UART_Init(57600);
#else
#if defined(SANJOSE)
	UART_Init(38400);
#endif
#endif
	scandal_delay(1000); /* wait for the UART clocks to settle */

	sc_time_t one_sec_timer = sc_get_timer(); /* Initialise the timer variable */

	/* Set LEDs to known states */
	red_led(1);
	yellow_led(0);

	// Set up the barometer
	long b5, pres, temp, alt, up, ut;
	readCalibrationValues();		//read calibration values

	/* Some GPS config stuff that isn't really necessary */

	/* We can send a reset command if need be
	mtk_send_command("$PMTK103"); */

	/* Set which messages to send out
	 * This sets us to receive GPRMC and GPGGA on every position fix */
	mtk_send_command("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0");

	/* put in 5Hz mode
	mtk_send_command("$PMTK220,200"); */

	/* Change the rate at which we fix the position. Presently every 1000ms */
	mtk_send_command("$PMTK300,1000,0,0,0,0*");

	/* We need a double buffer for reading the GPS messages while we parse them */
	UART_init_double_buffer(&nmea_buf_desc_1, nmea_buf_1, NMEA_BUFFER_SIZE,
								&nmea_buf_desc_2, nmea_buf_2, NMEA_BUFFER_SIZE);

	/* This is the main loop, go for ever! */
	while (1) {
		/* This checks whether there are pending requests from CAN, and sends a heartbeat message.
		 * The heartbeat message encodes some data in the first 4 bytes of the CAN message, such as
		 * the number of errors and the version of scandal */
		handle_scandal();

		if (sc_get_timer() > next_rtc_time){
		    ReadTime();
            UART_printf("Scandal Get Timer: %d\r\n", sc_get_timer_1());
		    next_rtc_time+=250;
		}

		/* Read a line from the UART into one of the buffers */
		nmea_current_buf = UART_readline_double_buffer(&nmea_buf_desc_1, &nmea_buf_desc_2);

		/* UART_readline_double_buffer will return a pointer to the current buffer. */
		if (nmea_current_buf != NULL) {

			/* If we didn't get a valid NMEA line */
			if(validate_nmea(nmea_current_buf) != 0)
				continue;

			/* check to see if we have a GGA message */ 
			if(strncmp(nmea_current_buf, "$GPGGA", 6) == 0){
				int res = parse_msg_gga(nmea_current_buf, &cur_point);
				if(res == 0){
                    if(initialGPSLock == 1) {
                        // Syncs GPS clock with RTC
                        SetTime(get_gga_time_array());
                        SetDate(get_rmc_date_array());
                        initialGPSLock = 0;
                    }
				   /*If the GPS is locked (res==0), send GPS data and set the RTC*/
					toggle_yellow_led();
                    
					cur_point_stamp = scandal_get_realtime32();
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_FIX,
											1, cur_point_stamp);
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_TIME,
											cur_point.time, cur_point_stamp);
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_LATITUDE,
											cur_point.lat, cur_point_stamp);
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_LONGITUDE,
											cur_point.lng, cur_point_stamp);
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_ALTITUDE,
											cur_point.alt, cur_point_stamp);
					/* Only sets the RTC if the seconds value is under 58
					 * This is done to make sure nothing ticks over in the
					 * middle of a write process as strange values may result
					 */
                    
				/* an actual parse error */
				} else if (res == -1) {
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_GGA_PARSE_ERROR_COUNT,
											gga_parse_errors++, cur_point_stamp);
				/* no fix yet */
				} else if (res == -2) {
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_FIX,
											0, cur_point_stamp);
					//gps_lock=0;
				}
			}

			/* check to see if we have an RMC message */ 
			if(strncmp(nmea_current_buf, "$GPRMC", 6) == 0) {
				if(parse_msg_rmc(nmea_current_buf, &cur_speed) == 0) {
					toggle_red_led();

					/* Milliseconds since the epoch */
                    /* Converts date to milliseconds, adds time */
					uint64_t timestamp = days_since_epoch(getRTCDay(), getRTCMonth(), getRTCYear());
					timestamp *= 3600;
					timestamp *= 24;
					timestamp *= 1000;
                    timestamp += getRTCTimeSecond() * 1000;
                    timestamp += sc_get_timer_1();
                    
                    UART_printf("Day:%d Month:%d Year:%d\r\n", getRTCDay(), getRTCMonth(), getRTCYear());
                    
                    scandal_send_timesync(CRITICAL_PRIORITY, scandal_get_addr(), timestamp);
                    
                    scandal_set_realtime(timestamp);
                    
                    scandal_send_channel(CRITICAL_PRIORITY, GPSBAROMETER_SPEED, cur_speed.speed * 1000);
                    scandal_send_channel(CRITICAL_PRIORITY, GPSBAROMETER_MILLISECONDS_TODAY, cur_speed.time);
                    scandal_send_channel(CRITICAL_PRIORITY, GPSBAROMETER_DAYS_SINCE_EPOCH, cur_speed.date);
                    
					/* This is an evil hack to make sure that we get fairly consistent timestamps 
					Sometimes there seems to be a really long dela of ~0.3s on some timestamp
					messages. To get rid of this, we don't accept any time differences that are 
					more than 20ms later than we expect them to be. 
					This is pure evil, and the problem should really be fixed rather than 
					hacking around it like this */
					/*
                    if(last_timesync_time == 0)
						last_timesync_time = sc_get_timer();

					timediff = (sc_get_timer() - last_timesync_time) % 1000;
					if((timediff < 50) || (timediff > 600)) {
                        
						last_timesync_time = sc_get_timer();
						scandal_send_timesync(CRITICAL_PRIORITY, scandal_get_addr(), timestamp);
					}

					scandal_set_realtime(timestamp);

					scandal_send_channel(CRITICAL_PRIORITY, GPSBAROMETER_SPEED, cur_speed.speed * 1000);
					scandal_send_channel(CRITICAL_PRIORITY, GPSBAROMETER_MILLISECONDS_TODAY, cur_speed.time);
					scandal_send_channel(CRITICAL_PRIORITY, GPSBAROMETER_DAYS_SINCE_EPOCH, cur_speed.date);
                    */
					/*                    if(cur_point_stamp  != 0){
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_TIME,
													cur_point.time, cur_point_stamp);
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_LATITUDE,
													cur_point.lat, cur_point_stamp);
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_LONGITUDE,
													cur_point.lng, cur_point_stamp);
					scandal_send_channel_with_timestamp(CRITICAL_PRIORITY, GPSBAROMETER_ALTITUDE,
													cur_point.alt, cur_point_stamp);
					cur_point_stamp = 0; 
					}
					*/

				}
			}
		}

#if 1
		if (sc_get_timer() > next_baro_read){

		    ut = bmp085ReadUT();					//read uncompensated temperature
		    scandal_delay(5);					//delay 4.5ms
		    up = bmp085ReadUP();					//read uncompensated pressure
		    scandal_delay(5);					//delay 4.5ms
		    b5 = bmp085Getb5(ut);				//calculate temperature constant
		    temp = bmp085GetTemperature(ut, b5);			//calculate true temperature
		    pres = bmp085GetPressure(up, b5);		//calculate true pressure
		    alt = bmp085GetAltitude(pres);				//estimate the altitude
		    next_baro_read=(sc_get_timer()+1000);
		}

#endif
		//UART_printf("B5: %d,Temp: %d:%d, Pres: %d:%d, Alt: %d\r\n", (int) b5,(int) ut, (int) temp, (int)up,(int) pres, (int) alt);
//		UART_printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",(int) barometerCal -> AC1,(int) barometerCal -> AC2,(int) barometerCal -> AC3,(int) barometerCal -> AC4,(int) barometerCal -> AC5,(int) barometerCal -> AC6,(int) barometerCal -> B1,(int) barometerCal -> B2,(int) barometerCal -> MB,(int) barometerCal -> MC,(int) barometerCal -> MD);

		#if 0
		/* Flash an LED every second */
		if(sc_get_timer() >= one_sec_timer + 1000) {
			toggle_red_led();
			one_sec_timer = sc_get_timer();
		}
#endif
	}
}

