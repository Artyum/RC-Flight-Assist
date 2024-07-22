/*
 * This file is part of RC Flight Assist (RCFA).
 *
 * RCFA is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RCFA is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RC Flight Assist. If not, see <http://www.gnu.org/licenses/>.
 *
 * Created for F3A pattern flying training
 * Developed in 2014/2015
 */

#include "../config.h"
#if RADIO_MODE==0

#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "radio_rx.h"
#include "../RCFA/rcfa_rx.h"
#include "../RCFA/rcfa.h"
#include "../SPI/spi.h"
#include "../UART/uart.h"
#include "../GPS/gps.h"
#include "../SOUND/sound.h"

#if PCB_REVISION==0
#include "../CPU/cpu_1.0.h"
#elif PCB_REVISION==1
#include "../CPU/cpu_1.1.h"
#endif

//Bufor odczytu z REG_FIFO
uint8_t rxdtab[14];

//Kolejka odtwarzania
uint8_t q[2];

void radio_hop_rx(int8_t num) {
	int8_t sub;

	//Ustalenie nast�pnego kana�u
	sub = fhss.ch + num;
	if (sub>=FHSS_NUM) sub = sub-FHSS_NUM;
	if (sub<0) sub = FHSS_NUM+sub;
	fhss.ch = sub;

	//uart_putul(fhss.ch,10); uart_nl();

	//Ustawienie cz�stotliwo�ci
	radio_set_freq(get_freq(fhss.list[fhss.ch]));
	radio_mode_fs();
	radio_mode_rx();
}

void radio_init_rx() {
	//SPI - po��czenie z radiem
	spi_enable(SPI_MODE_4);

	//Sprawdzenie czy jest po��czenie z radiem
	radio.enabled = radio_detect();
	if (!radio.enabled) {
		while (1) {
			play_freq(FC5,1000,100);
			LED1_ON; LED2_OFF;
			delay_ms(SYSTEM_STOP_DELAY);
			play_freq(FD5,1000,100);
			LED1_OFF; LED2_ON;
			delay_ms(SYSTEM_STOP_DELAY);
		}
	}

	//Wsp�lne ustawienia dla Rx i Tx
	radio_init_common();

	//Przerwania DioMapping
	//radio_cmd(REG_DIOMAPPING1,0b01000000);	//DIO0 Payload Ready
	//radio_cmd(REG_DIOMAPPING2,0b00000111);	//CLKOUT Off

	//Over Current Protection
	radio_cmd(REG_OCP,0b00011010);		//OCP enabled | 95mA

	//Test Registers
	radio_cmd(REG_TESTLNA,0x1B);		//Sensitivity boost - Normal mode
	radio_cmd(REG_TESTPA1,0x55);		//Normal mode RX or PA0
	radio_cmd(REG_TESTPA2,0x70);		//Normal mode RX or PA0
	radio_cmd(REG_TESTDAGC,0x30);		//Dagc On
	//radio_cmd(REG_TESTDAGC,0x20);		//Dagc On

	//RegLna
	radio_cmd(REG_LNA,0x00);			//LNA 50ohms | gain set by internal AGC

	//Wej�cie w tryb odbioru
	radio_mode_rx();
}

void radio_rx_data() {
	static uint8_t siglost;
	static uint16_t cnt_info;
	static uint16_t cnt_alarm;

	//if (!rf_ModeRxReady()) return;
	//if (!rf_PayloadReady()) return;

	//Blokada wielokrotnego odbierania danych
	if (cnt_info>0) cnt_info--;

	//Wy��czenie d�wi�ku alarmu je�li zbyt d�ugo nie odbierano danych
	if (cnt_alarm>0) cnt_alarm--;
	else snd.type &= ~(SND_TYPE_ALARM | SND_TYPE_VALARM);

	//Brak odebranych danych
	if (!INT_CRCOK) {
		if (t_rx==0) {
			//uart_putc('$');
			#if DISABLE_SND_NO_SIG==0
			snd.type |= SND_TYPE_NOSIG;
			#endif

			//Przesuni�cie cz�stotliwo�ci odbiorczej
			if (siglost<1) {
				radio_hop_rx(2);
				t_rx = RADIO_TX*4;
				siglost++;
				//uart_putc('+');
			}
			//Poszukiwanie sygna�u
			else if (siglost<10) {
				radio_hop_rx(1);
				t_rx = RADIO_TX;
				siglost++;
				//uart_putc('.');
			}
			//Przeskok o po�ow� pasma i oczekiwanie
			else if (siglost==10) {
				radio_hop_rx(FHSS_NUM/2-1);
				t_rx = (FHSS_NUM*15);
				siglost++;
				//uart_putc('~');
			}
			//Brak sygna�u - przeskok na cz�stotliwo�� pocz�tkow�
			else if (siglost==11) {
				radio_set_freq(get_freq(fhss.list[0]));
				siglost++;
				//uart_putc('@');
			}
		}
		return;
	}

	//Czas do nast�pnego odbioru
	t_rx = RX_MAX_WAIT;
	siglost = 0;

	//Liczba odebranych bajt�w
	rxdtab[0] = 0;

	//Odczyt bufora
	uint8_t i = 0;

	//while (rf_FifoNotEmpty()) dane[i++] = spi_read(REG_FIFO);
	//while (INT_FIFO_NOT_EMPTY) dane[i++] = spi_read(REG_FIFO);

	//Wy��czenie Rx na czas odczytu Fifo
	radio_mode_standby();
	//radio_mode_fs();

	//Burst read
	spi_open();
	SPDR = (0x7F & REG_FIFO);
	while (!(SPSR & (1<<SPIF)));
	while (INT_FIFO_NOT_EMPTY) {
		SPDR = (0x7F & REG_FIFO);
		while (!(SPSR & (1<<SPIF)));
		rxdtab[i++] = SPDR;
	}
	spi_close();

	//FHSS hop
	radio_hop_rx(1);

	//rxd.gain = getLnaCurrentGain();
	//rxd.rssi = getRssiValue();

	//1 bajt - Numsat
	if (rxdtab[0]==1) {
		rxd.numsat = (uint8_t)rxdtab[1];
		if (rxd.numsat<GPS_MIN_NUMSAT) snd.type |= SND_TYPE_NOFIX;

		#if CHK_RX==1
		uart_putsP(PSTR("1"));
		#endif

		return;
	}

	//2 bajty - Type + Alarm|VAlarm
	else if (rxdtab[0]==2) {
		uint8_t type = (rxdtab[1] & 0x80)>>7;
		int16_t alarm = ((uint16_t)(rxdtab[1] & 0x7F)<<8) | ((uint16_t)rxdtab[2]);
		alarm -= ALARM_PUSH;

		cnt_alarm = CNT_ALARM_TIME;

		if (type==TX2_TYPE_ALARM) {
			rxd.alarm = alarm;
			snd.type |= SND_TYPE_ALARM;
			#if CHK_RX==1
			uart_putsP(PSTR("2a"));
			#endif
		}
		else {
			rxd.valarm = alarm;
			snd.type |= SND_TYPE_VALARM;
			#if CHK_RX==1
			uart_putsP(PSTR("2b"));
			#endif
		}

		return;
	}

	//3 bajty - type + pos
	else if (rxdtab[0]==3) {
		//Je�li nie ma pozycji referencyjnej to wyj�cie
		if ((rxd.status & RX_REFPOS_SET)==0) return;

		//Znacznik type
		uint8_t type = (rxdtab[1] & 0xC0)>>6;

		//Znacznik czasu
		uint8_t t = (rxdtab[1] & 0x20)>>5;

		if ((nmea.t!=t) || (nmea.t<0) || (t_rxpos==0)) {
			nmea.t = t;
			nmea.np0 = 0;
			nmea.np1 = 0;
			nmea.np2 = 0;
		}

		//Ustawienie czasu oczekiwania na kolejny odczyt
		t_rxpos = RX_POS_WAIT;

		uint32_t tmp = ((uint32_t)rxdtab[1]<<16UL) | ((uint32_t)rxdtab[2]<<8UL) | ((uint32_t)rxdtab[3]);

		if (type==TX3_TYPE_NMEA0) {
			nmea.np0 = tmp;
			#if CHK_RX==1
			uart_putsP(PSTR("3a"));
			#endif
		}
		else if (type==TX3_TYPE_NMEA1) {
			nmea.np1 = tmp;
			#if CHK_RX==1
			uart_putsP(PSTR("3b"));
			#endif
		}
		else if (type==TX3_TYPE_NMEA2) {
			nmea.np2 = tmp;
			#if CHK_RX==1
			uart_putsP(PSTR("3c"));
			#endif
		}

		if ((nmea.np0!=0) && (nmea.np1!=0) && (nmea.np2!=0)) {
			nmea_decode();
			nmea.t = -1;
			#if CHK_RX==1
			uart_putsP(PSTR("3!"));
			#endif
		}

		return;
	}

	//4 bajty - Konfiguracja odbiornika | Informacje
	else if (rxdtab[0]==4) {
		if (rxdtab[1]==INFO_CONFIG) {
			if (cnt_info>0) return; else cnt_info = CNT_INFO_TIME;

			rx_init();

			snd.type = SND_TYPE_CONF;
			rxd.status = RX_CONFIG_SET;

			//G�o�no��
			snd.vol = rxdtab[2]*10;

			//Tryb vario
			conf.vmodeinit = rxdtab[3];

			//Nieu�ywane
			//rxdtab[4];

			//Pocz�tkowe ustawienie trybu wariometru
			if (conf.vmodeinit<=1) rxd.variomode=0;
			else rxd.variomode=1;

			#if CHK_RX==1
			uart_putsP(PSTR("C"));
			#endif
			return;
		}

		else if (rxdtab[1]==INFO_P1) {
			if (cnt_info>0) return; else cnt_info = CNT_INFO_TIME;

			snd.type |= SND_TYPE_P1;
			rxd.status &= ~(RX_RTB_SET | RX_BLK_SET);

			#if CHK_RX==1
			uart_putsP(PSTR("P1"));
			#endif
			return;
		}

		else if (rxdtab[1]==INFO_P2) {
			if (cnt_info>0) return; else cnt_info = CNT_INFO_TIME;

			snd.type |= SND_TYPE_P2;
			rxd.status &= ~(RX_RTB_SET | RX_BLK_SET);

			#if CHK_RX==1
			uart_putsP(PSTR("P2"));
			#endif
			return;
		}

		//Odebrano znacznik strefy gotowej do zablokowania
		else if (rxdtab[1]==INFO_RTB) {
			rxd.status |= RX_RTB_SET;

			#if CHK_RX==1
			uart_putsP(PSTR("Rtb"));
			#endif
			return;
		}

		//Odebrano znacznik strefy zablokowanej
		else if (rxdtab[1]==INFO_BLK) {
			rxd.status |= RX_BLK_SET;

			#if CHK_RX==1
			uart_putsP(PSTR("Blk"));
			#endif
			return;
		}

		//Znacznik ustawienia wysoko�ci referencyjnej
		else if (rxdtab[1]==INFO_REFALT_SET) {
			rxd.status |= RX_REFALT_SET;

			#if CHK_RX==1
			uart_putsP(PSTR("Alt"));
			#endif
			return;
		}
	}

	//10 bajt�w - Pozycja referencyjna + data
	else if (rxdtab[0]==10) {
		char str[3];
		uint8_t i;
		int32_t tmp;

		//Sk�adanie daty ddmmyy 01|23|45|\0
		strcpy(nmea.cdate,"");
		//dd
		i = (rxdtab[1] & 0x1F);
		if (i<10) nmea.cdate[0]='0'; itoa(i,str,10); strcat(nmea.cdate,str);
		//mm
		i = ((rxdtab[1] & 0xE0)>>5) | ((rxdtab[2] & 0x1)<<3);
		if (i<10) nmea.cdate[2]='0'; itoa(i,str,10); strcat(nmea.cdate,str);
		//yy
		i = ((rxdtab[2] & 0xFE)>>1);
		if (i<10) nmea.cdate[4]='0'; itoa(i,str,10); strcat(nmea.cdate,str);
		nmea.cdate[6]='\0';

		//Lat
		tmp = ((uint32_t)rxdtab[3]) | ((uint32_t)rxdtab[4]<<8) | ((uint32_t)rxdtab[5]<<16UL) | ((uint32_t)rxdtab[6]<<24UL);
		nmea.ref.lat = (double)((int32_t)tmp-(int32_t)((int32_t)LAT_PUSH*(int32_t)ACC6))/(double)ACC6;

		//Lon
		tmp = ((uint32_t)rxdtab[7]) | ((uint32_t)rxdtab[8]<<8) | ((uint32_t)rxdtab[9]<<16UL) | ((uint32_t)rxdtab[10]<<24UL);
		nmea.ref.lon = (double)((int32_t)tmp-(int32_t)((int32_t)LON_PUSH*(int32_t)ACC6))/(double)ACC6;

		//uart_puts("Ref.="); uart_putd(nmea.ref.lat,6); uart_puts("|"); uart_putd(nmea.ref.lon,6); uart_puts(" data="); uart_puts(nmea.cdate); uart_nl();

		//snd.type |= SND_TYPE_REFPOS;
		rxd.status |= RX_REFPOS_SET;

		#if CHK_RX==1
		uart_putsP(PSTR("Ref"));
		#endif
		return;
	}
}

#endif
