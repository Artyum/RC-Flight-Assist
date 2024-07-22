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

#include "rcfa.h"

#include "../UART/uart.h"
#include "../SD/sd.h"
#include "../config.h"

#if PCB_REVISION==0
#include "../CPU/cpu_1.0.h"
#elif PCB_REVISION==1
#include "../CPU/cpu_1.1.h"
#endif

//G³ówna zmianna F3A
s_rcfa rcfa;

//Zmienne z pliku ini
s_ini ini;

//Status inicjalizacji sprzêtowej
// Bit	0	Radio
//		1	GPS
//		2	Odczyt ini z SD
//      3   Wykrycie pliku log.txt
uint8_t hw_status;

s_hw hw;

double knots2kph(double k) { return k*1.852; }
double kph2knots(double k) { return k*0.539956803; }

#if FUNC_MAP==1
double map(double v, double f1, double f2, double t1, double t2) {
	if (v<f1) return t1;
	if (v>f2) return t2;
	double a = (v-f1)/(f2-f1);
	return (t2-t1)*a+t1;
}
#endif

void system_stop(void) {
	#if RADIO_MODE==1
	sd_logp(1, &txt86);
	#else
	uart_putsP(PSTR("System Stop"),1);
	#endif

	LED1_ON;
	LED2_OFF;
	while (1) {
		delay_ms(SYSTEM_STOP_DELAY);
		LED1_TOG;
		LED2_TOG;
	}
}
