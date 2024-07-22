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

#include <avr/io.h>
#include <math.h>
#include "kalman.h"

#include "../config.h"
#if RADIO_MODE==1

//Liczba próbek wy¿arzania filtra
//uint8_t kalman_prep;

//Zmienne do filtru kalmana
s_kalman klat;
s_kalman klon;
s_kalman kalt;
s_kalman kvario;

double kalman(double zk, double *xk, double *Pk, double R, double Q) {
	//double xk_1 = *xk;
	//double Pk_1 = *Pk;

	//Faza predykcji
	//*Xk = Xk_1;
	*Pk = *Pk + Q;

	//Faza korekcji
	double Kk = *Pk / (*Pk + R);
	*xk = *xk + Kk*(zk - *xk);
	*Pk = (1 - Kk)*(*Pk);

	//Zwrot nowej estymaty
	return *xk;
}

//Sprawdzenie czy filtr jest w trakcie wy¿arzania
//0 - wy¿arzanie trwa
//1 - wy¿arzanie zakoñczone
uint8_t filter_ready(double a, double b, double wsp) {
	if (fabs(a-b) <= wsp) return 1;
	else return 0;
}

void filter_reset() {
	klat.ready = 0;
	klon.ready = 0;
	kalt.ready = 0;
	//klat.cnt = FILTER_PREP;
	//klon.cnt = FILTER_PREP;
	//kalt.cnt = FILTER_PREP;
}

#endif
