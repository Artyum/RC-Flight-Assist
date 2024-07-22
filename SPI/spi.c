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
#include <util/delay.h>
#include "spi.h"
#include "../config.h"
#include "../UART/uart.h"
#include "../SD/sd.h"

#if PCB_REVISION==0
#include "../CPU/cpu_1.0.h"
#elif PCB_REVISION==1
#include "../CPU/cpu_1.1.h"
#endif

#if RADIO_MODE==0

void spi_enable(uint8_t mode) {
	SPI_DDR |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS_ORG);
	SPI_PORT |= (1<<SPI_MOSI);

	SPI_SS_DDR |= (1<<SPI_SS);
	SPI_SS_PORT |= (1<<SPI_SS);

	//CLK
	if (mode & SPI_MODE_16)			SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);				// F_CPU / 16
	else if (mode & SPI_MODE_64)	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);				// F_CPU / 64
	else if (mode & SPI_MODE_128)	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);		// F_CPU / 128
	else							SPCR = (1<<SPE)|(1<<MSTR);							// F_CPU / 4 (domy�lnie)

	//Double rate
	if (mode & SPI_MODE_DR)			SPSR |= (1<<SPI2X);

	delay_ms(1);
}

void spi_disable() {
	//SPCR = 0;
	//SPSR &= ~(1<<SPI2X);
	//SPI_DDR &= ~((1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS));
	//SPI_PORT &= ~((1<<SPI_MOSI)|(1<<SPI_SS));
	//delay_ms(1);
}

#else

void spi_enable(uint8_t mode) {
	//Wyj�cia MOSI, SCK
	SPI_DDR |= (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS_ORG);
	//SPI_PORT |= (1<<SPI_MOSI);

	//Wyj�cia SS_Radio i SS_SD
	SPI_SS_DDR |= (1<<SPI_SS)|(1<<SPI_SS_SD);
	SPI_SS_PORT |= (1<<SPI_SS)|(1<<SPI_SS_SD);

	//CLK
	if (mode & SPI_MODE_16)			SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);				// F_CPU / 16
	else if (mode & SPI_MODE_64)	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);				// F_CPU / 64
	else if (mode & SPI_MODE_128)	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);		// F_CPU / 128
	else							SPCR = (1<<SPE)|(1<<MSTR);							// F_CPU / 4 (domy�lnie)

	//Double rate
	if (mode & SPI_MODE_DR)			SPSR |= (1<<SPI2X);

	_delay_ms(0.5);
}

void spi_disable() {
	//Wy��czenie SPI | Stan wysoki na wej�ciach HiZ
	SPCR = 0;
	SPSR &= ~(1<<SPI2X);

	SPI_DDR &= ~((1<<SPI_MOSI)|(1<<SPI_SCK));
	SPI_PORT &= ~((1<<SPI_MOSI)|(1<<SPI_SCK));

	SPI_SS_DDR &= ~((1<<SPI_SS)|(1<<SPI_SS_SD));
	SPI_SS_PORT &= ~((1<<SPI_SS)|(1<<SPI_SS_SD));

	_delay_ms(0.5);
}

#endif

void spi_tx(char Data) {
	SPDR = Data;
	while (!(SPSR & (1<<SPIF)));
}

/*char spi_rx(void) {
	while (!(SPSR & (1<<SPIF)));
	return SPDR;
}*/

void spi_open() {
	BIT_LOW(SPI_SS_PORT,SPI_SS);
	//delay_ms(1);
	_delay_ms(0.5);
}

void spi_close() {
	BIT_HIGH(SPI_SS_PORT,SPI_SS);
	//delay_ms(1);
}

void spi_read_burst(uint8_t adr, uint8_t *tab, uint8_t cnt) {
	spi_open();

	spi_tx(0x7F & adr);
	while (cnt--) {
		SPDR = (0x7F & adr);
		while (!(SPSR & (1<<SPIF)));
		*tab = SPDR;
		tab++;
	}

	spi_close();
}

void spi_write_burst(uint8_t adr, uint8_t *tab, uint8_t cnt) {
	spi_open();

	spi_tx(0x80 | adr);
	while (cnt--) {
		spi_tx(*tab);
		tab++;
	}

	spi_close();
}

//Komunikacja z Radiem
void spi_write(uint8_t adr, uint8_t Data) {
	spi_open();

	//Wys�anie adresu z flag� 1 (write)
	spi_tx(0x80 | adr);

	//Wys�anie danych do zapisu
	spi_tx(Data);

	spi_close();
}

uint8_t spi_read(uint8_t adr) {
	uint8_t data;

	spi_open();

	//Wys�anie adresu z flag� 0 (read)
	SPDR = (0x7F & adr);	while (!(SPSR & (1<<SPIF)));
	SPDR = (0x7F & adr);	while (!(SPSR & (1<<SPIF)));
	data = SPDR;

	spi_close();

	return data;
}
