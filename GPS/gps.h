#ifndef _GPS_H_
#define _GPS_H_

#include <avr/io.h>
#include "../config.h"
#include "../RCFA/rcfa.h"

//6 miejsc po przecinku dla wsp�rz�dnych GPS
#define ACC6				1000000

//1 miejsce po przecinku dla wysoko�ci i pr�dko�ci
#define ACC1				10

//Przesuni�cie wsp�rz�dnych przed wys�aniem przez radio
#define LAT_PUSH			90
#define LON_PUSH			180
#define ALT_PUSH			500

#define PUSH_16BIT			32767UL
#define PUSH_17BIT			65535UL
#define PUSH_18BIT			131071UL
#define PUSH_19BIT			262143UL
#define PUSH_20BIT			524287UL
#define MAX_6BIT			63
#define MAX_7BIT			127
#define MAX_8BIT			255
#define MAX_9BIT			511
#define MAX_10BIT			1023UL
#define MAX_14BIT			16383UL
#define MAX_15BIT			32767UL
#define MAX_16BIT			65535UL
#define MAX_17BIT			131071UL
#define MAX_18BIT			262143UL
#define MAX_19BIT			524287UL
#define MAX_20BIT			1048575UL

#define ALT_DIV				6553.5
#define SPD_DIV				127.5

//Minimalna liczba satelit�w uznawana za fixa
#define GPS_MIN_NUMSAT		4

//Liczba pomijanych odczyt�w GPS po utracie fixa
#define GPS_RECOVERY_CNT	10

//Liczba przesy�anych bajt�w
#define NMEA_TAB_CNT		3
#define REFPOS_TAB_CNT		11

#if RADIO_MODE==1

//Pomini�cie pocz�tkowych odczyt�w GPS przy uruchomieniu systemu
#define GPS_START			30

//Maksymalny dopouszczalny czas pomi�dzy kolejnymi odczytami GPS (zalezny od logger.gpshz)
#define GPS_MAX_BREAK		5000
#define GPS_BREAK			2500

//Maksymalny dopuszczalny przeskok pomi�dzy punktami GPS [m]
#define GPS_MAX_HOP_M		200

//Maksymalna dopuszczalna wysoko�� odczytana z GPS [m]
//#define GPS_MAX_ALT		8500

//Rodzaj GPSa
#define PROTO_PMTK			1
#define PROTO_UBX			2

#define GPS_LOG_FIX			0x01
#define GPS_LOG_LOWACC		0x02

//Liczba pr�bek wysoko�ci do wst�pnego ustalenia wysoko�ci punktu Z (P1)
#define REFALT_CNT			30

//Oznaczenie gotowo�ci odczytu wysoko�ci
#define REFALT_READY		0
#define REFALTGPS_NOT_SET	999999

typedef struct {
	//Tabela danych NMEA do wys�ania przez radio
	uint32_t tab[NMEA_TAB_CNT];

	//Znacznik czasu dla ramki
	uint8_t t;

	//Znacznik gotowo�ci danych
	uint8_t ready;
} s_nmea;

typedef struct {
	//s_wsp w;
	double lat;
	double lon;
	double speed;		//Pr�dko�� w w�z�ach
	//double heading;

	double alt;			//Wysoko�� z uwzgl�dnieniem HOG
	double hog;			//Height of geoid

	//uint8_t fix;		//0-brak fixa; >0 fix
	uint8_t numsat;		//Liczba �ledzonych satelit�w
	uint8_t numsat_prev;
	uint8_t numsat_min;
	uint8_t numsat_max;

	char ctime[7];		//Czas UTC	hhmmss\0
	char cdate[7];		//Data		ddmmyy\0
} s_gpsData;

extern volatile uint16_t t_getGPSdata;
extern volatile uint16_t t_gpsBreak;
extern s_gpsData gpsData;
extern s_nmea nmea;

//Funkcja uzupe�nia i wysy�a komend� PMTK
//void gpsCmd(char *cmd);

//Sprawdzenie crc odczytanego zdania nmea; 1-crc ok; 0-b��d
//uint8_t checkCRC(char *s);

//Inicjalizacja GPS poprzez wys�anie odpowiednich komend
void gsp_init(void);

//Zamiana deg.min na float
double gps2float(int st, double min);

//Odczyt danych z rekord�w NMEA
void decodeRMC(s_gpsData *gps);
void decodeGGA(s_gpsData *gps);

uint8_t getGPSdata();			//Odczytanie wsp�rz�dnych oraz samplowanie po�rednich z podanym skokiem rate/sek
uint16_t getGpsTimeSec();		//Zwraca aktualn� godzin� w sekundach
uint8_t chk_angles(s_wsp w);
uint8_t position_valid();
void nmea_encode();

#endif

void gps_crc(char *s, char *scrc);

#endif /* _GPS_H_ */
