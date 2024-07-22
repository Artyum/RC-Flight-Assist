#ifndef _F3A_H_
#define _F3A_H_

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "../config.h"

//Warto�ci domy�lne strefy
#define F3A_DEF_D			150.0
#define F3A_DEF_K			60.0
#define F3A_DEF_SB			270.0
#define F3A_DEF_SC			30.0
#define F3A_DEF_MARGIN		0

#define F3A_D_MIN			10.0
#define F3A_D_MAX			2000.0
#define F3A_K_MIN			1.0
#define F3A_K_MAX			85.0
#define F3A_SB_MIN			10.0
#define F3A_SB_MAX			5000.0
#define F3A_SC_MIN			1.0

//Min i Max odleg�o�ci mi�dzy punktami Z i P w metrach
#define F3A_MIN_ZP			0.1
#define F3A_MAX_ZP			500.0

//Przesuni�cie przedzia�u alarmu z -10k;10k => 10k;30k - by zapobiec przesy�aniu przez radio liczb ca�kowitych bliskich 0
#define ALARM_PUSH			20000L
#define ALARM_MIN			-10000L
#define ALARM_MAX			10000L

//Maksymalna dopuszczalna odleg�o�� mi�dzy L i P1 do wyliczenia strefy
#define RCFA_MAX_DIST_LP1	200

//Maksymalna pr�dko�� przy kt�rej dzia�a filtr dolnoprzepustowy na wysoko�� i pozycj� [w�z�y]
#define STEADY_SPD			1.5

//Cz�stotliwo�� odczytu danych z GPS
#define GPSDATA_DELAY		50

//Cz�stotliwo�� obliczania pozycji w strefie i naliczania punkt�w
#define CHKPOS_DELAY		GPSDATA_DELAY

//Szybko�� migania diodami
#define SYSTEM_STOP_DELAY	200

//Znacznik odczytanych wsp�rz�dnych z SD (4 warto�ci)
#define AREA_SET_SD			4

//Cz�stotliwo�� sprawdzania vario
#define VARIO_DELAY			100

//Liczba pr�bek do u�redniania valarm (0,5 sek)
#define VARIO_MAX_AVG_CNT	(1000/VARIO_DELAY/2)

//Czas migni�cia diod�
#define BLINK_TIME			300


//Wsp�rz�dne geograficzne
typedef struct {
	double lat;		//Szeroko��
	double lon; 	//D�ugo��
} s_wsp;


//Po�o�enie gps + wysoko��
typedef struct {
	s_wsp w;		//Wsp�rz�dne
	double alt;		//Wysoko��
} s_wsp3d;


typedef struct {
	double dist;	//Odleg�o�� od linii lot�w
	double angle;	//K�t strefy [st.] 1-179 st.
	double sb;		//Szeroko�� Boxa [m]; strefa alarmu = +/-(sb/2) od lini lotu
	double sc;		//Szeroko�� linni lotu [m]; = +/-(sc/2) od linni lotu
	uint8_t margin;	//Alarm graniczny A/B/W

	double h;		//Wysoko�� strefy [m]
	double ss;		//Ca�kowita szeroko�� strefy [m] od pkt A do B
	double radius;	//Radii of Curvature

	//Pomocnicze
	double sb2;	//=sb/2
	double sc2;	//=sc/2

	s_wsp P1;		//Zawodnik
	s_wsp P2;		//Kierunek na �rodek
	s_wsp C;		//Punkt �rodka strefy
	s_wsp A;		//Punkt A
	s_wsp B;		//Punkt B
	s_wsp3d L;		//Aktualne po�o�enie samolotu
	s_wsp3d prevL;	//Poprzednie po�o�enie (do filtrowania)

	//Wsp�rz�dne wierzcho�k�w boxa wyznaczaj�cego stref� linii lotu
	s_wsp SAn;		//A near
	s_wsp SAf;		//A far
	s_wsp SBn;		//B near
	s_wsp SBf;		//B far

	s_wsp AAn;		//A near
	s_wsp AAf;		//A far
	s_wsp ABn;		//B near
	s_wsp ABf;		//B far

	//Stopie� alarmu dla pozycji w strefie lub wariometru
	//Przedzia�: <-10'000;10'000>
	int16_t alarm;

	//Warto�� wariometru
	//Przedzia�: <-10'000;10'000>
	int16_t	valarm;

	//Status: 0-Nieaktualne; 1-Aktualne
	// bit 0 - Punkt Z
	// bit 1 - Punkt P
	// bit 2 - Punkty S,A,B
	// bit 3 - Punkty ustalone - blokada przycisku
	// bit 4 - Wczytano poprzedni� stref�
	uint8_t status;

	//Punkt referencyjny dla NMEA
	s_wsp R;

	//Wysoko�� referencyjna z GPS i barometru
	double refalt;		//Wysoko�� z baro lub z gps
	double refalt_gps;	//Wysoko�� z GPS

	//Znaczniki wst�pnego ustalenia wysoko�ci
	uint8_t refalt_prep;

	//Licznik pomijanych odczyt�w GPS
	uint8_t gpspos_prep;

	double bearing;
} s_rcfa;


typedef struct {
	//Wsp�rz�dne z INI
	uint8_t iniwspcnt;	//Znacznik czy odczytano 4 wsp�rz�dne z ini. Je�li =4 to znaczy �e odczytano 4 wsp�rz�dne.

	//W��cznik modu�u radiowego
	uint8_t radio;

	//W��cznik barometru (MPL3115A2)
	uint8_t baro;

	//Uwzgl�dnienie wysoko�ci strefy przy obliczaniu alarmu
	uint16_t hlimit;
	uint16_t hlowlimit;

	//Pr�dko�� przy kt�rej nast�puje automatyczna blokada strefy w kph
	uint8_t lockspeed;

	//Szczeg�owo�� logowania (0-wy�)
	uint8_t loglevel;

	//Prze��cznik wariometru
	//0 - wy��czony, dane nie s� wysy�ane do odbiornika
	//1 - w��czony - odbiornik ustawiony w trybie akrobatycznym
	//2 - w��czony - odbiornik ustawiony w trybie wariometru
	uint8_t variomode;

	//Maksymalna pr�dko�� pionowa, poni�ej kt�rej uznaje si� lot poziomy
	double hspmin;

	//Maksymalna wykrywana pr�dko�� pionowa
	double hspmax;

	//Konfiguracja filtru kalmana
	uint8_t filter;
	double cr;
	double cq;
	double ar;
	double aq;
	double vr;
	double vq;

	//Pr�dko�� odczytu GPS w Hz: 1|5|10
	uint8_t gpshz;

	//Timezone
	int8_t tzone;

	//Co ile sekund nast�pi logowanie wsp�rz�dnych
	double logspeed;

	//Tryb loggera
	// 0 - Wy��czony | 1 - Automatyczny | 2 - Manualny
	uint8_t loggermode;

	//Minimalna liczba satelit, przy kt�rej sygna� uznawany jest za dobry. Poni�ej tej liczby pojawia si� alarm ma�ej ma�ej dok�adno�ci GPS
	uint8_t lowacc;

	//Znacznik czy schodzi� z wysoko�ci� poni�ej 0
	uint8_t floor;

	//G�o�no�� startowa
	uint8_t vol;

	//Test urz�dzenia - odczyt danych z pami�ci EEMEM i inne testy
	// 0 - wy��czone
	// 1 - Everest
	// 2 - Everest + Memtest
	uint8_t test;

	//W��cznik wysy�ania danych NMEA
	uint8_t txpos;

	//Minimalna liczba satelit�w potrzebna do uzyskania odczytu wysoko�ci referencyjnej
	uint8_t refsat;

	//Wysoko�� referencyjna ustawiana r�cznie
	double refalt_gps;
} s_ini;

typedef struct {
    uint8_t radio:1;
    uint8_t gps:1;
    uint8_t mpl:1;
    uint8_t sdmount:1;
    uint8_t sdlog:1;
    uint8_t sdini:1;
    uint8_t bit6:1;
    uint8_t bit7:1;
} s_hw;

//Operacje bitowe
#define SET_P1			0x01
#define SET_P2			0x02
#define SET_CAB			0x04
#define SET_BLK			0x08
#define SET_PREV		0x10
#define SET_P1P2		(SET_P1 | SET_P2)
#define SET_RTB			(SET_P1P2 | SET_CAB)

//Status sprz�towy
//#define HW_RADIO		0x01
//#define HW_GPS		0x02
//#define HW_SD_MOUNT	0x04
//#define HW_SD_INI		0x08
//#define HW_MPL		0x10
//#define HW_SD_LOG		0x20
//#define HW_ALL_OK		(HW_RADIO | HW_GPS | HW_SD_MOUNT | HW_SD_INI | HW_MPL)

double knots2kph(double k);
double kph2knots(double k);

#if FUNC_MAP==1
double map(double v, double f1, double f2, double t1, double t2);
#endif
void system_stop(void);

extern s_rcfa rcfa;
extern s_ini ini;
extern uint8_t hw_status;
extern s_hw hw;

#endif /* _F3A_H_ */
