#ifndef _RADIO_TX_H_
#define _RADIO_TX_H_

#include "../config.h"
#if RADIO_MODE==1

#include "../RCFA/rcfa.h"

extern volatile uint16_t t_radio_tx;

//Inicjalizacja radia
void radio_init_tx();

//Wys³anie wspó³rzêdnych w trybie Variable payload length
void radio_tx_numsat(uint8_t b);

void radio_tx_type_val(uint8_t type, int16_t val);
void radio_tx_pos(uint8_t type, uint32_t val);

void radio_tx_info(uint8_t type);

//void radio_tx_alarm_vario(int16_t alarm, int16_t vario);
//void radio_tx_wsp3d(s_wsp w, double alt);
//void radio_tx_nmea();

//void radio_prep_refpos();
void radio_tx_refpos();

//Wys³anie Payload - wejscie w tryb Tx a po wys³aniu w Standby
//Zwraca:
// 0  - mo¿na wysy³aæ
// >0 - w trakcie wysy³ania
uint8_t radio_tx_payload();

//void radio_hop_tx(int8_t num);

#endif
#endif /* _RADIO_TX_H_ */
