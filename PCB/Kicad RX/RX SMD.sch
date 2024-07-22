EESchema Schematic File Version 2
LIBS:RX SMD-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:MyLib
LIBS:RX SMD-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "RC Flight Assist"
Date ""
Rev ""
Comp "LittleCircuit"
Comment1 "Odbiornik"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L GND #PWR01
U 1 1 558400B6
P 1875 1775
F 0 "#PWR01" H 1875 1525 50  0001 C CNN
F 1 "GND" H 1875 1625 50  0000 C CNN
F 2 "" H 1875 1775 60  0000 C CNN
F 3 "" H 1875 1775 60  0000 C CNN
	1    1875 1775
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR02
U 1 1 55840E3D
P 1875 1100
F 0 "#PWR02" H 1875 950 50  0001 C CNN
F 1 "+BATT" H 1875 1240 50  0000 C CNN
F 2 "" H 1875 1100 60  0000 C CNN
F 3 "" H 1875 1100 60  0000 C CNN
	1    1875 1100
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 558435D5
P 3450 1700
F 0 "R2" V 3530 1700 50  0000 C CNN
F 1 "390" V 3450 1700 50  0000 C CNN
F 2 "Footprints:R_0805" V 3380 1700 30  0001 C CNN
F 3 "" H 3450 1700 30  0000 C CNN
	1    3450 1700
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5584392C
P 4000 1300
F 0 "R3" V 4080 1300 50  0000 C CNN
F 1 "240" V 4000 1300 50  0000 C CNN
F 2 "Footprints:R_0805" V 3930 1300 30  0001 C CNN
F 3 "" H 4000 1300 30  0000 C CNN
	1    4000 1300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 55844024
P 3450 2000
F 0 "#PWR03" H 3450 1750 50  0001 C CNN
F 1 "GND" H 3450 1850 50  0000 C CNN
F 2 "" H 3450 2000 60  0000 C CNN
F 3 "" H 3450 2000 60  0000 C CNN
	1    3450 2000
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 55846733
P 2300 3150
F 0 "R1" V 2380 3150 50  0000 C CNN
F 1 "10K" V 2300 3150 50  0000 C CNN
F 2 "Footprints:R_0805" V 2230 3150 30  0001 C CNN
F 3 "" H 2300 3150 30  0000 C CNN
	1    2300 3150
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR04
U 1 1 55849273
P 4450 1050
F 0 "#PWR04" H 4450 900 50  0001 C CNN
F 1 "+3.3V" H 4450 1190 50  0000 C CNN
F 2 "" H 4450 1050 60  0000 C CNN
F 3 "" H 4450 1050 60  0000 C CNN
	1    4450 1050
	1    0    0    -1  
$EndComp
Text GLabel 4050 3950 2    47   Input ~ 0
SCK
Text GLabel 4050 3850 2    47   Input ~ 0
MISO
Text GLabel 4050 3750 2    47   Input ~ 0
MOSI
$Comp
L GND #PWR05
U 1 1 5584C620
P 1900 5850
F 0 "#PWR05" H 1900 5600 50  0001 C CNN
F 1 "GND" H 1900 5700 50  0000 C CNN
F 2 "" H 1900 5850 60  0000 C CNN
F 3 "" H 1900 5850 60  0000 C CNN
	1    1900 5850
	1    0    0    -1  
$EndComp
Text GLabel 7650 1100 2    47   Input ~ 0
SS
Text GLabel 7650 1200 2    47   Input ~ 0
MOSI
Text GLabel 7650 1300 2    47   Input ~ 0
MISO
Text GLabel 7650 1400 2    47   Input ~ 0
SCK
$Comp
L GND #PWR06
U 1 1 5585023C
P 7750 1800
F 0 "#PWR06" H 7750 1550 50  0001 C CNN
F 1 "GND" H 7750 1650 50  0000 C CNN
F 2 "" H 7750 1800 60  0000 C CNN
F 3 "" H 7750 1800 60  0000 C CNN
	1    7750 1800
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR07
U 1 1 558508F4
P 5800 1500
F 0 "#PWR07" H 5800 1350 50  0001 C CNN
F 1 "+3.3V" H 5800 1640 50  0000 C CNN
F 2 "" H 5800 1500 60  0000 C CNN
F 3 "" H 5800 1500 60  0000 C CNN
	1    5800 1500
	1    0    0    -1  
$EndComp
Text GLabel 6250 1100 0    47   Input ~ 0
INT_PS
Text GLabel 6250 1300 0    47   Input ~ 0
INT_FNE
Text GLabel 6250 1500 0    47   Input ~ 0
INT_MR
$Comp
L LED-RESCUE-RX_SMD D2
U 1 1 558663BC
P 9750 4950
F 0 "D2" H 9750 5050 50  0000 C CNN
F 1 "LED0805W-RD-160" H 9750 4850 50  0000 C CNN
F 2 "Footprints:LED_0805" H 9750 4950 60  0001 C CNN
F 3 "" H 9750 4950 60  0000 C CNN
	1    9750 4950
	-1   0    0    1   
$EndComp
$Comp
L LED-RESCUE-RX_SMD D3
U 1 1 558664B1
P 9750 5550
F 0 "D3" H 9750 5650 50  0000 C CNN
F 1 "LED0805W-YL-160" H 9750 5450 50  0000 C CNN
F 2 "Footprints:LED_0805" H 9750 5550 60  0001 C CNN
F 3 "" H 9750 5550 60  0000 C CNN
	1    9750 5550
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR08
U 1 1 55866B37
P 10100 5850
F 0 "#PWR08" H 10100 5600 50  0001 C CNN
F 1 "GND" H 10100 5700 50  0000 C CNN
F 2 "" H 10100 5850 60  0000 C CNN
F 3 "" H 10100 5850 60  0000 C CNN
	1    10100 5850
	1    0    0    -1  
$EndComp
Text GLabel 8850 3850 0    47   Input ~ 0
BTN_1
Text GLabel 8850 4950 0    47   Input ~ 0
LED_1
Text GLabel 8850 5550 0    47   Input ~ 0
LED_2
$Comp
L +BATT #PWR09
U 1 1 558839FD
P 2800 1050
F 0 "#PWR09" H 2800 900 50  0001 C CNN
F 1 "+BATT" H 2800 1190 50  0000 C CNN
F 2 "" H 2800 1050 60  0000 C CNN
F 3 "" H 2800 1050 60  0000 C CNN
	1    2800 1050
	1    0    0    -1  
$EndComp
Text GLabel 6250 1000 0    47   Input ~ 0
RESET
$Comp
L C C7
U 1 1 558940B1
P 5800 1900
F 0 "C7" H 5825 2000 50  0000 L CNN
F 1 "1uF" H 5825 1800 50  0000 L CNN
F 2 "Footprints:C_0805" H 5838 1750 30  0001 C CNN
F 3 "" H 5800 1900 60  0000 C CNN
	1    5800 1900
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR010
U 1 1 55895393
P 5800 2150
F 0 "#PWR010" H 5800 1900 50  0001 C CNN
F 1 "GND" H 5800 2000 50  0000 C CNN
F 2 "" H 5800 2150 60  0000 C CNN
F 3 "" H 5800 2150 60  0000 C CNN
	1    5800 2150
	1    0    0    -1  
$EndComp
$Comp
L LM317AEMP U1
U 1 1 558A551F
P 3450 1150
F 0 "U1" H 3250 1350 40  0000 C CNN
F 1 "LM317D" H 3450 1350 40  0000 L CNN
F 2 "Footprints:LM317_DPAK" H 3450 1250 30  0000 C CIN
F 3 "" H 3450 1150 60  0000 C CNN
	1    3450 1150
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 558BB7EB
P 1700 3450
F 0 "C2" H 1725 3550 50  0000 L CNN
F 1 "1uF" H 1725 3350 50  0000 L CNN
F 2 "Footprints:C_0805" H 1738 3300 30  0001 C CNN
F 3 "" H 1700 3450 60  0000 C CNN
	1    1700 3450
	0    1    1    0   
$EndComp
$Comp
L C C8
U 1 1 558A2396
P 6150 1900
F 0 "C8" H 6175 2000 50  0000 L CNN
F 1 "100nF" H 6175 1800 50  0000 L CNN
F 2 "Footprints:C_0805" H 6188 1750 30  0001 C CNN
F 3 "" H 6150 1900 60  0000 C CNN
	1    6150 1900
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P2
U 1 1 558A8CA1
P 4800 7100
F 0 "P2" H 4800 7300 50  0000 C CNN
F 1 "CONN_01X03" V 4900 7100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x03" H 4800 7100 60  0001 C CNN
F 3 "" H 4800 7100 60  0000 C CNN
	1    4800 7100
	1    0    0    -1  
$EndComp
Text GLabel 4050 5550 2    47   Input ~ 0
RESET
Text GLabel 4050 5650 2    47   Input ~ 0
INT_PS
Text GLabel 4050 5750 2    47   Input ~ 0
INT_FNE
Text GLabel 4050 3450 2    47   Input ~ 0
INT_MR
$Comp
L CONN_01X03 P3
U 1 1 55895528
P 6350 7100
F 0 "P3" H 6350 7300 50  0000 C CNN
F 1 "CONN_01X03" V 6450 7100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x03" H 6350 7100 60  0001 C CNN
F 3 "" H 6350 7100 60  0000 C CNN
	1    6350 7100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 558958EC
P 5750 7300
F 0 "#PWR011" H 5750 7050 50  0001 C CNN
F 1 "GND" H 5750 7150 50  0000 C CNN
F 2 "" H 5750 7300 60  0000 C CNN
F 3 "" H 5750 7300 60  0000 C CNN
	1    5750 7300
	1    0    0    -1  
$EndComp
Text GLabel 4050 5050 2    47   Input ~ 0
L_RX
Text GLabel 4050 5150 2    47   Input ~ 0
TX
Text GLabel 4500 7100 0    47   Input ~ 0
H_RX
Text GLabel 4050 5350 2    47   Input ~ 0
FQ_1
Text GLabel 6050 7000 0    47   Input ~ 0
FQ_1
Text GLabel 6050 7200 0    47   Input ~ 0
FQ_2
Text GLabel 4050 5250 2    47   Input ~ 0
FQ_2
Text GLabel 4600 4700 2    47   Input ~ 0
SS
Text Notes 10150 5050 0    47   ~ 0
LED czerw. 120-160mcd\n1,9..2.4V\n20mA
Text Notes 10150 5625 0    47   ~ 0
LED ziel. 120-160mcd\n1,9..2.4V\n20mA
$Comp
L R R6
U 1 1 558AD816
P 9200 5550
F 0 "R6" V 9280 5550 50  0000 C CNN
F 1 "130" V 9200 5550 50  0000 C CNN
F 2 "Footprints:R_0805" V 9130 5550 30  0001 C CNN
F 3 "" H 9200 5550 30  0000 C CNN
	1    9200 5550
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 558ADC2F
P 9200 4950
F 0 "R5" V 9280 4950 50  0000 C CNN
F 1 "130" V 9200 4950 50  0000 C CNN
F 2 "Footprints:R_0805" V 9130 4950 30  0001 C CNN
F 3 "" H 9200 4950 30  0000 C CNN
	1    9200 4950
	0    1    1    0   
$EndComp
Text Notes 4250 6800 0    47   ~ 0
Złącze RX/TX
Text Notes 5700 6800 0    47   ~ 0
Zworka częstotliwości
Text Notes 2150 6800 0    47   ~ 0
Konwerter napięcia na RX
Text Notes 850  1050 0    47   ~ 0
BATT\n7-12V
Text GLabel 4050 4400 2    47   Input ~ 0
BTN_2
Text GLabel 4050 3550 2    47   Input ~ 0
BUZZER
$Comp
L C C1
U 1 1 558D778D
P 1700 3150
F 0 "C1" H 1725 3250 50  0000 L CNN
F 1 "100nF" H 1725 3050 50  0000 L CNN
F 2 "Footprints:C_0805" H 1738 3000 30  0001 C CNN
F 3 "" H 1700 3150 60  0000 C CNN
	1    1700 3150
	0    1    1    0   
$EndComp
$Comp
L CONN_01X02 P1
U 1 1 5583F92D
P 1050 1450
F 0 "P1" H 1050 1600 50  0000 C CNN
F 1 "CONN_01X02" V 1150 1450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x02" H 1050 1450 60  0001 C CNN
F 3 "" H 1050 1450 60  0000 C CNN
	1    1050 1450
	-1   0    0    1   
$EndComp
$Comp
L Crystal Y1
U 1 1 558EA38A
P 1100 4450
F 0 "Y1" H 1100 4600 50  0000 C CNN
F 1 "11.0592MHz" H 1100 4300 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-U_Vertical" H 1100 4450 60  0001 C CNN
F 3 "" H 1100 4450 60  0000 C CNN
	1    1100 4450
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 558EC13F
P 850 4800
F 0 "C3" H 875 4900 50  0000 L CNN
F 1 "22pF" H 875 4700 50  0000 L CNN
F 2 "Footprints:C_0805" H 888 4650 30  0001 C CNN
F 3 "" H 850 4800 60  0000 C CNN
	1    850  4800
	-1   0    0    1   
$EndComp
$Comp
L C C4
U 1 1 558EC293
P 1350 4800
F 0 "C4" H 1375 4900 50  0000 L CNN
F 1 "22pF" H 1375 4700 50  0000 L CNN
F 2 "Footprints:C_0805" H 1388 4650 30  0001 C CNN
F 3 "" H 1350 4800 60  0000 C CNN
	1    1350 4800
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR012
U 1 1 558EC46B
P 1450 3550
F 0 "#PWR012" H 1450 3300 50  0001 C CNN
F 1 "GND" H 1450 3400 50  0000 C CNN
F 2 "" H 1450 3550 60  0000 C CNN
F 3 "" H 1450 3550 60  0000 C CNN
	1    1450 3550
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR013
U 1 1 558A022E
P 1950 3000
F 0 "#PWR013" H 1950 2850 50  0001 C CNN
F 1 "+3.3V" H 1950 3140 50  0000 C CNN
F 2 "" H 1950 3000 60  0000 C CNN
F 3 "" H 1950 3000 60  0000 C CNN
	1    1950 3000
	1    0    0    -1  
$EndComp
Text GLabel 8850 4350 0    47   Input ~ 0
BTN_2
$Comp
L C C5
U 1 1 55911810
P 2900 1500
F 0 "C5" H 2925 1600 50  0000 L CNN
F 1 "100nF" H 2925 1400 50  0000 L CNN
F 2 "Footprints:C_0805" H 2938 1350 30  0001 C CNN
F 3 "" H 2900 1500 60  0000 C CNN
	1    2900 1500
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 5591188C
P 4350 1500
F 0 "C6" H 4375 1600 50  0000 L CNN
F 1 "100nF" H 4375 1400 50  0000 L CNN
F 2 "Footprints:C_0805" H 4388 1350 30  0001 C CNN
F 3 "" H 4350 1500 60  0000 C CNN
	1    4350 1500
	1    0    0    -1  
$EndComp
$Comp
L CP C11
U 1 1 55914958
P 1775 1450
F 0 "C11" H 1800 1550 50  0000 L CNN
F 1 "100uF" H 1800 1350 50  0000 L CNN
F 2 "Capacitors_SMD:c_elec_6.3x5.3" H 1813 1300 30  0001 C CNN
F 3 "" H 1775 1450 60  0000 C CNN
	1    1775 1450
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P4
U 1 1 55931FC2
P 8050 950
F 0 "P4" H 8050 1050 50  0000 C CNN
F 1 "CONN_01X01" V 8150 950 50  0000 C CNN
F 2 "Footprints:1Pin" H 8050 950 60  0001 C CNN
F 3 "" H 8050 950 60  0000 C CNN
	1    8050 950 
	0    -1   -1   0   
$EndComp
Text GLabel 4050 4600 2    47   Input ~ 0
BTN_1
$Comp
L ATMEGA168PA-A IC1
U 1 1 558F3B49
P 2950 4550
F 0 "IC1" H 2200 5800 40  0000 L BNN
F 1 "ATMEGA168PA-A" H 3350 3150 40  0000 L BNN
F 2 "Housings_QFP:TQFP-32_7x7mm_Pitch0.8mm" H 2950 4550 30  0000 C CIN
F 3 "" H 2950 4550 60  0000 C CNN
	1    2950 4550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 559007E5
P 1100 5150
F 0 "#PWR014" H 1100 4900 50  0001 C CNN
F 1 "GND" H 1100 5000 50  0000 C CNN
F 2 "" H 1100 5150 60  0000 C CNN
F 3 "" H 1100 5150 60  0000 C CNN
	1    1100 5150
	1    0    0    -1  
$EndComp
Text GLabel 850  4300 1    47   Input ~ 0
XTAL1
Text GLabel 1350 4300 1    47   Input ~ 0
XTAL2
Text GLabel 4050 4050 2    47   Input ~ 0
XTAL1
Text GLabel 4050 4150 2    47   Input ~ 0
XTAL2
Text Label 7900 1600 0    47   ~ 0
ANA
Text GLabel 4050 4900 2    47   Input ~ 0
RST
Text GLabel 2600 3150 2    47   Input ~ 0
RST
$Comp
L RFM69HW_SMD RFM1
U 1 1 55951168
P 6950 1350
F 0 "RFM1" H 6950 900 60  0000 C CNN
F 1 "RFM69HW" H 6950 1800 60  0000 C CNN
F 2 "Footprints:RFM69HW_SMD" H 6900 1200 60  0001 C CNN
F 3 "" H 6900 1200 60  0000 C CNN
	1    6950 1350
	1    0    0    -1  
$EndComp
Text Notes 7475 4900 0    47   ~ 0
6..15V\n140ohm\n40mA
$Comp
L CONN_01X01 P6
U 1 1 5596F0E3
P 10700 850
F 0 "P6" H 10700 950 50  0000 C CNN
F 1 "CONN_01X01" V 10800 850 50  0000 C CNN
F 2 "Footprints:1Pin" H 10700 850 60  0001 C CNN
F 3 "" H 10700 850 60  0000 C CNN
	1    10700 850 
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X01 P7
U 1 1 5596F63D
P 10500 850
F 0 "P7" H 10500 950 50  0000 C CNN
F 1 "CONN_01X01" V 10600 850 50  0000 C CNN
F 2 "Footprints:1Pin" H 10500 850 60  0001 C CNN
F 3 "" H 10500 850 60  0000 C CNN
	1    10500 850 
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X01 P8
U 1 1 5596F69B
P 10300 850
F 0 "P8" H 10300 950 50  0000 C CNN
F 1 "CONN_01X01" V 10400 850 50  0000 C CNN
F 2 "Footprints:1Pin" H 10300 850 60  0001 C CNN
F 3 "" H 10300 850 60  0000 C CNN
	1    10300 850 
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X01 P9
U 1 1 5596F6FB
P 10100 850
F 0 "P9" H 10100 950 50  0000 C CNN
F 1 "CONN_01X01" V 10200 850 50  0000 C CNN
F 2 "Footprints:1Pin" H 10100 850 60  0001 C CNN
F 3 "" H 10100 850 60  0000 C CNN
	1    10100 850 
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X01 P10
U 1 1 5596F761
P 9900 850
F 0 "P10" H 9900 950 50  0000 C CNN
F 1 "CONN_01X01" V 10000 850 50  0000 C CNN
F 2 "Footprints:1Pin" H 9900 850 60  0001 C CNN
F 3 "" H 9900 850 60  0000 C CNN
	1    9900 850 
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X01 P11
U 1 1 5596F947
P 9700 850
F 0 "P11" H 9700 950 50  0000 C CNN
F 1 "CONN_01X01" V 9800 850 50  0000 C CNN
F 2 "Footprints:1Pin" H 9700 850 60  0001 C CNN
F 3 "" H 9700 850 60  0000 C CNN
	1    9700 850 
	0    -1   -1   0   
$EndComp
Text GLabel 10700 1100 3    47   Input ~ 0
MISO
Text GLabel 10500 1100 3    47   Input ~ 0
MOSI
Text GLabel 10300 1100 3    47   Input ~ 0
SCK
Text GLabel 10100 1100 3    47   Input ~ 0
RST
$Comp
L GND #PWR015
U 1 1 559736E4
P 9700 1100
F 0 "#PWR015" H 9700 850 50  0001 C CNN
F 1 "GND" H 9700 950 50  0000 C CNN
F 2 "" H 9700 1100 60  0000 C CNN
F 3 "" H 9700 1100 60  0000 C CNN
	1    9700 1100
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR016
U 1 1 5597397F
P 9900 1100
F 0 "#PWR016" H 9900 950 50  0001 C CNN
F 1 "+3.3V" H 9900 1240 50  0000 C CNN
F 2 "" H 9900 1100 60  0000 C CNN
F 3 "" H 9900 1100 60  0000 C CNN
	1    9900 1100
	-1   0    0    1   
$EndComp
Text Notes 7475 3350 0    47   ~ 0
Headphones\n8-32ohm\n
$Comp
L D D1
U 1 1 55A3DC36
P 1525 1200
F 0 "D1" H 1525 1300 50  0000 C CNN
F 1 "SM4007" H 1525 1100 50  0000 C CNN
F 2 "Footprints:MELF_Standard" H 1525 1200 60  0001 C CNN
F 3 "" H 1525 1200 60  0000 C CNN
	1    1525 1200
	-1   0    0    1   
$EndComp
Text Notes 9550 650  0    47   ~ 0
Wejście ISP
Text GLabel 4500 7000 0    47   Input ~ 0
TX
$Comp
L R R8
U 1 1 55B327B3
P 2650 6950
F 0 "R8" V 2730 6950 50  0000 C CNN
F 1 "10K" V 2650 6950 50  0000 C CNN
F 2 "Footprints:R_0805" V 2580 6950 30  0001 C CNN
F 3 "" H 2650 6950 30  0000 C CNN
	1    2650 6950
	0    1    1    0   
$EndComp
$Comp
L R R9
U 1 1 55B336B0
P 2900 7200
F 0 "R9" V 2980 7200 50  0000 C CNN
F 1 "20K" V 2900 7200 50  0000 C CNN
F 2 "Footprints:R_0805" V 2830 7200 30  0001 C CNN
F 3 "" H 2900 7200 30  0000 C CNN
	1    2900 7200
	1    0    0    -1  
$EndComp
Text GLabel 2400 6950 0    47   Input ~ 0
H_RX
$Comp
L GND #PWR017
U 1 1 55B36681
P 2900 7400
F 0 "#PWR017" H 2900 7150 50  0001 C CNN
F 1 "GND" H 2900 7250 50  0000 C CNN
F 2 "" H 2900 7400 60  0000 C CNN
F 3 "" H 2900 7400 60  0000 C CNN
	1    2900 7400
	1    0    0    -1  
$EndComp
Text GLabel 3050 6950 2    47   Input ~ 0
L_RX
$Comp
L GND #PWR018
U 1 1 55B3AC96
P 4500 7250
F 0 "#PWR018" H 4500 7000 50  0001 C CNN
F 1 "GND" H 4500 7100 50  0000 C CNN
F 2 "" H 4500 7250 60  0000 C CNN
F 3 "" H 4500 7250 60  0000 C CNN
	1    4500 7250
	1    0    0    -1  
$EndComp
Text Notes 2200 7100 0    47   ~ 0
5V
Text Notes 3050 7100 0    47   ~ 0
3,333V
$Comp
L 10-XX S1
U 1 1 55B52151
P 9450 3900
F 0 "S1" H 9400 3750 50  0000 L BNN
F 1 "SM612A" H 9600 4100 50  0000 L BNN
F 2 "Footprints:B3F_10XX" H 9450 4200 50  0001 C CNN
F 3 "" V 9450 3950 47  0000 C CNN
	1    9450 3900
	1    0    0    -1  
$EndComp
$Comp
L 10-XX S2
U 1 1 55B552D6
P 9450 4400
F 0 "S2" H 9400 4250 50  0000 L BNN
F 1 "SM612A" H 9600 4600 50  0000 L BNN
F 2 "Footprints:B3F_10XX" H 9450 4700 50  0001 C CNN
F 3 "" V 9450 4450 47  0000 C CNN
	1    9450 4400
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P18
U 1 1 55C07B8F
P 9525 2500
F 0 "P18" H 9525 2600 50  0000 C CNN
F 1 "CONN_01X01" V 9625 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 9525 2500 60  0001 C CNN
F 3 "" H 9525 2500 60  0000 C CNN
	1    9525 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR019
U 1 1 55C07E1D
P 9525 2750
F 0 "#PWR019" H 9525 2500 50  0001 C CNN
F 1 "GND" H 9525 2600 50  0000 C CNN
F 2 "" H 9525 2750 60  0000 C CNN
F 3 "" H 9525 2750 60  0000 C CNN
	1    9525 2750
	1    0    0    -1  
$EndComp
Text Notes 9500 2300 0    47   ~ 0
Przelotki GND
$Comp
L CONN_01X01 P19
U 1 1 55C12CEE
P 9725 2500
F 0 "P19" H 9725 2600 50  0000 C CNN
F 1 "CONN_01X01" V 9825 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 9725 2500 60  0001 C CNN
F 3 "" H 9725 2500 60  0000 C CNN
	1    9725 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR020
U 1 1 55C12CF4
P 9725 2750
F 0 "#PWR020" H 9725 2500 50  0001 C CNN
F 1 "GND" H 9725 2600 50  0000 C CNN
F 2 "" H 9725 2750 60  0000 C CNN
F 3 "" H 9725 2750 60  0000 C CNN
	1    9725 2750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P20
U 1 1 55C12DCF
P 9925 2500
F 0 "P20" H 9925 2600 50  0000 C CNN
F 1 "CONN_01X01" V 10025 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 9925 2500 60  0001 C CNN
F 3 "" H 9925 2500 60  0000 C CNN
	1    9925 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR021
U 1 1 55C12DD5
P 9925 2750
F 0 "#PWR021" H 9925 2500 50  0001 C CNN
F 1 "GND" H 9925 2600 50  0000 C CNN
F 2 "" H 9925 2750 60  0000 C CNN
F 3 "" H 9925 2750 60  0000 C CNN
	1    9925 2750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P21
U 1 1 55C12DDC
P 10125 2500
F 0 "P21" H 10125 2600 50  0000 C CNN
F 1 "CONN_01X01" V 10225 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 10125 2500 60  0001 C CNN
F 3 "" H 10125 2500 60  0000 C CNN
	1    10125 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR022
U 1 1 55C12DE2
P 10125 2750
F 0 "#PWR022" H 10125 2500 50  0001 C CNN
F 1 "GND" H 10125 2600 50  0000 C CNN
F 2 "" H 10125 2750 60  0000 C CNN
F 3 "" H 10125 2750 60  0000 C CNN
	1    10125 2750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P22
U 1 1 55C12E62
P 10325 2500
F 0 "P22" H 10325 2600 50  0000 C CNN
F 1 "CONN_01X01" V 10425 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 10325 2500 60  0001 C CNN
F 3 "" H 10325 2500 60  0000 C CNN
	1    10325 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR023
U 1 1 55C12E68
P 10325 2750
F 0 "#PWR023" H 10325 2500 50  0001 C CNN
F 1 "GND" H 10325 2600 50  0000 C CNN
F 2 "" H 10325 2750 60  0000 C CNN
F 3 "" H 10325 2750 60  0000 C CNN
	1    10325 2750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P23
U 1 1 55C12EE6
P 10525 2500
F 0 "P23" H 10525 2600 50  0000 C CNN
F 1 "CONN_01X01" V 10625 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 10525 2500 60  0001 C CNN
F 3 "" H 10525 2500 60  0000 C CNN
	1    10525 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR024
U 1 1 55C12EEC
P 10525 2750
F 0 "#PWR024" H 10525 2500 50  0001 C CNN
F 1 "GND" H 10525 2600 50  0000 C CNN
F 2 "" H 10525 2750 60  0000 C CNN
F 3 "" H 10525 2750 60  0000 C CNN
	1    10525 2750
	1    0    0    -1  
$EndComp
Text GLabel 4050 4300 2    47   Input ~ 0
LED_1
Text GLabel 4050 4500 2    47   Input ~ 0
LED_2
Text GLabel 5250 3150 0    47   Input ~ 0
BUZZER
$Comp
L R R11
U 1 1 560EDAAB
P 6000 3150
F 0 "R11" V 6080 3150 50  0000 C CNN
F 1 "390" V 6000 3150 50  0000 C CNN
F 2 "Footprints:R_0805" V 5930 3150 30  0001 C CNN
F 3 "" H 6000 3150 30  0000 C CNN
	1    6000 3150
	0    -1   -1   0   
$EndComp
$Comp
L C C13
U 1 1 562165B2
P 6250 3400
F 0 "C13" H 6275 3500 50  0000 L CNN
F 1 "220nF" H 6275 3300 50  0000 L CNN
F 2 "Footprints:C_0805" H 6288 3250 30  0001 C CNN
F 3 "" H 6250 3400 60  0000 C CNN
	1    6250 3400
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 562166D2
P 5500 3150
F 0 "R12" V 5580 3150 50  0000 C CNN
F 1 "390" V 5500 3150 50  0000 C CNN
F 2 "Footprints:R_0805" V 5430 3150 30  0001 C CNN
F 3 "" H 5500 3150 30  0000 C CNN
	1    5500 3150
	0    -1   -1   0   
$EndComp
$Comp
L JC-128 U3
U 1 1 5621FCEE
P 7525 3350
F 0 "U3" H 7575 3050 47  0000 C CNN
F 1 "JC-128" H 7575 3650 47  0000 C CNN
F 2 "Footprints:JC-128" H 7875 3350 47  0001 C CNN
F 3 "" H 7875 3350 47  0000 C CNN
	1    7525 3350
	-1   0    0    1   
$EndComp
$Comp
L SPEAKER SP1
U 1 1 5622485C
P 7175 4950
F 0 "SP1" H 7075 5200 50  0000 C CNN
F 1 "LD-BZEN-1212 6-15V" H 7375 4575 50  0000 C CNN
F 2 "Footprints:BUZZER_12x6.5" H 7175 4950 60  0001 C CNN
F 3 "" H 7175 4950 60  0000 C CNN
	1    7175 4950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR025
U 1 1 56238C72
P 5750 3600
F 0 "#PWR025" H 5750 3350 50  0001 C CNN
F 1 "GND" H 5750 3450 50  0000 C CNN
F 2 "" H 5750 3600 60  0000 C CNN
F 3 "" H 5750 3600 60  0000 C CNN
	1    5750 3600
	1    0    0    -1  
$EndComp
Text GLabel 6825 3875 2    47   Input ~ 0
AMPL
$Comp
L GND #PWR026
U 1 1 56251D5B
P 6775 5525
F 0 "#PWR026" H 6775 5275 50  0001 C CNN
F 1 "GND" H 6775 5375 50  0000 C CNN
F 2 "" H 6775 5525 60  0000 C CNN
F 3 "" H 6775 5525 60  0000 C CNN
	1    6775 5525
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 5624D6A2
P 6250 3600
F 0 "#PWR027" H 6250 3350 50  0001 C CNN
F 1 "GND" H 6250 3450 50  0000 C CNN
F 2 "" H 6250 3600 60  0000 C CNN
F 3 "" H 6250 3600 60  0000 C CNN
	1    6250 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR028
U 1 1 5624E364
P 6875 3600
F 0 "#PWR028" H 6875 3350 50  0001 C CNN
F 1 "GND" H 6875 3450 50  0000 C CNN
F 2 "" H 6875 3600 60  0000 C CNN
F 3 "" H 6875 3600 60  0000 C CNN
	1    6875 3600
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P5
U 1 1 562558B1
P 10725 2500
F 0 "P5" H 10725 2600 50  0000 C CNN
F 1 "CONN_01X01" V 10825 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 10725 2500 60  0001 C CNN
F 3 "" H 10725 2500 60  0000 C CNN
	1    10725 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR029
U 1 1 5625597A
P 10725 2750
F 0 "#PWR029" H 10725 2500 50  0001 C CNN
F 1 "GND" H 10725 2600 50  0000 C CNN
F 2 "" H 10725 2750 60  0000 C CNN
F 3 "" H 10725 2750 60  0000 C CNN
	1    10725 2750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P24
U 1 1 56257C35
P 9325 2500
F 0 "P24" H 9325 2600 50  0000 C CNN
F 1 "CONN_01X01" V 9425 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 9325 2500 60  0001 C CNN
F 3 "" H 9325 2500 60  0000 C CNN
	1    9325 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR030
U 1 1 56257CFD
P 9325 2750
F 0 "#PWR030" H 9325 2500 50  0001 C CNN
F 1 "GND" H 9325 2600 50  0000 C CNN
F 2 "" H 9325 2750 60  0000 C CNN
F 3 "" H 9325 2750 60  0000 C CNN
	1    9325 2750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P27
U 1 1 56258880
P 9125 2500
F 0 "P27" H 9125 2600 50  0000 C CNN
F 1 "CONN_01X01" V 9225 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 9125 2500 60  0001 C CNN
F 3 "" H 9125 2500 60  0000 C CNN
	1    9125 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR031
U 1 1 5625894B
P 9125 2750
F 0 "#PWR031" H 9125 2500 50  0001 C CNN
F 1 "GND" H 9125 2600 50  0000 C CNN
F 2 "" H 9125 2750 60  0000 C CNN
F 3 "" H 9125 2750 60  0000 C CNN
	1    9125 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR032
U 1 1 56258A0D
P 8925 2750
F 0 "#PWR032" H 8925 2500 50  0001 C CNN
F 1 "GND" H 8925 2600 50  0000 C CNN
F 2 "" H 8925 2750 60  0000 C CNN
F 3 "" H 8925 2750 60  0000 C CNN
	1    8925 2750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P26
U 1 1 56258ACF
P 8925 2500
F 0 "P26" H 8925 2600 50  0000 C CNN
F 1 "CONN_01X01" V 9025 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 8925 2500 60  0001 C CNN
F 3 "" H 8925 2500 60  0000 C CNN
	1    8925 2500
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X01 P28
U 1 1 56263B9F
P 8700 2500
F 0 "P28" H 8700 2600 50  0000 C CNN
F 1 "CONN_01X01" V 8800 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 8700 2500 60  0001 C CNN
F 3 "" H 8700 2500 60  0000 C CNN
	1    8700 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR033
U 1 1 56263C76
P 8700 2750
F 0 "#PWR033" H 8700 2500 50  0001 C CNN
F 1 "GND" H 8700 2600 50  0000 C CNN
F 2 "" H 8700 2750 60  0000 C CNN
F 3 "" H 8700 2750 60  0000 C CNN
	1    8700 2750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P29
U 1 1 562FA57C
P 8500 2500
F 0 "P29" H 8500 2600 50  0000 C CNN
F 1 "CONN_01X01" V 8600 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 8500 2500 60  0001 C CNN
F 3 "" H 8500 2500 60  0000 C CNN
	1    8500 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR034
U 1 1 562FA661
P 8500 2750
F 0 "#PWR034" H 8500 2500 50  0001 C CNN
F 1 "GND" H 8500 2600 50  0000 C CNN
F 2 "" H 8500 2750 60  0000 C CNN
F 3 "" H 8500 2750 60  0000 C CNN
	1    8500 2750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P30
U 1 1 562FB088
P 10925 2500
F 0 "P30" H 10925 2600 50  0000 C CNN
F 1 "CONN_01X01" V 11025 2500 50  0000 C CNN
F 2 "Footprints:1Pin" H 10925 2500 60  0001 C CNN
F 3 "" H 10925 2500 60  0000 C CNN
	1    10925 2500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR035
U 1 1 562FB168
P 10925 2750
F 0 "#PWR035" H 10925 2500 50  0001 C CNN
F 1 "GND" H 10925 2600 50  0000 C CNN
F 2 "" H 10925 2750 60  0000 C CNN
F 3 "" H 10925 2750 60  0000 C CNN
	1    10925 2750
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 563C910F
P 4450 4450
F 0 "R10" V 4530 4450 50  0000 C CNN
F 1 "10K" V 4450 4450 50  0000 C CNN
F 2 "Footprints:R_0805" V 4380 4450 30  0001 C CNN
F 3 "" H 4450 4450 30  0000 C CNN
	1    4450 4450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR036
U 1 1 563C921C
P 4450 4250
F 0 "#PWR036" H 4450 4100 50  0001 C CNN
F 1 "+3.3V" H 4450 4390 50  0000 C CNN
F 2 "" H 4450 4250 60  0000 C CNN
F 3 "" H 4450 4250 60  0000 C CNN
	1    4450 4250
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 568D1ACA
P 5750 3400
F 0 "C9" H 5775 3500 50  0000 L CNN
F 1 "220nF" H 5775 3300 50  0000 L CNN
F 2 "Footprints:C_0805" H 5788 3250 30  0001 C CNN
F 3 "" H 5750 3400 60  0000 C CNN
	1    5750 3400
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 568D3588
P 6550 3150
F 0 "C10" H 6575 3250 50  0000 L CNN
F 1 "1uF" H 6575 3050 50  0000 L CNN
F 2 "Footprints:C_0805" H 6588 3000 30  0001 C CNN
F 3 "" H 6550 3150 60  0000 C CNN
	1    6550 3150
	0    -1   -1   0   
$EndComp
Text GLabel 5675 5100 0    47   Input ~ 0
AMPL
$Comp
L R R7
U 1 1 568DC13F
P 5800 5325
F 0 "R7" V 5880 5325 50  0000 C CNN
F 1 "12K" V 5800 5325 50  0000 C CNN
F 2 "Footprints:R_0805" V 5730 5325 30  0001 C CNN
F 3 "" H 5800 5325 30  0000 C CNN
	1    5800 5325
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 568DC2AD
P 5800 4875
F 0 "R4" V 5880 4875 50  0000 C CNN
F 1 "300K" V 5800 4875 50  0000 C CNN
F 2 "Footprints:R_0805" V 5730 4875 30  0001 C CNN
F 3 "" H 5800 4875 30  0000 C CNN
	1    5800 4875
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR037
U 1 1 568DC9E9
P 5800 4375
F 0 "#PWR037" H 5800 4225 50  0001 C CNN
F 1 "+BATT" H 5800 4515 50  0000 C CNN
F 2 "" H 5800 4375 60  0000 C CNN
F 3 "" H 5800 4375 60  0000 C CNN
	1    5800 4375
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR038
U 1 1 568E2EC2
P 6250 4375
F 0 "#PWR038" H 6250 4225 50  0001 C CNN
F 1 "+BATT" H 6250 4515 50  0000 C CNN
F 2 "" H 6250 4375 60  0000 C CNN
F 3 "" H 6250 4375 60  0000 C CNN
	1    6250 4375
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR039
U 1 1 568E3AE1
P 5800 5525
F 0 "#PWR039" H 5800 5275 50  0001 C CNN
F 1 "GND" H 5800 5375 50  0000 C CNN
F 2 "" H 5800 5525 60  0000 C CNN
F 3 "" H 5800 5525 60  0000 C CNN
	1    5800 5525
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR040
U 1 1 568E3BB8
P 6250 5525
F 0 "#PWR040" H 6250 5275 50  0001 C CNN
F 1 "GND" H 6250 5375 50  0000 C CNN
F 2 "" H 6250 5525 60  0000 C CNN
F 3 "" H 6250 5525 60  0000 C CNN
	1    6250 5525
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 568E4F89
P 6200 4575
F 0 "R13" V 6280 4575 50  0000 C CNN
F 1 "2K" V 6200 4575 50  0000 C CNN
F 2 "Footprints:R_0805" V 6130 4575 30  0001 C CNN
F 3 "" H 6200 4575 30  0000 C CNN
	1    6200 4575
	-1   0    0    1   
$EndComp
$Comp
L C C12
U 1 1 568E313A
P 6600 4850
F 0 "C12" H 6625 4950 50  0000 L CNN
F 1 "10uF" H 6625 4750 50  0000 L CNN
F 2 "Footprints:C_0805" H 6638 4700 30  0001 C CNN
F 3 "" H 6600 4850 60  0000 C CNN
	1    6600 4850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4000 1100 4000 1150
Connection ~ 4000 1100
Wire Wire Line
	2800 1100 3050 1100
Wire Wire Line
	4450 1100 4450 1050
Wire Wire Line
	3950 3950 4050 3950
Wire Wire Line
	3950 3850 4050 3850
Wire Wire Line
	3950 3750 4050 3750
Wire Wire Line
	3850 1100 4450 1100
Wire Wire Line
	3450 1400 3450 1550
Wire Wire Line
	4000 1500 4000 1450
Connection ~ 3450 1500
Wire Wire Line
	7550 1100 7650 1100
Wire Wire Line
	7550 1200 7650 1200
Wire Wire Line
	7550 1300 7650 1300
Wire Wire Line
	7550 1400 7650 1400
Wire Wire Line
	7550 1700 7750 1700
Wire Wire Line
	7750 1500 7750 1800
Wire Wire Line
	7550 1500 7750 1500
Connection ~ 7750 1700
Wire Wire Line
	6350 1100 6250 1100
Wire Wire Line
	6350 1300 6250 1300
Wire Wire Line
	6350 1500 6250 1500
Wire Wire Line
	10100 3850 10100 5850
Wire Wire Line
	9950 4950 10100 4950
Connection ~ 10100 4950
Wire Wire Line
	9950 5550 10100 5550
Connection ~ 10100 5550
Wire Wire Line
	9050 5550 8850 5550
Wire Wire Line
	4350 1350 4350 1100
Connection ~ 4350 1100
Wire Wire Line
	4000 1500 3450 1500
Wire Wire Line
	6350 1000 6250 1000
Wire Wire Line
	4350 1900 4350 1650
Connection ~ 3450 1900
Wire Wire Line
	3450 1850 3450 2000
Wire Wire Line
	2900 1900 4350 1900
Wire Wire Line
	5800 1500 5800 1750
Wire Wire Line
	6150 1700 6150 1750
Connection ~ 6150 1700
Connection ~ 5800 1700
Wire Wire Line
	5800 2050 5800 2150
Wire Wire Line
	6150 2100 6150 2050
Wire Wire Line
	3950 5550 4050 5550
Wire Wire Line
	3950 5650 4050 5650
Wire Wire Line
	3950 5750 4050 5750
Wire Wire Line
	3950 3450 4050 3450
Wire Wire Line
	5750 7300 5750 7100
Wire Wire Line
	5750 7100 6150 7100
Wire Wire Line
	3950 5350 4050 5350
Wire Wire Line
	6150 7000 6050 7000
Wire Wire Line
	6150 7200 6050 7200
Wire Wire Line
	3950 5250 4050 5250
Wire Wire Line
	9350 5550 9550 5550
Wire Wire Line
	9350 4950 9550 4950
Wire Wire Line
	9050 4950 8850 4950
Wire Wire Line
	3950 3550 4050 3550
Wire Wire Line
	2800 1100 2800 1050
Wire Wire Line
	2900 1350 2900 1100
Connection ~ 2900 1100
Wire Wire Line
	2900 1650 2900 1900
Wire Wire Line
	1350 4300 1350 4650
Wire Wire Line
	1350 4450 1250 4450
Connection ~ 1350 4450
Wire Wire Line
	850  4300 850  4650
Wire Wire Line
	850  4450 950  4450
Connection ~ 850  4450
Wire Wire Line
	850  5050 850  4950
Connection ~ 10100 4350
Wire Wire Line
	2050 5550 1900 5550
Wire Wire Line
	1900 5550 1900 5850
Wire Wire Line
	2050 5650 1900 5650
Connection ~ 1900 5650
Wire Wire Line
	2050 5750 1900 5750
Connection ~ 1900 5750
Wire Wire Line
	3950 4900 4050 4900
Wire Wire Line
	1850 3150 2150 3150
Wire Wire Line
	1950 3000 1950 3750
Wire Wire Line
	1850 3450 2050 3450
Connection ~ 1950 3450
Wire Wire Line
	1450 3450 1550 3450
Wire Wire Line
	1450 3150 1450 3550
Wire Wire Line
	1450 3150 1550 3150
Connection ~ 1950 3150
Wire Wire Line
	1100 5050 1100 5150
Wire Wire Line
	3950 4050 4050 4050
Wire Wire Line
	3950 4150 4050 4150
Connection ~ 1450 3450
Wire Wire Line
	2450 3150 2600 3150
Wire Wire Line
	1950 3750 2050 3750
Wire Wire Line
	7550 1600 8125 1600
Wire Wire Line
	8050 1600 8050 1150
Wire Wire Line
	10700 1050 10700 1100
Wire Wire Line
	10500 1050 10500 1100
Wire Wire Line
	10300 1050 10300 1100
Wire Wire Line
	10100 1050 10100 1100
Wire Wire Line
	1350 5050 1350 4950
Wire Wire Line
	850  5050 1350 5050
Connection ~ 1100 5050
Wire Wire Line
	2050 3550 1950 3550
Connection ~ 1950 3550
Wire Wire Line
	1875 1700 1875 1775
Wire Wire Line
	1300 1400 1250 1400
Wire Wire Line
	1300 1200 1375 1200
Wire Wire Line
	1250 1500 1300 1500
Wire Wire Line
	1300 1400 1300 1200
Wire Wire Line
	1300 1500 1300 1700
Wire Wire Line
	2800 6950 3050 6950
Wire Wire Line
	2900 6950 2900 7050
Wire Wire Line
	2500 6950 2400 6950
Wire Wire Line
	2900 7350 2900 7400
Connection ~ 2900 6950
Wire Wire Line
	3950 4500 4050 4500
Wire Wire Line
	3950 5050 4050 5050
Wire Wire Line
	3950 5150 4050 5150
Wire Wire Line
	9650 4350 10100 4350
Wire Wire Line
	9650 4450 9750 4450
Wire Wire Line
	9750 4450 9750 4350
Connection ~ 9750 4350
Wire Wire Line
	9650 3850 10100 3850
Wire Wire Line
	9650 3950 9750 3950
Wire Wire Line
	9750 3950 9750 3850
Connection ~ 9750 3850
Wire Wire Line
	9900 1050 9900 1100
Wire Wire Line
	9700 1050 9700 1100
Wire Wire Line
	9525 2700 9525 2750
Wire Wire Line
	9725 2700 9725 2750
Wire Wire Line
	9925 2700 9925 2750
Wire Wire Line
	10125 2700 10125 2750
Wire Wire Line
	10325 2700 10325 2750
Wire Wire Line
	10525 2700 10525 2750
Wire Wire Line
	8850 3850 9250 3850
Wire Wire Line
	3950 4600 4050 4600
Wire Wire Line
	5250 3150 5350 3150
Wire Wire Line
	5650 3150 5850 3150
Wire Wire Line
	5750 3150 5750 3250
Connection ~ 5750 3150
Wire Wire Line
	6250 3150 6250 3250
Connection ~ 6250 3150
Connection ~ 6875 3150
Wire Wire Line
	6925 3250 6725 3250
Wire Wire Line
	6725 3250 6725 3875
Wire Wire Line
	6725 3350 6925 3350
Connection ~ 6725 3350
Wire Wire Line
	6875 3150 6875 3450
Wire Wire Line
	6875 3450 6925 3450
Wire Wire Line
	6925 3550 6875 3550
Wire Wire Line
	6875 3550 6875 3600
Wire Wire Line
	5750 3550 5750 3600
Wire Wire Line
	6875 5050 6775 5050
Wire Wire Line
	6775 5050 6775 5525
Wire Wire Line
	6725 3875 6825 3875
Wire Wire Line
	6250 3600 6250 3550
Wire Wire Line
	10725 2700 10725 2750
Wire Wire Line
	9325 2700 9325 2750
Wire Wire Line
	8925 2700 8925 2750
Wire Wire Line
	9125 2700 9125 2750
Wire Wire Line
	8700 2700 8700 2750
Wire Wire Line
	4500 7000 4600 7000
Wire Wire Line
	4600 7200 4500 7200
Wire Wire Line
	4500 7200 4500 7250
Wire Wire Line
	4500 7100 4600 7100
Wire Wire Line
	8500 2700 8500 2750
Wire Wire Line
	10925 2700 10925 2750
Wire Wire Line
	4450 4300 4450 4250
Wire Wire Line
	3950 4400 4050 4400
Wire Wire Line
	4450 4600 4450 4700
Wire Wire Line
	3950 4300 4050 4300
Wire Wire Line
	3950 4700 4600 4700
Connection ~ 4450 4700
Wire Wire Line
	6700 3150 6925 3150
Wire Wire Line
	6150 3150 6400 3150
Wire Wire Line
	5800 4375 5800 4725
Wire Wire Line
	5800 5025 5800 5175
Wire Wire Line
	5675 5100 5950 5100
Connection ~ 5800 5100
Wire Wire Line
	6250 5525 6250 5300
Wire Wire Line
	5800 5525 5800 5475
Wire Wire Line
	6250 4725 6250 4900
Wire Wire Line
	6250 4425 6250 4375
Wire Wire Line
	6875 4850 6750 4850
Wire Wire Line
	6450 4850 6250 4850
Connection ~ 6250 4850
Wire Wire Line
	1675 1200 1875 1200
Wire Wire Line
	1875 1200 1875 1100
Wire Wire Line
	1300 1700 1875 1700
Wire Wire Line
	1775 1600 1775 1700
Connection ~ 1775 1700
Wire Wire Line
	1775 1300 1775 1200
Connection ~ 1775 1200
$Comp
L SMA J1
U 1 1 568EE29D
P 8225 1600
F 0 "J1" H 8375 1650 60  0000 L CNN
F 1 "SMA" H 8375 1550 60  0000 L CNN
F 2 "Footprints:SMA90" H 8375 1450 50  0000 L CNN
F 3 "" H 8275 1300 47  0000 C CNN
	1    8225 1600
	1    0    0    -1  
$EndComp
Connection ~ 8050 1600
$Comp
L GND #PWR041
U 1 1 568EEED8
P 8250 1800
F 0 "#PWR041" H 8250 1550 50  0001 C CNN
F 1 "GND" H 8250 1650 50  0000 C CNN
F 2 "" H 8250 1800 60  0000 C CNN
F 3 "" H 8250 1800 60  0000 C CNN
	1    8250 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8175 1750 8325 1750
Connection ~ 8225 1750
Connection ~ 8275 1750
Wire Wire Line
	8250 1800 8250 1750
Connection ~ 8250 1750
$Comp
L R R14
U 1 1 568F1134
P 6300 4575
F 0 "R14" V 6380 4575 50  0000 C CNN
F 1 "2K" V 6300 4575 50  0000 C CNN
F 2 "Footprints:R_0805" V 6230 4575 30  0001 C CNN
F 3 "" H 6300 4575 30  0000 C CNN
	1    6300 4575
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 4725 6300 4725
Connection ~ 6250 4725
Wire Wire Line
	6200 4425 6300 4425
Connection ~ 6250 4425
Wire Wire Line
	9250 4450 9150 4450
Wire Wire Line
	9150 4450 9150 4350
Wire Wire Line
	9150 4350 8850 4350
Wire Wire Line
	5800 1700 6350 1700
Wire Wire Line
	5800 2100 6150 2100
Connection ~ 5800 2100
$Comp
L BC847 Q1
U 1 1 56C20C30
P 6150 5100
F 0 "Q1" H 6350 5175 50  0000 L CNN
F 1 "BC847B" H 6350 5100 50  0000 L CNN
F 2 "Footprints:BC847" H 6350 5025 50  0000 L CIN
F 3 "" H 6150 5100 50  0000 L CNN
	1    6150 5100
	1    0    0    -1  
$EndComp
$EndSCHEMATC