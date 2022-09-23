EESchema Schematic File Version 5
EELAYER 36 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
Comment5 ""
Comment6 ""
Comment7 ""
Comment8 ""
Comment9 ""
$EndDescr
Connection ~ 2800 2600
Connection ~ 4200 3600
Connection ~ 4600 3600
Wire Wire Line
	2200 2600 2800 2600
Wire Wire Line
	2200 2900 2200 2600
Wire Wire Line
	2800 2500 2800 2600
Wire Wire Line
	2800 2600 2900 2600
Wire Wire Line
	3300 4800 4900 4800
Wire Wire Line
	3400 3500 4200 3500
Wire Wire Line
	3400 3600 4200 3600
Wire Wire Line
	3400 4400 3550 4400
Wire Wire Line
	3400 4900 4800 4900
Wire Wire Line
	4200 3200 4600 3200
Wire Wire Line
	4200 3500 4200 3200
Wire Wire Line
	4200 3600 4200 3800
Wire Wire Line
	4200 3600 4300 3600
Wire Wire Line
	4200 4050 4200 4100
Wire Wire Line
	4200 4050 4600 4050
Wire Wire Line
	4500 3600 4600 3600
Wire Wire Line
	4600 3200 4600 3600
Wire Wire Line
	4600 3600 4600 3800
Wire Wire Line
	4800 4900 4800 5600
Wire Wire Line
	4800 5600 4900 5600
$Comp
L power:+3.3V #PWR?
U 1 1 00000000
P 2800 2500
F 0 "#PWR?" H 2800 2350 50  0001 C CNN
F 1 "+3.3V" H 2800 2700 50  0000 C CNN
F 2 "" H 2800 2500 50  0001 C CNN
F 3 "" H 2800 2500 50  0001 C CNN
	1    2800 2500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 00000000
P 3851 4401
F 0 "#PWR?" H 3851 4251 50  0001 C CNN
F 1 "+3.3V" V 3851 4551 50  0000 L CNN
F 2 "" H 3851 4401 50  0001 C CNN
F 3 "" H 3851 4401 50  0001 C CNN
	1    3851 4401
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 00000000
P 2800 5600
F 0 "#PWR?" H 2800 5350 50  0001 C CNN
F 1 "GND" H 2800 5400 50  0000 C CNN
F 2 "" H 2800 5600 50  0001 C CNN
F 3 "" H 2800 5600 50  0001 C CNN
	1    2800 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 00000000
P 4600 4100
F 0 "#PWR?" H 4600 3850 50  0001 C CNN
F 1 "GND" H 4600 3900 50  0000 C CNN
F 2 "" H 4600 4100 50  0001 C CNN
F 3 "" H 4600 4100 50  0001 C CNN
	1    4600 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 00000000
P 5100 5000
F 0 "#PWR?" H 5100 4750 50  0001 C CNN
F 1 "GND" H 5100 4800 50  0000 C CNN
F 2 "" H 5100 5000 50  0001 C CNN
F 3 "" H 5100 5000 50  0001 C CNN
	1    5100 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 00000000
P 5151 5801
F 0 "#PWR?" H 5151 5551 50  0001 C CNN
F 1 "GND" H 5151 5602 50  0000 C CNN
F 2 "" H 5151 5801 50  0001 C CNN
F 3 "" H 5151 5801 50  0001 C CNN
	1    5151 5801
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R?
U 1 1 00000000
P 3700 4400
F 0 "R?" V 3450 4400 50  0000 C CNN
F 1 "10K" V 3550 4400 50  0000 C CNN
F 2 "" V 3740 4390 50  0001 C CNN
F 3 "~" H 3700 4400 50  0001 C CNN
	1    3700 4400
	0    1    1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 00000000
P 4200 3900
F 0 "C?" H 4350 3950 50  0000 L CNN
F 1 "C" H 4350 3850 50  0000 L CNN
F 2 "" H 4238 3750 50  0001 C CNN
F 3 "~" H 4200 3900 50  0001 C CNN
	1    4200 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 00000000
P 4600 3900
F 0 "C?" H 4750 3950 50  0000 L CNN
F 1 "C" H 4750 3850 50  0000 L CNN
F 2 "" H 4638 3750 50  0001 C CNN
F 3 "~" H 4600 3900 50  0001 C CNN
	1    4600 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y?
U 1 1 00000000
P 4400 3600
F 0 "Y?" H 4400 3900 50  0000 C CNN
F 1 "16MHz" H 4400 3800 50  0000 C CNN
F 2 "" H 4400 3600 50  0001 C CNN
F 3 "HC49US16.000MABJ-UB" H 4400 3600 50  0001 C CNN
	1    4400 3600
	1    0    0    -1  
$EndComp
$Comp
L Doug_symbol_library:FQP30N06L Q?
U 1 1 00000000
P 5050 4800
F 0 "Q?" H 5250 4850 50  0000 L CNN
F 1 "FQP30N06L" H 5250 4750 50  0000 L CNN
F 2 "" H 5300 5050 50  0001 C CNN
F 3 "" H 5300 5050 50  0001 C CNN
	1    5050 4800
	1    0    0    -1  
$EndComp
$Comp
L Doug_symbol_library:FQP30N06L Q?
U 1 1 00000000
P 5100 5600
F 0 "Q?" H 5300 5650 50  0000 L CNN
F 1 "FQP30N06L" H 5300 5550 50  0000 L CNN
F 2 "" H 5350 5850 50  0001 C CNN
F 3 "" H 5350 5850 50  0001 C CNN
	1    5100 5600
	1    0    0    -1  
$EndComp
$Comp
L MCU_Microchip_ATmega:ATmega328-P U?
U 1 1 00000000
P 2800 4100
F 0 "U?" H 1950 5471 50  0000 C CNN
F 1 "ATmega328-P" H 1950 5371 50  0000 C CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 2800 4100 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 2800 4100 50  0001 C CNN
	1    2800 4100
	1    0    0    -1  
$EndComp
$EndSCHEMATC
