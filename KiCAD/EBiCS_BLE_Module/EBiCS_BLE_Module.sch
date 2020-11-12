EESchema Schematic File Version 4
LIBS:EBiCS_BLE_Module-cache
EELAYER 30 0
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
$EndDescr
$Comp
L HM-10:HM-10 U2
U 1 1 5FA814D7
P 6100 3800
F 0 "U2" H 6050 4837 60  0000 C CNN
F 1 "HM-10" H 6050 4731 60  0000 C CNN
F 2 "" H 5850 4550 60  0001 C CNN
F 3 "" H 5850 4550 60  0001 C CNN
F 4 "Bluetooth 4.0 BLE module" H 6350 4350 61  0001 C CNN "DESC"
F 5 "JNHuaMao Technology" H 6150 4150 61  0001 C CNN "MFG_NAME"
F 6 "HM-10" H 6250 4250 61  0001 C CNN "MPN"
	1    6100 3800
	1    0    0    -1  
$EndComp
$Comp
L DC-DC:LM2574N-3.3 U1
U 1 1 5FA8238A
P 3300 2550
F 0 "U1" H 3300 3099 50  0000 C CNN
F 1 "LM2574N-3.3" H 3300 3008 50  0000 C CNN
F 2 "DIP-8" H 3300 2917 50  0000 C CIN
F 3 "www.national.com/ds/LM/LM2574.pdf" H 3300 2826 50  0000 C CNN
	1    3300 2550
	1    0    0    -1  
$EndComp
$Comp
L JST:HEADER_5 J1
U 1 1 5FA837D2
P 1550 3850
F 0 "J1" H 1542 3413 60  0000 C CNN
F 1 "HEADER_5" H 1542 3519 60  0000 C CNN
F 2 "" H 1550 3850 60  0000 C CNN
F 3 "" H 1550 3850 60  0000 C CNN
	1    1550 3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	1650 4050 4300 4050
Wire Wire Line
	4300 4050 4300 3050
Wire Wire Line
	4300 3050 5350 3050
Wire Wire Line
	1650 3950 4250 3950
Wire Wire Line
	4250 3950 4250 3150
Wire Wire Line
	4250 3150 5350 3150
Wire Wire Line
	1650 3850 3400 3850
Wire Wire Line
	3400 3850 3400 2850
Wire Wire Line
	3400 3850 3400 4250
Wire Wire Line
	3400 4250 5350 4250
Connection ~ 3400 3850
Wire Wire Line
	1650 3650 2550 3650
Wire Wire Line
	2550 3650 2550 2650
Wire Wire Line
	2550 2450 2800 2450
Wire Wire Line
	2550 2650 2800 2650
Connection ~ 2550 2650
Wire Wire Line
	2550 2650 2550 2450
Wire Wire Line
	3800 2650 3800 4150
Wire Wire Line
	3800 4150 5350 4150
$Comp
L Device:Q_PMOS_GSD Q2
U 1 1 5FA88256
P 9100 3400
F 0 "Q2" H 9306 3354 50  0000 L CNN
F 1 "IRLF9014" H 9306 3445 50  0000 L CNN
F 2 "" H 9300 3500 50  0001 C CNN
F 3 "~" H 9100 3400 50  0001 C CNN
	1    9100 3400
	1    0    0    1   
$EndComp
$Comp
L Device:Q_NPN_BCE Q1
U 1 1 5FA88D7B
P 8000 3600
F 0 "Q1" H 8191 3646 50  0000 L CNN
F 1 "Q_NPN_BCE" H 8191 3555 50  0000 L CNN
F 2 "" H 8200 3700 50  0001 C CNN
F 3 "~" H 8000 3600 50  0001 C CNN
	1    8000 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5FA8B0CA
P 8450 3250
F 0 "R5" H 8520 3296 50  0000 L CNN
F 1 "20k" H 8520 3205 50  0000 L CNN
F 2 "" V 8380 3250 50  0001 C CNN
F 3 "~" H 8450 3250 50  0001 C CNN
	1    8450 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2450 2550 1800
Wire Wire Line
	2550 1800 7250 1800
Connection ~ 2550 2450
Wire Wire Line
	9200 1800 9200 3200
Wire Wire Line
	8100 3400 8450 3400
$Comp
L Device:R R3
U 1 1 5FA8D17F
P 7650 3600
F 0 "R3" V 7443 3600 50  0000 C CNN
F 1 "4k7" V 7534 3600 50  0000 C CNN
F 2 "" V 7580 3600 50  0001 C CNN
F 3 "~" H 7650 3600 50  0001 C CNN
	1    7650 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	3400 4250 3400 5500
Wire Wire Line
	3400 5500 7250 5500
Wire Wire Line
	8100 5500 8100 3800
Connection ~ 3400 4250
Wire Wire Line
	6750 3950 7500 3950
Wire Wire Line
	7500 3950 7500 3600
Wire Wire Line
	9200 3600 9200 5350
Wire Wire Line
	9200 5350 1950 5350
Wire Wire Line
	1950 5350 1950 3750
Wire Wire Line
	1950 3750 1650 3750
$Comp
L Device:R R1
U 1 1 5FA8FDE2
P 7250 3200
F 0 "R1" H 7320 3246 50  0000 L CNN
F 1 "10k" H 7320 3155 50  0000 L CNN
F 2 "" V 7180 3200 50  0001 C CNN
F 3 "~" H 7250 3200 50  0001 C CNN
	1    7250 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5FA9045D
P 7250 3500
F 0 "R2" H 7320 3546 50  0000 L CNN
F 1 "420" H 7320 3455 50  0000 L CNN
F 2 "" V 7180 3500 50  0001 C CNN
F 3 "~" H 7250 3500 50  0001 C CNN
	1    7250 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 1800 7250 3050
Connection ~ 7250 1800
Wire Wire Line
	7250 3650 7250 5500
Connection ~ 7250 5500
Wire Wire Line
	7250 5500 8100 5500
Wire Wire Line
	7250 3350 7000 3350
Wire Wire Line
	7000 3350 7000 3750
Wire Wire Line
	7000 3750 6750 3750
Connection ~ 7250 3350
$Comp
L Device:LED D1
U 1 1 5FAC254D
P 8450 2050
F 0 "D1" V 8489 1933 50  0000 R CNN
F 1 "LED" V 8398 1933 50  0000 R CNN
F 2 "" H 8450 2050 50  0001 C CNN
F 3 "~" H 8450 2050 50  0001 C CNN
	1    8450 2050
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 5FAC4085
P 8450 2550
F 0 "R4" H 8520 2596 50  0000 L CNN
F 1 "10k" H 8520 2505 50  0000 L CNN
F 2 "" V 8380 2550 50  0001 C CNN
F 3 "~" H 8450 2550 50  0001 C CNN
	1    8450 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 1800 8450 1800
Wire Wire Line
	8450 1800 8450 1900
Connection ~ 8450 1800
Wire Wire Line
	8450 1800 9200 1800
Wire Wire Line
	8450 2200 8450 2400
Wire Wire Line
	8450 2700 8450 2900
Wire Wire Line
	8450 2900 8900 2900
Wire Wire Line
	8900 2900 8900 3400
Connection ~ 8450 2900
Wire Wire Line
	8450 2900 8450 3100
Text Notes 1150 3700 0    50   ~ 0
Batt+
Text Notes 950  3800 0    50   ~ 0
Power Lock
Text Notes 1200 3900 0    50   ~ 0
GND
Text Notes 1050 4000 0    50   ~ 0
UART Rx
Text Notes 1050 4100 0    50   ~ 0
UART Tx
$EndSCHEMATC
