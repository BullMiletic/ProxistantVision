EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 5
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
L Connector:Screw_Terminal_01x02 POWER?
U 1 1 60F9E443
P 2000 2350
AR Path="/60F9E443" Ref="POWER?"  Part="1" 
AR Path="/60F9CBE2/60F9E443" Ref="OUT501"  Part="1" 
AR Path="/60F9F677/60F9E443" Ref="POWER801"  Part="1" 
AR Path="/60FA01AF/60F9E443" Ref="OUT201"  Part="1" 
F 0 "OUT201" H 1918 2025 50  0000 C CNN
F 1 "24V_input" H 1918 2116 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 2000 2350 50  0001 C CNN
F 3 "~" H 2000 2350 50  0001 C CNN
F 4 "Phoenix" H 2000 2350 50  0001 C CNN "MFGR"
F 5 "1729128" H 2000 2350 50  0001 C CNN "MPN"
	1    2000 2350
	-1   0    0    1   
$EndComp
$Comp
L power:GNDS #PWR?
U 1 1 60F9E449
P 2350 2500
AR Path="/60F9E449" Ref="#PWR?"  Part="1" 
AR Path="/60F9CBE2/60F9E449" Ref="#PWR0137"  Part="1" 
AR Path="/60F9F677/60F9E449" Ref="#PWR0136"  Part="1" 
AR Path="/60FA01AF/60F9E449" Ref="#PWR0110"  Part="1" 
F 0 "#PWR0110" H 2350 2250 50  0001 C CNN
F 1 "GNDS" H 2355 2327 50  0000 C CNN
F 2 "" H 2350 2500 50  0001 C CNN
F 3 "" H 2350 2500 50  0001 C CNN
	1    2350 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 2350 2350 2350
Wire Wire Line
	2350 2350 2350 2500
$Comp
L Device:D D?
U 1 1 60F9E453
P 2700 2200
AR Path="/60F9E453" Ref="D?"  Part="1" 
AR Path="/60F9CBE2/60F9E453" Ref="D901"  Part="1" 
AR Path="/60F9F677/60F9E453" Ref="D801"  Part="1" 
AR Path="/60FA01AF/60F9E453" Ref="D201"  Part="1" 
F 0 "D201" V 2654 2279 50  0000 L CNN
F 1 "D" V 2745 2279 50  0000 L CNN
F 2 "Diode_THT:D_DO-15_P10.16mm_Horizontal" H 2700 2200 50  0001 C CNN
F 3 "~" H 2700 2200 50  0001 C CNN
F 4 "Littelfuse" H 2700 2200 50  0001 C CNN "MFGR"
F 5 "SA5.0A" H 2700 2200 50  0001 C CNN "MPN"
	1    2700 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	2350 2350 2700 2350
Connection ~ 2350 2350
Wire Wire Line
	2200 2250 2450 2250
Wire Wire Line
	2450 2250 2450 2050
Wire Wire Line
	2450 2050 2700 2050
Wire Wire Line
	2700 2050 2950 2050
Connection ~ 2700 2050
Text HLabel 2950 2050 2    50   Input ~ 0
pin
$EndSCHEMATC
