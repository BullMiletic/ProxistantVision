EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 6
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
L Connector:XLR5 XLR?
U 1 1 60E25FB5
P 3600 2400
AR Path="/60E25FB5" Ref="XLR?"  Part="1" 
AR Path="/60E23437/60E25FB5" Ref="XLR201"  Part="1" 
AR Path="/60E266D3/60E25FB5" Ref="XLR?"  Part="1" 
AR Path="/60E26AC7/60E25FB5" Ref="XLR301"  Part="1" 
AR Path="/60E26BB2/60E25FB5" Ref="XLR401"  Part="1" 
AR Path="/60E26BB5/60E25FB5" Ref="XLR501"  Part="1" 
AR Path="/60E26D80/60E25FB5" Ref="XLR601"  Part="1" 
F 0 "XLR601" V 3554 2628 50  0000 L CNN
F 1 "XLR5" V 3645 2628 50  0000 L CNN
F 2 "XLR:IO-XLR5-F-EH" H 3600 2400 50  0001 C CNN
F 3 " ~" H 3600 2400 50  0001 C CNN
F 4 "IO Audio Technologies" H 3600 2400 50  0001 C CNN "MFGR"
F 5 "IO-XLR5-F-EH" H 3600 2400 50  0001 C CNN "MPN"
	1    3600 2400
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60E25FBB
P 3250 1300
AR Path="/60E25FBB" Ref="#PWR?"  Part="1" 
AR Path="/60E23437/60E25FBB" Ref="#PWR0109"  Part="1" 
AR Path="/60E266D3/60E25FBB" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E25FBB" Ref="#PWR0115"  Part="1" 
AR Path="/60E26BB2/60E25FBB" Ref="#PWR0121"  Part="1" 
AR Path="/60E26BB5/60E25FBB" Ref="#PWR0127"  Part="1" 
AR Path="/60E26D80/60E25FBB" Ref="#PWR0133"  Part="1" 
F 0 "#PWR0133" H 3250 1150 50  0001 C CNN
F 1 "+5V" H 3265 1473 50  0000 C CNN
F 2 "" H 3250 1300 50  0001 C CNN
F 3 "" H 3250 1300 50  0001 C CNN
	1    3250 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GNDS #PWR?
U 1 1 60E25FC1
P 3900 1950
AR Path="/60E25FC1" Ref="#PWR?"  Part="1" 
AR Path="/60E23437/60E25FC1" Ref="#PWR0110"  Part="1" 
AR Path="/60E266D3/60E25FC1" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E25FC1" Ref="#PWR0116"  Part="1" 
AR Path="/60E26BB2/60E25FC1" Ref="#PWR0122"  Part="1" 
AR Path="/60E26BB5/60E25FC1" Ref="#PWR0128"  Part="1" 
AR Path="/60E26D80/60E25FC1" Ref="#PWR0134"  Part="1" 
F 0 "#PWR0134" H 3900 1700 50  0001 C CNN
F 1 "GNDS" H 3905 1777 50  0000 C CNN
F 2 "" H 3900 1950 50  0001 C CNN
F 3 "" H 3900 1950 50  0001 C CNN
	1    3900 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 60E25FC7
P 3800 3250
AR Path="/60E25FC7" Ref="#PWR?"  Part="1" 
AR Path="/60E23437/60E25FC7" Ref="#PWR0111"  Part="1" 
AR Path="/60E266D3/60E25FC7" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E25FC7" Ref="#PWR0117"  Part="1" 
AR Path="/60E26BB2/60E25FC7" Ref="#PWR0123"  Part="1" 
AR Path="/60E26BB5/60E25FC7" Ref="#PWR0129"  Part="1" 
AR Path="/60E26D80/60E25FC7" Ref="#PWR0135"  Part="1" 
F 0 "#PWR0135" H 3800 3000 50  0001 C CNN
F 1 "GND" H 3805 3077 50  0000 C CNN
F 2 "" H 3800 3250 50  0001 C CNN
F 3 "" H 3800 3250 50  0001 C CNN
	1    3800 3250
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR?
U 1 1 60E25FCD
P 3250 2800
AR Path="/60E25FCD" Ref="#PWR?"  Part="1" 
AR Path="/60E23437/60E25FCD" Ref="#PWR0112"  Part="1" 
AR Path="/60E266D3/60E25FCD" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E25FCD" Ref="#PWR0118"  Part="1" 
AR Path="/60E26BB2/60E25FCD" Ref="#PWR0124"  Part="1" 
AR Path="/60E26BB5/60E25FCD" Ref="#PWR0130"  Part="1" 
AR Path="/60E26D80/60E25FCD" Ref="#PWR0136"  Part="1" 
F 0 "#PWR0136" H 3250 2650 50  0001 C CNN
F 1 "+24V" H 3265 2973 50  0000 C CNN
F 2 "" H 3250 2800 50  0001 C CNN
F 3 "" H 3250 2800 50  0001 C CNN
	1    3250 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 2700 3500 2900
Wire Wire Line
	3500 2900 3250 2900
Wire Wire Line
	3250 2900 3250 2800
Wire Wire Line
	3600 2100 3600 1850
Wire Wire Line
	3600 1850 3900 1850
Wire Wire Line
	3900 1850 3900 1950
$Comp
L Connector:Screw_Terminal_01x02 TestButton?
U 1 1 60E25FDB
P 2650 2200
AR Path="/60E25FDB" Ref="TestButton?"  Part="1" 
AR Path="/60E23437/60E25FDB" Ref="TestButton201"  Part="1" 
AR Path="/60E266D3/60E25FDB" Ref="TestButton?"  Part="1" 
AR Path="/60E26AC7/60E25FDB" Ref="TestButton301"  Part="1" 
AR Path="/60E26BB2/60E25FDB" Ref="TestButton401"  Part="1" 
AR Path="/60E26BB5/60E25FDB" Ref="TestButton501"  Part="1" 
AR Path="/60E26D80/60E25FDB" Ref="TestButton601"  Part="1" 
F 0 "TestButton601" H 2568 1875 50  0000 C CNN
F 1 "Button" H 2568 1966 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 2650 2200 50  0001 C CNN
F 3 "~" H 2650 2200 50  0001 C CNN
F 4 "Phoenix" H 2650 2200 50  0001 C CNN "MFGR"
F 5 "1729128" H 2650 2200 50  0001 C CNN "MPN"
	1    2650 2200
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60E25FE1
P 3050 1950
AR Path="/60E25FE1" Ref="#PWR?"  Part="1" 
AR Path="/60E23437/60E25FE1" Ref="#PWR0113"  Part="1" 
AR Path="/60E266D3/60E25FE1" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E25FE1" Ref="#PWR0119"  Part="1" 
AR Path="/60E26BB2/60E25FE1" Ref="#PWR0125"  Part="1" 
AR Path="/60E26BB5/60E25FE1" Ref="#PWR0131"  Part="1" 
AR Path="/60E26D80/60E25FE1" Ref="#PWR0137"  Part="1" 
F 0 "#PWR0137" H 3050 1800 50  0001 C CNN
F 1 "+5V" H 3065 2123 50  0000 C CNN
F 2 "" H 3050 1950 50  0001 C CNN
F 3 "" H 3050 1950 50  0001 C CNN
	1    3050 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 2100 3050 2100
Wire Wire Line
	3050 2100 3050 1950
Wire Wire Line
	3300 2400 3200 2400
Wire Wire Line
	2850 2400 2850 2200
$Comp
L Device:LED D?
U 1 1 60E25FEB
P 2850 2700
AR Path="/60E25FEB" Ref="D?"  Part="1" 
AR Path="/60E23437/60E25FEB" Ref="D202"  Part="1" 
AR Path="/60E266D3/60E25FEB" Ref="D?"  Part="1" 
AR Path="/60E26AC7/60E25FEB" Ref="D302"  Part="1" 
AR Path="/60E26BB2/60E25FEB" Ref="D402"  Part="1" 
AR Path="/60E26BB5/60E25FEB" Ref="D502"  Part="1" 
AR Path="/60E26D80/60E25FEB" Ref="D602"  Part="1" 
F 0 "D602" V 2889 2583 50  0000 R CNN
F 1 "LED" V 2798 2583 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 2850 2700 50  0001 C CNN
F 3 "~" H 2850 2700 50  0001 C CNN
F 4 "any" H 2850 2700 50  0001 C CNN "MFGR"
F 5 "any" H 2850 2700 50  0001 C CNN "MPN"
	1    2850 2700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 60E25FF1
P 2850 3200
AR Path="/60E25FF1" Ref="R?"  Part="1" 
AR Path="/60E23437/60E25FF1" Ref="R201"  Part="1" 
AR Path="/60E266D3/60E25FF1" Ref="R?"  Part="1" 
AR Path="/60E26AC7/60E25FF1" Ref="R301"  Part="1" 
AR Path="/60E26BB2/60E25FF1" Ref="R401"  Part="1" 
AR Path="/60E26BB5/60E25FF1" Ref="R501"  Part="1" 
AR Path="/60E26D80/60E25FF1" Ref="R601"  Part="1" 
F 0 "R601" H 2920 3246 50  0000 L CNN
F 1 "4.7R" H 2920 3155 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2780 3200 50  0001 C CNN
F 3 "~" H 2850 3200 50  0001 C CNN
F 4 "any" H 2850 3200 50  0001 C CNN "MFGR"
F 5 "any" H 2850 3200 50  0001 C CNN "MPN"
	1    2850 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 2400 2850 2550
Connection ~ 2850 2400
Wire Wire Line
	2850 2850 2850 3050
$Comp
L power:GNDS #PWR?
U 1 1 60E25FFA
P 2850 3350
AR Path="/60E25FFA" Ref="#PWR?"  Part="1" 
AR Path="/60E23437/60E25FFA" Ref="#PWR0114"  Part="1" 
AR Path="/60E266D3/60E25FFA" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E25FFA" Ref="#PWR0120"  Part="1" 
AR Path="/60E26BB2/60E25FFA" Ref="#PWR0126"  Part="1" 
AR Path="/60E26BB5/60E25FFA" Ref="#PWR0132"  Part="1" 
AR Path="/60E26D80/60E25FFA" Ref="#PWR0138"  Part="1" 
F 0 "#PWR0138" H 2850 3100 50  0001 C CNN
F 1 "GNDS" H 2855 3177 50  0000 C CNN
F 2 "" H 2850 3350 50  0001 C CNN
F 3 "" H 2850 3350 50  0001 C CNN
	1    2850 3350
	1    0    0    -1  
$EndComp
Text HLabel 2350 2400 0    50   Input ~ 0
Signal
Wire Wire Line
	2850 2400 2350 2400
$Comp
L Device:CP C202
U 1 1 60E2A260
P 3550 3250
AR Path="/60E23437/60E2A260" Ref="C202"  Part="1" 
AR Path="/60E26D80/60E2A260" Ref="C602"  Part="1" 
AR Path="/60E26AC7/60E2A260" Ref="C302"  Part="1" 
AR Path="/60E26BB2/60E2A260" Ref="C402"  Part="1" 
AR Path="/60E26BB5/60E2A260" Ref="C502"  Part="1" 
F 0 "C602" V 3805 3250 50  0000 C CNN
F 1 "CP" V 3714 3250 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P2.50mm" H 3588 3100 50  0001 C CNN
F 3 "~" H 3550 3250 50  0001 C CNN
F 4 "ECE-A1HN101U" H 3550 3250 50  0001 C CNN "MPN"
F 5 "Panasonic" H 3550 3250 50  0001 C CNN "MFGR"
	1    3550 3250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3400 3250 3250 3250
Wire Wire Line
	3250 3250 3250 2900
Connection ~ 3250 2900
Wire Wire Line
	3700 3250 3800 3250
Wire Wire Line
	3600 2900 3800 2900
Wire Wire Line
	3800 2900 3800 3250
Wire Wire Line
	3600 2700 3600 2900
Connection ~ 3800 3250
$Comp
L Device:CP C201
U 1 1 60E2BDB2
P 3550 1300
AR Path="/60E23437/60E2BDB2" Ref="C201"  Part="1" 
AR Path="/60E26D80/60E2BDB2" Ref="C601"  Part="1" 
AR Path="/60E26AC7/60E2BDB2" Ref="C301"  Part="1" 
AR Path="/60E26BB2/60E2BDB2" Ref="C401"  Part="1" 
AR Path="/60E26BB5/60E2BDB2" Ref="C501"  Part="1" 
F 0 "C601" V 3805 1300 50  0000 C CNN
F 1 "CP" V 3714 1300 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P2.50mm" H 3588 1150 50  0001 C CNN
F 3 "~" H 3550 1300 50  0001 C CNN
F 4 "ECE-A1HN101U" H 3550 1300 50  0001 C CNN "MPN"
F 5 "Panasonic" H 3550 1300 50  0001 C CNN "MFGR"
	1    3550 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3700 1300 3900 1300
Wire Wire Line
	3900 1300 3900 1850
Connection ~ 3900 1850
Wire Wire Line
	3500 1650 3250 1650
Wire Wire Line
	3250 1650 3250 1300
Wire Wire Line
	3500 1650 3500 2100
Wire Wire Line
	3250 1300 3400 1300
Connection ~ 3250 1300
$Comp
L Device:D D201
U 1 1 60E2E014
P 3200 2000
AR Path="/60E23437/60E2E014" Ref="D201"  Part="1" 
AR Path="/60E26D80/60E2E014" Ref="D601"  Part="1" 
AR Path="/60E26AC7/60E2E014" Ref="D301"  Part="1" 
AR Path="/60E26BB2/60E2E014" Ref="D401"  Part="1" 
AR Path="/60E26BB5/60E2E014" Ref="D501"  Part="1" 
F 0 "D601" V 3154 2079 50  0000 L CNN
F 1 "D" V 3245 2079 50  0000 L CNN
F 2 "Diode_THT:D_DO-15_P10.16mm_Horizontal" H 3200 2000 50  0001 C CNN
F 3 "~" H 3200 2000 50  0001 C CNN
F 4 "LittelFuse" H 3200 2000 50  0001 C CNN "MFGR"
F 5 "SA5.0A" H 3200 2000 50  0001 C CNN "MPN"
	1    3200 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	3200 2400 3200 2150
Connection ~ 3200 2400
Wire Wire Line
	3200 2400 2850 2400
Wire Wire Line
	3200 1850 3600 1850
Connection ~ 3600 1850
$EndSCHEMATC
