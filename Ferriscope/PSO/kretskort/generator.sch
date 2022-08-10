EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 6
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
L Connector:Screw_Terminal_01x05 Signals101
U 1 1 60E1D31F
P 6500 1350
F 0 "Signals101" H 6418 925 50  0000 C CNN
F 1 "Signal" H 6418 1016 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-5-5.08_1x05_P5.08mm_Horizontal" H 6500 1350 50  0001 C CNN
F 3 "~" H 6500 1350 50  0001 C CNN
F 4 "Phoenix" H 6500 1350 50  0001 C CNN "MFGR"
F 5 "1729157" H 6500 1350 50  0001 C CNN "MPN"
	1    6500 1350
	-1   0    0    1   
$EndComp
Text GLabel 7050 1550 2    50   Input ~ 0
S1
Text GLabel 7050 1450 2    50   Input ~ 0
S2
Text GLabel 7050 1350 2    50   Input ~ 0
S3
Text GLabel 7050 1250 2    50   Input ~ 0
S4
Text GLabel 7050 1150 2    50   Input ~ 0
S5
Wire Wire Line
	6700 1550 7050 1550
Wire Wire Line
	6700 1450 7050 1450
Wire Wire Line
	6700 1350 7050 1350
Wire Wire Line
	6700 1250 7050 1250
Wire Wire Line
	6700 1150 7050 1150
$Sheet
S 8550 1150 1350 200 
U 60E23437
F0 "Output1" 50
F1 "XLR-output.sch" 50
F2 "Signal" I L 8550 1250 50 
$EndSheet
$Sheet
S 8550 1600 1350 200 
U 60E26AC7
F0 "Output2" 50
F1 "XLR-output.sch" 50
F2 "Signal" I L 8550 1700 50 
$EndSheet
$Sheet
S 8550 2050 1350 200 
U 60E26BB2
F0 "Output3" 50
F1 "XLR-output.sch" 50
F2 "Signal" I L 8550 2150 50 
$EndSheet
$Sheet
S 8550 2500 1350 200 
U 60E26BB5
F0 "Output4" 50
F1 "XLR-output.sch" 50
F2 "Signal" I L 8550 2600 50 
$EndSheet
$Sheet
S 8550 2950 1350 200 
U 60E26D80
F0 "Output5" 50
F1 "XLR-output.sch" 50
F2 "Signal" I L 8550 3050 50 
$EndSheet
Text GLabel 8550 1250 0    50   Input ~ 0
S1
Text GLabel 8550 1700 0    50   Input ~ 0
S2
Text GLabel 8550 2150 0    50   Input ~ 0
S3
Text GLabel 8550 2600 0    50   Input ~ 0
S4
Text GLabel 8550 3050 0    50   Input ~ 0
S5
$Comp
L Connector:Screw_Terminal_01x02 24VINPUT101
U 1 1 60E27B58
P 6550 2150
F 0 "24VINPUT101" H 6630 2142 50  0000 L CNN
F 1 "24VINPUT" H 6630 2051 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 6550 2150 50  0001 C CNN
F 3 "~" H 6550 2150 50  0001 C CNN
F 4 "Phoenix" H 6550 2150 50  0001 C CNN "MFGR"
F 5 "1729128" H 6550 2150 50  0001 C CNN "MPN"
	1    6550 2150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 5VINPUT101
U 1 1 60E2F7DC
P 6600 3050
F 0 "5VINPUT101" H 6680 3042 50  0000 L CNN
F 1 "5VINPUT" H 6680 2951 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 6600 3050 50  0001 C CNN
F 3 "~" H 6600 3050 50  0001 C CNN
F 4 "Phoenix" H 6600 3050 50  0001 C CNN "MFGR"
F 5 "1729128" H 6600 3050 50  0001 C CNN "MPN"
	1    6600 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 60E322F3
P 6350 2250
AR Path="/60E322F3" Ref="#PWR0101"  Part="1" 
AR Path="/60E23437/60E322F3" Ref="#PWR?"  Part="1" 
AR Path="/60E266D3/60E322F3" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E322F3" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB2/60E322F3" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB5/60E322F3" Ref="#PWR?"  Part="1" 
AR Path="/60E26D80/60E322F3" Ref="#PWR?"  Part="1" 
F 0 "#PWR0101" H 6350 2000 50  0001 C CNN
F 1 "GND" H 6355 2077 50  0000 C CNN
F 2 "" H 6350 2250 50  0001 C CNN
F 3 "" H 6350 2250 50  0001 C CNN
	1    6350 2250
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR0102
U 1 1 60E322F9
P 6350 2150
AR Path="/60E322F9" Ref="#PWR0102"  Part="1" 
AR Path="/60E23437/60E322F9" Ref="#PWR?"  Part="1" 
AR Path="/60E266D3/60E322F9" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E322F9" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB2/60E322F9" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB5/60E322F9" Ref="#PWR?"  Part="1" 
AR Path="/60E26D80/60E322F9" Ref="#PWR?"  Part="1" 
F 0 "#PWR0102" H 6350 2000 50  0001 C CNN
F 1 "+24V" H 6365 2323 50  0000 C CNN
F 2 "" H 6350 2150 50  0001 C CNN
F 3 "" H 6350 2150 50  0001 C CNN
	1    6350 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GNDS #PWR0103
U 1 1 60E322FF
P 6400 3150
AR Path="/60E322FF" Ref="#PWR0103"  Part="1" 
AR Path="/60E23437/60E322FF" Ref="#PWR?"  Part="1" 
AR Path="/60E266D3/60E322FF" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E322FF" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB2/60E322FF" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB5/60E322FF" Ref="#PWR?"  Part="1" 
AR Path="/60E26D80/60E322FF" Ref="#PWR?"  Part="1" 
F 0 "#PWR0103" H 6400 2900 50  0001 C CNN
F 1 "GNDS" H 6405 2977 50  0000 C CNN
F 2 "" H 6400 3150 50  0001 C CNN
F 3 "" H 6400 3150 50  0001 C CNN
	1    6400 3150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0104
U 1 1 60E32305
P 6400 3050
AR Path="/60E32305" Ref="#PWR0104"  Part="1" 
AR Path="/60E23437/60E32305" Ref="#PWR?"  Part="1" 
AR Path="/60E266D3/60E32305" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E32305" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB2/60E32305" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB5/60E32305" Ref="#PWR?"  Part="1" 
AR Path="/60E26D80/60E32305" Ref="#PWR?"  Part="1" 
F 0 "#PWR0104" H 6400 2900 50  0001 C CNN
F 1 "+5V" H 6415 3223 50  0000 C CNN
F 2 "" H 6400 3050 50  0001 C CNN
F 3 "" H 6400 3050 50  0001 C CNN
	1    6400 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x05 GND101
U 1 1 60E35E8F
P 5500 2400
F 0 "GND101" H 5418 1975 50  0000 C CNN
F 1 "GND" H 5418 2066 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical" H 5500 2400 50  0001 C CNN
F 3 "~" H 5500 2400 50  0001 C CNN
	1    5500 2400
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 60E36F6B
P 5850 2600
AR Path="/60E36F6B" Ref="#PWR0105"  Part="1" 
AR Path="/60E23437/60E36F6B" Ref="#PWR?"  Part="1" 
AR Path="/60E266D3/60E36F6B" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E36F6B" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB2/60E36F6B" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB5/60E36F6B" Ref="#PWR?"  Part="1" 
AR Path="/60E26D80/60E36F6B" Ref="#PWR?"  Part="1" 
F 0 "#PWR0105" H 5850 2350 50  0001 C CNN
F 1 "GND" H 5855 2427 50  0000 C CNN
F 2 "" H 5850 2600 50  0001 C CNN
F 3 "" H 5850 2600 50  0001 C CNN
	1    5850 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2600 5850 2500
Wire Wire Line
	5850 2200 5700 2200
Wire Wire Line
	5700 2300 5850 2300
Connection ~ 5850 2300
Wire Wire Line
	5850 2300 5850 2200
Wire Wire Line
	5700 2400 5850 2400
Connection ~ 5850 2400
Wire Wire Line
	5850 2400 5850 2300
Wire Wire Line
	5700 2500 5850 2500
Connection ~ 5850 2500
Wire Wire Line
	5850 2500 5850 2400
Wire Wire Line
	5700 2600 5850 2600
Connection ~ 5850 2600
$Comp
L power:+24V #PWR0106
U 1 1 60E38283
P 5850 1400
AR Path="/60E38283" Ref="#PWR0106"  Part="1" 
AR Path="/60E23437/60E38283" Ref="#PWR?"  Part="1" 
AR Path="/60E266D3/60E38283" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E38283" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB2/60E38283" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB5/60E38283" Ref="#PWR?"  Part="1" 
AR Path="/60E26D80/60E38283" Ref="#PWR?"  Part="1" 
F 0 "#PWR0106" H 5850 1250 50  0001 C CNN
F 1 "+24V" H 5865 1573 50  0000 C CNN
F 2 "" H 5850 1400 50  0001 C CNN
F 3 "" H 5850 1400 50  0001 C CNN
	1    5850 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 1800 5850 1800
Wire Wire Line
	5850 1800 5850 1700
Wire Wire Line
	5700 1700 5850 1700
Connection ~ 5850 1700
Wire Wire Line
	5850 1700 5850 1600
Wire Wire Line
	5700 1600 5850 1600
Connection ~ 5850 1600
Wire Wire Line
	5850 1600 5850 1500
Wire Wire Line
	5700 1500 5850 1500
Connection ~ 5850 1500
Wire Wire Line
	5850 1500 5850 1400
Wire Wire Line
	5700 1400 5850 1400
Connection ~ 5850 1400
$Comp
L Connector:Screw_Terminal_01x05 5V101
U 1 1 60E3A587
P 7700 4050
F 0 "5V101" H 7618 3625 50  0000 C CNN
F 1 "5V" H 7900 3400 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical" H 7700 4050 50  0001 C CNN
F 3 "~" H 7700 4050 50  0001 C CNN
	1    7700 4050
	-1   0    0    1   
$EndComp
$Comp
L Connector:Screw_Terminal_01x05 GND5101
U 1 1 60E3A593
P 7700 4850
F 0 "GND5101" H 7618 4425 50  0000 C CNN
F 1 "GND5" H 7618 4516 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical" H 7700 4850 50  0001 C CNN
F 3 "~" H 7700 4850 50  0001 C CNN
	1    7700 4850
	-1   0    0    1   
$EndComp
Wire Wire Line
	8050 5050 8050 4950
Wire Wire Line
	8050 4650 7900 4650
Wire Wire Line
	7900 4750 8050 4750
Connection ~ 8050 4750
Wire Wire Line
	8050 4750 8050 4650
Wire Wire Line
	7900 4850 8050 4850
Connection ~ 8050 4850
Wire Wire Line
	8050 4850 8050 4750
Wire Wire Line
	7900 4950 8050 4950
Connection ~ 8050 4950
Wire Wire Line
	8050 4950 8050 4850
Wire Wire Line
	7900 5050 8050 5050
Wire Wire Line
	7900 4250 8050 4250
Wire Wire Line
	8050 4250 8050 4150
Wire Wire Line
	7900 4150 8050 4150
Connection ~ 8050 4150
Wire Wire Line
	8050 4150 8050 4050
Wire Wire Line
	7900 4050 8050 4050
Connection ~ 8050 4050
Wire Wire Line
	8050 4050 8050 3950
Wire Wire Line
	7900 3950 8050 3950
Connection ~ 8050 3950
Wire Wire Line
	8050 3950 8050 3850
Wire Wire Line
	7900 3850 8050 3850
$Comp
L power:+5V #PWR0107
U 1 1 60E44713
P 8050 3850
AR Path="/60E44713" Ref="#PWR0107"  Part="1" 
AR Path="/60E23437/60E44713" Ref="#PWR?"  Part="1" 
AR Path="/60E266D3/60E44713" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E44713" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB2/60E44713" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB5/60E44713" Ref="#PWR?"  Part="1" 
AR Path="/60E26D80/60E44713" Ref="#PWR?"  Part="1" 
F 0 "#PWR0107" H 8050 3700 50  0001 C CNN
F 1 "+5V" H 8065 4023 50  0000 C CNN
F 2 "" H 8050 3850 50  0001 C CNN
F 3 "" H 8050 3850 50  0001 C CNN
	1    8050 3850
	1    0    0    -1  
$EndComp
Connection ~ 8050 3850
$Comp
L power:GNDS #PWR0108
U 1 1 60E44D66
P 8050 5050
AR Path="/60E44D66" Ref="#PWR0108"  Part="1" 
AR Path="/60E23437/60E44D66" Ref="#PWR?"  Part="1" 
AR Path="/60E266D3/60E44D66" Ref="#PWR?"  Part="1" 
AR Path="/60E26AC7/60E44D66" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB2/60E44D66" Ref="#PWR?"  Part="1" 
AR Path="/60E26BB5/60E44D66" Ref="#PWR?"  Part="1" 
AR Path="/60E26D80/60E44D66" Ref="#PWR?"  Part="1" 
F 0 "#PWR0108" H 8050 4800 50  0001 C CNN
F 1 "GNDS" H 8055 4877 50  0000 C CNN
F 2 "" H 8050 5050 50  0001 C CNN
F 3 "" H 8050 5050 50  0001 C CNN
	1    8050 5050
	1    0    0    -1  
$EndComp
Connection ~ 8050 5050
$Comp
L Connector:Screw_Terminal_01x05 24V101
U 1 1 60E3454E
P 5500 1600
F 0 "24V101" H 5418 1175 50  0000 C CNN
F 1 "24V" H 5700 950 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x05_P2.54mm_Vertical" H 5500 1600 50  0001 C CNN
F 3 "~" H 5500 1600 50  0001 C CNN
	1    5500 1600
	-1   0    0    1   
$EndComp
$Comp
L Mechanical:MountingHole H101
U 1 1 60E6FE37
P 6150 4250
F 0 "H101" H 6250 4296 50  0000 L CNN
F 1 "MountingHole" H 6250 4205 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_DIN965_Pad" H 6150 4250 50  0001 C CNN
F 3 "~" H 6150 4250 50  0001 C CNN
	1    6150 4250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H102
U 1 1 60E70196
P 6150 4550
F 0 "H102" H 6250 4596 50  0000 L CNN
F 1 "MountingHole" H 6250 4505 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_DIN965_Pad" H 6150 4550 50  0001 C CNN
F 3 "~" H 6150 4550 50  0001 C CNN
	1    6150 4550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H103
U 1 1 60E70357
P 6150 4850
F 0 "H103" H 6250 4896 50  0000 L CNN
F 1 "MountingHole" H 6250 4805 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_DIN965_Pad" H 6150 4850 50  0001 C CNN
F 3 "~" H 6150 4850 50  0001 C CNN
	1    6150 4850
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H104
U 1 1 60E70671
P 6150 5100
F 0 "H104" H 6250 5146 50  0000 L CNN
F 1 "MountingHole" H 6250 5055 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_DIN965_Pad" H 6150 5100 50  0001 C CNN
F 3 "~" H 6150 5100 50  0001 C CNN
	1    6150 5100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H105
U 1 1 60E7DBB6
P 3700 6350
F 0 "H105" H 3800 6396 50  0000 L CNN
F 1 "Image" H 3800 6305 50  0000 L CNN
F 2 "images:globe" H 3700 6350 50  0001 C CNN
F 3 "~" H 3700 6350 50  0001 C CNN
	1    3700 6350
	1    0    0    -1  
$EndComp
$EndSCHEMATC
