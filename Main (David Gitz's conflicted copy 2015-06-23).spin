{{               
********************************************************
* ICARUS Rover Sonic Controller                        *
* Author: David Gitz                                   *
* Copyright (c) 2014 David Gitz/FAST Robotics          *     
********************************************************

TODO
 - Documentation
   
{{
TARGET:
  - FAST Robotics Quickstart/Robot Shield
PURPOSE:
  - ICARUS Rover Sonar Controller, 
}}
{{
SETUP:
}}

{{
INDICATOR LIGHTS
 Startup LED: On when Booting, Off otherwise.
 Run LED: Blink On/Off at ~10 Hz when Running normally.
}}
{{
LOG:
  DPG 6-OCT-2014.  Created Program.
  DPG 22-MAR-2015. Updated Program for Sonic Controller Code for ICARUS Rover.  Currently 3 Sonars implemented. 
 
  
}}
CON

' Timing             
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000 
  
' ASCII Constants
  CR = 13
  LF = 10
  SPACE = 32
  PERIOD = 46
  COMMA =  44
  TAB = 9

 'Loop Rates
  FAST_LOOP = 1 '1000 Hz, 1 mS
  MEDIUM_LOOP = 50 '20 Hz, 50 mS
  SLOW_LOOP = 100 '10 Hz, 100 mS

  PRIORITY_HIGH = 0
  PRIORITY_MEDIUM = 1
  PRIORITY_LOW = 2  

  ' UART Definitions
    debug_port = 0
    rc_port = 1

      
  ' Hardware Pins************************************
  PINNOTUSED = -1                    

  ' UART Pins
  usbtx = 30  'USB TX Line
  usbrx = 31  'USB RX Line
  comtx = 8  'Robot Shield TX Line, FIX
  comrx = 7  'Robot Shield RX Line, FIX

' Led's

  LED4 = 17
  LED3 = 18
  LED2 = 19
  LED1 = 20

' Motor Out Pin: Set to -1 if not used
  SERVO1 = 16
  SERVO2 = 15
  SERVO3 = 14
  SERVO4 = 13

  'ADC Pins/Misc
  ADCuartPin = 21
  ADCclkPin = 22
  ADCcsPin = 23


  'Analog Sensors
  AnalogSense1 = 0
  AnalogSense2 = 1
  AnalogSense3 = 2
  AnalogSense4 = 3
  AnalogSense5 = 4
  AnalogSense6 = 5
  AnalogSense7 = 6
  AnalogSense8 = 7

 'GPIO Pin Definitions
  GPIO1 = 6
  GPIO2 = 5
  GPIO3 = 4
  GPIO4 = 3
  GPIO5 = 2

  'US Pin Definitions.  These are just more GPIO
  US1 = 13
  US2 = 12
  US3 = 11
  US4 = 10

  'SD Card Pin Definitions
  SD_DO = 24
  SD_SCLK = 25
  SD_DI = 26
  SD_CS = 27

  'Input Switch Pin Definitions
  SW1 = 0
  SW2 = 1

' ****************************************************  

  'ADC Definitions
  ADC_VOLTAGE_CONVERSION = 18   
  ADC_LOWER_THRESHOLD = 100  'Threshold for converting Analog Input to Boolean states.  Below this is FALSE.
  ADC_UPPER_THRESHOLD = 3900   'Threshold for converting Analog Input to Boolean states.  Above this is TRUE.
  ADC_MIN_VALUE = 0
  ADC_MAX_VALUE = 4096
  ADCmode = $00FF  
  
 'PWM/Motor Definitions

  PWM_MIN_VALUE = 1000
  PWM_MAX_VALUE = 2000
  PWM_NEUTRAL_VALUE = 1500 


  'FAST Protocol Definitions
  SD = $24
  ED = $2A
  'Message/Sub-Message Type Definitions
  MT_ERROR = $01
  SMT_NOERROR = $01


  'Value Type Definitions
  VT_NODATA = $FF
  VT_1INT = $01
   
  ' User Pin Definitions
  Startup_LED = LED1
  Run_LED = LED2
  Armed_LED = LED3
  Error_LED = LED4

  'Ultrasonic Sensor Definitions
  PING1 = GPIO5
  PING2 = GPIO4
  PING3 = GPIO3






OBJ
  uart:         "pcFullDuplexSerial4FC"
  pwmout:       "PWM_32_v4"
  util:         "Util"
  adc:          "MCP3208"
  str:          "STRINGS2"
  sdcard:           "fsrw"
  ping:         "ping"
  math:         "spin_trigpack"
  timer:        "Timer32"
  numbers:      "Numbers"

VAR
  'Program Variables
  byte cogsused

  'Timing Variables
  long wait_mS
  long elapsedtime
  long slow_loop_count
  long medium_loop_count
  long fast_loop_count
  byte prioritylevel
  word ontime
  long armedtimer
  byte armedstart                                              

  'UART Variables
  long stack[10]  
  byte rxbyte,rxinit  
  byte tempbyte
  byte tempstr1[50]
  byte  stringbuffer[100]
  byte comrxbuffer[100] 
  byte Debug

  'Sensor Variables
  long ping1_distance
  long ping2_distance
  long ping3_distance

  
  'SD Card Variables
  byte tbuf[20]
  byte bigbuf[256]

PUB init | i,j,sum, tempstr, temp1,fileerror
  
  ping1_distance := 0
  ping2_distance := 0
  ping3_distance := 0
  
  wait_mS := clkfreq/1000 '1 mS




  'Set LED Pins as outputs if they should be.  We don't know if they are contiguous or not.
  DIRA[Startup_LED]~~
  DIRA[Run_LED]~~
  DIRA[Armed_LED]~~
  DIRA[Error_LED]~~
  OUTA[Startup_LED] := TRUE
  OUTA[Run_LED] := FALSE
  OUTA[Armed_LED] := FALSE
  OUTA[Error_LED] := FALSE

  'Wait a couple seconds before doing anything
  
  cogsused := 0
  slow_loop_count := medium_loop_count := fast_loop_count := 0
  i := 0      
  'Set GPIO Pins as Inputs/Outputas as needed.


   'Initialize all Ports
  uart.init                    

  uart.AddPort(debug_port,usbrx,usbtx,uart#PINNOTUSED,uart#PINNOTUSED,uart#DEFAULTTHRESHOLD,uart#NOMODE,uart#BAUD115200)
  uart.AddPort(rc_port,comrx,comtx,uart#PINNOTUSED,uart#PINNOTUSED,uart#DEFAULTTHRESHOLD,uart#NOMODE,uart#BAUD115200)
  waitcnt(clkfreq*5 + cnt)
                         
  if (temp1 := uart.Start) 
      cogsused += 1 'Should be 1
      uart.rxflush(debug_port)
      uart.rxflush(debug_port)  
  if (temp1 := pwmout.start) 'servo.start
    cogsused += 1 'Should be 2
  else
    uart.str(debug_port,string("pwmout not started"))
    uart.dec(debug_port,temp1)

  
  'Start sensors

  if (temp1 := adc.start(ADCuartPin, ADCclkPin, ADCcsPin, ADCmode))
    cogsused += 1 'Should be 5
  else
    uart.str(debug_port,string("adc not started"))
    uart.dec(debug_port,temp1)

    
  if (temp1 := sdcard.mount(SD_DO))

  else
    uart.str(debug_port,string("sdcard card not mounted"))
    uart.dec(debug_port,temp1)

  math.Start_Driver
  numbers.Init
  uart.str(debug_port,string("Cog's Used:"))
  uart.dec(debug_port,cogsused)
  uart.str(debug_port,string(CR,LF))
   
  OUTA[Startup_LED] := FALSE
  mainloop

PUB testuart
  repeat
    waitcnt(clkfreq/10 + cnt)
   ' uart.tx(debug_port,uart.rxtime(pi_port,1))
    uart.str(rc_port,string("HI BACK",CR,LF))
    'uart.tx(rc_port,uart.rx(debug_port))
    'uart.tx(debug_port,uart.rx(debug_port))
    !outa[LED1]



PUB testping
  repeat
    waitcnt(clkfreq/10 + cnt)
      uart.str(debug_port,string("Distance: "))
      uart.dec(debug_port,ping.Inches(PING1))
      uart.str(debug_port,string(" in",CR,LF))
      !outa[LED1]


PUB mainloop | i, j, value1, value2, value3,bootmode,ledmode,ledpin , tempstrA,tempstrB,tempstrC,storagecount,temp1, lasttime
  ontime := 0
  prioritylevel := -1
  fast_loop_count := medium_loop_count := slow_loop_count := 0
  repeat
    waitcnt(1*wait_mS + cnt) 
    ontime++
    if ontime > 1000
      ontime := 1
      fast_loop_count := medium_loop_count := slow_loop_count := 0
    if (ontime - fast_loop_count) > FAST_LOOP
      fast_loop_count := ontime 
      prioritylevel := PRIORITY_HIGH        
     
    elseif (ontime - medium_loop_count) > MEDIUM_LOOP
      medium_loop_count := ontime
      prioritylevel := PRIORITY_MEDIUM      
     
    elseif (ontime - slow_loop_count) > SLOW_LOOP
      slow_loop_count := ontime
      prioritylevel := PRIORITY_LOW 
    else
      prioritylevel := -1                 

   
    case prioritylevel
           
      PRIORITY_HIGH: 'Read Mode (Arm/Disarm) Commands

        

      PRIORITY_MEDIUM:
        uart.str(rc_port,string("$SON,"))
        uart.dec(rc_port,ping.Inches(PING1))
        uart.str(rc_port,string(","))
        uart.dec(rc_port,ping.Inches(PING2))
        uart.str(rc_port,string(","))
        uart.dec(rc_port,ping.Inches(PING3))
        uart.str(rc_port,string("*",CR,LF))

      PRIORITY_LOW:  
  
        !OUTA[Run_LED]

      OTHER:

  