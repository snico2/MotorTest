
 Test unipolar & bipolar stepper motor, firmware for ESP8266/ESP32

 Use a serial connection to 115200 and select menu options

 From Primary Menu you can Configure & Move the motor:
 ===========================================
    PRIMARY MENU
 ===========================================
 C) Configure motor
 M) Move motor
 P) show Pin wiring
 V) View configuration
 A) About
 Q) Quit


 From Configuration Menu you can set motor's attribute:
 ===========================================
    MOTOR CONFIGURATION
 ===========================================
 1) set A1 pin
 2) set B1 pin
 3) set A2 pin
 4) set B2 pin
 A) set Anti-clockwise rotation
 C) set Clockwise rotation
 F) set Full step mode
 H) set Half step mode
 M) set Microstepping mode
 W) set Wave mode
 D) set step Delay
 R) set Rpm
 S) set revolution Steps
 P) show Pin wiring
 V) View configuration
 Q) Quit


 From Moving Menu you can init the motor and move step/gradess/rotation:
 ===========================================
    MOTOR MOVE
 ===========================================
 1) run one step
 2) run one grade
 3) run one rotation
 4) run number steps
 5) run number gradees
 6) run number rotations
 I) Init motor
 S) Stop motor
 V) View config
 Q) Quit


 For how to wiring the momtor you can use "show Pin wiring" menu choice.


   Unipolar Stepper Motor Coils - Step sequence: A1, B1, A2, B2

   A2 o----+
           |
           @
     A o---@
           @
           @
           |
   A1 o----+
                B1 o---@@@@@@@---o B2
                          |
                          B


   Bipolar Stepper Motor Coils - Step sequence: A1, B1, A2, B2

   A2 o----+
           |
           @
           @
           @
           @
           |
   A1 o----+
                B1 o---@@@@@@@---o B2


 1) Test performed on 28BYJ-48 Unipolar Stepper Motor with ULN2003 and WemosMini ESP8266 boards
    In Full-Step mode:
        - steps for revolution: 2048
        - step delay: 2000 us
    In Half-Step mode:
        - steps for revolution: 4096
        - step delay: 1000 us

      MCU    WEMOS   ULN2003   28BYJ-48    COILS
     -----------------------------------------------
     gpio12   D6      IN4       orange      A1
     gpio13   D7      IN3       yellow      B1
     gpio14   D5      IN2       pink        A2
     gpio15   D8      IN1       blue        B2


2) Test performed on 28BYJ-48 Unipolar Stepper Motor with L293D and WemosMini ESP8266 boards
    Vcc 28BYJ-48 NOT connected
    In Full-Step mode:
        - steps for revolution: 2048
        - step delay: 2000 us
    In Half-Step mode:
        - steps for revolution: 4096
        - step delay: 1000 us

      MCU    BOARD   L923D    BOARD   28BYJ-48   COILS
     -----------------------------------------------
     gpio12   D6      IN2      A+     orange      A1
     gpio13   D7      IN3      B-     yellow      B1
     gpio14   D5      IN1      A-     pink        A2
     gpio15   D8      IN4      B+     blue        B2


3) Test performed on BP485725 Bipolar Stepper Motor with L293D and WemosMini ESP8266 boards
    In Full-Step mode:
        - steps for revolution: 48
        - step delay: 8000 us
    In Half-Step mode:
        - steps for revolution: 96
        - step delay: 4000 us

      MCU    BOARD   L923D    BOARD   BP485725   COILS
     -----------------------------------------------
     gpio12   D6      IN2      A+     red         A1
     gpio13   D7      IN3      B-     yellow      B1
     gpio14   D5      IN1      A-     brown       A2
     gpio15   D8      IN4      B+     orange      B2

