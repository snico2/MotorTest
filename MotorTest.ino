/******************************************************************************************
 *  File:           MotorTest.ino
 *  Author:         Nicola Sellitto     nicosellitto@yahoo.it
 *  Version:        0.20
 *  Date:           31 may 2024
 *  Description:    Test Motor: Unipolar, Bipolar stepper motor & Servo
 *  Note:           Use a serial connection to 115200 and select menu options
 *****************************************************************************************/

#include <Arduino.h>
#if defined ESP8266
    #include <EEPROM.h>
#elif defined ESP32
    #include <Preferences.h>
#else
    #error "CHIP type error"
#endif
#include <math.h>
#include "MotorTest.h"

struct sHeader       Header;
struct sMicrostep    Microstep;
struct sStepper      Stepper;
struct sMotorStep    MotorStep;
struct sServo        Servo;

 uint8_t MotorDriver = NOT_DEFINED;                 // Motor drive type 
    bool MotorInitialized = false;                  // Flag init ok
 uint8_t MenuChoice = 0;
 uint8_t MenuSubChoice = 0;
 uint8_t MenuCurrent = MENU_ROOT;
 uint8_t MenuState = STATE_DISPLAY_MENU;
    bool MenuRunnig = true;
const char* ChoiceRange;    
uint32_t ValueMin;
uint32_t ValueMax;
uint32_t MoveStartTime;                             // time starting moving in ms
uint32_t MSPulseTimer;
uint16_t Overhead = 10;                             // time in us  

volatile uint32_t NumSteps;                         // Number of Stpes to move
volatile bool     PulseHigh;
volatile bool     PulseNew;


char PrintBuffer[256];				                // Buffer per funzioni di Print
const char* LineSeparator = "===========================================";

#if defined ESP32
    Preferences PrefsArea;
    hw_timer_t *PTimer = NULL;
#endif


/******************************************************************************************
 * 
 *****************************************************************************************/
void IRAM_ATTR pulseTimer() {
uint8_t numPhase=3;

    switch (MotorDriver) {
        case DRIVE_MICROSTEP:
            digitalWrite(Microstep.pinStep, HIGH);
            delayMicroseconds(MICROSTEP_PULSE_HIGH);                    // 5us
            digitalWrite(Microstep.pinStep, LOW);
            break;
        case DRIVE_DARLINGTON:
        case DRIVE_HBRIDGE:
            switch (Stepper.stepSequence) {
                case SEQUENCE_ONEPHASE:  runOnePhase();  break;
                case SEQUENCE_FULLSTEP:  runFullStep();  break;
                case SEQUENCE_HALFSTEP:  runHalfStep();  numPhase = 7;  break;
            }
            if (MotorStep.rotationType == ROTATION_CLOCKWISE) {
                if (++Stepper.idPhase > numPhase) Stepper.idPhase = 0;
            } else {
                if (--Stepper.idPhase < 0) Stepper.idPhase = numPhase;
            }
            break;
        case DRIVE_SERVO:
        	if (PulseHigh) {
	            digitalWrite(Servo.pinPwm, HIGH);
                timer1_write(Servo.ticksPulseHigh);
                PulseNew = true;
            } else {
	            digitalWrite(Servo.pinPwm, LOW);
                timer1_write(Servo.ticksPulseLow);   	
            }
            PulseHigh = !PulseHigh;
            return;
    }

    if (--NumSteps == 0) {
        #if defined ESP8266
            timer1_disable();
        #elif defined ESP32
            timerStop(PTimer);                     // Stop Timer counter
        #endif
    }
}

/******************************************************************************************
 * 
 *****************************************************************************************/
void setup() {

    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial);                              // wait connect, needed for native USB port
	Serial.println("Waiting 5 sec");		
    delay(5000);
    showAbout();
    configLoad();
    calculateRpmStepDelay();
    menuInit();
    // Prescaler  16   => 80 MHz/16  =  5000.0 KHz  =>   5.0     ticks/us  =>  Min: (1/  5.0)   = 0,20   us   Max: (2^23 /   5,0)   us = ~ 1,677s
    // Prescaler 256   => 80 MHz/256 =   312.5 KHz  =>   0.3125  ticks/us  =>  Min: (1/ 0.3125) = 3,2000 us   Max: (2^23 /  0.3125) us = ~26.800s
    #if defined ESP8266
        timer1_attachInterrupt(pulseTimer);
        timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
    #elif defined ESP32
        PTimer = timerBegin(0, 16, true);                        // Counter UP (true)
        timerAttachInterrupt(PTimer, &pulseTimer, true);  
    #endif
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void loop() {  

    if (MenuRunnig) menuLoop();
}


/******************************************************************************************
 *
 *****************************************************************************************/
void configLoad() {

    Serial.println("Load Configuration");

#if defined ESP8266
    // PSEUDO EEPROM
	EEPROM.begin(CONFIG_SIZE);
    EEPROM.get(CONFIG_START_HEADER,    Header);
    EEPROM.get(CONFIG_START_MOTORSTEP, MotorStep);
    EEPROM.get(CONFIG_START_STEPPER,   Stepper);
    EEPROM.get(CONFIG_START_MICROSTEP, Microstep);
    EEPROM.get(CONFIG_START_SERVO,     Servo);
    EEPROM.end();
#elif defined ESP32
    PrefsArea.begin("cfgmotor", true);                         // Open in Read-Only
    if (PrefsArea.isKey("Header")) {
        PrefsArea.getBytes("Header",     &Header,     CONFIG_SIZE_HEADER);
        PrefsArea.getBytes("MotorStep",  &MotorStep,  CONFIG_SIZE_MOTORSTEP);
        PrefsArea.getBytes("Stepper",    &Stepper,    CONFIG_SIZE_STEPPER);
        PrefsArea.getBytes("Microstep",  &Microstep,  CONFIG_SIZE_MICROSTEP);
        PrefsArea.getBytes("Servo",      &Servo,      CONFIG_SIZE_SERVO);
        PrefsArea.end();
    } else {
        PrefsArea.end();
        Serial.println("Configuration missing - Reset Value");
        configDefault();
        configSave();
    }
#endif

    if (Header.id != CONFIG_ID || Header.version != CONFIG_VERSION) {
		Serial.println("Configuration Invalid");		
        configDefault();
        configSave();
	} else {
  		Serial.println("Configuration Laded");		
    }

}


/******************************************************************************************
 * @brief   Write configuration storage
 * @param   n/a
 * @returns n/a
 *****************************************************************************************/
void configSave() {

#if defined ESP8266
	EEPROM.begin(CONFIG_SIZE);
    EEPROM.put(CONFIG_START_HEADER,     Header);
    EEPROM.put(CONFIG_START_MOTORSTEP,  MotorStep);
    EEPROM.put(CONFIG_START_STEPPER,    Stepper);
    EEPROM.put(CONFIG_START_MICROSTEP,  Microstep);
    EEPROM.put(CONFIG_START_SERVO,      Servo);
	EEPROM.commit();
    EEPROM.end();
#elif defined ESP32
	PrefsArea.begin("cfgmotor", false);			  							// Open in Read-Write
    PrefsArea.putBytes("Header",     &Header,     CONFIG_SIZE_HEADER);
    PrefsArea.putBytes("MotorStep",  &MotorStep,  CONFIG_SIZE_MOTORSTEP);
    PrefsArea.putBytes("Stepper",    &Stepper,    CONFIG_SIZE_STEPPER);
    PrefsArea.putBytes("Microstep",  &Microstep,  CONFIG_SIZE_MICROSTEP);
    PrefsArea.putBytes("Servo",      &Servo,      CONFIG_SIZE_SERVO);
    Serial.print("Preferences Write ");
    if (PrefsArea.isKey("Header")) Serial.println("OK");
    else Serial.println("ERROR");
    PrefsArea.end();
#endif
    Serial.println("Configuration Saved");		
}

/******************************************************************************************
 * 
 *****************************************************************************************/
void configReset() {
uint16_t i;
char* ptr;

    // Reset Config
    ptr = (char*) &Header;
    for (i=0; i<CONFIG_SIZE_HEADER; i++) *ptr++ = 0;
    ptr = (char*) &MotorStep;
    for (i=0; i<CONFIG_SIZE_MOTORSTEP; i++) *ptr++ = 0;
    ptr = (char*) &Stepper;
    for (i=0; i<CONFIG_SIZE_STEPPER; i++) *ptr++ = 0;
    ptr = (char*) &Microstep;
    for (i=0; i<CONFIG_SIZE_MICROSTEP; i++) *ptr++ = 0;
    ptr = (char*) &Servo;
    for (i=0; i<CONFIG_SIZE_SERVO; i++) *ptr++ = 0;
    Serial.println("Configuration reset to Null");		    
}



/******************************************************************************************
 * 
 *****************************************************************************************/
void configDefault() {

  	Header.id      = CONFIG_ID; 
	Header.version = CONFIG_VERSION;
    initMotorStep();
    initMicrostep();
    initStepper();
    initServo();
    Serial.println("Configuration set to Default");		    
}


/******************************************************************************************
 *
 *****************************************************************************************/
void initMotorStep() {

    MotorStep.stepDelay = DEAFULT_STEP_DELAY;               // default time in us
    MotorStep.rpm = DEAFULT_RPM;                            // default Revolution per Minute
    MotorStep.stepsRevolution = DEAFULT_STEPS_REVOLUTION;   // default number of step for a revolution
    MotorStep.stepResolution = DEAFULT_STEP_RESOLUTION;     // Default value   
    MotorStep.rotationType = DEAFULT_ROTATION;              // Default value
    calculateRpmStepDelay();                                // time in us
}


/******************************************************************************************
 *
 *****************************************************************************************/
void initServo() {

    Servo.pinPwm      = NOT_DEFINED;
    Servo.pulsePeriod = DEAFULT_SERVO_PERIOD;               // 20000 time in us
    Servo.pulseMin    = DEAFULT_SERVO_PULSE_MIN;            //  1000 time in us
    Servo.pulseMax    = DEAFULT_SERVO_PULSE_MAX;            //  2000 time in us
    Servo.pulseTime   = DEAFULT_SERVO_PULSE_TIME;           //  1500 time in us
    Servo.gradees     = DEAFULT_SERVO_GRADEES;              //  
}


/******************************************************************************************
 *
 *****************************************************************************************/
void initStepper() {

    Stepper.stepSequence = DEAFULT_SEQUENCE;                 // Default value
    Stepper.idPhase = 0;
    Stepper.pinA1 = NOT_DEFINED;
    Stepper.pinA2 = NOT_DEFINED;
    Stepper.pinB1 = NOT_DEFINED;
    Stepper.pinB2 = NOT_DEFINED;
}


/******************************************************************************************
 *
 *****************************************************************************************/
void initMicrostep() {

    Microstep.pinReset    = NOT_DEFINED;
    Microstep.pinEnable   = NOT_DEFINED;
    Microstep.pinSleep    = NOT_DEFINED;
    Microstep.pinS1       = NOT_DEFINED;
    Microstep.pinS2       = NOT_DEFINED;
    Microstep.pinS3       = NOT_DEFINED;
    Microstep.pinStep     = NOT_DEFINED;
    Microstep.pinDir      = NOT_DEFINED;
    Microstep.levelEnable = NOT_DEFINED;
    Microstep.levelSleep  = NOT_DEFINED;
    Microstep.levelReset  = NOT_DEFINED;
    Microstep.levelDir    = HIGH;                       // Default value
    Microstep.valueS1     = NOT_DEFINED;
    Microstep.valueS2     = NOT_DEFINED;
    Microstep.valueS3     = NOT_DEFINED;
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void microstepEnable(bool enable) {

    if (MotorDriver != DRIVE_MICROSTEP) {
    	Serial.println();
       	Serial.println("ABORT - not using microstep motor");
        return;
    }

    if (Microstep.pinEnable == NOT_DEFINED) {
    	Serial.println();
        Serial.println("ABORT - pin ENABLE not defined");
        return;
    }

    if (Microstep.levelEnable == NOT_DEFINED) {
    	Serial.println();
        Serial.println("ABORT - level ENABLE not defined");
        return;
    }

    if (enable) {
        (Microstep.levelEnable == 'H') ? digitalWrite(Microstep.pinEnable, HIGH) : digitalWrite(Microstep.pinEnable, LOW);  
    	Serial.println("Microstep motor Enabled");
    } else {
        (Microstep.levelEnable == 'H') ? digitalWrite(Microstep.pinEnable, LOW)  : digitalWrite(Microstep.pinEnable, HIGH);
    	Serial.println("Microstep motor Disabled");
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void microstepSleep(bool sleep) {

    if (MotorDriver != DRIVE_MICROSTEP) {
    	Serial.println();
       	Serial.println("ABORT - not using microstep motor");
        return;
    }

    if (Microstep.pinSleep == NOT_DEFINED) {
    	Serial.println();
        Serial.println("ABORT - pin SLEEP not defined");
        return;
    }

    if (Microstep.levelSleep == NOT_DEFINED) {
    	Serial.println();
        Serial.println("ABORT - level SLEEP not defined");
        return;
    }

    if (sleep) {
        (Microstep.levelSleep == 'H') ? digitalWrite(Microstep.pinSleep, HIGH) : digitalWrite(Microstep.pinSleep, LOW);  
    	Serial.println();
    	Serial.println("Microstep motor Slept");

    } else {
        (Microstep.levelSleep == 'H') ? digitalWrite(Microstep.pinSleep, LOW)  : digitalWrite(Microstep.pinSleep, HIGH);
    	Serial.println();
    	Serial.println("Microstep motor Wakeup");
    }

}



/******************************************************************************************
 * 
 *****************************************************************************************/
void motorInit() {

    switch (MotorDriver) {
        case NOT_DEFINED:
        	Serial.println();
        	Serial.println("ABORT - no motor driver is activated");
            return;
        case DRIVE_DARLINGTON:
        case DRIVE_HBRIDGE:
            motorInitStepper();
            return;
        case DRIVE_MICROSTEP:
            motorInitMicrostep();
            return;
        case DRIVE_SERVO:
            motorInitServo();
            return;
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void motorInitStepper() {

    if (Stepper.pinA1 == NOT_DEFINED) {
      	Serial.println();
        Serial.println("ABORT - A1 pin not defined");
        return;
    }
    if (Stepper.pinA2 == NOT_DEFINED) {
      	Serial.println();
    	Serial.println("ABORT - pin A2 pin not defined");
        return;
    }
    if (Stepper.pinB1 == NOT_DEFINED) {
      	Serial.println();
    	Serial.println("ABORT - pin B1 pin not defined");
        return;
    }
    if (Stepper.pinB2 == NOT_DEFINED) {
      	Serial.println();
    	Serial.println("ABORT - pin B2 pin not defined");
        return;
    }    
    pinMode(Stepper.pinA1, OUTPUT);
    pinMode(Stepper.pinB1, OUTPUT);
    pinMode(Stepper.pinA2, OUTPUT);
    pinMode(Stepper.pinB2, OUTPUT); 
    digitalWrite(Stepper.pinA1, LOW);
    digitalWrite(Stepper.pinB1, LOW);
    digitalWrite(Stepper.pinA2, LOW);
    digitalWrite(Stepper.pinB2, LOW);
    Stepper.idPhase = 0;
    MotorInitialized = true;
    Serial.println("Stepper Motor Initialized");
}



/******************************************************************************************
 * 
 *****************************************************************************************/
void motorInitMicrostep() {

    if (Microstep.levelReset != NOT_DEFINED) {
        if (Microstep.pinReset == NOT_DEFINED) {
          	Serial.println();
            Serial.println("ABORT - pin RESET not defined \n");
            return;
        }
        pinMode(Microstep.pinReset, OUTPUT); 
        // Default state is DISABLED
        (Microstep.levelReset == 'H') ? digitalWrite(Microstep.pinReset, LOW) : digitalWrite(Microstep.pinReset, HIGH);
    }

    if (Microstep.levelEnable != NOT_DEFINED) {
        if (Microstep.pinEnable == NOT_DEFINED) {
          	Serial.println();
            Serial.println("ABORT - pin ENABLE not defined");
            return;
        }
        pinMode(Microstep.pinEnable, OUTPUT); 
        // Default state is DISABLED
        (Microstep.levelEnable == 'H') ? digitalWrite(Microstep.pinEnable, LOW) : digitalWrite(Microstep.pinEnable, HIGH);
    }

    if (Microstep.levelSleep != NOT_DEFINED) {
        if (Microstep.pinSleep == NOT_DEFINED) {
          	Serial.println();
            Serial.println("ABORT - pin SLEEP not defined");
            return;
        }
        pinMode(Microstep.pinSleep, OUTPUT); 
        // Default state is WAKEUP
        (Microstep.levelSleep == 'H') ? digitalWrite(Microstep.pinSleep, LOW) : digitalWrite(Microstep.pinSleep, HIGH);
    }

    if (Microstep.valueS1 != NOT_DEFINED) {
        if (Microstep.pinS1==NOT_DEFINED) {
          	Serial.println();
            Serial.println("ABORT - pin MS1 not defined");
            return;
        }
        pinMode(Microstep.pinS1, OUTPUT); 
        (Microstep.valueS1 == 'H') ? digitalWrite(Microstep.pinS1, HIGH) : digitalWrite(Microstep.pinS1, LOW);
    }

    if (Microstep.valueS2 != NOT_DEFINED) {
        if (Microstep.pinS2==NOT_DEFINED) {
          	Serial.println();
            Serial.println("ABORT - pin MS2 not defined");
            return;
        }
        pinMode(Microstep.pinS2, OUTPUT); 
        (Microstep.valueS2 == 'H') ? digitalWrite(Microstep.pinS2, HIGH) : digitalWrite(Microstep.pinS2, LOW);
    }

    if (Microstep.valueS3 != NOT_DEFINED) {
        if (Microstep.pinS3==NOT_DEFINED) {
          	Serial.println();
            Serial.println("ABORT - pin MS3 not defined");
            return;
        }
        pinMode(Microstep.pinS3, OUTPUT); 
        (Microstep.valueS3 == 'H') ? digitalWrite(Microstep.pinS3, HIGH) : digitalWrite(Microstep.pinS3, LOW);
    }

    if (Microstep.pinStep == NOT_DEFINED) {
       	Serial.println();
        Serial.println("ABORT - pin STEP not defined");
        return;
    }
    pinMode(Microstep.pinStep, OUTPUT); 
    digitalWrite(Microstep.pinStep, LOW);

    if (Microstep.pinDir == NOT_DEFINED) {
       	Serial.println();
        Serial.println("ABORT - pin DIR not defined");
        return;
    }
    pinMode(Microstep.pinDir, OUTPUT); 
    setMicrostepRotation();

    Stepper.idPhase = 0;
    MotorInitialized = true;
    Serial.println("Microstep Motor Initialized");
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void motorInitServo() {

    if (Servo.pinPwm == NOT_DEFINED) {
       	Serial.println();
        Serial.println("ABORT - pin PWM not defined");
        return;
    }
    pinMode(Servo.pinPwm, OUTPUT); 
    digitalWrite(Servo.pinPwm, LOW);
    MotorInitialized = true;
    Serial.println("Servo Motor Initialized");
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void setMicrostepRotation() {

    if (MotorStep.rotationType == ROTATION_CLOCKWISE) {
        (Microstep.levelDir == 'H') ? digitalWrite(Microstep.pinDir, HIGH) : digitalWrite(Microstep.pinDir, LOW);
    } else {
        (Microstep.levelDir == 'H') ? digitalWrite(Microstep.pinDir, LOW) : digitalWrite(Microstep.pinDir, HIGH);
    }
}

/******************************************************************************************
 * 
 *****************************************************************************************/
void motorStop() {

    #if defined ESP8266
        timer1_disable();
    #elif defined ESP32
        timerStop(PTimer);                     // Stop Timer counter
    #endif
    switch (MotorDriver) {
        case DRIVE_DARLINGTON:
        case DRIVE_HBRIDGE:
            digitalWrite(Stepper.pinA1, LOW);
            digitalWrite(Stepper.pinB1, LOW);
            digitalWrite(Stepper.pinA2, LOW);
            digitalWrite(Stepper.pinB2, LOW);
            break;
        case DRIVE_MICROSTEP:
            microstepEnable(false);
            break;
        case DRIVE_SERVO:

            break;            
    }
    Serial.println("Stopped Motor");
}


/******************************************************************************************
 * StepDelay         2000 us
 * stepsRevolution   2048
 * 1 Revolution in   4096 ms   = 2000*2048
 * rpm  max         14,64      = 60000 /4096 
 * rpmStepDelay min  2092 us   = 60000000 / 14 / 2048
 * 
 * StepDelay         1000 us
 * stepsRevolution   2048
 * 1 Revolution in   2048 ms   = 1000*2048
 * rpm  max         29,29      = 60000 /2048 
 * rpmStepDelay min  1010 us   = 60000000 / 29 / 2048
 * 
 *****************************************************************************************/
void calculateRpmStepDelay() {
uint32_t usMinute = 60000000;
uint32_t usOneRevolution;

    MotorStep.rpmStepDelay = (usMinute/MotorStep.rpm)/MotorStep.stepsRevolution;
    if (MotorStep.rpmStepDelay < MotorStep.stepDelay) {
        MotorStep.rpmStepDelay = MotorStep.stepDelay;
        usOneRevolution = MotorStep.rpmStepDelay * MotorStep.stepsRevolution;
        MotorStep.rpm = usMinute / usOneRevolution;
    }
}


/******************************************************************************************
 * Rotate the motor of n degrees
 *****************************************************************************************/
void stepperMoveGradees(uint32_t  nGradees) {
uint32_t nSteps;

    if (!MotorInitialized) {
       	Serial.println();
    	Serial.println("ABORT - Motor not initialized");
        return;
    }
    nSteps = ceil((float) (MotorStep.stepsRevolution*nGradees*MotorStep.stepResolution)/360);   
    stepperMoveSteps(nSteps);
}


/******************************************************************************************
 * Rotate the motor of n revolutions
 *****************************************************************************************/
void stepperMoveRevolutions(uint32_t  nRevolutions) {
uint32_t nSteps;

    if (!MotorInitialized) {
       	Serial.println();
    	Serial.println("ABORT - Motor not initialized");
        return;
    }
    nSteps = nRevolutions * MotorStep.stepsRevolution * MotorStep.stepResolution;
    stepperMoveSteps(nSteps);
}


/******************************************************************************************
 * Rotate the motor of n steps
 *****************************************************************************************/
void stepperMoveSteps(uint32_t  nSteps) {
uint8_t pinState;
uint32_t timeout;


    if (!MotorInitialized) {
       	Serial.println();
    	Serial.println("ABORT - Motor not initialized");
        return;
    }

    sprintf(PrintBuffer, "\nRun steps: %u", nSteps);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "Resolution 1:%u", MotorStep.stepResolution);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "MotorStep.rpm Step Delay: %u us", MotorStep.rpmStepDelay);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "Step Delay Resolution: %u us", MotorStep.rpmStepDelay/MotorStep.stepResolution);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "Overhaed: %u us", Overhead);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "MotorStep.rpm: %u", MotorStep.rpm);
	Serial.println(PrintBuffer);
	Serial.print("Rotation: ");
    if (MotorStep.rotationType == ROTATION_CLOCKWISE) Serial.println("CLOCKWISE");
    else Serial.println("ANTI-CLOCKWISE");

	Serial.print("Drive Mode: ");
    if (MotorDriver == DRIVE_DARLINGTON || MotorDriver == DRIVE_HBRIDGE) {
        switch (Stepper.stepSequence) {
            case SEQUENCE_ONEPHASE:  Serial.println("ONE-PHASE");  break;
            case SEQUENCE_FULLSTEP:  Serial.println("FULL-STEP");  break;
            case SEQUENCE_HALFSTEP:  Serial.println("HALF-STEP");  break;
        }
        NumSteps = nSteps;
        timeout = (MotorStep.rpmStepDelay/MotorStep.stepResolution) - Overhead;
        MSPulseTimer = 5UL * timeout;                       // Prescal 16, 1 tick => 0.2 us
        sprintf(PrintBuffer, "Pulse timeout: %u", timeout);
	    Serial.println(PrintBuffer);
        sprintf(PrintBuffer, "MSPulseTimer: %u", MSPulseTimer);
	    Serial.println(PrintBuffer);
        MoveStartTime = millis();
        stepperMove1Step();
    } 
    
    if (MotorDriver == DRIVE_MICROSTEP) {
        Serial.println("MICROSTEP-PULSE");  
        if (Microstep.pinSleep != NOT_DEFINED) {
            pinState = digitalRead(Microstep.pinSleep); 
            if ((Microstep.levelSleep=='H' && pinState==HIGH) || (Microstep.levelSleep=='L' && pinState==LOW)) {
       	       	Serial.println();
                Serial.println("ABORT - microstep motor is slepping");
                return;
            }
        }
        NumSteps = nSteps;
        timeout = (MotorStep.rpmStepDelay/MotorStep.stepResolution) - Overhead;
        MSPulseTimer = 5UL * timeout;                       // Prescal 16, 1 tick => 0.2 us
        sprintf(PrintBuffer, "Pulse timeout: %u", timeout);
	    Serial.println(PrintBuffer);
        sprintf(PrintBuffer, "MSPulseTimer: %u", MSPulseTimer);
	    Serial.println(PrintBuffer);
        microstepEnable(true);
        MoveStartTime = millis();
        runOnePulse();        
    }
}


/******************************************************************************************
 * Rotate the motor of 1 step
 * MotorStep.rpmStepDelay     1010 us
 * MotorStep.stepResolution     16
 * timeout          63 = (1010/16) - 16
 *****************************************************************************************/
void stepperMove1Step() {
uint8_t numPhase=3;

    switch (Stepper.stepSequence) {
        case SEQUENCE_ONEPHASE:  runOnePhase();  break;
        case SEQUENCE_FULLSTEP:  runFullStep();  break;
        case SEQUENCE_HALFSTEP:  runHalfStep();  numPhase = 7;  break;
    }
    if (MotorStep.rotationType == ROTATION_CLOCKWISE) {
        if (++Stepper.idPhase > numPhase) Stepper.idPhase = 0;
    } else {
        if (--Stepper.idPhase < 0) Stepper.idPhase = numPhase;
    }

    NumSteps--;
    if (NumSteps) {
        #if defined ESP8266
            timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
            timer1_write(MSPulseTimer);                                 // schedule next pulse
        #elif defined ESP32
            timerAlarmDisable(PTimer);                                  // abilitiamo il timer      
            timerAlarmWrite(PTimer, MSPulseTimer, true);                // IRQ ogni 1 ms
            timerAlarmEnable(PTimer);                                   // abilitiamo il timer      
            timerStart(PTimer);
        #endif
        return;
    }
}


/******************************************************************************************
 * Pulse high for 5 us
 *****************************************************************************************/
void runOnePulse() {
uint32_t elapsedTime;

    digitalWrite(Microstep.pinStep, HIGH);
    delayMicroseconds(MICROSTEP_PULSE_HIGH);                            // 5us
    digitalWrite(Microstep.pinStep, LOW);
    NumSteps--;
    if (NumSteps) {
        #if defined ESP8266
            timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
            timer1_write(MSPulseTimer);                                 // schedule next pulse
        #elif defined ESP32
            timerAlarmDisable(PTimer);                                  // abilitiamo il timer      
            timerAlarmWrite(PTimer, MSPulseTimer, true);                // IRQ ogni 1 ms
            timerAlarmEnable(PTimer);                                   // abilitiamo il timer      
            timerStart(PTimer);
        #endif
        return;
    }
    // End move steps
    delayMicroseconds(MotorStep.stepDelay);
    microstepEnable(false);                     // disable driver
    elapsedTime = millis() - MoveStartTime;
  	Serial.println();
    sprintf(PrintBuffer, "ElapsedTime: %u ms",  elapsedTime);
	Serial.println(PrintBuffer);
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void runOnePhase() {

    switch (Stepper.idPhase) {
        case 0: writeStep(1,0,0,0); return;
        case 1: writeStep(0,1,0,0); return;
        case 2: writeStep(0,0,1,0); return;
        case 3: writeStep(0,0,0,1); return;
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void runFullStep() {

    switch (Stepper.idPhase) {
        case 0: writeStep(1,0,0,1); return;
        case 1: writeStep(1,1,0,0); return;
        case 2: writeStep(0,1,1,0); return;
        case 3: writeStep(0,0,1,1); return;
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void runHalfStep() {

    switch (Stepper.idPhase) {
        case 0: writeStep(1,0,0,1); return;
        case 1: writeStep(1,0,0,0); return;
        case 2: writeStep(1,1,0,0); return;
        case 3: writeStep(0,1,0,0); return;
        case 4: writeStep(0,1,1,0); return;
        case 5: writeStep(0,0,1,0); return;
        case 6: writeStep(0,0,1,1); return;
        case 7: writeStep(0,0,0,1); return;
    }
}



/******************************************************************************************
 * 
 *****************************************************************************************/
void writeStep(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {

    digitalWrite(Stepper.pinA1, p1);
    digitalWrite(Stepper.pinB1, p2);
    digitalWrite(Stepper.pinA2, p3);
    digitalWrite(Stepper.pinB2, p4);
}


/******************************************************************************************
 * Rotate the motor of n degrees
 *****************************************************************************************/
void servoGotoGradees(uint8_t nGradees) {
uint32_t ticksPeriod;

    if (!MotorInitialized) {
       	Serial.println();
    	Serial.println("ABORT - Motor not initialized");
        return;
    }

    Servo.gradees = nGradees;
    Servo.pulseTime = map(nGradees, 0, 180, Servo.pulseMin, Servo.pulseMax);    // Convert Gradees in Time
    ticksPeriod = 5UL * Servo.pulsePeriod;                                      // Convert Time in Ticks
    Servo.ticksPulseHigh = 5UL * Servo.pulseTime;	                            // Convert Time in Ticks
    Servo.ticksPulseLow  = ticksPeriod - Servo.ticksPulseHigh;

    sprintf(PrintBuffer, "Gradees: %u", Servo.gradees);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "PulseTime: %u", Servo.pulseTime);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "TicksHigh: %u", Servo.ticksPulseHigh);
	Serial.println(PrintBuffer);

    PulseHigh = true;
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);                               // 5 Ticks ogni 1 us
    timer1_write(Servo.ticksPulseHigh);						                    // Start Timer	    

}


/******************************************************************************************
 * Rotate the motor of n degrees
 *****************************************************************************************/
void servoMoveGradees(uint8_t nGradees, uint8_t direction) {
int16_t gradees;
int8_t goGradees;

    gradees = Servo.gradees;
    (direction == MOVE_FORWARD) ? gradees += nGradees : gradees -= nGradees;
    goGradees = constrain(gradees, 0, 180); 
    servoGotoGradees(goGradees);
}



/******************************************************************************************
 * 
 *****************************************************************************************/
void menuInit() {

	MenuRunnig = true;
	MenuCurrent = MENU_ROOT;
	MenuState = STATE_DISPLAY_MENU;
	MenuChoice = 0;
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void displayMenuHeader(const char* title) {

	Serial.println();
    Serial.println(LineSeparator);
    Serial.print("   ");
    Serial.println(title);
    Serial.println(LineSeparator);
}


/******************************************************************************************
 * @brief   Show tail menu text
 * @param   none
 * @returns none
 *****************************************************************************************/
void displayMenuTail() {

	Serial.println("Q) Quit");
    Serial.println();
    Serial.print("Choice: ");
	while (Serial.available() > 0) Serial.read();		// Svuota eventuale coda di char
    MenuState = STATE_WAIT_CHOICE;
}


/******************************************************************************************
 * @brief   Configuration loop, check and show actual menu on MenuCurrent id
 * @param   none
 * @returns none
 *****************************************************************************************/
void menuLoop() {

	if (!MenuRunnig) return;
	switch (MenuCurrent) {
		case MENU_ROOT:                menuRoot();     	 		   		 return;
        case MENU_CONFIGURE:           menuConfigure();  		   		 return;
        case MENU_HBRIDGE:             menuStepper("H BRIDGE drive");    return;
        case MENU_DARLINGTON:          menuStepper("DARLINGTON drive");  return;
        case MENU_MICROSTEP:           menuMicrostep();                  return;
        case MENU_SERVO:               menuServo();                      return;
        case MENU_MS_PIN_DEFINE:       menuMPinDefine();                 return;
        case MENU_MS_PIN_VALUE:        menuMPinValue();                  return;
        case MENU_MS_STEP_RESOLUTION:  menuMStepResoultion();            return;
        case MENU_RUN_MOVE:            menuRunMotor();         		   	 return;
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void menuRoot() {
uint8_t ret;
char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("PRIMARY MENU");
            Serial.println("C) Configure motor");
            Serial.println("D) Default configuration");
            Serial.println("L) Load    configuration");
            Serial.println("R) Reset   configuration");
            Serial.println("S) Save    configuration");
            Serial.println("M) Move motor");
            Serial.println("P) show Pin wiring");
            Serial.println("V) View configuration");
            Serial.println("A) About");
		    displayMenuTail();
            return;

	    case STATE_DISPLAY_NONE:
		    MenuState = STATE_DISPLAY_MENU;
            return;

	    case STATE_WAIT_CHOICE:
            ret = getChoice("CDLRSMPVAQ", &c);
	    	if (ret == RETURN_NONE) return;						// None or incomplete user input
		    MenuChoice = c;										// Save user choice
       	    if (c == 'Q') {                                     // Exit only after setting NodeId
			    menuInit();
			    return;
		    }
            MenuState = STATE_RUN_CHOICE;
            return;
	
	    case STATE_RUN_CHOICE:
		    MenuState = STATE_DISPLAY_MENU;						// Set default new state
            switch (MenuChoice) {
                case 'C':  MenuCurrent = MENU_CONFIGURE;  break;
                case 'M':  MenuCurrent = MENU_RUN_MOVE;   break;
                case 'L':  configLoad();        		  break;
                case 'D':  configDefault();        		  break;
                case 'R':  configReset();        		  break;
                case 'S':  configSave();	       		  break;
                case 'A':  showAbout();	         		  break;
                case 'P':  showPinWiring(); 		      break;
                case 'V':  configView(MotorDriver);		  break;
            }
            return;
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void menuConfigure() {
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("MOTOR DRIVER");
            Serial.println("1) configure DARLINGTON  (ULN2003 like)");
            Serial.println("2) configure H-BRIDGE    (L293D   like)");
            Serial.println("3) configure MICROSTEP   (A4988   like)");
            Serial.println("4) configure SERVO       (SG90    like)");
            Serial.println("5) active DARLINGTON");
            Serial.println("6) active H-BRIDGE");
            Serial.println("7) active MICROSTEP");
            Serial.println("8) active SERVO");
            Serial.println("P) show Pin wiring");
            Serial.println("V) View configuration");
		    displayMenuTail();
            return;

    	case STATE_WAIT_CHOICE:
            ret = getChoice("12345678PVQ", &c);
		    if (ret == RETURN_NONE) return; 						// None or incomplete user input
		    MenuChoice = c;											// Save user choice
       	    if (c == 'Q') {                                     	// Exit only after setting NodeId
    			MenuCurrent = MENU_ROOT;
	    		MenuState = STATE_DISPLAY_MENU;
		    	return;
	    	}
            MenuState = STATE_RUN_CHOICE;
            return;

	    case STATE_RUN_CHOICE:
		    MenuState = STATE_DISPLAY_MENU;						    // Set default new state
            switch (MenuChoice) {
                case '1':  
                    MenuCurrent = MENU_DARLINGTON;   
                    return;
                case '2':  
                    MenuCurrent = MENU_HBRIDGE;      
                    return;
                case '3':  
                    MenuCurrent = MENU_MICROSTEP;    
                    return;
                case '4':  
                    MenuCurrent = MENU_SERVO;    
                    return;
                case '5':  
                    MotorDriver = DRIVE_DARLINGTON;  
                    Serial.println("Driver DARLINGTON activated");		
                    return;
                case '6':  
                    MotorDriver = DRIVE_HBRIDGE;     
                    Serial.println("Driver HBRIDGE activated");		
                    return;
                case '7':  
                    MotorDriver = DRIVE_MICROSTEP;   
                    Serial.println("Driver MICROSTEP activated");		
                    return;
                case '8':  
                    MotorDriver = DRIVE_SERVO;   
                    Serial.println("Driver SERVO activated");		
                    return;
                case 'P':  
                    showPinWiring();				    
                    return;
                case 'V':  
                    configView(MotorDriver);                    
                    return;
            }
            return;
	}
}

/******************************************************************************************
 * 
 *****************************************************************************************/
void menuStepper(const char* title) {
uint32_t value;
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader(title);
            Serial.println("1) set A1 pin");
            Serial.println("2) set B1 pin");
            Serial.println("3) set A2 pin");
            Serial.println("4) set B2 pin");
            Serial.println("A) set Anti-clockwise rotation");
            Serial.println("C) set Clockwise rotation");
            Serial.println("F) set Full step sequence");
            Serial.println("H) set Half step sequence");
            Serial.println("O) set One phase sequence");
            Serial.println("D) set step Delay");
            Serial.println("R) set Rpm");
            Serial.println("S) set Steps revolution");
            Serial.println("P) show Pin wiring");
            Serial.println("V) View configuration");
		    displayMenuTail();
            return;

    	case STATE_WAIT_CHOICE:
            ret = getChoice("1234ACFHODRSPVQ", &c);
		    if (ret == RETURN_NONE) return; 						// None or incomplete user input
		    MenuChoice = c;											// Save user choice
       	    if (c == 'Q') {                                     	// Exit only after setting NodeId
    			MenuCurrent = MENU_CONFIGURE;
	    		MenuState = STATE_DISPLAY_MENU;
		    	return;
	    	}
            MenuState = STATE_RUN_CHOICE;
            return;

	    case STATE_RUN_CHOICE:
		    MenuState = STATE_DISPLAY_MENU;						    // Set default new state
            switch (MenuChoice) {
                case '1':
        			Serial.print("Enter A1 pin [0, 50]: ");
                    ValueMin = 0;
                    ValueMax = 50;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '2':
    	    		Serial.print("Enter B1 pin [0, 50]: ");
                    ValueMin = 0;
                    ValueMax = 50;
    	  			MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '3':
    		    	Serial.print("Enter A2 pin [0, 50]: ");
                    ValueMin = 0;
                    ValueMax = 50;
	      			MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '4':
        			Serial.print("Enter B2 pin [0, 50]: ");
                    ValueMin = 0;
                    ValueMax = 50;
	  		    	MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'A':
        			Serial.println("Set rotation to ANTI-CLOCKWISE");
                    MotorStep.rotationType = ROTATION_COUNTERCLOCKWISE;
                    return;
                case 'C':
        			Serial.println("Set rotation to CLOCKWISE");
                    MotorStep.rotationType = ROTATION_CLOCKWISE;
                    return;
                case 'F':
    	    		Serial.println("Set drive sequence to FULL-STEP");
                    MotorStep.stepResolution = STEP_RESOLUTION_FULL;                    
                    Stepper.stepSequence = SEQUENCE_FULLSTEP;
                    return;
                case 'H':
    	    		Serial.println("Set drive sequence to HALF-STEP");
                    MotorStep.stepResolution = STEP_RESOLUTION_HALF;                    
                    Stepper.stepSequence = SEQUENCE_HALFSTEP;
                    return;
                case 'O':
        			Serial.println("Set drive sequence to ONE-PHASE");
                    MotorStep.stepResolution = STEP_RESOLUTION_FULL;                    
                    Stepper.stepSequence = SEQUENCE_ONEPHASE;
                    return;
                case 'D':
        			Serial.print("Enter Step Delay us [100, 100000]: ");
                    ValueMin = 100;
                    ValueMax = 100000;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'R':
        			Serial.print("Enter MotorStep.rpm [1, 1000]: ");
                    ValueMin = 1;
                    ValueMax = 1000;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'S':
    	    		Serial.print("Set Revolution Steps [12, 8192]: ");
                    ValueMin = 12;
                    ValueMax = 8192;
    	  			MenuState = STATE_WAIT_USER_INPUT;
                    break;
                case 'P':
                    showPinWiring();
                    return;
                case 'V':
                    configViewStepper();
                    return;
            }
            return;

	    case STATE_WAIT_USER_INPUT:
            ret = getValueUnsigned(&value, ValueMin, ValueMax);
            if (ret == RETURN_NONE) return;						// None user input
            MenuState = STATE_DISPLAY_MENU;            
            if (ret == RETURN_ESC) return;                      // Abort user input
            // ret is RETUR_VALUE
            MenuState = STATE_DISPLAY_MENU;
            switch (MenuChoice) {
    	        case '1':  Stepper.pinA1 =  (uint8_t) value;                                      return;
	            case '2':  Stepper.pinB1 =  (uint8_t) value;                                      return;
        	    case '3':  Stepper.pinA2 =  (uint8_t) value;                                      return;
    	        case '4':  Stepper.pinB2 =  (uint8_t) value;                                      return;
	            case 'R':  MotorStep.rpm   = (uint16_t) value;            calculateRpmStepDelay();  return;
	            case 'D':  MotorStep.stepDelay = value;                   calculateRpmStepDelay();  return;
	            case 'S':  MotorStep.stepsRevolution = (uint16_t) value;  calculateRpmStepDelay();  return;
            }
            return;
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void menuMicrostep() {
uint32_t value;
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("MICROSTEP drive");
            Serial.println("1) define pin");
            Serial.println("2) set pin value");
            Serial.println("3) set step resolution");
            Serial.println("A) set Anti-clockwise rotation");
            Serial.println("C) set Clockwise rotation");
            Serial.println("D) set step Delay");
            Serial.println("E) rEset drive");
            Serial.println("U) Un-reset drive");
            Serial.println("R) set Rpm");
            Serial.println("S) set Steps revolution");
            Serial.println("P) show Pin wiring");
            Serial.println("V) View configuration");
		    displayMenuTail();
            return;

    	case STATE_WAIT_CHOICE:
            ret = getChoice("123ACDERSPVQ", &c);
		    if (ret == RETURN_NONE) return; 						// None or incomplete user input
		    MenuChoice = c;											// Save user choice
       	    if (c == 'Q') {                                     	// Exit only after setting NodeId
    			MenuCurrent = MENU_CONFIGURE;
	    		MenuState = STATE_DISPLAY_MENU;
		    	return;
	    	}
            MenuState = STATE_RUN_CHOICE;
            return;

	    case STATE_RUN_CHOICE:
		    MenuState = STATE_DISPLAY_MENU;						    // Set default new state
            switch (MenuChoice) {
                case '1':
                    MenuCurrent = MENU_MS_PIN_DEFINE;
                    return;
                case '2':
                    MenuCurrent = MENU_MS_PIN_VALUE;
                    return;
                case '3':
                    MenuCurrent = MENU_MS_STEP_RESOLUTION;
                    return;
                case 'A':
        			Serial.println("Set rotation to ANTI-CLOCKWISE");
                    MotorStep.rotationType = ROTATION_COUNTERCLOCKWISE;
                    return;
                case 'C':
        			Serial.println("Set rotation to CLOCKWISE");
                    MotorStep.rotationType = ROTATION_CLOCKWISE;
                    return;
                case 'D':
        			Serial.print("Enter Step Delay us [100, 100000]: ");
                    ValueMin = 100;
                    ValueMax = 100000;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'E':
                    (Microstep.levelReset == 'H') ? digitalWrite(Microstep.pinReset, HIGH) : digitalWrite(Microstep.pinReset, LOW);
                    return;
                case 'U':
                    (Microstep.levelReset == 'H') ? digitalWrite(Microstep.pinReset, LOW)  : digitalWrite(Microstep.pinReset, HIGH);
                    return;
                case 'R':
        			Serial.print("Enter MotorStep.rpm [1, 1000]: ");
                    ValueMin = 1;
                    ValueMax = 1000;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'S':
    	    		Serial.print("Set Revolution Steps [12, 8192]: ");
                    ValueMin = 12;
                    ValueMax = 8192;
    	  			MenuState = STATE_WAIT_USER_INPUT;
                    break;
                case 'P':
                    showPinWiring();
                    return;
                case 'V':
                    configViewMicrostep();
                    return;
            }
            return;
    
        case STATE_WAIT_USER_INPUT:
            ret = getValueUnsigned(&value, ValueMin, ValueMax);
            if (ret == RETURN_NONE) return;						// None user input
            MenuState = STATE_DISPLAY_MENU;            
            if (ret == RETURN_ESC) return;                      // Abort user input
            // ret is RETUR_VALUE
            MenuState = STATE_DISPLAY_MENU;
            switch (MenuChoice) {
	            case 'D':  MotorStep.stepDelay = value;                   calculateRpmStepDelay();  return;
	            case 'R':  MotorStep.rpm = (uint16_t) value;              calculateRpmStepDelay();  return;
	            case 'S':  MotorStep.stepsRevolution = (uint16_t) value;  calculateRpmStepDelay();  return;
            }
            return;
    }
}



/******************************************************************************************
 * 
 *****************************************************************************************/
void menuMPinDefine() {
uint32_t value;    
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("PIN DEFINE");
            Serial.println("1) set pin MS1          (optional)");
            Serial.println("2) set pin MS2          (optional)");
            Serial.println("3) set pin MS3          (optional)");
            Serial.println("4) set pin Enable       (optional)");
            Serial.println("5) set pin Sleep        (optional)");
            Serial.println("6) set pin Reset        (optional)");
            Serial.println("7) set pin dIr          (mandatory)");
            Serial.println("8) set pin Step         (mandatory)");
            Serial.println("V) View configuration");
		    displayMenuTail();
            return;

	    case STATE_DISPLAY_NONE:
		    MenuState = STATE_DISPLAY_MENU;
            return;

	    case STATE_WAIT_CHOICE:
            ret = getChoice("12345678VQ", &c);
	    	if (ret == RETURN_NONE) return;						// None or incomplete user input
		    MenuChoice = c;										// Save user choice
       	    if (c == 'Q') {                                     // Exit only after setting NodeId
    			MenuCurrent = MENU_MICROSTEP;
	    		MenuState = STATE_DISPLAY_MENU;
			    return;
		    }
            MenuState = STATE_RUN_CHOICE;
            return;
	
	    case STATE_RUN_CHOICE:
            if (MenuChoice == 'V') {
                configViewMicrostep();
    		    MenuState = STATE_DISPLAY_MENU;						   
                return;
            }
    		Serial.print("Enter ");
            switch (MenuChoice) {
                case '1':  Serial.print("MS1");    break;
                case '2':  Serial.print("MS2");    break;
                case '3':  Serial.print("MS3");    break;
                case '4':  Serial.print("ENABLE"); break;
                case '5':  Serial.print("SLEEP");  break;
                case '6':  Serial.print("RESET");  break;
                case '7':  Serial.print("DIR");    break;
                case '8':  Serial.print("STEP");   break;
            }
  			Serial.print(" pin [0, 50]: ");
            ValueMin = 0;
            ValueMax = 50;
  			MenuState = STATE_WAIT_USER_INPUT;
            return;

        case STATE_WAIT_USER_INPUT:
            ret = getValueUnsigned(&value, ValueMin, ValueMax);
            if (ret == RETURN_NONE) return;						// None user input
            MenuState = STATE_DISPLAY_MENU;            
            if (ret == RETURN_ESC) return;                      // Abort user input
            // ret is RETUR_VALUE
            MenuState = STATE_DISPLAY_MENU;
            switch (MenuChoice) {
    	        case '1':  Microstep.pinS1     =  (uint8_t) value;  return;
    	        case '2':  Microstep.pinS2     =  (uint8_t) value;  return;
    	        case '3':  Microstep.pinS3     =  (uint8_t) value;  return;
    	        case '4':  Microstep.pinEnable =  (uint8_t) value;  return;
    	        case '5':  Microstep.pinSleep  =  (uint8_t) value;  return;
    	        case '6':  Microstep.pinReset  =  (uint8_t) value;  return;
    	        case '7':  Microstep.pinDir    =  (uint8_t) value;  return;
    	        case '8':  Microstep.pinStep   =  (uint8_t) value;  return;
            }
    }
}    



/******************************************************************************************
 * 
 *****************************************************************************************/
void menuMPinValue() {
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("PIN VALUE");
            Serial.println("1) set MS1 value            (default NC)");
            Serial.println("2) set MS2 value            (default NC)");
            Serial.println("3) set MS3 value            (default NC)");
            Serial.println("4) set ENABLE active level  (default NC)");
            Serial.println("5) set SLEEP  active level  (default NC)");
            Serial.println("6) set RESET  active level  (default NC)");
            Serial.println("7) set DIR clockwise level  (default HIGH)");
            Serial.println("V) View configuration");
		    displayMenuTail();
            return;

	    case STATE_DISPLAY_NONE:
		    MenuState = STATE_DISPLAY_MENU;
            return;

	    case STATE_WAIT_CHOICE:
            ret = getChoice("1234567VQ", &c);
	    	if (ret == RETURN_NONE) return;						// None or incomplete user input
		    MenuChoice = c;										// Save user choice
       	    if (c == 'Q') {                                     // Exit only after setting NodeId
    			MenuCurrent = MENU_MICROSTEP;
	    		MenuState = STATE_DISPLAY_MENU;
			    return;
		    }
            MenuState = STATE_RUN_CHOICE;
            return;
	
	    case STATE_RUN_CHOICE:
		    MenuState = STATE_DISPLAY_MENU;						    // Set default new state
            switch (MenuChoice) {
                case '1':
        			Serial.print("Enter MS1 value [High, Low, Not connected]: ");
                    ChoiceRange = "HLN";
	  			    MenuState = STATE_WAIT_SUBCHOICE;
                    return;
                case '2':
        			Serial.print("Enter MS2 value [High, Low, Not connected]: ");
                    ChoiceRange = "HLN";
	  			    MenuState = STATE_WAIT_SUBCHOICE;
                    return;
                case '3':
        			Serial.print("Enter MS3 value [High, Low, Not connected]: ");
                    ChoiceRange = "HLN";
	  			    MenuState = STATE_WAIT_SUBCHOICE;
                    return;
                case '4':
        			Serial.print("Enter ENABLE activation level [High, Low, Not connected]: ");
                    ChoiceRange = "HLN";
	  			    MenuState = STATE_WAIT_SUBCHOICE;
                    return;
                case '5':
        			Serial.print("Enter SLEEP activation level [High, Low, Not connected]: ");
                    ChoiceRange = "HLN";
	  			    MenuState = STATE_WAIT_SUBCHOICE;
                    return;
                case '6':
        			Serial.print("Enter RESET activation level [High, Low, Not connected]: ");
                    ChoiceRange = "HLN";
	  			    MenuState = STATE_WAIT_SUBCHOICE;
                    return;
                case '7':
        			Serial.print("Enter DIR value [High, Low]: ");
                    ChoiceRange = "HL";
	  			    MenuState = STATE_WAIT_SUBCHOICE;
                    return;
                case 'V':
                    configViewMicrostep();
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
            }
            return;

	    case STATE_WAIT_SUBCHOICE:
            ret = getChoice(ChoiceRange, &c);
		    if (ret == RETURN_NONE) return; 						// None or incomplete user input
            MenuState = STATE_DISPLAY_MENU;            
            switch (MenuChoice) {
    	        case '1':  
                    if (Microstep.pinS1 == NOT_DEFINED && (c=='H' || c=='L')) {
            			Serial.println();
            			Serial.println("ABORT - Pin MS1 not defined");
                        return;
                    }
                    Microstep.valueS1 = (uint8_t) c;
          			Serial.print("Set Pin MS1 to ");
                    Serial.println(c);
                    if (MotorInitialized) {
                        (c == 'H') ? digitalWrite(Microstep.pinS1, HIGH) : digitalWrite(Microstep.pinS1, LOW);
                        Serial.print("Set Pin MS1 to ");
                        uint8_t pinState = digitalRead(Microstep.pinS1); 
                        Serial.println(pinState);
                    }                                
                    return;
    	        case '2':  
                    if (Microstep.pinS2 == NOT_DEFINED && (c=='H' || c=='L')) {
            			Serial.println();
            			Serial.println("ABORT - Pin MS2 not defined");
                        return;
                    }
                    Microstep.valueS2 = (uint8_t) c;
          			Serial.print("Set Pin MS2 to ");
                    Serial.println(c);
                    if (MotorInitialized) {
                        (c == 'H') ? digitalWrite(Microstep.pinS2, HIGH) : digitalWrite(Microstep.pinS2, LOW);
                    }                                
                    return;
    	        case '3':  
                    if (Microstep.pinS3 == NOT_DEFINED && (c=='H' || c=='L')) {
            			Serial.println();
            			Serial.println("ABORT - Pin MS3 not defined");
                        return;
                    }
                    Microstep.valueS3 = (uint8_t) c;
          			Serial.print("Set Pin MS3 to ");
                    Serial.println(c);
                    if (MotorInitialized) {
                        (c == 'H') ? digitalWrite(Microstep.pinS3, HIGH) : digitalWrite(Microstep.pinS3, LOW);
                    }                                
                    return;
    	        case '4':  
                    if (Microstep.pinEnable == NOT_DEFINED && (c=='H' || c=='L')) {
            			Serial.println();
            			Serial.println("ABORT - Pin ENABLE not defined");
                        return;
                    }
                    Microstep.levelEnable = (uint8_t) c;
                    return;
    	        case '5':  
                    if (Microstep.pinSleep == NOT_DEFINED && (c=='H' || c=='L')) {
            			Serial.println();
            			Serial.println("ABORT - Pin SLEEP not defined");
                        return;
                    }
                    Microstep.levelSleep = (uint8_t) c;
                    return;
    	        case '6':  
                    if (Microstep.pinReset == NOT_DEFINED && (c=='H' || c=='L')) {
            			Serial.println();
            			Serial.println("ABORT - Pin RESET not defined");
                        return;
                    }
                    Microstep.levelReset = (uint8_t) c;
                    return;
    	        case '7':  
                    if (Microstep.pinDir == NOT_DEFINED && (c=='H' || c=='L')) {
            			Serial.println();
            			Serial.println("ABORT - Pin DIR not defined");
                        return;
                    }
                    Microstep.levelDir = (uint8_t) c;                          
            }
            return;

     }
}    



/******************************************************************************************
 * 
 *****************************************************************************************/
void menuMStepResoultion() {
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("STEP RESOLUTION");
            Serial.println("F) set to Full");
            Serial.println("H) set to Half");
            Serial.println("U) set to qUarter");
            Serial.println("E) set to Eighth");
            Serial.println("S) set to Sixteenth");
            Serial.println("T) set to Thirty-second");         
            Serial.println("V) View configuration");
		    displayMenuTail();
            return;

	    case STATE_DISPLAY_NONE:
		    MenuState = STATE_DISPLAY_MENU;
            return;

	    case STATE_WAIT_CHOICE:
            ret = getChoice("FHUESTVQ", &c);
	    	if (ret == RETURN_NONE) return;						// None or incomplete user input
		    MenuChoice = c;										// Save user choice
       	    if (c == 'Q') {                                     // Exit only after setting NodeId
    			MenuCurrent = MENU_MICROSTEP;
	    		MenuState = STATE_DISPLAY_MENU;
			    return;
		    }
            MenuState = STATE_RUN_CHOICE;
            return;

	    case STATE_RUN_CHOICE:
		    MenuState = STATE_DISPLAY_MENU;						    // Set default new state
            if (MenuChoice == 'V') {
                configViewMicrostep();
                return;
            }

  			Serial.print("Set step resolution to ");
            switch (MenuChoice) {
                case 'F':
          			Serial.println("FULL");
                    MotorStep.stepResolution = STEP_RESOLUTION_FULL;
                    return;
                case 'H':
        			Serial.println("HALF");
                    MotorStep.stepResolution = STEP_RESOLUTION_HALF;
                    return;
                case 'U':
        			Serial.println("QUARTER");
                    MotorStep.stepResolution = STEP_RESOLUTION_QUARTER;
                    return;
                case 'E':
        			Serial.println("EIGHTH");
                    MotorStep.stepResolution = STEP_RESOLUTION_EIGHTH;
                    return;
                case 'S':
        			Serial.println("SIXTEENTH");
                    MotorStep.stepResolution = STEP_RESOLUTION_SIXTEENTH;
                    return;
                case 'T':
        			Serial.println("THIRTY-SECOND");
                    MotorStep.stepResolution = STEP_RESOLUTION_THIRTY2;
                    return;
            }
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void menuServo() {
uint32_t value;
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("SERVO drive");
            Serial.println("1) define PWM pin");
            Serial.println("2) pulse Period time");
            Serial.println("3) pulse Min time");
            Serial.println("4) pulse Max time");
            Serial.println("V) View configuration");
		    displayMenuTail();
            return;

    	case STATE_WAIT_CHOICE:
            ret = getChoice("1234VQ", &c);
		    if (ret == RETURN_NONE) return; 						// None or incomplete user input
		    MenuChoice = c;											// Save user choice
       	    if (c == 'Q') {                                     	// Exit only after setting NodeId
    			MenuCurrent = MENU_CONFIGURE;
	    		MenuState = STATE_DISPLAY_MENU;
		    	return;
	    	}
            MenuState = STATE_RUN_CHOICE;
            return;

	    case STATE_RUN_CHOICE:
		    MenuState = STATE_DISPLAY_MENU;						    // Set default new state
            switch (MenuChoice) {
                case '1':
        			Serial.print("Enter PWM pin [0, 50]: ");
                    ValueMin = 0;
                    ValueMax = 50;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '2':
        			Serial.print("Enter pulse Pediod time us [10000, 30000]: ");
                    ValueMin = 10000;
                    ValueMax = 30000;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '3':
        			Serial.print("Enter pulse min time us [300, 2900]: ");
                    ValueMin = 300;
                    ValueMax = 2900;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '4':
        			Serial.print("Enter pulse max time us [400, 2900]: ");
                    ValueMin = 400;
                    ValueMax = 2900;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'V':
                    configViewServo();
                    return;
            }
            return;
    
        case STATE_WAIT_USER_INPUT:
            ret = getValueUnsigned(&value, ValueMin, ValueMax);
            if (ret == RETURN_NONE) return;						// None user input
            MenuState = STATE_DISPLAY_MENU;            
            if (ret == RETURN_ESC) return;                      // Abort user input
            // ret is RETUR_VALUE
            MenuState = STATE_DISPLAY_MENU;
            switch (MenuChoice) {
	            case '1':  Servo.pinPwm      = (uint8_t)  value;  return;
	            case '2':  Servo.pulsePeriod = (uint16_t) value;  return;
	            case '3':  Servo.pulseMin    = (uint16_t) value;  return;
	            case '4':  Servo.pulseMax    = (uint16_t) value;  return;
            }
            return;
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void menuRunMotor() {

    switch (MotorDriver) {
        case DRIVE_DARLINGTON:
        case DRIVE_HBRIDGE:
        case DRIVE_MICROSTEP:
            menuRunStepper();
            break;
        case DRIVE_SERVO:
            menuRunServo();
            break;
        default:
           	Serial.println();
        	Serial.println("ABORT - no motor is defined");
   		    MenuState = STATE_DISPLAY_MENU;
            MenuCurrent = MENU_ROOT;
            return;
    }
}



/******************************************************************************************
 * 
 *****************************************************************************************/
void menuRunServo() {
uint32_t value;
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("SERVO MOTOR MOVE");
            Serial.println("1) goto     0 gradees");
            Serial.println("2) goto    90 gradees");
            Serial.println("3) goto   180 gradees");
            Serial.println("4) goto number gradees");
            Serial.println("5) move one grade forward");
            Serial.println("6) move one grade backward");
            Serial.println("7) move number gradees forward");
            Serial.println("8) move number gradees backward");
            Serial.println("I) Init motor");
            Serial.println("H) set overHead");
            Serial.println("R) Read gradees");
            Serial.println("S) Stop motor");
            Serial.println("V) View config");
		    displayMenuTail();
            break;

    	case STATE_WAIT_CHOICE:
            ret = getChoice("12345678IHRSVQ", &c);
		    if (ret == RETURN_NONE) return; 						// None or incomplete user input
		    MenuChoice = c;											// Save user choice
       	    if (c == 'Q') {                                     	// Exit only after setting NodeId
    			MenuCurrent = MENU_ROOT;
	    		MenuState = STATE_DISPLAY_MENU;
		    	return;
	    	}
            MenuState = STATE_RUN_CHOICE;
            break;

	    case STATE_RUN_CHOICE:
		    MenuState = STATE_DISPLAY_MENU;						    // Set default new state
            switch (MenuChoice) {
                case '1':
                    servoGotoGradees(0);
                    return;
                case '2':
                    servoGotoGradees(90);
                    return;
                case '3':
                    servoGotoGradees(180);
                    return;
                case '4':
        			Serial.print("Enter Gradees goto [0, 180]: ");
                    ValueMin = 0;
                    ValueMax = 180;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '5':
                    servoMoveGradees(1, MOVE_FORWARD);    
                    return;
                case '6':
                    servoMoveGradees(1, MOVE_BACKWARD);    
                    return;
                case '7':
    	    		Serial.print("Enter Gradees Forward to move [1, 180]: ");
                    ValueMin = 1;
                    ValueMax = 100;
    	  			MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '8':
    	    		Serial.print("Enter Gradees Backward to move [1, 180]: ");
                    ValueMin = 1;
                    ValueMax = 100;
    	  			MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'I':
                    motorInit();
                    return;
                case 'H':
        			Serial.print("Enter OVERHEAD time us [0, 100]: ");
                    ValueMin = 0;
                    ValueMax = 100;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'R':
                    sprintf(PrintBuffer, "Position greeds: %u", Servo.gradees);
	                Serial.println(PrintBuffer);
                    return;
                case 'S':
                    motorStop();
                    return;
                case 'V':
                    configViewServo();
                    return;
            }
            return;

	    case STATE_WAIT_USER_INPUT:
            ret = getValueUnsigned(&value, ValueMin, ValueMax);
            if (ret == RETURN_NONE) return;						// None user input
            MenuState = STATE_DISPLAY_MENU;            
            if (ret == RETURN_ESC) return;                      // Abort user input
            // ret is RETUR_VALUE
            switch (MenuChoice) {
    	        case '4':  
                    servoGotoGradees((uint8_t)value);      
                    return;
    	        case '7':  
                    servoMoveGradees((uint8_t)value, MOVE_FORWARD);      
                    return;
    	        case '8':  
                    servoMoveGradees((uint8_t)value, MOVE_BACKWARD);      
                    return;
    	        case 'H':  
                    Overhead = (uint16_t) value;          
                    return;
            }
            return;
	}
}



/******************************************************************************************
 * 
 *****************************************************************************************/
void menuRunStepper() {
uint32_t value;
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("STEPPER MOTOR MOVE");
            Serial.println("1) move one step");
            Serial.println("2) move one grade");
            Serial.println("3) move one rotation");
            Serial.println("4) move number steps");
            Serial.println("5) move number gradees");
            Serial.println("6) move number rotations");
            Serial.println("7) set  clockwise rotation");
            Serial.println("8) set  anti-clockwise rotation");
            Serial.println("D) Disable microstep motor");
            Serial.println("E) Enable  microstep motor");
            Serial.println("L) sLeep   microstep motor");
            Serial.println("W) Wakeup  microstep motor");
            Serial.println("I) Init motor");
            Serial.println("H) set overHead");
            Serial.println("R) set Rpm");
            Serial.println("S) Stop motor");
            Serial.println("V) View config");
		    displayMenuTail();
            break;

    	case STATE_WAIT_CHOICE:
            ret = getChoice("12345678DELWIHRSVQ", &c);
		    if (ret == RETURN_NONE) return; 						// None or incomplete user input
		    MenuChoice = c;											// Save user choice
       	    if (c == 'Q') {                                     	// Exit only after setting NodeId
    			MenuCurrent = MENU_ROOT;
	    		MenuState = STATE_DISPLAY_MENU;
		    	return;
	    	}
            MenuState = STATE_RUN_CHOICE;
            break;

	    case STATE_RUN_CHOICE:
		    MenuState = STATE_DISPLAY_MENU;						    // Set default new state
            switch (MenuChoice) {
                case '1':
                    stepperMoveSteps(1);
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case '2':
                    stepperMoveGradees(1);
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case '3':
                    stepperMoveRevolutions(1);
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case '4':
        			Serial.print("Enter Steps value to move [1, 10000]: ");
                    ValueMin = 1;
                    ValueMax = 10000;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '5':
        			Serial.print("Enter Gradees value to move [1, 360]: ");
                    ValueMin = 1;
                    ValueMax = 360;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '6':
    	    		Serial.print("Enter Revolutions value to move [1, 100]: ");
                    ValueMin = 1;
                    ValueMax = 100;
    	  			MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case '7':
                    MotorStep.rotationType = ROTATION_CLOCKWISE;
                    if (MotorDriver == DRIVE_MICROSTEP) setMicrostepRotation();
           			Serial.println("Set rotation to CLOCKWISE");
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case '8':
                    MotorStep.rotationType = ROTATION_COUNTERCLOCKWISE;
                    if (MotorDriver == DRIVE_MICROSTEP) setMicrostepRotation();
           			Serial.println("Set rotation to ANTI-CLOCKWISE");
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'D':
                    microstepEnable(false);
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'E':
                    microstepEnable(true);
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'L':
                    microstepSleep(true);
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'W':
                    microstepSleep(false);
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'I':
                    motorInit();
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'H':
        			Serial.print("Enter OVERHEAD time us [0, 100]: ");
                    ValueMin = 0;
                    ValueMax = 100;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'R':
        			Serial.print("Enter MotorStep.rpm [1, 1000]: ");
                    ValueMin = 1;
                    ValueMax = 1000;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'S':
                    motorStop();
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'V':
                    configView(MotorDriver);
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
            }
            return;

	    case STATE_WAIT_USER_INPUT:
            ret = getValueUnsigned(&value, ValueMin, ValueMax);
            if (ret == RETURN_NONE) return;						// None user input
            MenuState = STATE_DISPLAY_MENU;            
            if (ret == RETURN_ESC) return;                      // Abort user input
            // ret is RETUR_VALUE
            switch (MenuChoice) {
    	        case '4':  
                    stepperMoveSteps(value);      
                    return;
    	        case '5':  
                    stepperMoveGradees(value);     
                    return;
    	        case '6':  
                    stepperMoveRevolutions(value);  
                    return;
    	        case 'H':  
                    Overhead = (uint16_t) value;          
                    return;
    	        case 'R':  
                    MotorStep.rpm = (uint16_t) value;          
                    calculateRpmStepDelay(); 
                    return;
            }
            return;
	}
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void showPinWiring() {

    Serial.println();
    Serial.println("  UNIPOLAR STEPPER MOTOR COILS - Step sequence: A1, B1, A2, B2");
    Serial.println();
    Serial.println("  A2 o----+");
    Serial.println("          |");   
    Serial.println("          @");   
    Serial.println("    A o---@");   
    Serial.println("          @");   
    Serial.println("          @");   
    Serial.println("          |");   
    Serial.println("  A1 o----+");
    Serial.println("               B1 o---@@@@@@@---o B2");
    Serial.println("                         |");
    Serial.println("                         B");
    Serial.println();
    Serial.println();
    Serial.println("  BIPOLAR STEPPER MOTOR COILS - Step sequence: A1, B1, A2, B2");
    Serial.println();
    Serial.println("  A2 o----+");
    Serial.println("          |");   
    Serial.println("          @");   
    Serial.println("          @");   
    Serial.println("          @");   
    Serial.println("          @");   
    Serial.println("          |");   
    Serial.println("  A1 o----+");
    Serial.println("               B1 o---@@@@@@@---o B2");
    Serial.println();
    Serial.println();
    Serial.println("Test performed on 28BYJ-48 Unipolar Stepper Motor with ULN2003 and WemosMini ESP8266 boards");
    Serial.println("In Full-Step mode:");
    Serial.println(" - steps for revolution: 2048 ");
    Serial.println(" - step delay: 2000 us");
    Serial.println("In Half-Step mode:");
    Serial.println(" - steps for revolution: 4096 ");
    Serial.println(" - step delay: 1000 us");
    Serial.println();
    Serial.println(" MCU    WEMOS   ULN2003   28BYJ-48    COILS");
    Serial.println("-----------------------------------------------");
    Serial.println("gpio12   D6      IN2       pink        A2");
    Serial.println("gpio16   D0      IN4       orange      A1");
    Serial.println("gpio14   D5      IN3       yellow      B1");
	Serial.println("gpio13   D7      IN1       blue        B2");   
    Serial.println();
    Serial.println();
    Serial.println("Test performed on 28BYJ-48 Unipolar Stepper Motor with L293D and WemosMini ESP8266 boards");
    Serial.println("Vcc 28BYJ-48 NOT connected");
    Serial.println("In Full-Step mode:");
    Serial.println(" - steps for revolution: 2048 ");
    Serial.println(" - step delay: 2000 us");
    Serial.println("In Half-Step mode:");
    Serial.println(" - steps for revolution: 4096 ");
    Serial.println(" - step delay: 1000 us");
    Serial.println();
    Serial.println(" MCU    BOARD   L923D    BOARD   28BYJ-48   COILS");
    Serial.println("-----------------------------------------------");
    Serial.println("gpio12   D6      IN1      A-     pink        A2");
    Serial.println("gpio16   D0      IN2      A+     orange      A1");
    Serial.println("gpio14   D5      IN3      B-     yellow      B1");
    Serial.println("gpio13   D7      IN4      B+     blue        B2");   
    Serial.println();
    Serial.println();
    Serial.println("Test performed on BP485725 Bipolar Stepper Motor with L293D and WemosMini ESP8266 boards");
    Serial.println("In Full-Step mode:");
    Serial.println(" - steps for revolution: 48 ");
    Serial.println(" - step delay: 8000 us");
    Serial.println("In Half-Step mode:");
    Serial.println(" - steps for revolution: 96 ");
    Serial.println(" - step delay: 4000 us");
    Serial.println();
    Serial.println(" MCU    BOARD   L923D    BOARD   BP485725   COILS");
    Serial.println("-----------------------------------------------");
    Serial.println("gpio12   D6      IN2      A+     red         A1");
    Serial.println("gpio13   D7      IN3      B-     yellow      B1");
    Serial.println("gpio14   D5      IN1      A-     brown       A2");
    Serial.println("gpio15   D8      IN4      B+     orange      B2");
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println("  BIPOLAR MICROSTEP MOTOR COILS");
    Serial.println();
    Serial.println("  A+ o----+");
    Serial.println("          |");   
    Serial.println("          @");   
    Serial.println("          @");   
    Serial.println("          @");   
    Serial.println("          @");   
    Serial.println("          |");   
    Serial.println("  A- o----+");
    Serial.println("               B+ o---@@@@@@@---o B-");
    Serial.println();
    Serial.println();
    Serial.println("Test performed on NEMA 17HE15-1504S bipolar Step Motor with A4988 driver and WemosMini ESP8266 boards");
    Serial.println("In Full-Step mode:");
    Serial.println(" - steps for revolution: 200 ");
    Serial.println(" - step delay: 2000 us");
    Serial.println();
    Serial.println(" MCU    WEMOS   A4988    NEMA17");
    Serial.println("-----------------------------------------------");
    Serial.println("gpio15   D8     enable  ");
    Serial.println("gpio13   D7     ms1     ");
    Serial.println("gpio12   D6     ms2     ");
    Serial.println("gpio14   D5     ms3     ");
    Serial.println("gpio16   D0     reset   ");
    Serial.println("gpio05   D1     sleep   ");
    Serial.println("gpio04   D2     step    ");
    Serial.println("gpio00   D3     dir     ");
    Serial.println("                B2       A+ ");
    Serial.println("                A2       A- ");
    Serial.println("                A1       B+ ");
    Serial.println("                B1       B- ");
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void showAbout() {

    Serial.println("\n");
    Serial.println(FIRMWARE_NAME);
    sprintf(PrintBuffer, "%s - %s", FIRMWARE_VERSION, FIRMWARE_DATE);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%s - %s", FIRMWARE_AUTHOR, FIRMWARE_EMAIL);
	Serial.println(PrintBuffer);
}

/******************************************************************************************
 * 
 *****************************************************************************************/
void configView(uint8_t mDriver) {

    switch (mDriver) {
        case DRIVE_DARLINGTON:
        case DRIVE_HBRIDGE:
            configViewStepper();
            return;
        case DRIVE_MICROSTEP:
            configViewMicrostep();
            return;
        case DRIVE_SERVO:
            configViewServo();
            return;
        case NOT_DEFINED:
           	Serial.println();
        	Serial.println("ABORT - no motor is defined");
            return;
    }
}

/******************************************************************************************
 * 
 *****************************************************************************************/
void configViewStepper() {
char pA1[] = "NC ";
char pA2[] = "NC ";
char pB1[] = "NC ";
char pB2[] = "NC ";


	Serial.println();
    Serial.println(LineSeparator);
	Serial.println("STEPPER CONFIGURATION");
    Serial.println(LineSeparator);

	if (Stepper.pinA1 != NOT_DEFINED) itoa(Stepper.pinA1, pA1, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin A1",  pA1);
	Serial.println(PrintBuffer);

	if (Stepper.pinB1 != NOT_DEFINED) itoa(Stepper.pinB1, pB1, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin B1",  pB1);
	Serial.println(PrintBuffer);

	if (Stepper.pinA2 != NOT_DEFINED) itoa(Stepper.pinA2, pA2, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin A2",  pA2);
	Serial.println(PrintBuffer);

	if (Stepper.pinB2 != NOT_DEFINED) itoa(Stepper.pinB2, pB2, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin B2",  pB2);
	Serial.println(PrintBuffer);

    sprintf(PrintBuffer, "%20s: %s", "Rotation", (MotorStep.rotationType == ROTATION_CLOCKWISE) ? "CLOCKWISE" : "ANTI-CLOCKWISE");
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u", "Steps Revolution",  MotorStep.stepsRevolution);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u us", "Step Delay",  MotorStep.stepDelay);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u", "Rpm",  MotorStep.rpm);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u us", "Rpm Step Delay",  MotorStep.rpmStepDelay/MotorStep.stepResolution);
	Serial.println(PrintBuffer);

    switch (Stepper.stepSequence) {
        case SEQUENCE_ONEPHASE:  sprintf(PrintBuffer, "%20s: %s", "Step sequence", "ONE PHASE");  break;
        case SEQUENCE_FULLSTEP:  sprintf(PrintBuffer, "%20s: %s", "Step sequence", "FULL STEP");  break;
        case SEQUENCE_HALFSTEP:  sprintf(PrintBuffer, "%20s: %s", "Step sequence", "HALF STEP");  break;
    }
	Serial.println(PrintBuffer);
}




/******************************************************************************************
 * 
 *****************************************************************************************/
void configViewMicrostep() {
char pEnable[] = "NC ";
char pSleep[]  = "NC ";
char pReset[]  = "NC ";
char pMS1[]    = "NC ";
char pMS2[]    = "NC ";
char pMS3[]    = "NC ";
char pStep[]   = "NC ";
char pDir[]    = "NC ";
const char* sLevel;
const char* sState;
uint8_t pinState;

	Serial.println();
    Serial.println(LineSeparator);
	Serial.println("MICROSTEP CONFIGURATION");
    Serial.println(LineSeparator);

    // Check ENABLE
	if (Microstep.pinEnable != NOT_DEFINED) itoa(Microstep.pinEnable, pEnable, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin ENABLE",  pEnable);
	Serial.println(PrintBuffer);
	     if (Microstep.levelEnable == NOT_DEFINED) sLevel = "NC";
	else if (Microstep.levelEnable == 'H') sLevel = "High";
	else if (Microstep.levelEnable == 'L') sLevel = "Low";
    sprintf(PrintBuffer, "%20s: %s", "Level ENABLE",  sLevel);
	Serial.println(PrintBuffer);
    if (Microstep.pinEnable!=NOT_DEFINED && Microstep.levelEnable!=NOT_DEFINED) {
        pinState = digitalRead(Microstep.pinEnable); 
        sState = (pinState==HIGH) ? "High" : "Low";   
    } else {
        sState = "NC";
    }
    sprintf(PrintBuffer, "%20s: %s", "State ENABLE",  sState);
	Serial.println(PrintBuffer);

    // Check SLEEP
	if (Microstep.pinSleep != NOT_DEFINED) itoa(Microstep.pinSleep, pSleep, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin SLEEP", pSleep);
	Serial.println(PrintBuffer);
	     if (Microstep.levelSleep == NOT_DEFINED) sLevel = "NC";
	else if (Microstep.levelSleep == 'H') sLevel = "High";
	else if (Microstep.levelSleep == 'L') sLevel = "Low";
    sprintf(PrintBuffer, "%20s: %s", "Level SLEEP",  sLevel);
	Serial.println(PrintBuffer);
    if (Microstep.pinSleep!=NOT_DEFINED && Microstep.levelSleep!=NOT_DEFINED) {
        pinState = digitalRead(Microstep.pinSleep); 
        sState = (pinState==HIGH) ? "High" : "Low";   
    } else {
        sState = "NC";
    }
    sprintf(PrintBuffer, "%20s: %s", "State SLEEP",  sState);
	Serial.println(PrintBuffer);

    // Check RESET
	if (Microstep.pinReset != NOT_DEFINED) itoa(Microstep.pinReset, pReset, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin RESET", pReset);
	Serial.println(PrintBuffer);
	     if (Microstep.levelReset == NOT_DEFINED) sLevel = "NC";
	else if (Microstep.levelReset == 'H') sLevel = "High";
	else if (Microstep.levelReset == 'L') sLevel = "Low";
    sprintf(PrintBuffer, "%20s: %s", "Level RESET",  sLevel);
	Serial.println(PrintBuffer);
    if (Microstep.pinReset!=NOT_DEFINED && Microstep.levelReset!=NOT_DEFINED) {
        pinState = digitalRead(Microstep.pinReset); 
        sState = (pinState==HIGH) ? "High" : "Low";   
    } else {
        sState = "NC";
    }
    sprintf(PrintBuffer, "%20s: %s", "State RESET",  sState);
	Serial.println(PrintBuffer);

    // Check MS1
	if (Microstep.pinS1 != NOT_DEFINED) itoa(Microstep.pinS1, pMS1, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin MS1",  pMS1);
	Serial.println(PrintBuffer);
    if (Microstep.pinS1 != NOT_DEFINED) {
        pinState = digitalRead(Microstep.pinS1); 
        sState = (pinState==HIGH) ? "High" : "Low";   
    } else {
        sState = "NC";
    }
    sprintf(PrintBuffer, "%20s: %s", "State MS1",  sState);
	Serial.println(PrintBuffer);

    // Check MS2
	if (Microstep.pinS2 != NOT_DEFINED) itoa(Microstep.pinS2, pMS2, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin MS2",  pMS2);
	Serial.println(PrintBuffer);
    if (Microstep.pinS2 != NOT_DEFINED) {
        pinState = digitalRead(Microstep.pinS2); 
        sState = (pinState==HIGH) ? "High" : "Low";   
    } else {
        sState = "NC";
    }
    sprintf(PrintBuffer, "%20s: %s", "State MS2",  sState);
	Serial.println(PrintBuffer);

    // Check MS3
	if (Microstep.pinS3 != NOT_DEFINED) itoa(Microstep.pinS3, pMS3, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin MS3",  pMS3);
	Serial.println(PrintBuffer);
    if (Microstep.pinS3 != NOT_DEFINED) {
        pinState = digitalRead(Microstep.pinS3); 
        sState = (pinState==HIGH) ? "High" : "Low";   
    } else {
        sState = "NC";
    }
    sprintf(PrintBuffer, "%20s: %s", "State MS3",  sState);
	Serial.println(PrintBuffer);

    // Check STEP
	if (Microstep.pinStep != NOT_DEFINED) itoa(Microstep.pinStep, pStep, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin STEP",  pStep);
	Serial.println(PrintBuffer);
    if (Microstep.pinStep != NOT_DEFINED) {
        pinState = digitalRead(Microstep.pinStep); 
        sState = (pinState==HIGH) ? "High" : "Low";   
    } else {
        sState = "NC";
    }
    sprintf(PrintBuffer, "%20s: %s", "State STEP",  sState);
	Serial.println(PrintBuffer);

    // Check DIR
	if (Microstep.pinDir != NOT_DEFINED) itoa(Microstep.pinDir, pDir, 10);
    (Microstep.levelDir == 'H') ? sLevel = "High" : sLevel = "Low";
    sprintf(PrintBuffer, "%20s: %s", "Pin DIR",  pDir);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %s", "Clockwise Level",  sLevel);
	Serial.println(PrintBuffer);
    if (Microstep.pinDir != NOT_DEFINED) {
        pinState = digitalRead(Microstep.pinDir); 
        sState = (pinState==HIGH) ? "High" : "Low";   
    } else {
        sState = "NC";
    }
    sprintf(PrintBuffer, "%20s: %s", "State DIR",  sState);
	Serial.println(PrintBuffer);

    sprintf(PrintBuffer, "%20s: %s", "Rotation", (MotorStep.rotationType == ROTATION_CLOCKWISE) ? "CLOCKWISE" : "ANTI-CLOCKWISE");
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u", "Steps Revolution",  MotorStep.stepsRevolution);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u us", "Step Delay",  MotorStep.stepDelay);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u", "Rpm",  MotorStep.rpm);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u us", "Rpm Step Delay",  MotorStep.rpmStepDelay);
	Serial.println(PrintBuffer);

    sprintf(PrintBuffer, "%20s: ", "Step resolution");
	Serial.print(PrintBuffer);
    switch (MotorStep.stepResolution) {
        case STEP_RESOLUTION_FULL:      Serial.println("FULL");           break;
        case STEP_RESOLUTION_HALF:      Serial.println("HALF");           break;
        case STEP_RESOLUTION_QUARTER:   Serial.println("QUARTER");        break;
        case STEP_RESOLUTION_EIGHTH:    Serial.println("EIGHTH");         break;
        case STEP_RESOLUTION_SIXTEENTH: Serial.println("SIXTEENTH");      break;
        case STEP_RESOLUTION_THIRTY2:   Serial.println("THIRTY-SECOND");  break;        
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void configViewServo() {
char pPwm[] = "NC ";

	Serial.println();
    Serial.println(LineSeparator);
	Serial.println("SERVO CONFIGURATION");
    Serial.println(LineSeparator);

	if (Servo.pinPwm != NOT_DEFINED) itoa(Servo.pinPwm, pPwm, 10);
    sprintf(PrintBuffer, "%20s: %s", "Pin pwm",  pPwm);
	Serial.println(PrintBuffer);

    sprintf(PrintBuffer, "%20s: %u us", "Pulse Period", Servo.pulsePeriod);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u us", "Pulse Min time", Servo.pulseMin);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u us", "Pulse Max time", Servo.pulseMax);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u us", "Pulse Time", Servo.pulseTime);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u", "Grades", Servo.gradees);
	Serial.println(PrintBuffer);

}


/******************************************************************************************
 * 
 *****************************************************************************************/
uint8_t getChoice(const char* menu, char* value) {
char c;

	// Serial Code
	if (Serial.available() == 0) return RETURN_NONE;
    c = Serial.read();
    if (c > 90) c -= 32;
    if (strchr(menu, c) == NULL) return RETURN_NONE;
    Serial.println(c);
    *value = c;
	Serial.println();
    return RETURN_VALUE;	
}



/******************************************************************************************
 * 
 *****************************************************************************************/
uint8_t getValueUnsigned(uint32_t *value, uint32_t min, uint32_t max) {
static uint32_t tot=0;                                  		// valore totale numero digitato
static uint8_t i;                                      			// contatore caratteri digitati
uint32_t sum=0;                                  				// somma parziale numero digitato
uint8_t c;                                        				// carattere digitato

	if (Serial.available() == 0) return RETURN_NONE;
	c = Serial.read();
	if (c==27) {												// ESC
		Serial.println(" - abort");
		tot = 0;												// reset index
		i = 0;
	    return RETURN_ESC;      								// ESC, annulla digitazione
	}
    if (c==13 && i>0 && tot >= min && tot <= max) {				// CR, check end
    	*value = tot;
		tot = 0;												// reset index
		i = 0;
	    Serial.println();
	    return RETURN_VALUE;	    				
	}

	if (c >= '0' && c <= '9') {                 				// Pesa la cifra digitata
    	sum = (10 * tot) + (c-'0');             				// rispetto alle precedenti
	    if (sum <= max) {
    	    Serial.write(c);
        	i++;
            tot = sum;
		}
	}
	return RETURN_NONE;
}
