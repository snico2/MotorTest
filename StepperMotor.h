// --------------------------------------------------------------------------------------
//  Header file StepperMotor.h
// --------------------------------------------------------------------------------------

#ifndef __STEPPERMOTOR_H
#define __STEPPERMOTOR_H


// --------------------------------------------------------------------------------------
// CONSTANTS DEFINITION
// --------------------------------------------------------------------------------------

#define FIRMWARE_NAME                   "Motor Stepper Test"
#define FIRMWARE_VERSION                "v0.16"
#define FIRMWARE_DATE                   "26/05/2024"
#define FIRMWARE_AUTHOR                 "nicola sellitto"
#define FIRMWARE_EMAIL                  "nicosellitto@yahoo.it"

#define SERIAL_BAUD_RATE               115200

// ------------------------------------------------------------------
// CONFIG MEMORY CONSTANT
// ------------------------------------------------------------------
#define CONFIG_ID                            0xCF
#define CONFIG_VERSION                       0xA1
#define CONFIG_SIZE                            44
#define CONFIG_SIZE_HEADER                      4
#define CONFIG_SIZE_MOTORSTEP                  16 
#define CONFIG_SIZE_STEPPER                     8  
#define CONFIG_SIZE_MICROSTEP                  16

#define CONFIG_START_HEADER                     0
#define CONFIG_START_MOTORSTEP                  4
#define CONFIG_START_STEPPER                   20 
#define CONFIG_START_MICROSTEP                 28 


#define STATE_DISPLAY_NONE              0
#define STATE_DISPLAY_MENU              1
#define STATE_RUN_CHOICE                2
#define STATE_WAIT_CHOICE               3
#define STATE_WAIT_USER_INPUT           4
#define STATE_WAIT_SUBCHOICE            5

#define ROTATION_CLOCKWISE              1
#define ROTATION_COUNTERCLOCKWISE       2

#define SEQUENCE_FULLSTEP               1
#define SEQUENCE_ONEPHASE               2
#define SEQUENCE_HALFSTEP               3

#define STEP_RESOLUTION_FULL            1
#define STEP_RESOLUTION_HALF            2
#define STEP_RESOLUTION_QUARTER         4
#define STEP_RESOLUTION_EIGHTH          8
#define STEP_RESOLUTION_SIXTEENTH      16
#define STEP_RESOLUTION_THIRTY2        32

#define DRIVE_DARLINGTON                1
#define DRIVE_HBRIDGE                   2
#define DRIVE_MICROSTEP                 3

#define RETURN_NONE                     0
#define RETURN_ESC                      1
#define RETURN_VALUE                    2

#define MENU_ROOT                       1    
#define MENU_CONFIGURE                  2
#define MENU_DARLINGTON                 3
#define MENU_HBRIDGE                    4
#define MENU_MICROSTEP                  5
#define MENU_MS_PIN_DEFINE              6
#define MENU_MS_PIN_VALUE               7
#define MENU_MS_STEP_RESOLUTION         8
#define MENU_RUN_MOVE                   9


#define SM_STATE_STOP                   0
#define SM_STATE_WAIT_TIMEOUT           1

#define NOT_DEFINED                   255

#define MICROSTEP_PULSE_HIGH            5          // time in us

// ------------------------------------------------------------------
// DEFAULT CONSTANT
// ------------------------------------------------------------------
#define DEAFULT_STEP_DELAY                   2000
#define DEAFULT_RPM                             4
#define DEAFULT_STEPS_REVOLUTION              200
#define DEAFULT_STEP_RESOLUTION              STEP_RESOLUTION_FULL
#define DEAFULT_ROTATION                     ROTATION_CLOCKWISE
#define DEAFULT_SEQUENCE                     SEQUENCE_FULLSTEP


// ------------------------------------------------------------------
// STRUCTURE DEFINITION
// ------------------------------------------------------------------


// 4 byte Size
struct sHeader {
    uint8_t id;                                                   // Configuration id fix to 0xCF
    uint8_t version;                                              // Configuration version id
    uint8_t reserved[2];
};

// 16 byte Size
struct sMotorStep {
   uint32_t stepDelay;                                                  // [100, 100000] time in us
   uint32_t rpmStepDelay;
   uint16_t rpm;                                                        // [1, 1000]   
   uint16_t stepsRevolution;                                            // [12, 8192]
    uint8_t stepResolution;                                             // Full, Half, Quarter, Eighth, Sixteenth, Thirty2
    uint8_t rotationType;                                               // Clockwise - AntiClockwise     
    uint8_t reserved[2];
};

// 8 byte Size
struct sStepper {
    uint8_t pinA1;                                                      // 
    uint8_t pinA2;                                                      // 
    uint8_t pinB1;                                                      // 
    uint8_t pinB2;                                                      // 
     int8_t stepSequence;
     int8_t idPhase;
    uint8_t reserved[2];
};

// 16 byte Size
struct sMicrostep {
    uint8_t pinEnable;                                                  // 
    uint8_t pinSleep;                                                   // 
    uint8_t pinReset;                                                   // 
    uint8_t pinStep;                                                    // 
    uint8_t pinDir;                                                     // 
    uint8_t pinS1;                                                      // 
    uint8_t pinS2;                                                      // 
    uint8_t pinS3;                                                      // 
    uint8_t levelEnable;                                                // 
    uint8_t levelSleep;                                                 // 
    uint8_t levelReset;                                                 // 
    uint8_t levelDir;                                                   // 
    uint8_t valueS1;                                                    // 
    uint8_t valueS2;                                                    // 
    uint8_t valueS3;                                                    // 
    uint8_t reserved;
};


// Motor State Machine
struct sStateMachine {
    uint32_t start;                                                     // Start Time in ms, tempo assoluto
    uint32_t timeout;                                                   // Timeout in ms, intervallo di tempo
     uint8_t state;                                                     // step for value increment
};



// --------------------------------------------------------------------------------------
// FUNCTION PROTOTYPE
// --------------------------------------------------------------------------------------

   void calculateRpmStepDelay(void);
   void displayMenuHeader(const char* title);
   void displayMenuTail(void);
uint8_t getChoice(const char* menu, char* value);
uint8_t getValueUnsigned(uint32_t *value, uint32_t min, uint32_t max);
   void configRead(void);
   void configReset(void);
   void configWrite(void);
   void initMotorStep(void);
   void initMicrostep(void);
   void initStepper(void);
   void menuConfigure(void);
   void menuInit(void);
   void menuLoop(void);
   void menuMicrostep(void);
   void menuMPinDefine(void);
   void menuMPinValue(void);
   void menuMStepResoultion(void);
   void menuRoot(void);
   void menuRunMove(void);
   void menuStepper(const char* title);
   void microstepEnable(bool enable);
   void microstepSleep(bool enable);
   void motorInit(void);
   void MotorInitStepper(void);   
   void MotorInitMicrostep(void);   
   void motorStop(void);
   void moveGradees(uint32_t nGradees);
   void moveOneStep(void); 
   void moveRevolutions(uint32_t nRotations);
   void moveSteps(uint32_t nSteps);
   void runFullStep(void);
   void runHalfStep(void);
   void runOnePhase(void);
   void runOnePulse(void);
   void showAbout(void);
   void showPinWiring(void);
   void stateMachineCheck(void);
   void stateMachineInit(void);
   void viewConfig(uint8_t mDriver);
   void viewConfigMicrostep(void);
   void viewConfigStepper(void);
   void writeStep(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4);

#endif
