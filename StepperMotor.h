// --------------------------------------------------------------------------------------
//  Header file config.h
// --------------------------------------------------------------------------------------

#ifndef __STEPPERMOTOR_H
#define __STEPPERMOTOR_H

// --------------------------------------------------------------------------------------
// Constants definition
// --------------------------------------------------------------------------------------

#define FIRMWARE_NAME                   "Motor Stepper Test"
#define FIRMWARE_VERSION                "v0.04"
#define FIRMWARE_DATE                   "10/05/2024"
#define FIRMWARE_AUTHOR                 "nicola sellitto"
#define FIRMWARE_EMAIL                  "nicosellitto@yahoo.it"

#define SERIAL_BAUD_RATE            115200

#define STATE_DISPLAY_NONE              0
#define STATE_DISPLAY_MENU              1
#define STATE_RUN_CHOICE                2
#define STATE_WAIT_CHOICE               3
#define STATE_WAIT_USER_INPUT           4

#define ROTATION_CLOCKWISE              1
#define ROTATION_COUNTERCLOCKWISE       2

#define MODE_FULLSTEP                   1
#define MODE_WAVE                       2
#define MODE_HALFSTEP                   3
#define MODE_MICROSTEPPING              4

#define RETURN_NONE                     0
#define RETURN_ESC                      1
#define RETURN_VALUE                    2

#define MENU_ROOT                       1    
#define MENU_CONFIGURE                  2
#define MENU_RUN_MOVE                   3    

#define SM_STATE_STOP                   0
#define SM_STATE_WAIT_TIMEOUT           1


// ------------------------------------------------------------------
// STRUCTURE DEFINITION
// ------------------------------------------------------------------

// Motor State Machine
struct sStateMachine {
    uint32_t start;                                                     // Start Time in ms, tempo assoluto
    uint32_t timeout;                                                   // Timeout in ms, intervallo di tempo
     uint8_t state;                                                     // step for value increment
};


// --------------------------------------------------------------------------------------
// Functions prototype
// --------------------------------------------------------------------------------------

   void calculateRpmStepDelay(void);
   void displayMenuHeader(const char* title);
   void displayMenuTail(void);
uint8_t getChoice(const char* menu, char* value);
uint8_t getValueUnsigned(uint32_t *value, uint32_t min, uint32_t max);
   void menuConfigure(void);
   void menuInit(void);
   void menuLoop(void);
   void menuRoot(void);
   void menuRunMove(void);
   void motorInit(void);
   void motorStop(void);
   void moveGradees(uint32_t nGradees);
   void moveOneStep(void);
   void moveRotations(uint32_t nRotations);
   void moveSteps(uint32_t nSteps);
   void runFullStep(void);
   void runHalfStep(void);
   void runMicrostepping(void);
   void runWave(void);
   void showAbout(void);
   void showPinWiring(void);
   void stateMachineCheck(void);
   void stateMachineInit(void);
   void viewConfig(void);
   void writeStep(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4);

#endif
