/******************************************************************************************
 *  File:           StepperMotor.ino
 *  Author:         Nicola Sellitto     nicosellitto@yahoo.it
 *  Version:        0.05
 *  Date:           11 may 2024
 *  Description:    Test Unipola & Bipolar stepper motor
 *  Note:           Use a serial connection to 115200 and select menu options
 *****************************************************************************************/

#include <math.h>
#include <Arduino.h>
#include "StepperMotor.h"


 uint8_t PinA1 = 255;
 uint8_t PinB1 = 255;
 uint8_t PinA2 = 255;
 uint8_t PinB2 = 255;
 uint8_t RotationType = ROTATION_CLOCKWISE;
 uint8_t DriveMode = MODE_FULLSTEP;
  int8_t StepId = 0;
uint16_t RPM = 4;                                 // default Revolution per Minute
uint32_t StepsRevolution = 2048;                  // default number of step for a revolution
uint32_t StepDelay = 2000;                        // default time in us
uint32_t RpmStepDelay;                            // time in us
    bool MotorInitialized = false;                // Flag init ok
 uint8_t MenuChoice = 0;
 uint8_t MenuCurrent = MENU_ROOT;
 uint8_t MenuState = STATE_DISPLAY_MENU;
    bool MenuRunnig = true;
uint32_t ValueMin;
uint32_t ValueMax;
uint32_t NumSteps;                                // Number of Stpes to move
uint32_t MoveStartTime;                           // time starting moving in ms

struct sStateMachine MotorSM;

char PrintBuffer[256];				              // Buffer per funzioni di Print
const char* LineSeparator = "===========================================";


/******************************************************************************************
 * 
 *****************************************************************************************/
void setup() {

    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial);                                        // wait connect, needed for native USB port

    showAbout();
    calculateRpmStepDelay();
    stateMachineInit();
    menuInit();
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void loop() {  

    if (MenuRunnig) menuLoop();
    stateMachineCheck();    
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void stateMachineInit() {

    MotorSM.state = SM_STATE_STOP;
    MotorSM.start = 0;
    MotorSM.timeout = 0;
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void stateMachineCheck() {
uint32_t elapsedTime;

    if (MotorSM.state == SM_STATE_STOP) return;

    // SM is in SM_STATE_WAIT_TIMEOUT:
    elapsedTime = micros() - MotorSM.start;
    if (MotorSM.timeout > elapsedTime) return;          // None Timeout            
    MotorSM.state = SM_STATE_STOP;                      // Clear flag after Timeout
    if (NumSteps) moveOneStep();                        // Check for other step to run   
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void motorInit() {

    if (PinA1 == 255) {
    	Serial.println("\n ABORT - pin A1 not defined \n");
        return;
    }
    if (PinA2 == 255) {
    	Serial.println("\n ABORT - pin A2 not defined \n");
        return;
    }
    if (PinB1 == 255) {
    	Serial.println("\n ABORT - pin B1 not defined \n");
        return;
    }
    if (PinB2 == 255) {
    	Serial.println("\n ABORT - pin B2 not defined \n");
        return;
    }    

    pinMode(PinA1, OUTPUT);
    pinMode(PinB1, OUTPUT);
    pinMode(PinA2, OUTPUT);
    pinMode(PinB2, OUTPUT); 
    digitalWrite(PinA1, LOW);
    digitalWrite(PinB1, LOW);
    digitalWrite(PinA2, LOW);
    digitalWrite(PinB2, LOW);
    StepId = 0;
    MotorInitialized = true;
    Serial.println("Stepper Motor Initialized");
}



/******************************************************************************************
 * 
 *****************************************************************************************/
void motorStop() {

    digitalWrite(PinA1, LOW);
    digitalWrite(PinB1, LOW);
    digitalWrite(PinA2, LOW);
    digitalWrite(PinB2, LOW);
    MotorSM.state = SM_STATE_STOP;
    Serial.println("Stopped Motor");
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void calculateRpmStepDelay() {
uint32_t usMinute = 60000000;
uint32_t usOneRevolution;

    RpmStepDelay = (usMinute/RPM)/StepsRevolution;
    if (RpmStepDelay < StepDelay) {
        RpmStepDelay = StepDelay;
        usOneRevolution = RpmStepDelay * StepsRevolution;
        RPM = usMinute / usOneRevolution;
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void moveSteps(uint32_t  nSteps) {

    if (!MotorInitialized) {
    	Serial.println("\n ABORT - Motor not initialized \n");
        return;
    }
    if (MotorSM.state != SM_STATE_STOP) {
    	Serial.println("\n ABORT - Motor is alread running \n");
        return;
    }

    sprintf(PrintBuffer, "\nRun steps: %u", nSteps);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "RPM: %u", RPM);
	Serial.println(PrintBuffer);

	Serial.print("Rotation: ");
    if (RotationType == ROTATION_CLOCKWISE) Serial.println("CLOCKWISE");
    else Serial.println("ANTI-CLOCKWISE");

	Serial.print("Drive Mode: ");
    switch (DriveMode) {
        case MODE_WAVE:      	  Serial.println("WAVE");           break;
        case MODE_FULLSTEP:       Serial.println("FULL-STEP");      break;
        case MODE_HALFSTEP:       Serial.println("HALF-STEP");      break;
        case MODE_MICROSTEPPING:  Serial.println("MICROSTEPPING");  break;
    }

    if (DriveMode == MODE_MICROSTEPPING) {
        Serial.println("ABORT - Microsteping mode is TBD");
        return;
    }

    NumSteps = nSteps;
    MoveStartTime = millis();
    moveOneStep();
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void moveGradees(uint32_t  nGradees) {
uint32_t nSteps;

    if (!MotorInitialized) {
    	Serial.println("\n ABORT - Motor not initialized \n");
        return;
    }
    if (MotorSM.state != SM_STATE_STOP) {
    	Serial.println("\n ABORT - Motor is alread running \n");
        return;
    }

    nSteps = ceil((float) (StepsRevolution*nGradees)/360);    
    moveSteps(nSteps);
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void moveRotations(uint32_t  nRotations) {
uint32_t nSteps;

    if (!MotorInitialized) {
    	Serial.println("\n ABORT - Motor not initialized \n");
        return;
    }
    if (MotorSM.state != SM_STATE_STOP) {
    	Serial.println("\n ABORT - Motor is alread running \n");
        return;
    }
    nSteps = nRotations * StepsRevolution;
    moveSteps(nSteps);
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void moveOneStep() {
uint8_t numPhase;
uint32_t elapsedTime;

    if (MotorSM.state != SM_STATE_STOP) return;         // Step alread running

    switch (DriveMode) {
        case MODE_WAVE:           runWave();           numPhase = 3;  break;
        case MODE_FULLSTEP:       runFullStep();       numPhase = 3;  break;
        case MODE_HALFSTEP:       runHalfStep();       numPhase = 7;  break;
        case MODE_MICROSTEPPING:  runMicrostepping();                 break; 
    }
    (RotationType == ROTATION_CLOCKWISE) ? StepId++ : StepId--;
    if (StepId > numPhase) StepId = 0;
    if (StepId < 0) StepId = numPhase;
    NumSteps--;
    if (NumSteps) {
        MotorSM.start = micros();
        MotorSM.timeout = RpmStepDelay - 10;            // 10 us for overhead
        MotorSM.state = SM_STATE_WAIT_TIMEOUT;
        return;
    }
    elapsedTime = millis() - MoveStartTime;
  	Serial.println();
    sprintf(PrintBuffer, "ElapsedTime: %u ms",  elapsedTime);
	Serial.println(PrintBuffer);
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void runWave() {

    switch (StepId) {
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

    switch (StepId) {
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

    switch (StepId) {
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
void runMicrostepping() {
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void writeStep(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {

    digitalWrite(PinA1, p1);
    digitalWrite(PinB1, p2);
    digitalWrite(PinA2, p3);
    digitalWrite(PinB2, p4);
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
		case MENU_ROOT:       menuRoot();     	return;
        case MENU_CONFIGURE:  menuConfigure();  return;
        case MENU_RUN_MOVE:   menuRunMove();    return;
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
            ret = getChoice("CMPVAQ", &c);
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
                case 'A':  showAbout();	         		  break;
                case 'P':  showPinWiring(); 		      break;
                case 'V':  viewConfig();	    		  break;
            }
            return;
    }

}


/******************************************************************************************
 * 
 *****************************************************************************************/
void menuConfigure() {
uint32_t value;
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("MOTOR CONFIGURATION");
            Serial.println("1) set A1 pin");
            Serial.println("2) set B1 pin");
            Serial.println("3) set A2 pin");
            Serial.println("4) set B2 pin");
            Serial.println("A) set Anti-clockwise rotation");
            Serial.println("C) set Clockwise rotation");
            Serial.println("F) set Full step mode");
            Serial.println("H) set Half step mode");
            Serial.println("M) set Microstepping mode");
            Serial.println("W) set Wave mode");
            Serial.println("D) set step Delay");
            Serial.println("R) set Rpm");
            Serial.println("S) set revolution Steps");
            Serial.println("P) show Pin wiring");
            Serial.println("V) View configuration");
		    displayMenuTail();
            return;

    	case STATE_WAIT_CHOICE:
            ret = getChoice("1234ACFHMWDRSPVQ", &c);
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
                    RotationType = ROTATION_COUNTERCLOCKWISE;
	  		    	MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'C':
        			Serial.println("Set rotation to CLOCKWISE");
                    RotationType = ROTATION_CLOCKWISE;
	  		    	MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'F':
    	    		Serial.println("Set drive mode to FULL-STEP");
                    DriveMode = MODE_FULLSTEP;
	  			    MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'H':
    	    		Serial.println("Set drive mode to HALF-STEP");
                    DriveMode = MODE_HALFSTEP;
	  			    MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'M':
        			Serial.println("Set drive mode to MICROSTEPPING");
                    DriveMode = MODE_MICROSTEPPING;
	  		    	MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'W':
        			Serial.println("Set drive mode to WAVE");
                    DriveMode = MODE_WAVE;
	  		        MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'D':
        			Serial.print("Enter Step Delay us [100, 100000]: ");
                    ValueMin = 100;
                    ValueMax = 100000;
	  			    MenuState = STATE_WAIT_USER_INPUT;
                    return;
                case 'R':
        			Serial.print("Enter RPM [1, 1000]: ");
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
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'V':
                    viewConfig();
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
            MenuState = STATE_DISPLAY_MENU;
            switch (MenuChoice) {
    	        case '1':  PinA1 =  (uint8_t) value;                                     return;
	            case '2':  PinB1 =  (uint8_t) value;                                     return;
        	    case '3':  PinA2 =  (uint8_t) value;                                     return;
    	        case '4':  PinB2 =  (uint8_t) value;                                     return;
	            case 'R':  RPM   = (uint16_t) value;            calculateRpmStepDelay();  return;
	            case 'D':  StepDelay = value;                   calculateRpmStepDelay();  return;
	            case 'S':  StepsRevolution = (uint16_t) value;  calculateRpmStepDelay();  return;
            }
            return;
    }
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void menuRunMove() {
uint32_t value;
 uint8_t ret;
    char c;

    switch (MenuState) {
	    case STATE_DISPLAY_MENU:
		    displayMenuHeader("MOTOR MOVE");
            Serial.println("1) run one step");
            Serial.println("2) run one grade");
            Serial.println("3) run one rotation");
            Serial.println("4) run number steps");
            Serial.println("5) run number gradees");
            Serial.println("6) run number rotations");
            Serial.println("I) Init motor");
            Serial.println("S) Stop motor");
            Serial.println("V) View config");
		    displayMenuTail();
            break;

    	case STATE_WAIT_CHOICE:
            ret = getChoice("123456ISVQ", &c);
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
                    moveSteps(1);
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case '2':
                    moveGradees(1);
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case '3':
                    moveRotations(1);
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
                case 'I':
                    motorInit();
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'S':
                    motorStop();
	  	    		MenuState = STATE_DISPLAY_MENU;
                    return;
                case 'V':
                    viewConfig();
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
    	        case '4':  moveSteps(value);      return;
    	        case '5':  moveGradees(value);     return;
    	        case '6':  moveRotations(value);  return;
            }
            return;
	}
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void showPinWiring() {

    Serial.println();
    Serial.println("  Unipolar Stepper Motor Coils - Step sequence: A1, B1, A2, B2");
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
    Serial.println("  Bipolar Stepper Motor Coils - Step sequence: A1, B1, A2, B2");
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
    Serial.println("gpio12   D6      IN4       orange      A1");
    Serial.println("gpio13   D7      IN3       yellow      B1");
    Serial.println("gpio14   D5      IN2       pink        A2");
    Serial.println("gpio15   D8      IN1       blue        B2");
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
    Serial.println("gpio12   D6      IN2      A+     orange      A1");
    Serial.println("gpio13   D7      IN3      B-     yellow      B1");
    Serial.println("gpio14   D5      IN1      A-     pink        A2");
    Serial.println("gpio15   D8      IN4      B+     blue        B2");
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
}


/******************************************************************************************
 * 
 *****************************************************************************************/
void showAbout() {

    Serial.println("\n");
    Serial.println(FIRMWARE_NAME);
    sprintf(PrintBuffer, "%s - %s \n", FIRMWARE_VERSION, FIRMWARE_DATE);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%s - %s \n", FIRMWARE_AUTHOR, FIRMWARE_EMAIL);
	Serial.println(PrintBuffer);
}

/******************************************************************************************
 * 
 *****************************************************************************************/
void viewConfig() {

	Serial.println();
    Serial.println(LineSeparator);
	Serial.println("STEPPER CONFIGURATION");
    Serial.println(LineSeparator);

    sprintf(PrintBuffer, "%20s: %u", "Pin IN1 B-",  PinA1);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u", "Pin IN2 A+",  PinB1);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u", "Pin IN3 B+",  PinA2);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u", "Pin IN4 A-",  PinB2);
	Serial.println(PrintBuffer);

    sprintf(PrintBuffer, "%20s: %s", "Rotation", (RotationType == ROTATION_CLOCKWISE) ? "CLOCKWISE" : "ANTI-CLOCKWISE");
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u", "Steps Revolution",  StepsRevolution);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u us", "Step Delay",  StepDelay);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u", "RPM",  RPM);
	Serial.println(PrintBuffer);
    sprintf(PrintBuffer, "%20s: %u us", "Rpm Step Delay",  RpmStepDelay);
	Serial.println(PrintBuffer);

    if (DriveMode == MODE_WAVE)          sprintf(PrintBuffer, "%20s: %s", "Drive Mode", "WAVE");
    if (DriveMode == MODE_FULLSTEP)      sprintf(PrintBuffer, "%20s: %s", "Drive Mode", "FULL STEP");
    if (DriveMode == MODE_HALFSTEP)      sprintf(PrintBuffer, "%20s: %s", "Drive Mode", "HALF STEP");
    if (DriveMode == MODE_MICROSTEPPING) sprintf(PrintBuffer, "%20s: %s", "Drive Mode", "MICROSTEPPING");
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
