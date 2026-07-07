// Code created from "LR_SerialPrinting_V1" on 6/19/2026 by Owen Murphy 
// used for testing

#include <Arduino.h>
#include <AccelStepper.h>
#include <math.h>

const int STEP_PIN = 26;
const int DIR_PIN = 27;
const int HOMING_PIN = 23;  // HIGH is pressed - now an NC switch, so HIGH when pressed, LOW when released
const int MS1_PIN = 32;
const int MS2_PIN = 33;

const int POS_JOG_PIN = 15;  // LOW is pressed
const int NEG_JOG_PIN = 16;   // LOW is pressed


const long REV_STEPS = 800;  // 1/4 step mode
const int PULLEY_TEETH = 16;
const float BELT_PITCH_MM = 2.0;
const float MM_PER_REV = PULLEY_TEETH * BELT_PITCH_MM;


const float STEPS_PER_MM = REV_STEPS / MM_PER_REV;

// Signed speeds.
// Negative = toward home switch.
// Positive = away from home switch.
const float HOMING_SPEED_SLOW = -100.0;
const float HOMING_SPEED_FAST = -600.0;
const float BACKOFF_SPEED = 200.0;
const float JOG_SPEED = 1200.0;

const unsigned long BACKOFF_DURATION_MS = 1000;
const unsigned long HOME_LED_DURATION_MS = 1000;
const unsigned long HOMING_TIMEOUT = 10000;

const float HOME_PULL_OFF_MM = 5.0;

const int ERROR_LED_PIN1 = 17; //
const int IDLE_LED_PIN = 18;  //GREEN
const int HOME_LED_PIN = 19;  //BLUE
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

bool homeButtonPressed() {
  return digitalRead(HOMING_PIN) == HIGH;  // HIGH is pressed - now an NC switch, so HIGH when pressed, LOW when released
}

bool posJogPressed() {
  return digitalRead(POS_JOG_PIN) == LOW;
}

bool negJogPressed() {
  return digitalRead(NEG_JOG_PIN) == LOW;
}

void microStep(bool MS1_STATE, bool MS2_STATE) {
  digitalWrite(MS1_PIN, MS1_STATE);
  digitalWrite(MS2_PIN, MS2_STATE);
}

long mmToSteps(float mm) {
  return lround(mm * STEPS_PER_MM);  //math library function to convert float to long
}

void goToMm(float mm) {
  stepper.moveTo(mmToSteps(mm));
}

enum class Mode {
  HOMING,
  IDLE,
  JOGGING,
  ERROR,

};

enum class HomingPhase {
  FAST_APPROACH,
  BACK_OFF,
  SLOW_APPROACH,
  SET_ZERO,
  HOMING_FINISHED,
};

enum class ErrorMode {
  NO_ERROR,
  HOMING_TIMEOUT,
  UNKNOWN
};

// Manual declare function prototypes so Arduino IDE doesn't guess wrong -- from ChatGPT - weird way IDE defines functions
void changeState(Mode nextState);
void changeHomingPhase(HomingPhase nextPhase);
void changeErrorMode(ErrorMode nextMode);
bool stateTimer(unsigned long stateDuration);
bool phaseTimer(unsigned long phaseDuration);
bool modeTimer(unsigned long modeDuration);


void homeLight(bool state);


//CGPT ADDITION - FOR PRINTING.........................................
  const char* modeText(Mode currentMode);
  const char* homingPhaseText(HomingPhase currentPhase);
  const char* errorModeText(ErrorMode currentError);
  const char* ledText(bool state);
  void printStatus();
//CGPT ADDITION - FOR PRINTING ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Mode mode = Mode::HOMING;
HomingPhase homingPhase = HomingPhase::FAST_APPROACH;
ErrorMode errorMode = ErrorMode::NO_ERROR;

unsigned long stateStartTime = 0;
unsigned long phaseStartTime = 0;
unsigned long modeStartTime = 0;

void changeState(Mode nextState) {
  mode = nextState;
  stateStartTime = millis();
}

void changeHomingPhase(HomingPhase nextPhase) {
  homingPhase = nextPhase;
  phaseStartTime = millis();
}

void changeErrorMode(ErrorMode nextMode) {
  errorMode = nextMode;
  modeStartTime = millis();
}


bool stateTimer(unsigned long stateDuration) {
  return millis() - stateStartTime >= stateDuration;
}

bool phaseTimer(unsigned long phaseDuration) {
  return millis() - phaseStartTime >= phaseDuration;
}

bool modeTimer(unsigned long modeDuration) {
  return millis() - modeStartTime >= modeDuration;
}


void homeLight(bool state) {
  digitalWrite(HOME_LED_PIN, state ? HIGH : LOW);
}

void errorLight1(bool state) {
  digitalWrite(ERROR_LED_PIN1, state ? HIGH : LOW);
}

void idleLight(bool state) {
  digitalWrite(IDLE_LED_PIN, state ? HIGH : LOW);
}

//CGPT ADDITION - FOR PRINTING..............................................
    const unsigned long PRINT_INTERVAL_MS = 250;  // PLACEHOLDER: choose print rate

  const char* modeText(Mode currentMode) {
    switch (currentMode) {
      case Mode::HOMING:
        return "HOMING";  // PLACEHOLDER MESSAGE

      case Mode::IDLE:
        return "IDLE";  // PLACEHOLDER MESSAGE

      case Mode::ERROR:
        return "ERROR";  // PLACEHOLDER MESSAGE

      default:
        return "UNKNOWN MODE";
    }
  }

  const char* homingPhaseText(HomingPhase currentPhase) {
    switch (currentPhase) {
      case HomingPhase::FAST_APPROACH:
        return "FAST_APPROACH";  // PLACEHOLDER MESSAGE

      case HomingPhase::BACK_OFF:
        return "BACK_OFF";  // PLACEHOLDER MESSAGE

      case HomingPhase::SLOW_APPROACH:
        return "SLOW_APPROACH";  // PLACEHOLDER MESSAGE

      case HomingPhase::SET_ZERO:
        return "SET_ZERO";  // PLACEHOLDER MESSAGE

      case HomingPhase::HOMING_FINISHED:
        return "HOMING_FINISHED";  // PLACEHOLDER MESSAGE

      default:
        return "UNKNOWN HOMING PHASE";
    }
  }

  const char* errorModeText(ErrorMode currentError) {
    switch (currentError) {
      case ErrorMode::NO_ERROR:
        return "NO_ERROR";  // PLACEHOLDER MESSAGE

      case ErrorMode::HOMING_TIMEOUT:
        return "HOMING_TIMEOUT";  // PLACEHOLDER MESSAGE

      case ErrorMode::UNKNOWN:
        return "UNKNOWN_ERROR";  // PLACEHOLDER MESSAGE

      default:
        return "UNKNOWN ERROR MODE";
    }
  }

  const char* ledText(bool state) {
    return state ? "ON" : "OFF";
  }
//CGPT ADDITION - FOR PRINTING ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//CGPT ADDITION - FOR PRINTING................................................
  void printStatus() {
    static unsigned long lastPrintTime = 0;

    if (millis() - lastPrintTime < PRINT_INTERVAL_MS) {
      return;
    }

    lastPrintTime = millis();

    float carriagePositionMm = stepper.currentPosition() / STEPS_PER_MM;
    float speedStepsPerSec = stepper.speed();
    float speedMmPerSec = speedStepsPerSec / STEPS_PER_MM;

    Serial.print("State: ");
    Serial.print(modeText(mode));

    Serial.print(" | Homing Phase: ");
    Serial.print(homingPhaseText(homingPhase));

    Serial.print(" | Error Mode: ");
    Serial.print(errorModeText(errorMode));

    Serial.print(" | Home LED: ");
    Serial.print(ledText(digitalRead(HOME_LED_PIN)));

    Serial.print(" | Idle LED: ");
    Serial.print(ledText(digitalRead(IDLE_LED_PIN)));

    Serial.print(" | Error LED 1: ");
    Serial.print(ledText(digitalRead(ERROR_LED_PIN1)));

    Serial.print(" | Position: ");
    Serial.print(carriagePositionMm, 2);
    Serial.print(" mm");

    Serial.print(" | Speed: ");
    Serial.print(speedStepsPerSec, 1);
    Serial.print(" steps/s");

    Serial.print(" | Speed: ");
    Serial.print(speedMmPerSec, 2);
    Serial.print(" mm/s");

    Serial.println();
  }
//CGPT ADDITION - FOR PRINTING ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


void setup() {

  delay(1000); // Delay homing routine for monitoring and upload stability
  Serial.begin(115200);

  pinMode(HOMING_PIN, INPUT_PULLUP);
  pinMode(POS_JOG_PIN, INPUT_PULLUP);
  pinMode(NEG_JOG_PIN, INPUT_PULLUP);

  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);

  pinMode(HOME_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN1, OUTPUT);
  pinMode(IDLE_LED_PIN, OUTPUT);
  homeLight(false);


  microStep(LOW, HIGH);

  stepper.setPinsInverted(true, false, false); 

  stepper.setMaxSpeed(6000);
  stepper.setAcceleration(3000);

  changeState(Mode::HOMING);
  changeHomingPhase(HomingPhase::FAST_APPROACH);
  changeErrorMode(ErrorMode::NO_ERROR);
}

void loop() {
  switch (mode) {

    case Mode::HOMING:
      switch (homingPhase) {

        case HomingPhase::FAST_APPROACH:
          if (homeButtonPressed()) {
            stepper.setSpeed(BACKOFF_SPEED);
            changeHomingPhase(HomingPhase::BACK_OFF);
          } else if (phaseTimer(HOMING_TIMEOUT)) {
            changeErrorMode(ErrorMode::HOMING_TIMEOUT);
            changeState(Mode::ERROR);
          } else {
            stepper.setSpeed(HOMING_SPEED_FAST);
            stepper.runSpeed();
          }
          break;

        case HomingPhase::BACK_OFF:
          if (phaseTimer(BACKOFF_DURATION_MS)) {
            stepper.setSpeed(HOMING_SPEED_SLOW);
            changeHomingPhase(HomingPhase::SLOW_APPROACH);
          } else {
            stepper.runSpeed();
          }
          break;

        case HomingPhase::SLOW_APPROACH:
          if (homeButtonPressed()) {

            stepper.setCurrentPosition(mmToSteps(-HOME_PULL_OFF_MM));
            goToMm(0);
            changeHomingPhase(HomingPhase::SET_ZERO);
          } else if (phaseTimer(HOMING_TIMEOUT)) {
            changeErrorMode(ErrorMode::HOMING_TIMEOUT);
            changeState(Mode::ERROR);
          } else {
            stepper.runSpeed();
          }
          break;

        case HomingPhase::SET_ZERO:
          if (stepper.distanceToGo() == 0) {
            stepper.setCurrentPosition(0);
            changeHomingPhase(HomingPhase::HOMING_FINISHED);
          } else {
            stepper.run();
          }
          break;

        case HomingPhase::HOMING_FINISHED:
          if (phaseTimer(HOME_LED_DURATION_MS)) {
            homeLight(LOW);
            goToMm(20);
            changeState(Mode::IDLE);
          } else {
            homeLight(HIGH);
          }
          break;
      }

      break;

    case Mode::IDLE:
      idleLight(HIGH);
      stepper.setSpeed(0);
      stepper.setCurrentPosition(stepper.currentPosition()); // Prevents motor from drifting when idle
      stepper.run();
        if (posJogPressed() || negJogPressed()) {
          changeState(Mode::JOGGING);
          idleLight(LOW);
        }
      break;
    
    case Mode::JOGGING:
        if (posJogPressed()) {
          stepper.setSpeed(JOG_SPEED);
          stepper.runSpeed();
        } else if (negJogPressed()) {
          stepper.setSpeed(-JOG_SPEED);
          stepper.runSpeed();          
        } else if (posJogPressed() && negJogPressed()) { // If both buttons are pressed, stop movement (safety feature)
          stepper.setSpeed(0); 
          stepper.run();
        } else {          
          stepper.moveTo(stepper.currentPosition()); //stops motor returning to initial idle position
          changeState(Mode::IDLE);
        }
        break;

    case Mode::ERROR:
      stepper.setSpeed(0); 
      stepper.stop(); 
      
      switch (errorMode) {
        case ErrorMode::NO_ERROR:
        
          break;
        case ErrorMode::HOMING_TIMEOUT:
          errorLight1(HIGH);
          break;
        case ErrorMode::UNKNOWN:
          
          break;
      }
      break;
  }


//CGPT ADDITION - FOR PRINTING....................
  printStatus();
//CGPT ADDITION - FOR PRINTING^^^^^^^^^^^^^^^^^^^^

}