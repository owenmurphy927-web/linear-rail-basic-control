// Code created from "LR_SerialPrinting_V1" on 6/19/2026 by Owen Murphy 
// used for testing

#include <Arduino.h>
#include <AccelStepper.h>
#include <math.h>
#include "Encoder.h"

const int STEP_PIN = 26;
const int DIR_PIN = 27;
const int HOMING_PIN = 23;  // HIGH is pressed - now an NC switch, so HIGH when pressed, LOW when released
const int FAR_LIMIT_PIN = 13;  // NC switch, HIGH = pressed (matches HOMING_PIN), INPUT_PULLUP. Far end-of-travel limit.
const int MS1_PIN = 32;
const int MS2_PIN = 33;

const int POS_JOG_PIN = 15;  // LOW is pressed
const int NEG_JOG_PIN = 16;   // LOW is pressed

const int ENCODER_SDA_PIN = 21;
const int ENCODER_SCL_PIN = 22;


const long REV_STEPS = 800;  // 1/4 step mode
const int PULLEY_TEETH = 16;
const float BELT_PITCH_MM = 2.0;
const float MM_PER_REV = PULLEY_TEETH * BELT_PITCH_MM;


const float STEPS_PER_MM = REV_STEPS / MM_PER_REV;

const float POSITION_MISMATCH_TOLERANCE_MM = 0.5;  // starting point -- tune after watching normal-run "Pos Diff mm"
const bool POSITION_MISMATCH_HALT_ENABLED = true;  // DEBUG: false disables the hard-stop so mismatch can be observed live. Set true once direction/scale calibration is confirmed.

// Signed speeds.
// Negative = toward home switch.
// Positive = away from home switch.
const float HOMING_SPEED_SLOW = -100.0;
const float HOMING_SPEED_FAST = -600.0;
const float BACKOFF_SPEED = 200.0;
const float JOG_SPEED = 400.0;

const unsigned long BACKOFF_DURATION_MS = 1000;
const unsigned long HOME_LED_DURATION_MS = 1000;
const unsigned long HOMING_TIMEOUT = 10000;

const float HOME_PULL_OFF_MM = 5.0;

const float FAR_LIMIT_POSITION_MM = 116.3;   // far switch trigger point, measured from zero
const float SOFT_LIMIT_BUFFER_MM  = 5.0;     // keep-out from each switch for jogging soft limits

// Soft jog bounds (open-loop step position, mm). Derived from the switch trigger points so they
// track HOME_PULL_OFF_MM automatically. Near switch sits at -HOME_PULL_OFF_MM.
const float JOG_MIN_POSITION_MM = -HOME_PULL_OFF_MM + SOFT_LIMIT_BUFFER_MM;     // = 0.0mm
const float JOG_MAX_POSITION_MM = FAR_LIMIT_POSITION_MM - SOFT_LIMIT_BUFFER_MM; // = 111.3mm

const int ERROR_LED_PIN1 = 17; //
const int IDLE_LED_PIN = 18;  //GREEN
const int HOME_LED_PIN = 19;  //BLUE
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

bool homeButtonPressed() {
  return digitalRead(HOMING_PIN) == HIGH;  // HIGH is pressed - now an NC switch, so HIGH when pressed, LOW when released
}

bool farLimitPressed() {
  return digitalRead(FAR_LIMIT_PIN) == HIGH;  // NC: HIGH is pressed
}

bool overTravelTriggered() {
  return homeButtonPressed() || farLimitPressed();  // either end-of-travel switch
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

float positionDiffMm() {
  float stepperMm = stepper.currentPosition() / STEPS_PER_MM;
  return stepperMm - railEncoder.positionMm();
}

bool positionMismatchDetected() {
  return fabs(positionDiffMm()) > POSITION_MISMATCH_TOLERANCE_MM;
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
  POSITION_MISMATCH,
  ENCODER_NOT_DETECTED,
  OVER_TRAVEL,
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

float maxAbsPositionDiffMm = 0.0f;

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
    const unsigned long PRINT_INTERVAL_MS = 500;  // PLACEHOLDER: choose print rate

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

      case ErrorMode::POSITION_MISMATCH:
        return "POSITION_MISMATCH";  // PLACEHOLDER MESSAGE

      case ErrorMode::ENCODER_NOT_DETECTED:
        return "ENCODER_NOT_DETECTED";  // PLACEHOLDER MESSAGE

      case ErrorMode::OVER_TRAVEL:
        return "OVER_TRAVEL";  // PLACEHOLDER MESSAGE

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

    Serial.print(" | Encoder mm: ");
    Serial.print(railEncoder.positionMm(), 2);

    Serial.print(" | Encoder deg: ");
    Serial.print(railEncoder.rawAngleDeg(), 1);

    Serial.print(" | Encoder ticks: ");
    Serial.print(railEncoder.rawTicks());

    Serial.print(" | Encoder Connected: ");
    Serial.print(railEncoder.isConnected() ? "YES" : "NO");

    Serial.print(" | Encoder Err: ");
    Serial.print(railEncoder.lastError());

    Serial.print(" | Pos Diff mm: ");
    Serial.print(carriagePositionMm - railEncoder.positionMm(), 3);

    Serial.print(" | Would Mismatch: ");
    Serial.print(positionMismatchDetected() ? "YES" : "NO");

    Serial.print(" | Max |Diff| mm: ");
    Serial.print(maxAbsPositionDiffMm, 3);

    Serial.print(" | Limits H/F: ");
    Serial.print(homeButtonPressed() ? "1" : "0");
    Serial.print(farLimitPressed() ? "1" : "0");

    Serial.println();
  }
//CGPT ADDITION - FOR PRINTING ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


void setup() {

  delay(1000); // Delay homing routine for monitoring and upload stability
  Serial.begin(115200);

  pinMode(HOMING_PIN, INPUT_PULLUP);
  pinMode(FAR_LIMIT_PIN, INPUT_PULLUP);
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

  bool encoderOk = railEncoder.begin(ENCODER_SDA_PIN, ENCODER_SCL_PIN, MM_PER_REV, AS5600_COUNTERCLOCK_WISE);

  if (!encoderOk) {
    changeErrorMode(ErrorMode::ENCODER_NOT_DETECTED);
    changeState(Mode::ERROR);
  } else {
    changeState(Mode::HOMING);
    changeHomingPhase(HomingPhase::FAST_APPROACH);
    changeErrorMode(ErrorMode::NO_ERROR);
  }
}

void loop() {
  railEncoder.update();

  // Over-travel failsafe: an end-of-travel switch trip in any mode but HOMING
  // immediately halts and latches into ERROR. Placed above switch(mode) so the
  // ERROR case runs this same iteration (no further step pulses issue). HOMING is
  // excluded so homing can still drive into the home switch; ERROR is excluded so
  // we don't re-stamp modeStartTime every cycle once already halted.
  if (mode != Mode::HOMING && mode != Mode::ERROR) {
    if (overTravelTriggered()) {
      stepper.setSpeed(0);
      changeErrorMode(ErrorMode::OVER_TRAVEL);
      changeState(Mode::ERROR);
    }
  }

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
            railEncoder.zero();
            maxAbsPositionDiffMm = 0.0f;
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
          if (stepper.currentPosition() >= mmToSteps(JOG_MAX_POSITION_MM)) {
            stepper.setSpeed(0);
            stepper.moveTo(stepper.currentPosition());
            stepper.run();                       // hold at soft max, stay in JOGGING
          } else {
            stepper.setSpeed(JOG_SPEED);
            stepper.runSpeed();
          }
        } else if (negJogPressed()) {
          if (stepper.currentPosition() <= mmToSteps(JOG_MIN_POSITION_MM)) {
            stepper.setSpeed(0);
            stepper.moveTo(stepper.currentPosition());
            stepper.run();                       // hold at soft min, stay in JOGGING
          } else {
            stepper.setSpeed(-JOG_SPEED);
            stepper.runSpeed();
          }
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
        case ErrorMode::POSITION_MISMATCH:
          errorLight1(HIGH);
          break;
        case ErrorMode::ENCODER_NOT_DETECTED:
          errorLight1(HIGH);
          break;
        case ErrorMode::OVER_TRAVEL:
          errorLight1(HIGH);
          break;
        case ErrorMode::UNKNOWN:
          
          break;
      }
      break;
  }

  if (mode == Mode::IDLE || mode == Mode::JOGGING) {
    if (!railEncoder.isConnected()) {
      changeErrorMode(ErrorMode::ENCODER_NOT_DETECTED);
      changeState(Mode::ERROR);
    } else {
      float diffMm = fabs(positionDiffMm());
      if (diffMm > maxAbsPositionDiffMm) {
        maxAbsPositionDiffMm = diffMm;
      }
      if (POSITION_MISMATCH_HALT_ENABLED && diffMm > POSITION_MISMATCH_TOLERANCE_MM) {
        changeErrorMode(ErrorMode::POSITION_MISMATCH);
        changeState(Mode::ERROR);
      }
    }
  }

//CGPT ADDITION - FOR PRINTING....................
  printStatus();
//CGPT ADDITION - FOR PRINTING^^^^^^^^^^^^^^^^^^^^

}