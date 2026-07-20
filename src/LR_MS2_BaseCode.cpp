// Code created from "LR_SerialPrinting_V1" on 6/19/2026 by Owen Murphy 
// used for testing

#include <Arduino.h>
#include <FastAccelStepper.h>
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

const int TEST_START_PIN = 25;  // LOW is pressed -- normally-open button to GND, INPUT_PULLUP. Starts the automated test sequence (Mode::TESTING).

const int ENCODER_SDA_PIN = 21;
const int ENCODER_SCL_PIN = 22;


// Microstep configuration -- single source of truth. Selecting ACTIVE_MICROSTEP sets BOTH the
// MS1/MS2 driver pins (applied in setup() via microStep()) AND steps-per-rev, so the two can never
// drift out of sync (previously REV_STEPS and the microStep() call were set independently). The
// pin<->resolution map is the empirically-measured bench table in hardware/MICROSTEPPING.md. A
// constexpr struct (not an enum) keeps this a plain compile-time config.
struct MicrostepMode { bool ms1; bool ms2; long stepsPerRev; };
constexpr MicrostepMode MS_HALF      {true,  false, 400 };   // 1/2  step
constexpr MicrostepMode MS_QUARTER   {false, true,  800 };   // 1/4  step
constexpr MicrostepMode MS_EIGHTH    {false, false, 1600};   // 1/8  step
constexpr MicrostepMode MS_SIXTEENTH {true,  true,  3200};   // 1/16 step

constexpr MicrostepMode ACTIVE_MICROSTEP = MS_QUARTER;       // <-- change this to reconfigure microstepping
const long REV_STEPS = ACTIVE_MICROSTEP.stepsPerRev;         // steps per motor revolution (derived)

const int PULLEY_TEETH = 16;
const float BELT_PITCH_MM = 2.0;
const float MM_PER_REV = PULLEY_TEETH * BELT_PITCH_MM;


const float STEPS_PER_MM = REV_STEPS / MM_PER_REV;

const float POSITION_MISMATCH_TOLERANCE_MM = 0.5;  // starting point -- tune after watching normal-run "Pos Diff mm"
const bool POSITION_MISMATCH_HALT_ENABLED = false;  // DEBUG: false disables the hard-stop so mismatch can be observed live. Set true once direction/scale calibration is confirmed.

// Homing / back-off signed speeds (safety- and homing-specific; NOT part of the swept test params).
// Negative = toward home switch. Positive = away from home switch.
const float HOMING_SPEED_SLOW = -100.0;
const float HOMING_SPEED_FAST = -600.0;
const float BACKOFF_SPEED = 200.0;

const unsigned long BACKOFF_DURATION_MS = 1000;
const unsigned long HOME_LED_DURATION_MS = 1000;
const unsigned long HOMING_TIMEOUT = 10000;
const unsigned long HOMING_SETTLE_MS = 150;  // let forceStop fully drain before issuing the next move command

const float HOME_PULL_OFF_MM = 5.0;

const float FAR_LIMIT_POSITION_MM = 116.3;   // far switch trigger point, measured from zero
const float SOFT_LIMIT_BUFFER_MM  = 8.0;     // keep-out from each switch for jogging soft limits

// Soft jog bounds (open-loop step position, mm). Derived from the switch trigger points so they
// track HOME_PULL_OFF_MM automatically. Near switch sits at -HOME_PULL_OFF_MM.
const float JOG_MIN_POSITION_MM = -HOME_PULL_OFF_MM + SOFT_LIMIT_BUFFER_MM;     // = 0.0mm
const float JOG_MAX_POSITION_MM = FAR_LIMIT_POSITION_MM - SOFT_LIMIT_BUFFER_MM; // = 111.3mm

const int ERROR_LED_PIN1 = 17; //
const int IDLE_LED_PIN = 18;  //GREEN
const int HOME_LED_PIN = 19;  //BLUE

// ---- MOTION TUNING (consolidated) ----
// One place to tune the speed/acceleration of positioning moves, kept independent of the control
// mode (open- vs closed-loop) so OL/CL test runs share identical motion params. Speed and accel are
// runtime-mutable via setMotionSpeed()/setMotionAccel() (defined below, near the command helpers) so
// test code can sweep them -- e.g. the acceleration ramp in docs/TESTING_PLAN.md -- without touching
// setup() or the state machine. Boot defaults match the previous MAX_SPEED_HZ / ACCEL_STEPS_PER_S2.
uint32_t motionSpeedHz         = 10000;   // positioning-move cruise speed (per moveTo), steps/s
uint32_t motionAccelStepsPerS2 = 12000;   // ramp acceleration, steps/s^2
const float JOG_SPEED          = motionSpeedHz; // manual jog velocity, steps/s

// ---- TEST CONFIG (automated acceleration / step-loss ramp -- Mode::TESTING) ----
// Everything the test sequence needs, in one place, so a run can be re-parametrized without touching
// the state machine. Started by pressing TEST_START_PIN while IDLE; see docs/TEST_BATTERY.md and
// docs/TESTING_PLAN.md. Speeds/accels are in steps/s (/s^2). At the active microstep (STEPS_PER_MM,
// = 25 at 1/4-step) mm/s^2 = steps-per-s^2 / STEPS_PER_MM -- the logged 'accel' column is steps/s^2.
//
// The ramp is 20 moves (10 back/forth) at a FIXED cruise speed; only the acceleration changes, one
// LINEAR step per move: move i (0-based) uses accel = TEST_ACCEL_START + i*TEST_ACCEL_STEP. Tune
// START/STEP so the later moves trip the open-loop position mismatch (see POSITION_MISMATCH_*), then
// bisect with a narrower START and smaller STEP. Control mode (OL/CL) and microstep are compile-time
// (controlMode / ACTIVE_MICROSTEP) -- reflash between them, keeping runs adjacent.
const uint32_t TEST_CRUISE_SPEED_HZ  = 6000;    // fixed cruise speed for every ramp move, steps/s
const uint32_t TEST_ACCEL_START      = 6000;    // acceleration of move 0, steps/s^2
const uint32_t TEST_ACCEL_STEP       = 1000;    // per-move acceleration increment, steps/s^2 - final accel is TEST_ACCEL_START + ((TEST_NUM_MOVES - 1)/2) * TEST_ACCEL_STEP
const int      TEST_NUM_MOVES        = 20;      // total moves (10 back-and-forth pairs)

// Travel endpoints for each move. Default to the jog soft limits so the test uses the full safe span.
const float TEST_NEAR_MM = JOG_MIN_POSITION_MM;  // near end (toward home)
const float TEST_FAR_MM  = JOG_MAX_POSITION_MM;  // far end  (away from home)

const unsigned long TEST_SETTLE_MS = 400;   // dwell after each move fully stops (clean stop + logged sample)

// Warm-up: run gentle back-and-forth for >=1 min before the recorded ramp (rows tagged phase=WARM so
// they can be dropped in post). Set TEST_WARMUP_ENABLED false to skip straight to the ramp.
const bool          TEST_WARMUP_ENABLED     = true;
const unsigned long TEST_WARMUP_DURATION_MS = 30000;  // >= 1 min per the testing plan
const uint32_t      TEST_WARMUP_ACCEL_HZ2   = 4000;   // gentle warm-up acceleration, steps/s^2
const uint32_t      TEST_WARMUP_SPEED_HZ    = 4000;   // gentle warm-up cruise speed, steps/s

const unsigned long TEST_PRINT_INTERVAL_MS  = 20;     // 50 Hz telemetry while testing (normal run stays PRINT_INTERVAL_MS)

// ---- Closed-loop position control (encoder feedback) ----
// Active ONLY when controlMode == CLOSED_LOOP (see below); the OPEN_LOOP path is unchanged.
// Proportional servo that drives the ENCODER (not the step count) onto the commanded
// setpoint. All LIVE-TUNED against real hardware, like POSITION_MISMATCH_TOLERANCE_MM.
const float    CL_KP                  = 0.8f;         // proportional gain: mm of target-adjust per mm of encoder error
const float    CL_DEADBAND_MM         = 0.10f;        // no correction while |setpoint - encoder| <= this (kills hold dither)
const float    CL_MAX_CORRECTION_MM   = 8.0f;         // HOLD clamp: per-tick target adjustment while holding (keeps target near actual -> no dither/ripple)
const float    CL_TRAVEL_MAX_CORRECTION_MM = 120.0f;  // TRAVEL clamp: large enough to never bind on this ~111mm rail, so the
                                                      // travel lookahead is CL_KP*err (not a fixed 8mm) and the jog reaches JOG_SPEED.
                                                      // A tight travel clamp caps cruise at sqrt(2*accel*clamp) -- far below JOG_SPEED.
const unsigned long CL_UPDATE_INTERVAL_MS = 20;       // servo tick throttle (>= encoder 10ms poll)
const uint32_t CL_CORRECTION_SPEED_HZ = motionSpeedHz; // speed cap for hold-correction moves (snapshot at init; not updated by setMotionSpeed())

// FastAccelStepper generates step pulses in hardware (ESP32 RMT/MCPWM), so pulse timing is
// independent of loop()/printStatus() timing -- loop() no longer services motion every iteration.
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;

// Command cache: the state machine still calls these every loop, but we only issue a command to the
// hardware when the intent actually changes -- this keeps the flat switch(mode) structure while
// avoiding re-triggering FastAccelStepper every iteration.
enum class MotionCmd { NONE, VELOCITY, POSITION };
MotionCmd lastCmd = MotionCmd::NONE;
float lastVelSps = 0.0f;
long  lastTarget = 0;
uint32_t lastMoveSpeedHz = 0;

void commandVelocity(float signedSps) {          // continuous move (jog / homing approach)
  if (!stepper) return;                          // stepper init failed -- no-op (see STEPPER_INIT_FAILED)
  if (lastCmd == MotionCmd::VELOCITY && signedSps == lastVelSps) return;
  lastCmd = MotionCmd::VELOCITY;
  lastVelSps = signedSps;
  if (signedSps == 0.0f) {
    stepper->stopMove();                         // decelerate to a stop and hold
    return;
  }
  stepper->setSpeedInHz((uint32_t)fabsf(signedSps));
  if (signedSps > 0) stepper->runForward(); else stepper->runBackward();
}

void commandMoveTo(long targetSteps, uint32_t speedHz = motionSpeedHz) {  // positioning move at speedHz
  if (!stepper) return;
  if (lastCmd == MotionCmd::POSITION && targetSteps == lastTarget && speedHz == lastMoveSpeedHz) return;
  lastCmd = MotionCmd::POSITION;
  lastTarget = targetSteps;
  lastMoveSpeedHz = speedHz;
  stepper->setSpeedInHz(speedHz);
  stepper->moveTo(targetSteps);
}

void commandHalt() {                             // immediate stop (errors / over-travel / homing hit)
  if (!stepper) return;
  lastCmd = MotionCmd::NONE;                     // force the next command to re-issue
  stepper->forceStop();
}

// Runtime motion-tuning knobs for the MOTION TUNING block above. Plain free functions -- they don't
// touch loop()/the state machine. setMotionAccel pushes straight to the stepper so a tuning sweep
// (e.g. the acceleration ramp in docs/TESTING_PLAN.md) takes effect on the next move; setMotionSpeed
// is picked up by commandMoveTo's default speed argument.
void setMotionSpeed(uint32_t hz)         { motionSpeedHz = hz; }
void setMotionAccel(uint32_t stepsPerS2) { motionAccelStepsPerS2 = stepsPerS2; if (stepper) stepper->setAcceleration(stepsPerS2); }

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

bool testStartPressed() {
  return digitalRead(TEST_START_PIN) == LOW;  // NO button to GND, INPUT_PULLUP: LOW is pressed
}

void microStep(bool MS1_STATE, bool MS2_STATE) {
  digitalWrite(MS1_PIN, MS1_STATE);
  digitalWrite(MS2_PIN, MS2_STATE);
}

long mmToSteps(float mm) {
  return lround(mm * STEPS_PER_MM);  //math library function to convert float to long
}

void goToMm(float mm) {
  commandMoveTo(mmToSteps(mm));
}

float positionDiffMm() {
  if (!stepper) return 0.0f;
  float stepperMm = stepper->getCurrentPosition() / STEPS_PER_MM;
  return stepperMm - railEncoder.positionMm();
}

bool positionMismatchDetected() {
  return fabs(positionDiffMm()) > POSITION_MISMATCH_TOLERANCE_MM;
}

// Closed-loop state. clSetpointMm is the position (shared mm frame) the encoder should
// reach/hold. clInHold latches a hold setpoint once on entry (see closedLoopHold). For a
// travel-to-endpoint the endpoint branches set clSetpointMm in BOTH control modes so the
// telemetry sp=/cerr= fields log the same target open- and closed-loop (comparable A/B
// data); the hold latch and all motion effects are closed-loop only.
float clSetpointMm = 0.0f;
bool  clInHold = false;

// Closed-loop proportional position servo. Called only from the CLOSED_LOOP branches of
// the state machine. Drives the ENCODER onto setpointMm by nudging the stepper's moveTo
// target (reuses the command cache via commandMoveTo -- never touches FastAccelStepper
// directly). Self-throttled to CL_UPDATE_INTERVAL_MS; between ticks the last commanded
// target simply holds. Deadband suppresses hold dither. maxCorrectionMm clamps the per-tick
// target adjustment: callers pass the tight CL_MAX_CORRECTION_MM for HOLD (target stays near
// actual -> no dither/ripple) and the large CL_TRAVEL_MAX_CORRECTION_MM for jog TRAVEL (clamp
// non-binding -> lookahead is CL_KP*err, so the move reaches JOG_SPEED then decelerates
// cleanly into the endpoint as err shrinks). A tight clamp on travel would cap cruise at
// sqrt(2*accel*clamp), far below JOG_SPEED.
void closedLoopServoTo(float setpointMm, uint32_t speedHz, float maxCorrectionMm) {
  if (!stepper) return;
  static unsigned long lastTick = 0;
  if (millis() - lastTick < CL_UPDATE_INTERVAL_MS) return;
  lastTick = millis();

  float errMm = setpointMm - railEncoder.positionMm();   // drive the encoder to setpoint
  if (fabsf(errMm) <= CL_DEADBAND_MM) return;             // in band: hold last target

  float corrMm = CL_KP * errMm;
  if (corrMm >  maxCorrectionMm) corrMm =  maxCorrectionMm;
  if (corrMm < -maxCorrectionMm) corrMm = -maxCorrectionMm;

  commandMoveTo(stepper->getCurrentPosition() + mmToSteps(corrMm), speedHz);
}

// Closed-loop "stop and hold here". First entry: if still moving, ramp to a natural stop
// exactly like the open-loop commandVelocity(0) (so the stopping feel is identical and the
// carriage doesn't snap back); only once actually stopped does it latch that rest position
// as the hold setpoint. Thereafter it servos the encoder to that setpoint to reject drift.
void closedLoopHold() {
  if (!stepper) return;
  if (!clInHold) {
    if (stepper->isRunning()) {
      commandVelocity(0);                       // decelerate to a stop, same as open loop
      return;                                   // latch only after it has actually stopped
    }
    clSetpointMm = railEncoder.positionMm();     // hold where it came to rest
    clInHold = true;
  }
  closedLoopServoTo(clSetpointMm, CL_CORRECTION_SPEED_HZ, CL_MAX_CORRECTION_MM);  // HOLD: tight clamp
}

enum class Mode {
  HOMING,
  IDLE,
  JOGGING,
  TESTING,
  ERROR,

};

// Sub-phases of Mode::TESTING (the automated acceleration ramp). Mirrors HomingPhase: a small
// non-blocking sub-state machine driven from the TESTING case in loop(). WARMUP oscillates to warm
// the mechanics (rows tagged so they can be dropped); MOVING drives one ramp move; SETTLING dwells
// after each move; FINISHED returns to IDLE. See the TEST CONFIG block and docs/TEST_BATTERY.md.
enum class TestPhase {
  WARMUP,
  MOVING,
  SETTLING,
  FINISHED,
};

enum class HomingPhase {
  FAST_APPROACH,
  BACK_OFF,
  SLOW_APPROACH,
  STOP_SETTLE,
  SET_ZERO,
  HOMING_FINISHED,
};

enum class ErrorMode {
  NO_ERROR,
  HOMING_TIMEOUT,
  POSITION_MISMATCH,
  ENCODER_NOT_DETECTED,
  OVER_TRAVEL,
  STEPPER_INIT_FAILED,
  UNKNOWN
};

// Open-loop step counting is what drives motion today; the encoder is verification only. This flips
// to CLOSED_LOOP once the toggleable closed-loop position mode lands -- printed as ctrl= so logs
// record which controller was active.
enum class ControlMode { OPEN_LOOP, CLOSED_LOOP };
ControlMode controlMode = ControlMode::OPEN_LOOP; //THIS IS MODE TOGGLE -- THIS IS MODE TOGGLE 

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
  const char* testPhaseText(TestPhase currentPhase);
  const char* errorModeText(ErrorMode currentError);
  const char* controlModeText(ControlMode m);
  void printStatus();
//CGPT ADDITION - FOR PRINTING ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Mode mode = Mode::HOMING;
HomingPhase homingPhase = HomingPhase::FAST_APPROACH;
ErrorMode errorMode = ErrorMode::NO_ERROR;

// Test-sequence state (Mode::TESTING). testPhase is the sub-state; testMoveIndex counts ramp moves
// 0..TEST_NUM_MOVES-1; testTargetIsFar toggles the endpoint each move; testCurrentAccelHz2 is the
// accel commanded for the current move (also mirrored into motionAccelStepsPerS2 and logged).
TestPhase testPhase = TestPhase::WARMUP;
int testMoveIndex = 0;
bool testTargetIsFar = true;
uint32_t testCurrentAccelHz2 = 0;
bool testStartLatched = false;   // require the start button to be released before a new run can arm

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

// Reuses phaseStartTime/phaseTimer (HOMING and TESTING never run at the same time) so the test
// sub-state machine gets the same non-blocking dwell timing as homing.
void changeTestPhase(TestPhase nextPhase) {
  testPhase = nextPhase;
  phaseStartTime = millis();
}

// Drive one automated test move toward targetMm, honoring the control mode -- the SAME OL/CL idiom as
// the JOGGING endpoints, so a test move behaves exactly like a jog to that endpoint (and sp/cerr log
// the target in both modes). speedHz is the cruise speed; the per-move acceleration is applied
// separately via setMotionAccel() in testBeginMove(). Called every iteration while MOVING (the OL
// commandMoveTo is cached; the CL servo needs the repeated call).
void testDriveTo(float targetMm, uint32_t speedHz) {
  clInHold = false;
  clSetpointMm = targetMm;
  if (controlMode == ControlMode::CLOSED_LOOP) {
    closedLoopServoTo(clSetpointMm, speedHz, CL_TRAVEL_MAX_CORRECTION_MM);  // TRAVEL: large clamp
  } else {
    commandMoveTo(mmToSteps(targetMm), speedHz);
  }
}

// A move is complete once the stepper has stopped; in closed-loop also require the encoder to be
// within the servo deadband of the target so a genuinely-settled endpoint is what advances the ramp.
bool testArrived(float targetMm) {
  if (!stepper) return true;
  if (stepper->isRunning()) return false;
  if (controlMode == ControlMode::CLOSED_LOOP) {
    return fabsf(targetMm - railEncoder.positionMm()) <= CL_DEADBAND_MM;
  }
  return true;
}

// Start ramp move testMoveIndex: compute its linear-ramp acceleration, push it to the stepper (takes
// effect on the next planned move), and enter MOVING. accel(move i) = TEST_ACCEL_START + i*TEST_ACCEL_STEP.
void testBeginMove() {
  testCurrentAccelHz2 = TEST_ACCEL_START + (uint32_t)testMoveIndex * TEST_ACCEL_STEP;
  setMotionAccel(testCurrentAccelHz2);
  changeTestPhase(TestPhase::MOVING);
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
    const unsigned long PRINT_INTERVAL_MS = 100;  // PLACEHOLDER: choose print rate

  const char* modeText(Mode currentMode) {
    switch (currentMode) {
      case Mode::HOMING:
        return "HOMING";  // PLACEHOLDER MESSAGE

      case Mode::IDLE:
        return "IDLE";  // PLACEHOLDER MESSAGE

      case Mode::JOGGING:
        return "JOGGING";

      case Mode::TESTING:
        return "TESTING";

      case Mode::ERROR:
        return "ERROR";  // PLACEHOLDER MESSAGE

      default:
        return "UNKNOWN MODE";
    }
  }

  const char* testPhaseText(TestPhase currentPhase) {
    switch (currentPhase) {
      case TestPhase::WARMUP:
        return "WARM";

      case TestPhase::MOVING:
        return "MOVE";

      case TestPhase::SETTLING:
        return "SET";

      case TestPhase::FINISHED:
        return "DONE";

      default:
        return "UNK_PHASE";
    }
  }

  const char* homingPhaseText(HomingPhase currentPhase) {
    switch (currentPhase) {
      case HomingPhase::FAST_APPROACH:
        return "FAST";

      case HomingPhase::BACK_OFF:
        return "BACK";

      case HomingPhase::SLOW_APPROACH:
        return "SLOW";

      case HomingPhase::STOP_SETTLE:
        return "SETTLE";

      case HomingPhase::SET_ZERO:
        return "ZERO";

      case HomingPhase::HOMING_FINISHED:
        return "DONE";

      default:
        return "UNK_PHASE";
    }
  }

  const char* errorModeText(ErrorMode currentError) {
    switch (currentError) {
      case ErrorMode::NO_ERROR:
        return "OK";

      case ErrorMode::HOMING_TIMEOUT:
        return "TIMEOUT";

      case ErrorMode::POSITION_MISMATCH:
        return "MISMATCH";

      case ErrorMode::ENCODER_NOT_DETECTED:
        return "NO_ENC";

      case ErrorMode::OVER_TRAVEL:
        return "OVERTRVL";

      case ErrorMode::STEPPER_INIT_FAILED:
        return "STP_FAIL";

      case ErrorMode::UNKNOWN:
        return "UNK";

      default:
        return "UNK_ERR";
    }
  }

  const char* controlModeText(ControlMode m) {
    return (m == ControlMode::CLOSED_LOOP) ? "CL" : "OL";
  }
//CGPT ADDITION - FOR PRINTING ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//CGPT ADDITION - FOR PRINTING................................................
  // Telemetry CSV column header, printed once at boot from setup(). Kept adjacent to printStatus() so
  // the column names and the snprintf field order below stay together -- update BOTH (and
  // docs/TELEMETRY.md) if you add, rename, or reorder a field.
  static const char* TELEMETRY_CSV_HEADER =
    "# t,ctrl,mode,phase,err,pos,vsps,vmms,enc,dpos,dmax,sp,cerr,mis,econn,eerr,lim,led,accel";

  void printStatus() {
    static unsigned long lastPrintTime = 0;

    // Faster sampling while a test runs (TEST_PRINT_INTERVAL_MS) so accel/decel transients are
    // resolved; normal operation stays at PRINT_INTERVAL_MS.
    unsigned long interval = (mode == Mode::TESTING) ? TEST_PRINT_INTERVAL_MS : PRINT_INTERVAL_MS;
    if (millis() - lastPrintTime < interval) {
      return;
    }

    lastPrintTime = millis();

    // One snprintf + one Serial.println instead of ~40 Serial.print calls: fits on a single terminal
    // line, and fewer trips through the TX path. Condensed to values-only CSV (no key= prefixes) to
    // minimize print cost for logging; field 1 is a millis() timestamp. The column legend is printed
    // once at boot (TELEMETRY_CSV_HEADER above) and in docs/TELEMETRY.md -- keep all three in lockstep.
    // Packed flag columns: lim = <near><far> (2 digits), led = <home><idle><err> (3 digits).
    // %f is safe here -- ESP32/newlib has full printf float support.
    float posMm  = stepper ? stepper->getCurrentPosition() / STEPS_PER_MM : 0.0f;
    float velSps = stepper ? stepper->getCurrentSpeedInMilliHz() / 1000.0f : 0.0f;
    float velMms = velSps / STEPS_PER_MM;
    float encMm  = railEncoder.positionMm();

    char buf[256];
    snprintf(buf, sizeof(buf),
      "%lu,%s,%s,%s,%s,"
      "%.2f,%.1f,%.2f,%.2f,%.3f,%.3f,"
      "%.2f,%.3f,"
      "%d,%d,%d,%d%d,%d%d%d,"
      "%lu",
      millis(),
      controlModeText(controlMode), modeText(mode),
      (mode == Mode::HOMING)  ? homingPhaseText(homingPhase) :
      (mode == Mode::TESTING) ? testPhaseText(testPhase)     : "-",
      (mode == Mode::ERROR)  ? errorModeText(errorMode)     : "-",
      posMm, velSps, velMms, encMm, posMm - encMm, maxAbsPositionDiffMm,
      clSetpointMm, clSetpointMm - encMm,
      positionMismatchDetected() ? 1 : 0,
      railEncoder.isConnected() ? 1 : 0, railEncoder.lastError(),
      homeButtonPressed() ? 1 : 0, farLimitPressed() ? 1 : 0,
      (digitalRead(HOME_LED_PIN)   == HIGH) ? 1 : 0,
      (digitalRead(IDLE_LED_PIN)   == HIGH) ? 1 : 0,
      (digitalRead(ERROR_LED_PIN1) == HIGH) ? 1 : 0,
      (unsigned long)motionAccelStepsPerS2);  // accel: live commanded acceleration, steps/s^2 (mm/s^2 = /STEPS_PER_MM)

    Serial.println(buf);
  }
//CGPT ADDITION - FOR PRINTING ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


void setup() {

  delay(1000); // Delay homing routine for monitoring and upload stability
  Serial.setTxBufferSize(1024);  // background TX ring buffer so Serial.print never blocks loop() (starves step timing otherwise)
  Serial.begin(115200);
  Serial.println(TELEMETRY_CSV_HEADER);  // one-time CSV column header for the telemetry stream (see printStatus())

  pinMode(HOMING_PIN, INPUT_PULLUP);
  pinMode(FAR_LIMIT_PIN, INPUT_PULLUP);
  pinMode(POS_JOG_PIN, INPUT_PULLUP);
  pinMode(NEG_JOG_PIN, INPUT_PULLUP);
  pinMode(TEST_START_PIN, INPUT_PULLUP);

  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);

  pinMode(HOME_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN1, OUTPUT);
  pinMode(IDLE_LED_PIN, OUTPUT);
  homeLight(false);


  microStep(ACTIVE_MICROSTEP.ms1, ACTIVE_MICROSTEP.ms2);  // pins derived from the single microstep config

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) {
    changeErrorMode(ErrorMode::STEPPER_INIT_FAILED);
    changeState(Mode::ERROR);
    return;                                        // no stepper -> don't attempt homing
  }
  // dirHighCountsUp = false matches the old setPinsInverted(true, ...): DIR is inverted so that
  // positive step counts move AWAY from the home switch. VERIFY ON BENCH before first homing --
  // if the carriage drives away from home, flip this boolean (see docs/ARCHITECTURE.md calibration).
  stepper->setDirectionPin(DIR_PIN, false);
  stepper->setAutoEnable(false);                   // no enable pin wired
  setMotionAccel(motionAccelStepsPerS2);           // apply consolidated accel tunable to the stepper

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
      commandHalt();
      changeErrorMode(ErrorMode::OVER_TRAVEL);
      changeState(Mode::ERROR);
    }
  }

  switch (mode) {

    case Mode::HOMING:
      switch (homingPhase) {

        case HomingPhase::FAST_APPROACH:
          if (homeButtonPressed()) {
            // Hard stop at the switch. Do NOT reverse via a velocity command here: that would
            // decelerate over v^2/(2a) (~2.4mm at HOMING_SPEED_FAST) PAST the switch and rack the
            // carriage into the mechanical end. forceStop() abandons the ramp and stops within the
            // hardware queue. BACK_OFF issues the reversal on the next iteration.
            commandHalt();
            changeHomingPhase(HomingPhase::BACK_OFF);
          } else if (phaseTimer(HOMING_TIMEOUT)) {
            changeErrorMode(ErrorMode::HOMING_TIMEOUT);
            changeState(Mode::ERROR);
          } else {
            commandVelocity(HOMING_SPEED_FAST);
          }
          break;

        case HomingPhase::BACK_OFF:
          // Wait out HOMING_SETTLE_MS so the fast-approach forceStop fully drains before commanding the
          // backoff -- otherwise runForward() lands in a draining forceStop and is dropped (and the
          // command cache never retries it), so the carriage never backs off. No forceStop on exit:
          // SLOW_APPROACH reverses smoothly from the backoff velocity.
          if (phaseTimer(BACKOFF_DURATION_MS)) {
            changeHomingPhase(HomingPhase::SLOW_APPROACH);
          } else if (phaseTimer(HOMING_SETTLE_MS)) {
            commandVelocity(BACKOFF_SPEED);
          }
          break;

        case HomingPhase::SLOW_APPROACH:
          if (homeButtonPressed()) {
            commandHalt();                                     // hard stop at the trigger point
            changeHomingPhase(HomingPhase::STOP_SETTLE);       // reposition only after forceStop drains
          } else if (phaseTimer(HOMING_TIMEOUT)) {
            changeErrorMode(ErrorMode::HOMING_TIMEOUT);
            changeState(Mode::ERROR);
          } else {
            commandVelocity(HOMING_SPEED_SLOW);
          }
          break;

        case HomingPhase::STOP_SETTLE:
          // After the settle window the motor is idle, so setCurrentPosition (safe only when stopped)
          // and the pull-off moveTo land cleanly rather than being dropped into the draining forceStop.
          if (phaseTimer(HOMING_SETTLE_MS)) {
            stepper->setCurrentPosition(mmToSteps(-HOME_PULL_OFF_MM));  // define trigger point as -5mm
            goToMm(0);                                                  // pull off to zero
            changeHomingPhase(HomingPhase::SET_ZERO);
          }
          break;

        case HomingPhase::SET_ZERO:
          if (!stepper->isRunning()) {                         // pull-off move to 0 complete
            stepper->setCurrentPosition(0);
            railEncoder.zero();
            maxAbsPositionDiffMm = 0.0f;
            clSetpointMm = 0.0f; clInHold = false;   // re-latch the closed-loop hold on next IDLE entry
            changeHomingPhase(HomingPhase::HOMING_FINISHED);
          }
          break;

        case HomingPhase::HOMING_FINISHED:
          if (phaseTimer(HOME_LED_DURATION_MS)) {
            homeLight(LOW);
            changeState(Mode::IDLE);                           // hold at 0mm (5mm off the switch)
          } else {
            homeLight(HIGH);
          }
          break;
      }

      break;

    case Mode::IDLE:
      idleLight(HIGH);
      if (controlMode == ControlMode::CLOSED_LOOP) {
        closedLoopHold();                        // hold position under encoder feedback
      } else {
        commandVelocity(0);                      // OPEN-LOOP: ensure stopped; hardware holds position when idle
      }
        if (!testStartPressed()) testStartLatched = false;  // re-arm once the button is released
        if (posJogPressed() || negJogPressed()) {
          changeState(Mode::JOGGING);
          idleLight(LOW);
        } else if (testStartPressed() && !testStartLatched) {
          // Start the automated acceleration test (latched so one press = one run, and a still-held
          // button after the run finishes doesn't immediately restart it).
          testStartLatched = true;
          idleLight(LOW);
          testMoveIndex = 0;
          testTargetIsFar = true;                // ramp starts heading to the far end
          clInHold = false;
          changeState(Mode::TESTING);
          if (TEST_WARMUP_ENABLED) {
            setMotionAccel(TEST_WARMUP_ACCEL_HZ2);
            testCurrentAccelHz2 = TEST_WARMUP_ACCEL_HZ2;
            changeTestPhase(TestPhase::WARMUP);
          } else {
            testBeginMove();                     // skip warm-up: set move-0 accel and go straight to MOVING
          }
        }
      break;

    case Mode::JOGGING:
        // Jog by moving TO the soft limit at JOG_SPEED rather than running at constant velocity and
        // reacting once the limit is crossed. FastAccelStepper plans the deceleration to land exactly
        // on the target, so the carriage decelerates INTO the soft limit and never overshoots into the
        // switch -- at any jog speed. (The old velocity+reactive-stop overshot by v^2/(2a), which grew
        // with jog speed until it reached the switch.) Held mid-travel it cruises at JOG_SPEED toward
        // the limit; released mid-travel it ramps to a stop.
        // Endpoint branches set clSetpointMm in BOTH modes so telemetry (sp=/cerr=) logs the
        // same target open- and closed-loop; the CLOSED_LOOP branch additionally servos the
        // encoder onto it while the OL statements stay byte-for-byte the original behavior.
        // Hold cases go through closedLoopHold() (CL) / commandVelocity(0) (OL). "Both
        // buttons" stays the first-checked stop (a deliberate safety stop).
        if (posJogPressed() && negJogPressed()) {
          if (controlMode == ControlMode::CLOSED_LOOP) {
            closedLoopHold();
          } else {
            commandVelocity(0);
          }
        } else if (posJogPressed()) {
          clInHold = false; clSetpointMm = JOG_MAX_POSITION_MM;
          if (controlMode == ControlMode::CLOSED_LOOP) {
            closedLoopServoTo(clSetpointMm, (uint32_t)JOG_SPEED, CL_TRAVEL_MAX_CORRECTION_MM);  // TRAVEL: large clamp
          } else {
            commandMoveTo(mmToSteps(JOG_MAX_POSITION_MM), (uint32_t)JOG_SPEED);
          }
        } else if (negJogPressed()) {
          clInHold = false; clSetpointMm = JOG_MIN_POSITION_MM;
          if (controlMode == ControlMode::CLOSED_LOOP) {
            closedLoopServoTo(clSetpointMm, (uint32_t)JOG_SPEED, CL_TRAVEL_MAX_CORRECTION_MM);  // TRAVEL: large clamp
          } else {
            commandMoveTo(mmToSteps(JOG_MIN_POSITION_MM), (uint32_t)JOG_SPEED);
          }
        } else {
          if (controlMode == ControlMode::CLOSED_LOOP) {
            closedLoopHold();                    // ramp to a stop, then hold, back to IDLE
          } else {
            commandVelocity(0);                  // release -> ramp to a stop and hold, back to IDLE
          }
          changeState(Mode::IDLE);
        }
        break;

    case Mode::TESTING:
        // Automated acceleration / step-loss ramp -- see the TEST CONFIG block and docs/TEST_BATTERY.md.
        // Non-blocking sub-state machine (WARMUP -> MOVING/SETTLING per move -> FINISHED). The
        // over-travel failsafe above still guards this mode; dmax and encoder health are tracked in the
        // post-switch guard below. Both jog buttons = manual abort back to IDLE (safety stop).
        if (posJogPressed() && negJogPressed()) {
          commandHalt();
          changeState(Mode::IDLE);
          break;
        }
        switch (testPhase) {
          case TestPhase::WARMUP: {
            if (phaseTimer(TEST_WARMUP_DURATION_MS)) {
              // Duration elapsed: come to a FULL STOP before the recorded ramp. A ramp move issued
              // while the carriage is still moving toward the far end can't decelerate at the low
              // move-0 accel and overshoots the soft limit into the switch (see docs/PROBLEM_LOG.md).
              // Start move 0 only from rest -- same rest guarantee SETTLING gives inter-move starts.
              commandVelocity(0);
              if (!stepper->isRunning()) {
                testMoveIndex = 0;
                testTargetIsFar = true;
                testBeginMove();                  // begin the recorded ramp from rest
              }
            } else {
              // Gentle back-and-forth to warm the mechanics; rows tag phase=WARM so they drop in post.
              float target = testTargetIsFar ? TEST_FAR_MM : TEST_NEAR_MM;
              testDriveTo(target, TEST_WARMUP_SPEED_HZ);
              if (testArrived(target)) testTargetIsFar = !testTargetIsFar;  // bounce to the other end
            }
            break;
          }
          case TestPhase::MOVING: {
            float target = testTargetIsFar ? TEST_FAR_MM : TEST_NEAR_MM;
            testDriveTo(target, TEST_CRUISE_SPEED_HZ);
            // Floor: don't test arrival until the just-issued move has had time to start (isRunning()
            // latches true), so the first MOVING iteration can't be misread as already-arrived. Every
            // ramp move spans near<->far (hundreds of ms), so this floor is never the limiting factor.
            if (phaseTimer(50) && testArrived(target)) changeTestPhase(TestPhase::SETTLING);
            break;
          }
          case TestPhase::SETTLING:
            // Sit stopped so a clean, settled sample is logged before advancing accel.
            if (controlMode == ControlMode::CLOSED_LOOP) closedLoopHold(); else commandVelocity(0);
            if (phaseTimer(TEST_SETTLE_MS)) {
              testMoveIndex++;
              testTargetIsFar = !testTargetIsFar;   // next move heads to the opposite end
              if (testMoveIndex >= TEST_NUM_MOVES) {
                changeTestPhase(TestPhase::FINISHED);
              } else {
                testBeginMove();
              }
            }
            break;
          case TestPhase::FINISHED:
            if (controlMode == ControlMode::CLOSED_LOOP) closedLoopHold(); else commandVelocity(0);
            changeState(Mode::IDLE);             // sequence complete -> back to normal hold
            break;
        }
        break;

    case Mode::ERROR:
      commandHalt();

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
        case ErrorMode::STEPPER_INIT_FAILED:
          errorLight1(HIGH);
          break;
        case ErrorMode::UNKNOWN:

          break;
      }
      break;
  }

  if (mode == Mode::IDLE || mode == Mode::JOGGING || mode == Mode::TESTING) {
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