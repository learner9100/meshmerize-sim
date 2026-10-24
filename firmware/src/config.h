#pragma once
// Everything about YOUR robot lives in this file: pins, sensor type, motor
// driver wiring, mechanical numbers and tuning values. The rest of the
// firmware only reads these constants. If a value is marked MEASURE, the
// serial menu has a test that tells you what to put there.

// ------------------------------------------------------------------ sensors
// Reflectance array across the front of the robot, left to right as seen
// from behind the robot. Analog (QTR-8A style) by default; set IR_ANALOG to
// false for arrays that give a digital 0/1 per sensor.
constexpr int  IR_COUNT = 8;
constexpr int  IR_PIN[IR_COUNT] = {36, 39, 34, 35, 32, 33, 25, 26};
constexpr bool IR_ANALOG = true;
constexpr int  IR_EMITTER_PIN = -1;        // LEDON / enable pin of the array, -1 if wired always on
constexpr bool IR_LINE_READS_HIGH = false; // QTR-8A reads LOW on white line; set true if your array is the other way
constexpr int  IR_SPACING_MM = 8;          // centre-to-centre distance between neighbouring sensors
constexpr int  IR_TO_AXLE_MM = 60;         // MEASURE: sensor row to wheel axle, so the robot can stop with the wheels on the junction
constexpr int  IR_ON_LINE = 500;           // normalised 0..1000 value above which a sensor counts as "on the line"

// ------------------------------------------------------------------ motors (TB6612FNG)
constexpr int  M_LEFT_PWM  = 16;
constexpr int  M_LEFT_IN1  = 17;
constexpr int  M_LEFT_IN2  = 5;
constexpr int  M_RIGHT_PWM = 4;
constexpr int  M_RIGHT_IN1 = 18;
constexpr int  M_RIGHT_IN2 = 19;
constexpr int  M_STBY      = 21;
constexpr bool M_LEFT_REVERSED  = false;   // flip if the motor test shows a wheel going backwards
constexpr bool M_RIGHT_REVERSED = false;
constexpr int  PWM_FREQ_HZ = 20000;
constexpr int  PWM_BITS    = 10;           // speeds below are 0..1023
constexpr int  PWM_MAX     = (1 << PWM_BITS) - 1;

// ------------------------------------------------------------------ encoders
constexpr int  E_LEFT_A  = 23;
constexpr int  E_LEFT_B  = 22;
constexpr int  E_RIGHT_A = 27;
constexpr int  E_RIGHT_B = 13;
constexpr bool E_LEFT_REVERSED  = false;   // flip if counts go down while the wheel drives forward
constexpr bool E_RIGHT_REVERSED = false;
constexpr float COUNTS_PER_WHEEL_REV = 700.0f;   // MEASURE with the encoder test
constexpr float WHEEL_DIAMETER_MM    = 32.0f;    // MEASURE
constexpr float WHEEL_BASE_MM        = 90.0f;    // MEASURE: distance between the two wheel contact points

// ------------------------------------------------------------------ buttons and lights
constexpr int BUTTON_PIN     = 0;          // BOOT button on most dev boards, active low
constexpr int LED_RED_PIN    = 15;         // the "I am at End" LED the rules ask for
constexpr int LED_STATUS_PIN = 2;          // onboard blue LED on most boards
constexpr int BUZZER_PIN     = -1;         // optional

// ------------------------------------------------------------------ driving
constexpr int   SPEED_DRY   = 450;         // base PWM while exploring
constexpr int   SPEED_RUN   = 750;         // base PWM in the Actual Run
constexpr int   SPEED_TURN  = 400;         // wheel PWM while turning in place
constexpr int   SPEED_MIN   = 120;         // below this the motors stall
constexpr float LINE_KP = 0.9f;            // line-following PID on the -1000..1000 line position
constexpr float LINE_KI = 0.0f;
constexpr float LINE_KD = 6.0f;
constexpr int   LOOP_MS = 5;               // control loop period

// ------------------------------------------------------------------ maze reading
constexpr int  NODE_SIDE_SENSORS = 2;      // outer sensors on one side lit = a branch to that side
constexpr int  NODE_CONFIRM_MM   = 15;     // keep seeing the branch over this distance before believing it
constexpr int  DEAD_END_MM       = 20;     // all sensors dark over this distance = dead end
constexpr int  END_MARK_MM       = 60;     // all sensors lit over this distance = End marker (assumption, verify at the venue)
constexpr int  TURN_SETTLE_MS    = 60;
constexpr int  TURN_OVERSHOOT_DEG = 10;    // turn slightly past 90 and let the line pull the robot straight

// ------------------------------------------------------------------ maze model (must match the simulator)
constexpr int CELL_MM  = 230;              // assumption until the organisers say otherwise
constexpr int MERGE_MM = 100;              // two positions closer than this are the same node
constexpr int START_HEADING = 1;           // which way the robot faces at Start: 0 N, 1 E, 2 S, 3 W
constexpr int START_EXITS   = 2;           // exits at Start as seen by the robot: bit0 left, bit1 straight, bit2 right

// Which exploration strategy the robot runs. Same names as `make compare`.
enum class Policy { FirstPath, DfsFull, NearestFull, Prune, PruneGrid, PruneGuard, NearestGuard };
constexpr Policy POLICY = Policy::PruneGrid;

// Robot timing the explorer uses for its time guard. Measure with the
// calibration procedure in the README and keep robot.cfg in the simulator
// in sync, so what you tested on the PC is what the robot believes.
constexpr int DRY_MM_PER_S = 500;
constexpr int DRY_NODE_MS  = 300;
constexpr int DRY_TURN_MS  = 400;
constexpr int DRY_UTURN_MS = 1000;
constexpr int DRY_LIMIT_MS = 180000;
constexpr int DRY_MARGIN_MS = 20000;
