    /*================================================================
    *  LINE FOLLOWING ROBOT — Competition Code
    *  ESP32-WROOM + L298N + 2x IR Sensors
    *  
    *  Board: ESP32-WROOM-DA Module (Arduino IDE)
    *  
    *  ═══════════════════════════════════════════════════════════
    *  SENSOR PLACEMENT GUIDE (CRITICAL FOR SUCCESS)
    *  ═══════════════════════════════════════════════════════════
    *  
    *  Line width: 1 inch (25.4 mm)
    *  
    *  Recommended sensor center-to-center spacing: 30-35 mm
    *  (We recommend 32 mm — about 1.26 inches)
    *  
    *  WHY 32 mm?
    *  - Each sensor sits ~3-4 mm OUTSIDE the edge of the 1" line
    *  - When centered: both sensors see WHITE (line is between them)
    *  - When drifting: one sensor crosses onto the BLACK line
    *  - Close enough to catch sharp turns before overshooting
    *  - Far enough that both don't trigger on straight sections
    *  
    *  POSITION ON THE CHASSIS:
    *  - Mount sensors at the FRONT of the robot, 10-15 mm ahead
    *    of the front rotating wheel (as far forward as possible)
    *  - Sensors should be 3-5 mm above the ground surface
    *  - Both sensors must be at EQUAL height from ground
    *  - Sensors should be SYMMETRICAL about the robot centerline
    *    (16 mm left of center, 16 mm right of center)
    *  
    *  TOP VIEW OF SENSOR PLACEMENT:
    *  
    *         [LEFT_IR]---32mm---[RIGHT_IR]
    *              |     16+16     |
    *              |   centerline  |
    *              |       |       |
    *         ═════╤═══════╤═══════╤═════  ← front edge
    *              |       |       |
    *          [LEFT WHEEL]   [RIGHT WHEEL]
    *  
    *  CALIBRATION CHECKLIST:
    *  1. Place sensor over WHITE surface → adjust pot until LED OFF
    *  2. Place sensor over BLACK line → confirm LED ON
    *  3. Signal: 0 = White, 1 = Black (as per your setup)
    *  4. Test at the EXACT height you'll mount them
    *  
    *  ═══════════════════════════════════════════════════════════
    *  WIRING REFERENCE (Your Setup)
    *  ═══════════════════════════════════════════════════════════
    *  
    *  Power: 2x 18650 → Switch → L298N 12V terminal
    *         L298N GND → Battery negative
    *         L298N 5V  → ESP32 VIN
    *         L298N GND → ESP32 GND (common ground)
    *  
    *  Motors:
    *    Right Motor → OUT1 & OUT2
    *    Left Motor  → OUT3 & OUT4
    *    ENA & ENB   → Jumpers ON (full voltage to motors)
    *  
    *  Logic (ESP32 → L298N):
    *    D12 → IN1 (Right Motor Forward)
    *    D13 → IN2 (Right Motor Backward)
    *    D25 → IN3 (Left Motor Forward)
    *    D26 → IN4 (Left Motor Backward)
    *  
    *  IR Sensors:
    *    VCC → ESP32 3.3V
    *    GND → ESP32 GND
    *    Left Sensor OUT  → D32
    *    Right Sensor OUT → D33
    *  
    *  ═══════════════════════════════════════════════════════════
    *  COMPETITION CHALLENGES HANDLED
    *  ═══════════════════════════════════════════════════════════
    *  
    *  1. Straight lines           → Both white, go straight
    *  2. Sharp 90° L-turns        → Detect via sensor, then spin-search
    *  3. Zigzag patterns          → Rapid left/right corrections
    *  4. Intersections (cross)    → Both black, go straight through
    *  5. Zigzag intersections     → Go straight through, resume zigzag
    *  6. Intersection after turn  → Brake to stabilize, then straight
    *  
    *  STRATEGY:
    *  - Sensors STRADDLE the line (both white when centered)
    *  - One sensor hits black → correct toward that side
    *  - Both sensors hit black → intersection → go STRAIGHT
    *  - Both sensors white too long → line lost → SPIN to recover
    *  - After completing a turn, brief brake before next decision
    *    (prevents false turns at immediate intersections)
    *  
    *  SPEED CONTROL:
    *  - Even though ENA/ENB jumpers are ON, we achieve speed
    *    control by sending PWM signals through IN1-IN4 pins.
    *    The L298N responds to PWM on these pins.
    *  
    *================================================================*/

    // ──────────────────────────────────────────────────────────────
    //  PIN DEFINITIONS
    // ──────────────────────────────────────────────────────────────
    #define IN1       12    // Right Motor Forward
    #define IN2       13    // Right Motor Backward
    #define IN3       25    // Left Motor Forward
    #define IN4       26    // Left Motor Backward

    #define IR_LEFT   32    // Left IR Sensor  (0=White, 1=Black)
    #define IR_RIGHT  33    // Right IR Sensor (0=White, 1=Black)

    // ──────────────────────────────────────────────────────────────
    //  PWM CONFIGURATION (LEDC)
    // ──────────────────────────────────────────────────────────────
    #define PWM_FREQ      5000   // 5 kHz PWM frequency
    #define PWM_RESOLUTION   8   // 8-bit resolution (0-255)

    // Note: ESP32 Arduino Core 3.x uses pin-based LEDC API
    // No channel assignments needed — ledcWrite() takes the pin directly

    // ──────────────────────────────────────────────────────────────
    //  TUNABLE SPEED PARAMETERS
    //  *** Adjust these values based on your track and surface ***
    // ──────────────────────────────────────────────────────────────
    #define BASE_SPEED          150   // Straight-line cruise speed
    #define TURN_OUTER_SPEED    180   // Outer wheel during correction
    #define TURN_INNER_SPEED     70   // Inner wheel during correction (Increased for torque)
    #define SHARP_SPIN_SPEED    160   // Spin speed for 90° turn recovery
    #define SCAN_SPEED          130   // Slow speed for scanning
    #define INTERSECTION_SPEED  120   // Speed through intersections
    #define BRAKE_PULSE_MS       50   // Reverse-pulse duration for hard brake

    // ──────────────────────────────────────────────────────────────
    //  TIMING PARAMETERS (milliseconds)
    // ──────────────────────────────────────────────────────────────
    #define LINE_LOST_THRESHOLD  150  // ms of (0,0) before entering SCAN
    #define SCAN_PAUSE_MS        400  // Pause to stabilize before scanning
    #define SCAN_SHORT_MS        625  // Small look (e.g., center to 7:30 o'clock / 4:30 o'clock = 150° total)
    #define SCAN_FULL_MS         1250 // Full sweep (e.g., 7:30 o'clock to 4:30 o'clock = 150° sweep)
    #define SCAN_MAX_STEPS       6    // Max times to look back and forth
    #define TURN_COOLDOWN_MS     300  // After a turn, ignore intersections
    #define INTERSECTION_FWD_MS  100  // Drive straight into intersection

    // ──────────────────────────────────────────────────────────────
    //  GRADUATED SCAN TIMING (milliseconds for each angle degree)
    //  Calculated from: SCAN_FULL_MS / 150° ≈ 8.33 ms per degree
    //  Updated for 30° increments: stops at 120°
    // ──────────────────────────────────────────────────────────────
    #define SCAN_TIME_30_DEG     250   // ~30° scan
    #define SCAN_TIME_60_DEG     500   // ~60° scan
    #define SCAN_TIME_90_DEG     750   // ~90° scan
    //#define SCAN_TIME_120_DEG    1000  // ~120° scan (maximum)
    // ──────────────────────────────────────────────────────────────
    enum State {
    ST_FOLLOW,          // Normal line following
    ST_CORRECT_LEFT,    // Correcting leftward (Line is on the left)
    ST_CORRECT_RIGHT,   // Correcting rightward (Line is on the right)
    ST_INTERSECTION,    // Both sensors on black — drive straight
    ST_SCAN_PAUSE,      // Line lost — stop first
    ST_SCAN_LEFT,       // Line lost — look left
    ST_SCAN_RIGHT,      // Line lost — look right
    ST_BRAKE,           // Brief brake for stabilization
    ST_STOP             // Emergency stop / Line truly gone
    };

    State state     = ST_FOLLOW;
    State prevState = ST_FOLLOW;

    // ──────────────────────────────────────────────────────────────
    //  TRACKING VARIABLES
    // ──────────────────────────────────────────────────────────────
    int  lastKnownDir       = 0;     // -1 = left, 0 = center, +1 = right
    int  scanStepCount      = 0;     // How many scan sweeps performed
    unsigned long stateStartTime = 0; // Timestamp when current state started
    unsigned long turnEndTime    = 0; // Timestamp when last turn ended
    unsigned long intStart       = 0; // Timestamp when intersection began
    bool recentTurn              = false; // True if we just completed a turn
    unsigned long lineLostTime   = 0; // Track how long line has been lost
    bool lineIsLost              = false; // Immediate sensor loss flag

    // Graduated scan tracking
    int  scanAngleDegrees  = 30;    // Current scan angle: 30, 60, 90, 120
    int  scanPhase         = 0;     // 0=scanFirstDir, 1=returnFromFirst, 2=scanSecondDir, 3=returnFromSecond
    unsigned long scanStartTime = 0; // Start time for current scan phase
    int  scanFirstDir      = -1;    // Which direction to scan first: -1=left, +1=right
    int  lastSensorDetected = -1;   // Which sensor LAST detected black: -1=left, +1=right
    bool finishLineReached  = false; // TRUE = permanent stop, no restart

    // Sensor debouncing
    int prevL = 0, prevR = 0;       // Previous sensor reads
    int debounceCount = 0;          // Counter for debounce confirmation
    bool systemReady = false;       // Flag to prevent startup noise

    // ──────────────────────────────────────────────────────────────
    //  MOTOR CONTROL (PWM via IN pins)
    // ──────────────────────────────────────────────────────────────

    // Drive right motor: positive = forward, negative = backward, 0 = stop
    void motorRight(int spd) {
    if (spd > 0) {
        ledcWrite(IN1, constrain(spd, 0, 255));
        ledcWrite(IN2, 0);
    } else if (spd < 0) {
        ledcWrite(IN1, 0);
        ledcWrite(IN2, constrain(-spd, 0, 255));
    } else {
        ledcWrite(IN1, 0);
        ledcWrite(IN2, 0);
    }
    }

    // Drive left motor: positive = forward, negative = backward, 0 = stop
    void motorLeft(int spd) {
    if (spd > 0) {
        ledcWrite(IN3, constrain(spd, 0, 255));
        ledcWrite(IN4, 0);
    } else if (spd < 0) {
        ledcWrite(IN3, 0);
        ledcWrite(IN4, constrain(-spd, 0, 255));
    } else {
        ledcWrite(IN3, 0);
        ledcWrite(IN4, 0);
    }
    }

    void goStraight(int spd) {
    motorLeft(spd);
    motorRight(spd);
    }

    void correctLeft(int inner, int outer) {
    motorLeft(inner);     // slow down left (inner wheel)
    motorRight(outer);    // speed up right (outer wheel)
    }

    void correctRight(int inner, int outer) {
    motorLeft(outer);     // speed up left (outer wheel)
    motorRight(inner);    // slow down right (inner wheel)
    }

    void stopMotors() {
    motorLeft(0);
    motorRight(0);
    }

    // Quick-stop: gentle coast (no reverse pulse - prevents buzzing)
    void hardBrake() {
    motorLeft(0);
    motorRight(0);
    delay(BRAKE_PULSE_MS);
    stopMotors();
    }

    // Spin in place to find the line
    void spinSearch(int dir) {
    // dir < 0 → spin left, dir > 0 → spin right
    if (dir <= 0) {
        motorLeft(-SHARP_SPIN_SPEED);
        motorRight(SHARP_SPIN_SPEED);
    } else {
        motorLeft(SHARP_SPIN_SPEED);
        motorRight(-SHARP_SPIN_SPEED);
    }
    }

    void setState(State s) {
    state = s;
    stateStartTime = millis();
    scanStartTime = millis();  // Also reset scan phase timer
    }

    // Get scan time in milliseconds for a given angle
    int getScanTimeForAngle(int angle) {
    if (angle == 30) return SCAN_TIME_30_DEG;
    if (angle == 60) return SCAN_TIME_60_DEG;
    if (angle == 90) return SCAN_TIME_90_DEG;
  //  if (angle == 120) return SCAN_TIME_120_DEG;
    return SCAN_TIME_30_DEG;  // default
    }

    // ──────────────────────────────────────────────────────────────
    //  SETUP
    // ──────────────────────────────────────────────────────────────
    void setup() {
    Serial.begin(115200);

    // Attach PWM to motor pins (ESP32 Core 3.x API)
    ledcAttach(IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(IN2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(IN3, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(IN4, PWM_FREQ, PWM_RESOLUTION);

    // IR sensor inputs
    pinMode(IR_LEFT,  INPUT);
    pinMode(IR_RIGHT, INPUT);

    // Start with motors off
    stopMotors();

    // 5-second countdown before starting (place robot on line during this time)
    Serial.println("=== LINE FOLLOWING ROBOT ===");
    Serial.println("Starting in 5 seconds...");
    delay(1000);
    Serial.println("4...");
    delay(1000);
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("GO!");
    
    // Stabilization period: let sensors settle for 500ms before starting
    unsigned long stabilizeStart = millis();
    while (millis() - stabilizeStart < 500) {
        digitalRead(IR_LEFT);
        digitalRead(IR_RIGHT);
        delay(10);
    }
    systemReady = true;  // System is now ready
    }

    // ──────────────────────────────────────────────────────────────
    //  MAIN LOOP — STATE MACHINE
    // ──────────────────────────────────────────────────────────────
    void loop() {
    // Safety check: don't process if system is not ready
    if (!systemReady) {
        delay(1);
        return;
    }
    
    // Read sensors with debouncing to prevent noise-triggered false starts
    int L = digitalRead(IR_LEFT);    // 0 = White, 1 = Black
    int R = digitalRead(IR_RIGHT);
    
    // Debounce: require 3 consecutive identical reads before accepting
    if (L == prevL && R == prevR) {
        debounceCount++;
    } else {
        debounceCount = 0;
    }
    prevL = L;
    prevR = R;
    
    // Use sensor values only after debounce threshold is reached
    if (debounceCount < 3) {
        L = prevL;  // Use previous stable values until debounce completes
        R = prevR;
    }
    
    unsigned long now = millis();

    // ════════════════════════════════════════════════════════════
    //  TRACK WHICH SENSOR LAST DETECTED BLACK
    //  This determines which direction to scan first when line is lost
    // ════════════════════════════════════════════════════════════
    if (L == 1 && R == 0) {
        lastSensorDetected = -1;  // Left sensor was last to see black
    } else if (L == 0 && R == 1) {
        lastSensorDetected = 1;   // Right sensor was last to see black
    }
    // If both black (1,1) or both white (0,0), keep the previous value

    // ════════════════════════════════════════════════════════════
    //  IMMEDIATE SENSOR LOSS DETECTION
    //  The moment BOTH sensors see white (line lost), STOP immediately
    //  This prevents coasting through intersections or off the map
    // ════════════════════════════════════════════════════════════
    if (L == 0 && R == 0) {
        // Both sensors white (line lost or centered)
        if (lineIsLost == false) {
            lineLostTime = now;  // Start tracking how long line is lost
            lineIsLost = true;
        }
        
        // If in a movement state (correcting left/right), stop IMMEDIATELY
        if (state == ST_CORRECT_LEFT || state == ST_CORRECT_RIGHT) {
            stopMotors();  // INSTANT STOP
            setState(ST_SCAN_PAUSE);
            lineIsLost = true;
        }
    } else {
        // Line detected (at least one sensor sees black)
        lineIsLost = false;
        lineLostTime = 0;
    }

    /*
    * SENSOR TRUTH TABLE (sensors straddle the line):
    * ┌───────┬───────┬─────────────────────────────────┐
    * │ LEFT  │ RIGHT │ MEANING                         │
    * ├───────┼───────┼─────────────────────────────────┤
    * │   0   │   0   │ Line between sensors (centered) │
    * │   1   │   0   │ Robot drifting RIGHT of line     │
    * │       │       │  → correct LEFT                  │
    * │   0   │   1   │ Robot drifting LEFT of line      │
    * │       │       │  → correct RIGHT                 │
    * │   1   │   1   │ Intersection / cross-line        │
    * │       │       │  → go STRAIGHT through           │
    * └───────┴───────┴─────────────────────────────────┘
    *
    * NOTE: When LEFT sensor sees black, the LINE is to the
    * left, meaning the ROBOT has drifted right → steer LEFT.
    */

    switch (state) {

        // ────────── NORMAL FOLLOWING ──────────
        case ST_FOLLOW:
        goStraight(BASE_SPEED);

        if (L == 0 && R == 0) {
            // Centered - if we stay centered too long without seeing black, scan
            if (now - stateStartTime > LINE_LOST_THRESHOLD) {
            setState(ST_SCAN_PAUSE);
            hardBrake();
            }
        }
        else if (L == 1 && R == 0) {
            // Line on left (robot drifted right) → steer left
            setState(ST_CORRECT_LEFT);
            lastKnownDir = -1;
        }
        else if (L == 0 && R == 1) {
            // Line on right (robot drifted left) → steer right
            setState(ST_CORRECT_RIGHT);
            lastKnownDir = 1;
        }
        else if (L == 1 && R == 1) {
            // Intersection
            if (recentTurn && (now - turnEndTime < TURN_COOLDOWN_MS)) {
            setState(ST_BRAKE);
            hardBrake();
            } else {
            setState(ST_INTERSECTION);
            intStart = now;
            }
        }
        break;

        // ────────── CORRECT LEFT ──────────
        case ST_CORRECT_LEFT:
        // PRE-CHECK: If line is lost, stop immediately
        if (lineIsLost) {
            stopMotors();
            setState(ST_SCAN_PAUSE);
            break;
        }
        
        correctLeft(TURN_INNER_SPEED, TURN_OUTER_SPEED);

        if (L == 0 && R == 0) {
            setState(ST_FOLLOW);
            turnEndTime = now;
            recentTurn = true;
        }
        else if (L == 0 && R == 1) {
            setState(ST_CORRECT_RIGHT);
            lastKnownDir = 1;
        }
        else if (L == 1 && R == 1) {
            setState(ST_INTERSECTION);
            intStart = now;
        }
        // If we lose the line while correcting, start scanning
        if (now - stateStartTime > LINE_LOST_THRESHOLD && L == 0 && R == 0) {
            setState(ST_SCAN_PAUSE);
            hardBrake();
        }
        break;

        // ────────── CORRECT RIGHT ──────────
        case ST_CORRECT_RIGHT:
        // PRE-CHECK: If line is lost, stop immediately
        if (lineIsLost) {
            stopMotors();
            setState(ST_SCAN_PAUSE);
            break;
        }
        
        correctRight(TURN_INNER_SPEED, TURN_OUTER_SPEED);

        if (L == 0 && R == 0) {
            setState(ST_FOLLOW);
            turnEndTime = now;
            recentTurn = true;
        }
        else if (L == 1 && R == 0) {
            setState(ST_CORRECT_LEFT);
            lastKnownDir = -1;
        }
        else if (L == 1 && R == 1) {
            setState(ST_INTERSECTION);
            intStart = now;
        }
        // If we lose the line while correcting, start scanning
        if (now - stateStartTime > LINE_LOST_THRESHOLD && L == 0 && R == 0) {
            setState(ST_SCAN_PAUSE);
            hardBrake();
        }
        break;

        // ────────── INTERSECTION ──────────
        case ST_INTERSECTION:
        goStraight(INTERSECTION_SPEED);

        if (L == 0 && R == 0) {
            setState(ST_FOLLOW);
            recentTurn = false;
        }
        else if (L == 1 && R == 0) {
            if (now - intStart > INTERSECTION_FWD_MS) {
            setState(ST_CORRECT_LEFT);
            lastKnownDir = -1;
            }
        }
        else if (L == 0 && R == 1) {
            if (now - intStart > INTERSECTION_FWD_MS) {
            setState(ST_CORRECT_RIGHT);
            lastKnownDir = 1;
            }
        }
        break;

        // ────────── BRAKE ──────────
        case ST_BRAKE:
        stopMotors();
        if (now - stateStartTime >= 150) {
            recentTurn = false;
            if (L == 1 && R == 1) setState(ST_INTERSECTION);
            else if (L == 0 && R == 0) setState(ST_FOLLOW);
            else if (L == 1 && R == 0) setState(ST_CORRECT_LEFT);
            else setState(ST_CORRECT_RIGHT);
        }
        break;

        // ────────── SCANNING SEQUENCE (GRADUATED) ──────────
        // Scans toward lastSensorDetected direction FIRST.
        // Left sensor last saw black → scan left first, then right.
        // Right sensor last saw black → scan right first, then left.
        // Increments: 30° → 60° → 90° → 120°.
        // After 120° exhausted on both sides → PERMANENT STOP (finish line).
        case ST_SCAN_PAUSE:
        stopMotors();
        if (now - stateStartTime > SCAN_PAUSE_MS) {
            scanAngleDegrees = 30;
            scanPhase = 0;
            scanStartTime = now;
            scanFirstDir = lastSensorDetected;  // -1=left first, +1=right first
            if (scanFirstDir <= 0) {
                setState(ST_SCAN_LEFT);
            } else {
                setState(ST_SCAN_RIGHT);
            }
        }
        // If line appears during pause, go back to follow
        if (L == 1 || R == 1) setState(ST_FOLLOW);
        break;

        case ST_SCAN_LEFT: {
        // If scanFirstDir <= 0: left is first (phase 0=scan, 1=return)
        // If scanFirstDir > 0:  left is second (phase 2=scan, 3=return)
        int leftBase = (scanFirstDir <= 0) ? 0 : 2;

        if (scanPhase == leftBase) {
            motorLeft(-SCAN_SPEED);
            motorRight(SCAN_SPEED);
            if (L == 1 || R == 1) { stopMotors(); setState(ST_FOLLOW); break; }
            int scanTime = getScanTimeForAngle(scanAngleDegrees);
            if (now - scanStartTime > scanTime) {
                scanPhase = leftBase + 1;
                scanStartTime = now;
            }
        }
        else if (scanPhase == leftBase + 1) {
            motorLeft(SCAN_SPEED);
            motorRight(-SCAN_SPEED);
            if (L == 1 || R == 1) { stopMotors(); setState(ST_FOLLOW); break; }
            int returnTime = getScanTimeForAngle(scanAngleDegrees);
            if (now - scanStartTime > returnTime) {
                stopMotors();
                if (scanFirstDir <= 0) {
                    // Left was first → now scan right (second)
                    scanPhase = 2;
                    scanStartTime = now;
                    setState(ST_SCAN_RIGHT);
                } else {
                    // Left was second → both sides done at this angle
                    if (scanAngleDegrees >= 90) {
                        finishLineReached = true;
                        setState(ST_STOP);
                    } else {
                        scanAngleDegrees += 30;
                        scanPhase = 0;
                        scanStartTime = now;
                        // scanFirstDir > 0 means right-first, so next cycle starts right
                        setState(ST_SCAN_RIGHT);
                    }
                }
            }
        }
        break;
        }

        case ST_SCAN_RIGHT: {
        // If scanFirstDir > 0:  right is first (phase 0=scan, 1=return)
        // If scanFirstDir <= 0: right is second (phase 2=scan, 3=return)
        int rightBase = (scanFirstDir > 0) ? 0 : 2;

        if (scanPhase == rightBase) {
            motorLeft(SCAN_SPEED);
            motorRight(-SCAN_SPEED);
            if (L == 1 || R == 1) { stopMotors(); setState(ST_FOLLOW); break; }
            int scanTime = getScanTimeForAngle(scanAngleDegrees);
            if (now - scanStartTime > scanTime) {
                scanPhase = rightBase + 1;
                scanStartTime = now;
            }
        }
        else if (scanPhase == rightBase + 1) {
            motorLeft(-SCAN_SPEED);
            motorRight(SCAN_SPEED);
            if (L == 1 || R == 1) { stopMotors(); setState(ST_FOLLOW); break; }
            int returnTime = getScanTimeForAngle(scanAngleDegrees);
            if (now - scanStartTime > returnTime) {
                stopMotors();
                if (scanFirstDir > 0) {
                    // Right was first → now scan left (second)
                    scanPhase = 2;
                    scanStartTime = now;
                    setState(ST_SCAN_LEFT);
                } else {
                    // Right was second → both sides done at this angle
                    if (scanAngleDegrees >= 120) {
                        finishLineReached = true;
                        setState(ST_STOP);
                    } else {
                        scanAngleDegrees += 30;
                        scanPhase = 0;
                        scanStartTime = now;
                        // scanFirstDir <= 0 means left-first, so next cycle starts left
                        setState(ST_SCAN_LEFT);
                    }
                }
            }
        }
        break;
        }

        // ────────── PERMANENT STOP (FINISH LINE) ──────────
        case ST_STOP:
        stopMotors();
        if (finishLineReached) {
            // Permanent stop — robot is done. No restart.
            break;
        }
        // Only restart if NOT a finish line stop
        if (L == 1 || R == 1) {
            debounceCount = 0;
            setState(ST_FOLLOW);
        }
        break;
    }

    // Reset recentTurn flag after cooldown expires
    if (recentTurn && (now - turnEndTime > TURN_COOLDOWN_MS * 2)) {
        recentTurn = false;
    }

    // Debug logging (comment out for competition to save CPU cycles)
    static unsigned long lastLog = 0;
    if (now - lastLog > 100) {
        Serial.printf("ST:%d L:%d R:%d DIR:%d RT:%d LOST:%d LSENS:%d\n",
                    state, L, R, lastKnownDir, recentTurn, lineIsLost, lastSensorDetected);
        lastLog = now;
    }

    // Reduced delay for faster responsiveness (1ms instead of 3ms)
    //delay(1);
    }

    /*================================================================
    *  TUNING GUIDE — Read this before your first test run!
    *================================================================
    *
    *  STEP 1: SENSOR TEST
    *    - Upload the code, open Serial Monitor at 115200 baud
    *    - Place robot on white surface → both should read 0
    *    - Slide black tape under left sensor → L should read 1
    *    - Slide black tape under right sensor → R should read 1
    *    - If inverted, swap the sensor wires or adjust pots
    *
    *  STEP 2: STRAIGHT LINE TEST
    *    - Place robot centered on a straight black line
    *    - It should go straight with minor corrections
    *    - If oscillating too much:
    *        → Increase TURN_INNER_SPEED (less aggressive turn)
    *        → Decrease TURN_OUTER_SPEED (less aggressive turn)
    *    - If too slow to correct:
    *        → Decrease TURN_INNER_SPEED
    *        → Increase TURN_OUTER_SPEED
    *
    *  STEP 3: SPEED TUNING
    *    - Start with BASE_SPEED = 140
    *    - Once stable, gradually increase by 10-15 at a time
    *    - Competition tip: fastest stable speed wins
    *    - If too fast for turns, lower BASE_SPEED
    *
    *  STEP 4: 90° TURN TEST (L-shape)
    *    - Place robot on an L-shaped track
    *    - It should detect the turn, lose the line briefly,
    *      then SPIN in the last known direction to recover
    *    - If overshooting turns:
    *        → Decrease LINE_LOST_THRESHOLD (react faster)
    *        → Decrease BASE_SPEED
    *    - If spinning wrong direction:
    *        → Check sensor wiring (left/right might be swapped)
    *
    *  STEP 5: INTERSECTION TEST
    *    - Place robot approaching a + intersection
    *    - It should detect both sensors black and go STRAIGHT
    *    - If turning at intersections:
    *        → Increase INTERSECTION_FWD_MS slightly
    *
    *  STEP 6: INTERSECTION-AFTER-TURN TEST (Challenge 3.5)
    *    - This is the hardest scenario
    *    - After a 90° turn, place an intersection immediately
    *    - Robot should BRAKE to stabilize, then go straight
    *    - If still turning at the intersection:
    *        → Increase TURN_COOLDOWN_MS (longer ignore window)
    *        → Increase brake duration in ST_BRAKE (the 150 value)
    *
    *  STEP 7: ZIGZAG TEST
    *    - Place robot on zigzag pattern
    *    - Should rapidly alternate left/right corrections
    *    - If oscillating wildly:
    *        → Lower BASE_SPEED
    *        → Reduce the difference between OUTER and INNER speed
    *
    *  MOTOR DIRECTION FIX:
    *    If a motor spins the wrong way, you have two options:
    *    1. Swap the two motor wires on the L298N terminal
    *    2. In code: swap the IN pin assignments for that motor
    *       e.g., swap IN1/IN2 values for right motor
    *
    *================================================================
    *  HOW THE STATE MACHINE WORKS (for your documentation)
    *================================================================
    *
    *  ST_FOLLOW ──→ Both white (centered) → go straight
    *     │
    *     ├─→ Left=Black  → ST_CORRECT_LEFT  (steer left)
    *     ├─→ Right=Black → ST_CORRECT_RIGHT (steer right)
    *     ├─→ Both=Black  → ST_INTERSECTION  (go straight)
    *     │                  or ST_BRAKE if just finished a turn
    *     └─→ White too long → ST_SEARCH (spin to find line)
    *
    *  ST_CORRECT_LEFT/RIGHT ──→ Re-centered → ST_FOLLOW
    *     │                        (marks recentTurn = true)
    *     ├─→ Overcorrected → switch to opposite correction
    *     └─→ Both=Black → ST_INTERSECTION or ST_BRAKE
    *
    *  ST_INTERSECTION ──→ Go straight until exiting
    *     └─→ Exits to ST_FOLLOW or ST_CORRECT_*
    *
    *  ST_BRAKE ──→ Stop for 150ms to stabilize
    *     └─→ Re-read sensors → appropriate state
    *
    *  ST_SEARCH ──→ Spin in lastKnownDir to find line
    *     ├─→ Found line → ST_CORRECT_* or ST_FOLLOW
    *     └─→ Timeout → ST_STOP
    *
    *  ST_STOP ──→ Dead stop (auto-restarts if line detected)
    *
    *================================================================*/

    