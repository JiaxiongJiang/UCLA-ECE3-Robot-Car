#include <ECE3.h>

// =====================================================
// ECE3 Robot Car - Single-file sketch
// All core logic and main program are combined here.
// No separate .h file is needed.
// =====================================================

// ---------- IR sensors: S0 = rightmost, S7 = leftmost ----------
const uint8_t SENSOR_COUNT = 8;
uint16_t sensorValues[SENSOR_COUNT];

const int weights[SENSOR_COUNT] = {15, 14, 12, 8, -8, -12, -14, -15};
int minValues[SENSOR_COUNT] = {794, 567, 612, 521, 589, 589, 657, 612};
int maxValues[SENSOR_COUNT] = {2500, 2282, 2213, 1677, 1910, 1840, 2330, 2500};

// ---------- Motor pins ----------
const int left_nslp_pin = 31;
const int right_nslp_pin = 11;
const int left_dir_pin = 29;
const int right_dir_pin = 30;
const int left_pwm_pin = 40;
const int right_pwm_pin = 39;
const int LED_RF = 41;

// ---------- PD parameters ----------
float KP = 0.06;
float KD = 0.3;
const int NORMAL_BASE_SPEED = 120;
int base_speed = NORMAL_BASE_SPEED;

float error = 0;
float prevError = 0;
float derivative = 0;
float correction = 0;

// ---------- Pivot parameters ----------
const int OUTER_SENSOR_THRESHOLD = 1500;
const int PIVOT_TRIGGER_ERROR_THRESHOLD = 2000;
const int PIVOT_ERROR_EXIT_THRESHOLD = 2000;
const int CENTER_SENSOR_THRESHOLD = 1500;
const int PIVOT_SPEED = 110;
const unsigned long PIVOT_CHECK_INTERVAL = 20;

bool pivotActive = false;
int pivotDirection = 0;
unsigned long lastPivotCheckTime = 0;

// ---------- Finish-line / black-line parameters ----------
const uint16_t FINISH_BLACK_LINE_THRESHOLD = 15000;
const uint8_t FINISH_CONFIRM_COUNT = 3;
const unsigned long FINISH_PRE_TURN_PAUSE_MS = 200;
const int FINISH_UTURN_SPEED = 220;
const unsigned long FINISH_UTURN_TIME_MS = 300;
const unsigned long FINISH_POST_TURN_PAUSE_MS = 200;

uint8_t finishLineCount = 0;
uint8_t finishLineConsecutiveCount = 0;
bool finishLineDetected = false;
bool finishLineStopped = false;
bool finishFeedbackActive = false;

int currentLeftSpeed = 0;
int currentRightSpeed = 0;

// ---------- Timing and logger parameters ----------
const unsigned long START_DELAY_TIME = 2000;
const unsigned long RUN_TIME = 50000;
const unsigned long SAMPLE_INTERVAL = 10;
const int MAX_SAMPLES = 300;

const unsigned long DEBOUNCE_TIME = 30;
int lastButtonReading = HIGH;
int stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

unsigned long delayStartTime = 0;
unsigned long runStartTime = 0;
unsigned long lastSampleTime = 0;

struct DataPoint
{
    unsigned long time_ms;
    uint16_t sensor[8];
    float error;
    float derivative;
    float correction;
    int base_speed;
    int left_speed;
    int right_speed;
    int pivot_active;
    int pivot_direction;
    int center_detected;
    unsigned long sensor_sum;
    int finish_line_detected;
    int finish_line_count;
    int finish_line_stopped;
};

DataPoint dataLog[MAX_SAMPLES];
int sampleCount = 0;

// ---------- Core motor methods ----------
void setMotor(int motor_speed, int dir_pin, int pwm_pin)
{
    motor_speed = constrain(motor_speed, -255, 255);
    if (motor_speed == 0)
    {
        analogWrite(pwm_pin, 0);
        return;
    }
    digitalWrite(dir_pin, motor_speed > 0 ? LOW : HIGH);
    analogWrite(pwm_pin, abs(motor_speed));
}

void stopMotors()
{
    setMotor(0, left_dir_pin, left_pwm_pin);
    setMotor(0, right_dir_pin, right_pwm_pin);
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
}

void driveMotors(int leftCommand, int rightCommand)
{
    leftCommand = constrain(leftCommand, -255, 255);
    rightCommand = constrain(rightCommand, -255, 255);
    setMotor(leftCommand, left_dir_pin, left_pwm_pin);
    setMotor(rightCommand, right_dir_pin, right_pwm_pin);
    currentLeftSpeed = leftCommand;
    currentRightSpeed = rightCommand;
}

// ---------- Button methods ----------
int readS1()
{
    return (P1->IN & BIT1) ? HIGH : LOW;
}

bool buttonPressed()
{
    int reading = readS1();
    if (reading != lastButtonReading)
        lastDebounceTime = millis();

    if (millis() - lastDebounceTime >= DEBOUNCE_TIME)
    {
        if (reading != stableButtonState)
        {
            stableButtonState = reading;
            if (stableButtonState == LOW)
            {
                lastButtonReading = reading;
                return true;
            }
        }
    }
    lastButtonReading = reading;
    return false;
}

// ---------- Sensor / PD methods ----------
float calculateError()
{
    float newError = 0;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        long normalized = (long)(sensorValues[i] - minValues[i]) * 1000L / (maxValues[i] - minValues[i]);
        normalized = constrain(normalized, 0, 1000);
        newError += (normalized * weights[i]) / 8;
    }
    return newError;
}

void calculatePD()
{
    error = calculateError();
    derivative = error - prevError;
    correction = KP * error + KD * derivative;
}

void controlMotors()
{
    int rightCommand = constrain(base_speed - (int)correction, -255, 255);
    int leftCommand = constrain(base_speed + (int)correction, -255, 255);
    driveMotors(leftCommand, rightCommand);
    prevError = error;
}

// ---------- Pivot methods ----------
void applyPivot()
{
    if (pivotDirection > 0)
        driveMotors(PIVOT_SPEED, -PIVOT_SPEED);
    else
        driveMotors(-PIVOT_SPEED, PIVOT_SPEED);
}

void startPivot(int direction)
{
    pivotActive = true;
    pivotDirection = direction;
    lastPivotCheckTime = millis();
    derivative = 0;
    correction = 0;
    applyPivot();
}

bool updatePivot()
{
    unsigned long now = millis();
    if (now - lastPivotCheckTime < PIVOT_CHECK_INTERVAL)
    {
        applyPivot();
        return false;
    }

    lastPivotCheckTime = now;
    error = calculateError();

    bool errorSmall = abs((int)error) < PIVOT_ERROR_EXIT_THRESHOLD;
    bool centerDetected =
        (sensorValues[3] > CENTER_SENSOR_THRESHOLD) ||
        (sensorValues[4] > CENTER_SENSOR_THRESHOLD);

    if (errorSmall && centerDetected)
    {
        pivotActive = false;
        pivotDirection = 0;
        prevError = error;
        derivative = 0;
        correction = 0;
        return true;
    }

    applyPivot();
    return false;
}

// ---------- Finish-line methods ----------
unsigned long getSensorSum()
{
    unsigned long total = 0;
    for (int i = 0; i < SENSOR_COUNT; i++)
        total += sensorValues[i];
    return total;
}

bool updateFinishLineDetection()
{
    if (finishLineStopped)
        return false;

    unsigned long total = getSensorSum();
    if (total >= FINISH_BLACK_LINE_THRESHOLD)
    {
        if (finishLineConsecutiveCount < FINISH_CONFIRM_COUNT)
        {
            finishLineConsecutiveCount++;
        }
    }
    else
    {
        finishLineConsecutiveCount = 0;
        finishFeedbackActive = false;
    }

    if (finishLineConsecutiveCount >= FINISH_CONFIRM_COUNT &&
        !finishFeedbackActive)
    {
        finishFeedbackActive = true;
        finishLineDetected = true;
        finishLineCount++;
        digitalWrite(LED_RF, HIGH);
        return true;
    }
    return false;
}

// ---------- Logger method ----------
void recordData()
{
    if (sampleCount >= MAX_SAMPLES)
        return;
    DataPoint &d = dataLog[sampleCount];
    d.time_ms = millis() - runStartTime;
    for (int i = 0; i < SENSOR_COUNT; i++)
        d.sensor[i] = sensorValues[i];

    d.error = error;
    d.derivative = derivative;
    d.correction = correction;
    d.base_speed = base_speed;
    d.left_speed = currentLeftSpeed;
    d.right_speed = currentRightSpeed;
    d.pivot_active = pivotActive ? 1 : 0;
    d.pivot_direction = pivotDirection;
    d.center_detected =
        (sensorValues[3] > CENTER_SENSOR_THRESHOLD ||
         sensorValues[4] > CENTER_SENSOR_THRESHOLD)
            ? 1
            : 0;
    d.sensor_sum = getSensorSum();
    d.finish_line_detected = finishLineDetected ? 1 : 0;
    d.finish_line_count = finishLineCount;
    d.finish_line_stopped = finishLineStopped ? 1 : 0;
    sampleCount++;
}

// =====================================================
// MAIN PROGRAM
// =====================================================

enum LoggerState
{
    READY,
    START_DELAY,
    RECORDING,
    DATA_READY,
    PRINTING
};

LoggerState state = READY;

enum FinishActionState
{
    FINISH_NORMAL,
    FINISH_PRE_PAUSE,
    FINISH_UTURN,
    FINISH_POST_PAUSE,
    FINISH_STOPPED
};

FinishActionState finishActionState = FINISH_NORMAL;
unsigned long finishActionStartTime = 0;

// -----------------------------------------------------
// Reset all run-specific values before a new run
// -----------------------------------------------------
void resetRun()
{
    sampleCount = 0;

    error = 0;
    prevError = 0;
    derivative = 0;
    correction = 0;

    base_speed = NORMAL_BASE_SPEED;

    pivotActive = false;
    pivotDirection = 0;
    lastPivotCheckTime = 0;

    finishLineCount = 0;
    finishLineConsecutiveCount = 0;
    finishLineDetected = false;
    finishLineStopped = false;
    finishFeedbackActive = false;

    finishActionState = FINISH_NORMAL;
    finishActionStartTime = 0;

    stopMotors();
    digitalWrite(LED_RF, LOW);
}

// -----------------------------------------------------
// Start sequence: press S1 -> wait 2 seconds -> run
// -----------------------------------------------------
void startDelay()
{
    resetRun();
    delayStartTime = millis();
    state = START_DELAY;
    digitalWrite(LED_RF, HIGH);
}

void beginRecording()
{
    runStartTime = millis();
    lastSampleTime = runStartTime;
    state = RECORDING;
    digitalWrite(LED_RF, LOW);
}

// -----------------------------------------------------
// Finish-line action
//
// First confirmed finish-line detection:
//   1. Stop for 200 ms
//   2. Pivot in place for 300 ms (approximate U-turn)
//   3. Stop for 200 ms
//   4. Reset PD history and resume line following
//
// Second confirmed finish-line detection:
//   Stop permanently.
// -----------------------------------------------------
void beginFinishLineAction()
{
    stopMotors();

    pivotActive = false;
    pivotDirection = 0;

    finishActionStartTime = millis();

    if (finishLineCount >= 2)
    {
        finishLineStopped = true;
        finishActionState = FINISH_STOPPED;
        digitalWrite(LED_RF, HIGH);
        Serial.println("FINISH LINE #2: ROBOT STOPPED");
    }
    else
    {
        finishActionState = FINISH_PRE_PAUSE;
        Serial.println("FINISH LINE #1: PAUSE THEN U-TURN");
    }
}

void updateFinishLineAction()
{
    unsigned long elapsed = millis() - finishActionStartTime;

    switch (finishActionState)
    {
    case FINISH_PRE_PAUSE:
        stopMotors();

        if (elapsed >= FINISH_PRE_TURN_PAUSE_MS)
        {
            finishActionStartTime = millis();
            finishActionState = FINISH_UTURN;
        }
        break;

    case FINISH_UTURN:
        // Opposite wheel directions. Without encoders, this is a
        // time-based approximation, not a guaranteed 180-degree turn.
        driveMotors(FINISH_UTURN_SPEED, -FINISH_UTURN_SPEED);

        if (elapsed >= FINISH_UTURN_TIME_MS)
        {
            stopMotors();
            finishActionStartTime = millis();
            finishActionState = FINISH_POST_PAUSE;
        }
        break;

    case FINISH_POST_PAUSE:
        stopMotors();

        if (elapsed >= FINISH_POST_TURN_PAUSE_MS)
        {
            // Prevent a derivative spike when PD control resumes.
            error = calculateError();
            prevError = error;
            derivative = 0;
            correction = 0;

            pivotActive = false;
            pivotDirection = 0;

            finishLineConsecutiveCount = 0;
            finishFeedbackActive = false;
            finishLineDetected = false;

            finishActionState = FINISH_NORMAL;
            digitalWrite(LED_RF, LOW);
            Serial.println("U-TURN COMPLETE: RESUME PD");
        }
        break;

    case FINISH_STOPPED:
        stopMotors();
        digitalWrite(LED_RF, HIGH);
        break;

    case FINISH_NORMAL:
    default:
        break;
    }
}

// -----------------------------------------------------
// CSV output
// -----------------------------------------------------
void printData()
{
    Serial.println(
        "time_ms,S0,S1,S2,S3,S4,S5,S6,S7,"
        "error,derivative,correction,base_speed,left_speed,right_speed,"
        "pivot_active,pivot_direction,center_detected,sensor_sum,"
        "finish_line_detected,finish_line_count,finish_line_stopped");

    for (int i = 0; i < sampleCount; i++)
    {
        DataPoint &d = dataLog[i];

        Serial.print(d.time_ms);

        for (int j = 0; j < SENSOR_COUNT; j++)
        {
            Serial.print(",");
            Serial.print(d.sensor[j]);
        }

        Serial.print(",");
        Serial.print(d.error, 2);
        Serial.print(",");
        Serial.print(d.derivative, 2);
        Serial.print(",");
        Serial.print(d.correction, 2);
        Serial.print(",");
        Serial.print(d.base_speed);
        Serial.print(",");
        Serial.print(d.left_speed);
        Serial.print(",");
        Serial.print(d.right_speed);
        Serial.print(",");
        Serial.print(d.pivot_active);
        Serial.print(",");
        Serial.print(d.pivot_direction);
        Serial.print(",");
        Serial.print(d.center_detected);
        Serial.print(",");
        Serial.print(d.sensor_sum);
        Serial.print(",");
        Serial.print(d.finish_line_detected);
        Serial.print(",");
        Serial.print(d.finish_line_count);
        Serial.print(",");
        Serial.println(d.finish_line_stopped);
    }

    Serial.println("END_OF_DATA");
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
    // Motor driver pins
    pinMode(left_nslp_pin, OUTPUT);
    pinMode(right_nslp_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);
    pinMode(left_pwm_pin, OUTPUT);
    pinMode(right_pwm_pin, OUTPUT);
    pinMode(LED_RF, OUTPUT);

    // S1 button = MSP432 P1.1, active LOW, internal pull-up
    P1->DIR &= ~BIT1;
    P1->REN |= BIT1;
    P1->OUT |= BIT1;

    // Wake up both motor drivers
    digitalWrite(left_nslp_pin, HIGH);
    digitalWrite(right_nslp_pin, HIGH);

    stopMotors();

    ECE3_Init();
    Serial.begin(115200);

    digitalWrite(LED_RF, LOW);

    lastButtonReading = readS1();
    stableButtonState = lastButtonReading;
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop()
{
    bool pressed = buttonPressed();

    if (state == READY)
    {
        stopMotors();

        // Press S1 to begin a new run.
        if (pressed)
        {
            startDelay();
        }
    }

    else if (state == START_DELAY)
    {
        stopMotors();

        if (millis() - delayStartTime >= START_DELAY_TIME)
        {
            beginRecording();
        }
    }

    else if (state == RECORDING)
    {
        // Refresh all eight IR sensor readings every loop.
        ECE3_read_IR(sensorValues);

        // Once the second finish line has been confirmed, remain stopped.
        if (finishActionState == FINISH_STOPPED || finishLineStopped)
        {
            stopMotors();
            digitalWrite(LED_RF, HIGH);
        }

        // Finish-line pause / U-turn / pause action takes priority over PD/Pivot.
        else if (finishActionState != FINISH_NORMAL)
        {
            updateFinishLineAction();
        }

        else
        {
            // Check the finish line before issuing normal motor commands.
            if (updateFinishLineDetection())
            {
                beginFinishLineAction();
            }

            // If no finish-line event occurred, continue with Pivot or PD.
            else if (pivotActive)
            {
                bool pivotExited = updatePivot();

                // Resume PD immediately on the same loop that Pivot exits.
                if (pivotExited)
                {
                    calculatePD();
                    controlMotors();
                }
                else
                {
                    derivative = 0;
                    correction = 0;
                }
            }

            else
            {
                error = calculateError();

                bool errorLarge =
                    abs((int)error) > PIVOT_TRIGGER_ERROR_THRESHOLD;

                // Right outer pair: S0 AND S1.
                bool rightOuterPair =
                    (sensorValues[0] > OUTER_SENSOR_THRESHOLD) &&
                    (sensorValues[1] > OUTER_SENSOR_THRESHOLD) &&
                    errorLarge;

                // Left outer pair: S6 AND S7.
                bool leftOuterPair =
                    (sensorValues[6] > OUTER_SENSOR_THRESHOLD) &&
                    (sensorValues[7] > OUTER_SENSOR_THRESHOLD) &&
                    errorLarge;

                if (rightOuterPair || leftOuterPair)
                {
                    int direction;

                    if (rightOuterPair && leftOuterPair)
                    {
                        direction = (error >= 0) ? 1 : -1;
                    }
                    else
                    {
                        direction = rightOuterPair ? 1 : -1;
                    }

                    startPivot(direction);
                }

                else
                {
                    calculatePD();
                    controlMotors();
                }
            }
        }

        // Record telemetry at the configured interval.
        if (millis() - lastSampleTime >= SAMPLE_INTERVAL)
        {
            recordData();
            lastSampleTime = millis();
        }

        // End the timed test. A finish-line permanent stop remains latched.
        if (millis() - runStartTime >= RUN_TIME && !finishLineStopped)
        {
            stopMotors();
            pivotActive = false;
            pivotDirection = 0;
            digitalWrite(LED_RF, LOW);
            state = DATA_READY;
        }
    }

    else if (state == DATA_READY)
    {
        stopMotors();

        // Press S1 again to print the captured CSV.
        if (pressed)
        {
            state = PRINTING;
        }
    }

    else if (state == PRINTING)
    {
        stopMotors();

        if (Serial)
        {
            printData();
        }

        state = READY;
    }
}
