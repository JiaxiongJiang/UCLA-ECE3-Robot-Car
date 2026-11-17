
#include <ECE3.h>

// ==========================================
// 1. SENSOR CONFIGURATION
// ==========================================

uint16_t sensorValues[8];

const uint8_t sensor_count = 8;

const int weights[8] = {
    8, 4, 2, 1, -1, -2, -4, -8};

int minValues[8] = {
    658, 636, 544, 612, 499, 635, 589, 635};

int maxValues[8] = {
    2500, 2500, 2500, 2500,
    2500, 2500, 2500, 2500};

// ==========================================
// 2. MOTOR CONFIGURATION
// ==========================================

const int left_nslp_pin = 31;
const int right_nslp_pin = 11;

const int left_dir_pin = 29;
const int right_dir_pin = 30;

const int left_pwm_pin = 40;
const int right_pwm_pin = 39;

const int LED_RF = 41;

// ==========================================
// 3. PD CONTROLLER
// ==========================================

float KP = 0.2;
float KD = 1.5;

float error = 0;
float prevError = 0;
float derivative = 0;
float correction = 0;

int base_speed = 95;

bool resetDerivative = false;

// ==========================================
// 4. FINISH LINE DETECTION
// ==========================================

const uint32_t FINISH_THRESHOLD = 14000;

const uint8_t CONFIRM_COUNT = 3;

uint8_t finishCounter = 0;

bool hasTurned = false;

// ==========================================
// 5. U-TURN CONFIGURATION
// ==========================================

const int UTURN_SPEED = 80;

// Initial estimate - requires calibration
const unsigned long UTURN_TIME = 500;

// ==========================================
// 6. MOTOR CONTROL FUNCTION
// ==========================================

void setMotor(int motor_speed, int dir_pin, int pwm_pin)
{
    motor_speed = constrain(motor_speed, -255, 255);

    if (motor_speed == 0)
    {
        analogWrite(pwm_pin, 0);
        return;
    }

    if (motor_speed > 0)
    {
        digitalWrite(dir_pin, LOW);
    }
    else
    {
        digitalWrite(dir_pin, HIGH);
    }

    analogWrite(pwm_pin, abs(motor_speed));
}

// ==========================================
// 7. STOP MOTORS
// ==========================================

void stopMotors()
{
    setMotor(0, left_dir_pin, left_pwm_pin);
    setMotor(0, right_dir_pin, right_pwm_pin);
}

// ==========================================
// 8. DETECT FINISH LINE
// ==========================================

bool detectFinishLine()
{
    uint32_t sensorSum = 0;

    for (uint8_t i = 0; i < sensor_count; i++)
    {
        sensorSum += sensorValues[i];
    }

    return sensorSum > FINISH_THRESHOLD;
}

// ==========================================
// 9. CONFIRM FINISH LINE
// ==========================================

bool confirmFinishLine()
{
    if (detectFinishLine())
    {
        finishCounter++;
    }
    else
    {
        finishCounter = 0;
    }

    if (finishCounter >= CONFIRM_COUNT)
    {
        finishCounter = 0;
        return true;
    }

    return false;
}

// ==========================================
// 10. 180 DEGREE U-TURN
// ==========================================

void turn180()
{
    // Stop before turning
    stopMotors();

    delay(200);

    // Rotate in place
    setMotor(UTURN_SPEED, left_dir_pin, left_pwm_pin);
    setMotor(-UTURN_SPEED, right_dir_pin, right_pwm_pin);

    delay(UTURN_TIME);

    // Stop after turning
    stopMotors();

    delay(200);

    // Reset PD controller history
    prevError = 0;
    resetDerivative = true;

    Serial.println("U-Turn completed!");
}

// ==========================================
// 11. SETUP
// ==========================================

void setup()
{
    pinMode(left_nslp_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(left_pwm_pin, OUTPUT);

    pinMode(right_nslp_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);
    pinMode(right_pwm_pin, OUTPUT);

    digitalWrite(left_nslp_pin, HIGH);
    digitalWrite(right_nslp_pin, HIGH);

    pinMode(LED_RF, OUTPUT);

    ECE3_Init();

    Serial.begin(9600);

    delay(2000);
}

// ==========================================
// 12. MAIN LOOP
// ==========================================

void loop()
{
    // Read all 8 sensors
    ECE3_read_IR(sensorValues);

    // Check finish line
    if (!hasTurned && confirmFinishLine())
    {

        Serial.println("FINISH LINE DETECTED!");

        hasTurned = true;

        turn180();

        return;
    }

    // ======================================
    // NORMAL LINE FOLLOWING
    // ======================================

    error = 0;

    for (int i = 0; i < sensor_count; i++)
    {
        long raw = sensorValues[i];

        long normNumber =
            (raw - minValues[i]) * 1000L / (maxValues[i] - minValues[i]);

        normNumber = constrain(normNumber, 0, 1000);

        int holdError = (normNumber * weights[i]) / 4;

        error += holdError;
    }

    // ======================================
    // PD CONTROLLER
    // ======================================

    if (resetDerivative)
    {
        derivative = 0;
        resetDerivative = false;
    }
    else
    {
        derivative = error - prevError;
    }

    correction = KP * error + KD * derivative;

    // ======================================
    // MOTOR SPEED
    // ======================================

    int right_speed = base_speed - (int)correction;
    int left_speed = base_speed + (int)correction;

    right_speed = constrain(right_speed, -255, 255);
    left_speed = constrain(left_speed, -255, 255);

    // ======================================
    // APPLY MOTOR CONTROL
    // ======================================

    setMotor(right_speed, right_dir_pin, right_pwm_pin);
    setMotor(left_speed, left_dir_pin, left_pwm_pin);

    prevError = error;
}
