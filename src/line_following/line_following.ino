#include <ECE3.h>

uint16_t sensorValues[8];

const uint8_t sensor_count = 8;

const int weights[8] = {
    10, 9, 4, 1, -1, -4, -9, -10};

int minValues[8] = {
    794, 567, 612, 521, 589, 589, 657, 612};

int maxValues[8] = {
    2500, 2282, 2213, 1677, 1910, 1840, 2330, 2500};

// Motor pins
const int left_nslp_pin = 31;
const int right_nslp_pin = 11;

const int left_dir_pin = 29;
const int right_dir_pin = 30;

const int left_pwm_pin = 40;
const int right_pwm_pin = 39;

const int LED_RF = 41;

// PD controller
float KP = 0.23;
float KD = 1.5;

float error = 0;
float prevError = 0;
float derivative = 0;
float correction = 0;

int base_speed = 110;

bool resetDerivative = false;

// Speed control
unsigned long startTime = 0;

void updateSpeed()
{
    unsigned long elapsedTime = millis() - startTime;

    if (elapsedTime < 4500)
    {
        base_speed = 90;
    }
    else if (elapsedTime < 5500)
    {
        base_speed = 200;
    }
    else if (elapsedTime < 18000)
    {
        base_speed = 95;
    }
    else
    {
        base_speed = 95;
    }
}

// Finish line detection
const uint32_t FINISH_THRESHOLD = 18000;

const uint8_t CONFIRM_COUNT = 3;

uint8_t finishCounter = 0;

uint8_t finishCount = 0;

// U-turn settings
const int UTURN_SPEED = 200;

const unsigned long UTURN_TIME = 300;

// Motor control
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

// Stop both motors
void stopMotors()
{
    setMotor(0, left_dir_pin, left_pwm_pin);
    setMotor(0, right_dir_pin, right_pwm_pin);
}

// Check sensor readings
bool detectFinishLine()
{
    uint32_t sensorSum = 0;

    for (uint8_t i = 0; i < sensor_count; i++)
    {
        sensorSum += sensorValues[i];
    }

    return sensorSum > FINISH_THRESHOLD;
}

// Confirm finish line detection
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

// Turn around
void turn180()
{
    stopMotors();

    delay(10);

    setMotor(UTURN_SPEED, left_dir_pin, left_pwm_pin);
    setMotor(-UTURN_SPEED, right_dir_pin, right_pwm_pin);

    delay(UTURN_TIME);

    stopMotors();

    delay(10);

    prevError = 0;
    resetDerivative = true;
}

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

    delay(2000);

    startTime = millis();
}

void loop()
{
    ECE3_read_IR(sensorValues);

    // Check finish line
    if (confirmFinishLine())
    {
        finishCount++;

        if (finishCount == 1)
        {
            turn180();
            return;
        }
        else if (finishCount >= 2)
        {
            stopMotors();

            while (true)
            {
            }
        }
    }

    // Update motor speed
    // updateSpeed();

    // Calculate error
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

    // PD control
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

    // Calculate motor speeds
    int right_speed = base_speed - (int)correction;
    int left_speed = base_speed + (int)correction;

    right_speed = constrain(right_speed, -255, 255);
    left_speed = constrain(left_speed, -255, 255);

    // Apply motor speeds
    setMotor(right_speed, right_dir_pin, right_pwm_pin);
    setMotor(left_speed, left_dir_pin, left_pwm_pin);

    prevError = error;
}
