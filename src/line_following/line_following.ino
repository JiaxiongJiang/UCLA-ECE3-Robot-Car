#include <ECE3.h>

uint16_t sensorValues[8];
const int left_nslp_pin = 31;
const int right_nslp_pin = 11;

const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_dir_pin = 30;
const int right_pwm_pin = 39;
const int LED_RF = 41;

const int weights[8] = {8, 4, 2, 1, -1, -2, -4, -8};

int minValues[8] = {658, 636, 544, 612, 499, 635, 589, 635};
int maxValues[8] = {2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500};

// PD controller
float KP = 0.2;
float KD = 1.5;

float error = 0;
float prevError = 0;
float derivative = 0;
float correction = 0;

int base_speed = 95;

// Motor dir
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

void loop()
{

    ECE3_read_IR(sensorValues);

    error = 0;

    for (int i = 0; i < 8; i++)
    {

        long raw = sensorValues[i];

        long normNumber = (raw - minValues[i]) * 1000 / (maxValues[i] - minValues[i]);
        normNumber = constrain(normNumber, 0, 1000);

        int holdError = (normNumber * weights[i]) / 4;

        error += holdError;
    }

    // PD controller
    derivative = error - prevError;

    correction = KP * error + KD * derivative;

    int right_speed = base_speed - (int)correction;
    int left_speed = base_speed + (int)correction;

    right_speed = constrain(right_speed, -255, 255);
    left_speed = constrain(left_speed, -255, 255);

    setMotor(right_speed, right_dir_pin, right_pwm_pin);
    setMotor(left_speed, left_dir_pin, left_pwm_pin);

    prevError = error;

    delay(0.1);
}