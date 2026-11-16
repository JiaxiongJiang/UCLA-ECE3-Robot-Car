#include <ECE3.h>

const uint8_t sensor_count = 8;
uint16_t sensor_values[sensor_count];
const uint16_t sensor_maximums[] = {2500, 2500, 2500, 2445, 2500, 2500, 2500, 2500};
const uint16_t sensor_minimums[] = {866, 750, 612, 612, 612, 589, 612, 726};
const int8_t sensor_weights[] = {-8, -4, -2, -1, 1, 2, 4, 8};

int16_t prev_error = 0;
const uint8_t base_speed = 50;
const uint8_t fast_speed = 255;
uint8_t low_error_count = 0;
const uint16_t low_error_thresh = 250;
const uint8_t low_error_loops = 50;
const float kp = 0.2;
const float kd = 1.5;

const uint16_t track_end_tresh = 7000;
uint8_t laps = 0;
uint8_t max_counts = 0;

const uint8_t left_nslp_pin = 31;
const uint8_t left_dir_pin = 29;
const uint8_t left_pwm_pin = 40;

const uint8_t right_nslp_pin = 11;
const uint8_t right_dir_pin = 30;
const uint8_t right_pwm_pin = 39;

void setup()
{
    pinMode(left_nslp_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(left_pwm_pin, OUTPUT);

    pinMode(right_nslp_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);
    pinMode(right_pwm_pin, OUTPUT);

    digitalWrite(left_dir_pin, LOW);
    digitalWrite(left_nslp_pin, HIGH);

    digitalWrite(right_dir_pin, LOW);
    digitalWrite(right_nslp_pin, HIGH);

    ECE3_Init();

    // Serial.begin(9600);
    delay(2000);
}

void loop()
{
    ECE3_read_IR(sensor_values);
    int16_t error = 0;
    uint16_t uw_sum = 0;

    for (uint8_t i = 0; i < sensor_count; i++)
    {
        int16_t norm = (sensor_values[i] - sensor_minimums[i]) * 1000 / (sensor_maximums[i] - sensor_minimums[i]);
        if (norm < 0)
        {
            norm = 0;
        }
        uw_sum += norm;
        error += norm * sensor_weights[i];
    }

    error /= 4;

    if (uw_sum > track_end_tresh)
    {
        max_counts += 1;
        if (max_counts >= 2)
        {
            laps += 1;
            max_counts = 0;
            if (laps >= 2)
            {
                digitalWrite(right_nslp_pin, LOW);
                digitalWrite(left_nslp_pin, LOW);
            }
            digitalWrite(left_dir_pin, LOW);
            digitalWrite(right_dir_pin, HIGH);
            analogWrite(left_pwm_pin, 200);
            analogWrite(right_pwm_pin, 200);
            delay(300);
            analogWrite(left_pwm_pin, 0);
            analogWrite(right_pwm_pin, 0);
        }
    }
    else
    {
        max_counts = 0;
    }

    int16_t correction = kp * error + kd * (error - prev_error);
    if (abs(error) < low_error_thresh)
    {
        if (low_error_count < low_error_loops)
        {
            low_error_count += 1;
        }
    }
    else
    {
        low_error_count = 0;
    }
    int16_t speed = (low_error_count >= low_error_loops) ? fast_speed : base_speed;
    int16_t left_speed = speed - correction;
    int16_t right_speed = speed + correction;

    if (left_speed > 0)
    {
        digitalWrite(left_dir_pin, LOW);
    }
    else
    {
        digitalWrite(left_dir_pin, HIGH);
    }
    if (right_speed > 0)
    {
        digitalWrite(right_dir_pin, LOW);
    }
    else
    {
        digitalWrite(right_dir_pin, HIGH);
    }

    analogWrite(left_pwm_pin, min(abs(left_speed), 255));
    analogWrite(right_pwm_pin, min(abs(right_speed), 255));

    prev_error = error;
}
