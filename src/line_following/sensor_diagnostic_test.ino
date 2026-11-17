#include <ECE3.h>

const uint8_t sensor_count = 8;
uint16_t sensor_values[sensor_count];
uint32_t sensorSum = 0;

void setup()
{
    Serial.begin(9600);
    ECE3_Init();

    Serial.println("ECE3 Sensor Diagnostic Test");
}

void loop()
{
    // Read all 8 IR sensors
    ECE3_read_IR(sensor_values);

    // Reset sum before every calculation
    sensorSum = 0;

    // Print sensor values
    for (uint8_t i = 0; i < sensor_count; i++)
    {

        Serial.print("S");
        Serial.print(i + 1);
        Serial.print(": ");

        Serial.print(sensor_values[i]);
        Serial.print("\t");

        // Calculate sensor sum
        sensorSum += sensor_values[i];
    }

    Serial.println();

    // Print total sensor sum
    Serial.print("Sensor Sum: ");
    Serial.println(sensorSum);

    delay(100);
}
