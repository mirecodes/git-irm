#include "hall_sensor.h"

// Function hall_sensor_get_field converts the measured voltage value to a magnetic field (in milli-tesla)

float hall_sensor_get_field(float voltage, float voltage_0)
{
   // Insert your code here
   int temperature = 25;
   int sensitivity_25 = 30;
   float stc = 0.0012;
   float sensitivity = sensitivity_25*(1+stc*(temperature-25));
   float B = (voltage - voltage_0) / sensitivity * 1000; // unit: [mT]

   return B;
}
