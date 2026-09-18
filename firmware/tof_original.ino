#include <Wire.h>
#include "SparkFun_VL53L1X.h"

SFEVL53L1X sensor;

void setup()
{
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 <= 2999) {
    delay(10);
  }

  Wire.begin();
  Wire.setClock(400000);

  while (sensor.begin() != 0) {
    Serial.println("ERR,VL53L1X init failed (check wiring/power)");
    delay(1000);
  }

  sensor.setDistanceModeShort();
  sensor.setTimingBudgetInMs(33);
  sensor.setIntermeasurementPeriod(33);
  sensor.startRanging();
}

void loop()
{
  if (!sensor.checkForDataReady()) {
    delay(1);
    return;
  }
  uint16_t d = sensor.getDistance();
  uint8_t s = sensor.getRangeStatus();
  sensor.clearInterrupt();
  Serial.print("D,");
  Serial.print(d);
  Serial.print(",");
  Serial.println(s);
}

