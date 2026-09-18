#include <Wire.h>
#include <VL53L1X.h>

// Palm TOF. Streams "D,<mm>,<status>" for gripper_sensors/tof_node.
// Every failure keeps reporting on a 1 s beat -- a message printed once at
// boot is lost, because the CDC drops writes until a host opens the port.

static const uint32_t kPeriodMs = 33;
static const uint32_t kBudgetUs = 25000;
static const uint16_t kIoTimeoutMs = 500;

VL53L1X sensor;

static bool ready;
static bool wire_up;
static const char * fault = "";
static uint32_t last_beat;

/// Read the bus as plain GPIO. Wire's write path spins on INTFLAG.MB with no
/// timeout, so a line stuck low hangs it forever with nothing reported.
static bool lines_idle()
{
  if (wire_up) {
    Wire.end();
    wire_up = false;
  }
  pinMode(PIN_WIRE_SDA, INPUT);
  pinMode(PIN_WIRE_SCL, INPUT);
  delayMicroseconds(200);
  const int sda = digitalRead(PIN_WIRE_SDA);
  const int scl = digitalRead(PIN_WIRE_SCL);
  Serial.print("I,lines,SDA=");
  Serial.print(sda);
  Serial.print(",SCL=");
  Serial.println(scl);

  // The internal pull-up is ~40k: it wins against a floating line but loses
  // against a short, which separates "no external pull-up" from "held down".
  pinMode(PIN_WIRE_SDA, INPUT_PULLUP);
  pinMode(PIN_WIRE_SCL, INPUT_PULLUP);
  delayMicroseconds(500);
  Serial.print("I,pullup,SDA=");
  Serial.print(digitalRead(PIN_WIRE_SDA));
  Serial.print(",SCL=");
  Serial.println(digitalRead(PIN_WIRE_SCL));
  pinMode(PIN_WIRE_SDA, INPUT);
  pinMode(PIN_WIRE_SCL, INPUT);

  return sda == HIGH && scl == HIGH;
}

static void scan_bus()
{
  uint8_t found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("I,addr,0x");
      Serial.println(a, HEX);
      found++;
    }
  }
  Serial.print("I,scan,");
  Serial.println(found);
}

static void start()
{
  ready = false;
  if (!lines_idle()) {
    fault = "ERR,I2C line held low (short or stuck slave)";
    return;
  }
  Wire.begin();
  Wire.setClock(400000);
  wire_up = true;
  scan_bus();

  sensor.setTimeout(kIoTimeoutMs);
  if (!sensor.init()) {
    fault = "ERR,VL53L1X init failed (check wiring/power)";
    return;
  }
  if (!sensor.setDistanceMode(VL53L1X::Long) ||
      !sensor.setMeasurementTimingBudget(kBudgetUs)) {
    fault = "ERR,VL53L1X configuration rejected";
    return;
  }
  sensor.startContinuous(kPeriodMs);
  fault = "";
  ready = true;
  Serial.println("I,ready");
}

void setup()
{
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  Serial.println("I,boot,tof_vl53l1x");
  start();
}

void loop()
{
  if (!ready) {
    if (millis() - last_beat >= 1000) {
      last_beat = millis();
      Serial.println(fault);
      start();
    }
    return;
  }
  if (!sensor.dataReady()) {
    return;
  }
  const uint16_t mm = sensor.read(false);
  if (sensor.timeoutOccurred()) {
    fault = "ERR,VL53L1X read timeout";
    ready = false;
    return;
  }
  Serial.print("D,");
  Serial.print(mm);
  Serial.print(",");
  Serial.println((int)sensor.ranging_data.range_status);
}

