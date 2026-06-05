/* ============================================================
 *  AETHON-X  ·  IMU SERIAL STREAMER  (ESP32 Classic Bluetooth SPP)
 *  ESP32 sketch — feeds the aethon_visualizer.py Python visualiser.
 *
 *  WHY ESP32 INSTEAD OF HC-05:
 *    HC-05 uses Classic Bluetooth SPP but macOS 12+ dropped native
 *    /dev/tty.* support for SPP devices. The ESP32 exposes the same
 *    SPP profile — macOS sees it as /dev/tty.AETHON-X — so the
 *    Python visualiser works with zero changes (still uses pyserial).
 *
 *  HARDWARE WIRING (breadboard):
 *
 *     MPU6050  -->  ESP32
 *       VCC   ->  3.3V   (MPU6050 runs on 3.3V — do NOT use 5V)
 *       GND   ->  GND
 *       SCL   ->  GPIO 22
 *       SDA   ->  GPIO 21
 *       INT   ->  (not connected)
 *
 *     NO external Bluetooth module needed — ESP32 has BT built-in.
 *     Power the ESP32 from a USB power bank via its USB-C / micro-USB port.
 *
 *  FIRST-TIME PAIRING (macOS):
 *    1. Flash this sketch.
 *    2. Open System Settings → Bluetooth.
 *    3. The device "AETHON-X" appears — click Connect.
 *    4. After pairing, the port appears as /dev/tty.AETHON-X
 *       (run `ls /dev/tty.*` in Terminal to confirm).
 *    5. Run: python aethon_visualizer.py --port /dev/tty.AETHON-X --baud 115200
 *
 *  ARDUINO IDE SETUP:
 *    Board   : "ESP32 Dev Module" (or your specific ESP32 board variant)
 *    Library : BluetoothSerial — included in the ESP32 Arduino core, no install needed.
 *    Partition scheme: Use "Default" or "Huge APP" if you get low-memory errors.
 *
 *  OUTPUT FORMAT (13 CSV fields):
 *    angleX,angleY,angleZ,gxR,gyR,gzR,axg,ayg,waz,thrX,thrY,thrZ,landed\n
 *    • angleX/Y/Z : complementary-filter fused angles (degrees)
 *    • gxR/gyR/gzR: bias-corrected gyro rates (deg/s)
 *    • axg/ayg    : raw body-frame accelerometer (g)
 *    • waz        : world-frame vertical acceleration (g); 1.0 = hover
 *    • thrX/Y/Z   : body-frame thrust vector components (g)
 *    • landed     : 1 if still for >1.5 s, else 0
 *
 *  BAUD NOTE:
 *    BluetoothSerial baud is irrelevant for the BT link — SPP tunnels
 *    data over the radio at full speed regardless. The 115200 baud below
 *    only applies to the USB Serial monitor. Set --baud 115200 in Python.
 * ============================================================ */

#include <Wire.h>
#include <BluetoothSerial.h>

// ── Bluetooth SPP instance ──────────────────────────────────
BluetoothSerial btSerial;
const char* BT_NAME = "AETHON-X";   // device name visible during pairing

// ── Serial baud (USB monitor only — BT speed is automatic) ──
const uint32_t USB_BAUD = 115200;

// ── MPU6050 ─────────────────────────────────────────────────
const uint8_t MPU_ADDR  = 0x68;
const float   DT_TARGET = 0.01f;   // 100 Hz loop target
const float   ALPHA     = 0.98f;   // complementary filter weight (gyro trust)

// ── I²C pins (ESP32 default) ─────────────────────────────────
// GPIO 21 = SDA, GPIO 22 = SCL — Wire uses these automatically on ESP32.
// If your board variant differs, call Wire.begin(SDA_PIN, SCL_PIN) instead.

float angleX = 0, angleY = 0, angleZ = 0;
float gyroXoff = 0, gyroYoff = 0, gyroZoff = 0;
uint32_t lastMicros = 0;
uint32_t stillFrames = 0;    // counts consecutive still frames for landed detection


// ── MPU6050 helpers ─────────────────────────────────────────
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuRead(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                         // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                 // skip temperature bytes
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}


// ── Gyro calibration (keep board still for ~1 s on power-up) ─
void calibrateGyro() {
  const int N = 500;
  long sx = 0, sy = 0, sz = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < N; i++) {
    mpuRead(ax, ay, az, gx, gy, gz);
    sx += gx; sy += gy; sz += gz;
    delay(2);
  }
  gyroXoff = sx / (float)N;
  gyroYoff = sy / (float)N;
  gyroZoff = sz / (float)N;
}


// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(USB_BAUD);

  // Start Classic Bluetooth SPP — advertises as "AETHON-X"
  if (!btSerial.begin(BT_NAME)) {
    Serial.println(F("AETHON-X: BluetoothSerial init failed — check ESP32 board selection."));
    while (true) delay(1000);   // halt; re-flash with correct board
  }
  Serial.print(F("AETHON-X: Bluetooth SPP started, device name = "));
  Serial.println(BT_NAME);

  // I²C — ESP32 default pins: SDA=21, SCL=22
  Wire.begin();
  Wire.setClock(400000);        // 400 kHz fast-mode I2C

  mpuWrite(0x6B, 0x00);  // PWR_MGMT_1  — wake MPU6050
  mpuWrite(0x1B, 0x08);  // GYRO_CONFIG — ±500 dps  (65.5 LSB/dps)
  mpuWrite(0x1C, 0x10);  // ACCEL_CONFIG— ±8 g      (4096 LSB/g)
  delay(100);

  Serial.println(F("AETHON-X: calibrating gyro — keep board still..."));
  calibrateGyro();
  Serial.println(F("AETHON-X: calibration done. Waiting for BT connection..."));
  Serial.println(F("          Pair 'AETHON-X' in macOS Bluetooth settings,"));
  Serial.println(F("          then run: python aethon_visualizer.py --port /dev/tty.AETHON-X --baud 115200"));

  lastMicros = micros();
}


// ── Main loop ────────────────────────────────────────────────
void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  uint32_t now = micros();      // timestamp BEFORE I²C read for accurate dt
  mpuRead(ax, ay, az, gx, gy, gz);

  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0 || dt > 0.5f) dt = DT_TARGET;   // guard: micros() wrap or stall
  lastMicros = now;

  // ── Accelerometer in g ───────────────────────────────────
  float axg = ax / 4096.0f;
  float ayg = ay / 4096.0f;
  float azg = az / 4096.0f;

  // ── Accel magnitude guard ────────────────────────────────
  // Only trust the accelerometer for tilt correction when |a| ≈ 1g.
  // Linear motion (horizontal push, lift jerk) shifts |a| away from 1g —
  // freezing the accel blend during those moments prevents false tilt readings.
  float aMag = sqrt(axg*axg + ayg*ayg + azg*azg);

  // ── Accel-derived pitch & roll ───────────────────────────
  float accPitch = atan2(-axg, sqrt(ayg * ayg + azg * azg)) * 57.2957795f;
  float accRoll  = atan2(-ayg, azg)                          * 57.2957795f;

  // ── Gyro rates (deg/s, bias-corrected) ──────────────────
  float gxR = (gx - gyroXoff) / 65.5f;
  float gyR = (gy - gyroYoff) / 65.5f;
  float gzR = -((gz - gyroZoff) / 65.5f);

  // ── Complementary filter ─────────────────────────────────
  // blend = 0 during linear motion (|a| outside 0.85–1.15 g band).
  float blend = (aMag > 0.85f && aMag < 1.15f) ? (1.0f - ALPHA) : 0.0f;
  angleX = ALPHA * (angleX + gxR * dt) + blend * accPitch;
  angleY = ALPHA * (angleY + gyR * dt) + blend * accRoll;
  angleZ += gzR * dt;   // yaw: gyro-only (no magnetometer on MPU6050)

  // ── World-frame vertical acceleration (waz) ──────────────
  // Rotates the body-frame accel into world frame using fused pitch & roll,
  // then projects onto the world vertical axis.
  // waz = 1.0 g at hover/rest (gravity baseline), > 1.0 = ascending thrust,
  // < 1.0 = descending. Algebraically proved to always equal exactly 1.0
  // under pure gravity regardless of attitude — linear motion is the only
  // thing that shifts it, which is exactly what we want to detect.
  float pRad    = angleX * 0.017453f;
  float rRad    = angleY * 0.017453f;
  float sinP    = sin(pRad), cosP = cos(pRad);
  float sinR    = sin(rRad), cosR = cos(rRad);
  float waz     = -sinP * axg
                  + cosP * sinR * (-ayg)
                  + cosP * cosR * azg;

  // ── Thrust vector (body-frame, scaled by waz) ───────────────
  // Projects the drone's "up" axis into world frame components,
  // then scales by waz so magnitude reflects actual thrust level.
  float thrX =  sinP         * waz;   // forward/back component
  float thrY = -cosP * sinR  * waz;   // left/right component
  float thrZ =  cosP * cosR  * waz;   // vertical component

  // ── Landed detection ─────────────────────────────────────────
  // Drone is "landed" when gyro rates AND angles are all near-zero
  // for more than 150 consecutive frames (~1.5 s at 100 Hz).
  bool isStill = (abs(gxR)    < 2.0f && abs(gyR)    < 2.0f && abs(gzR)    < 2.0f
               && abs(angleX) < 3.0f && abs(angleY) < 3.0f);
  stillFrames  = isStill ? stillFrames + 1 : 0;
  uint8_t landed = (stillFrames > 150) ? 1 : 0;

  // ── Transmit 13-field CSV over Bluetooth SPP ─────────────────
  btSerial.print(angleX, 2); btSerial.print(',');
  btSerial.print(angleY, 2); btSerial.print(',');
  btSerial.print(angleZ, 2); btSerial.print(',');
  btSerial.print(gxR,    2); btSerial.print(',');
  btSerial.print(gyR,    2); btSerial.print(',');
  btSerial.print(gzR,    2); btSerial.print(',');
  btSerial.print(axg,    3); btSerial.print(',');
  btSerial.print(ayg,    3); btSerial.print(',');
  btSerial.print(waz,    3); btSerial.print(',');
  btSerial.print(thrX,   3); btSerial.print(',');
  btSerial.print(thrY,   3); btSerial.print(',');
  btSerial.print(thrZ,   3); btSerial.print(',');
  btSerial.println(landed);

  // ── Mirror to USB serial (for Arduino IDE Serial Monitor) ────
  Serial.print(angleX, 2); Serial.print(',');
  Serial.print(angleY, 2); Serial.print(',');
  Serial.print(angleZ, 2); Serial.print(',');
  Serial.print(gxR,    2); Serial.print(',');
  Serial.print(gyR,    2); Serial.print(',');
  Serial.print(gzR,    2); Serial.print(',');
  Serial.print(axg,    3); Serial.print(',');
  Serial.print(ayg,    3); Serial.print(',');
  Serial.print(waz,    3); Serial.print(',');
  Serial.print(thrX,   3); Serial.print(',');
  Serial.print(thrY,   3); Serial.print(',');
  Serial.print(thrZ,   3); Serial.print(',');
  Serial.println(landed);

  // ── Pace loop to ~100 Hz ─────────────────────────────────
  // delay() yields to FreeRTOS between iterations — the BT stack tasks
  // (RFCOMM keep-alives, TX flush, channel negotiation) run during this
  // window. The previous bare spin-wait burned 100% CPU for the full
  // 10 ms slice, starving those tasks and causing instant disconnect.
  uint32_t elapsed_ms = (micros() - now) / 1000;
  if (elapsed_ms < 10) delay(10 - elapsed_ms);
}
