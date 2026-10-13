/* ============================================================================
   MKS ESP32 FOC V1.0 + WK5515 + AS5600 (I2C)
   OPEN-LOOP CONTINUOUS  +  CLOSED-LOOP FOC WITH LIVE FIXES
   ----------------------------------------------------------------------------
   !!! build_opt.h in this folder:   -DSIMPLEFOC_ESP32_USELEDC

   WHY THIS SKETCH EXISTS
     You saw the motor run SMOOTHLY during alignment, then vibrate once FOC
     took over. Alignment is pure OPEN LOOP - SimpleFOC commands the electrical
     angle and the rotor follows, sensor ignored. So your power stage and
     commutation are fine. The fault is in the sensor feedback path.

     The usual culprit is an inverted sensor_direction: the loop becomes
     positive feedback and the motor fights itself instantly and violently.

   ----------------------------------------------------------------------------
   MODES
     o<n>   OPEN LOOP continuous at n rad/s. Exactly what alignment does, but
            it never stops. Smooth, no sensor involved. Start here.
     D      DIAGNOSTIC: open-loop rotation while printing commanded electrical
            angle vs sensor-derived electrical angle side by side. If the
            sensor is healthy the DIFFERENCE stays roughly constant. If it
            drifts, jumps at the 0 crossing, or runs backwards, that is your
            bug and the printout names it.
     a      align, then e1 + t<n> for closed-loop FOC
     F      FLIP sensor_direction and re-run alignment. One keystroke test for
            the inverted-feedback theory.
     Z<n>   nudge zero_electric_angle by n radians, live, while running.

   COMMANDS
     o<n>  open loop rad/s (o0 = stop, stays energised)
     D     open-loop sensor diagnostic, 15 s
     c     sensor check             P  measure pole pairs
     a     align                    F  flip direction + realign
     Z<n>  offset zero angle        k  print calibration
     U     align with automatic direction detection
     N<n>  set pole pairs
     e1/e0 enable/disable           x  stop
     t<n>  closed-loop target       m<n> mode 0 torque 1 vel 2 angle
     p/i/f vel P, I, Tf             l<n> voltage limit
     ?     status
   ============================================================================ */

#include <SimpleFOC.h>
#include <Wire.h>

#define PH_A       32
#define PH_B       33
#define PH_C       25
#define EN_PIN     22
// AS5600 on the board's encoder-0 I2C header
#define I2C_SDA  19          // silkscreen SDA0
#define I2C_SCL  18          // silkscreen SCL0
#define I2C_FREQ 400000
#define AS5600_ADDR 0x36

#define POLE_PAIRS     14
#define SUPPLY_VOLTAGE 24.0f
#define R_PHASE        2.81f
#define U_MAX          (SUPPLY_VOLTAGE / 1.7320508f)   // 13.86 V on a 24 V bus
#define KE             0.208f    // V per mech rad/s, phase peak (23 rpm/V)

// ---------------------------------------------------------------------------
// CURRENT LIMITING WITHOUT A CURRENT SENSOR
//   In steady state the phase current is
//        I = (Uq - omega * Ke) / R_phase
//   so capping Uq at
//        Uq_max = I_limit * R_phase + omega * Ke
//   holds the current at I_limit regardless of speed. At standstill that is
//   just I*R, which protects the windings. As the motor speeds up the back-EMF
//   term grows and the cap rises toward the full 13.86 V, so you get maximum
//   speed AND a bounded current. A fixed voltage limit cannot do both.
//
//   I_LIMIT_PEAK reference points for this motor (2.81 ohm phase):
//     2.26 A peak = 1.60 A RMS = datasheet NOMINAL  = 21 W  = 0.69 N.m
//     3.00 A peak = 2.12 A RMS                      = 38 W  = 0.91 N.m
//     4.00 A peak = 2.83 A RMS                      = 67 W  = 1.22 N.m
//     4.53 A peak = 3.20 A RMS = datasheet STALL    = 86 W  = 1.38 N.m
//     4.93 A peak = 3.49 A RMS = bus maximum        = 103 W (above stall spec)
//   Nominal assumes the stator is bonded into a housing. Yours is bare on a
//   bench with no heat path, so anything above ~2.5 A peak is a short burst.
// ---------------------------------------------------------------------------
#define I_LIMIT_DEFAULT 3.0f     // A peak. Change live with the 'A' command.

// SVPWM ceiling on 12 V is V_bus/sqrt(3) = 6.93 V.
// 6.93 / 2.81 ohm = 2.47 A peak = 1.75 A RMS = 0.75 N.m.
// That is the most this bus can deliver - above the motor's 1.6 A RMS nominal
// and about 55% of its 3.2 A stall rating. No firmware setting exceeds it.
// Open-loop and alignment run without back-EMF compensation, so these are
// plain I*R values. 3.0 A peak * 2.81 ohm = 8.4 V.
#define ALIGN_VOLTAGE  8.4f
#define OPENLOOP_VOLT  8.4f
#define RUN_VOLTAGE    U_MAX

#define THERMAL_R_CW   7.0f
#define THERMAL_TAU_S  120.0f
#define AMBIENT_C      25.0f

BLDCMotor      motor   = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver  = BLDCDriver3PWM(PH_A, PH_B, PH_C, EN_PIN);

// SimpleFOC's Commander is NOT used. It only dispatches on a '\n' terminator,
// so if the Serial Monitor line ending is set to CR or None, every command is
// silently dropped. The parser below accepts LF, CR, or no terminator at all
// (100 ms idle timeout) and echoes everything it receives.

// AS5600: 12-bit magnetic encoder over I2C at address 0x36.
// 4096 counts/rev = 0.088 deg per step. That is 4x coarser than the AS5048A's
// 14-bit reading, so the velocity estimate is noisier - hence more filtering.
TwoWire I2Cbus = TwoWire(0);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// ------------------------------------------------------------- STATE
bool     openloop  = false;     // true = open-loop continuous mode
float    ol_speed  = 0.0f;      // rad/s mechanical, open loop
float    ol_theta  = 0.0f;      // electrical radians
float    ol_ramp   = 3.0f;      // rad/s^2
float    ol_actual = 0.0f;
uint32_t ol_last_us = 0;

float    target   = 0.0f;
bool     aligned  = false;
bool     running  = false;      // closed loop running
uint32_t last_msg = 0;

float    temp_est  = AMBIENT_C;
uint32_t temp_last = 0;
uint32_t loop_count = 0, loop_hz = 0, loop_t0 = 0;
bool     stall_guard = true;
float    i_limit_peak = I_LIMIT_DEFAULT;   // A peak
bool     current_limiter = true;           // 'X0' disables, fixed Uq instead

// ------------------------------------------------------------- THERMAL
void updateThermal() {
  uint32_t now = millis();
  float dt = (now - temp_last) * 1e-3f;
  if (dt <= 0.0f) return;
  temp_last = now;
  float u = 0.0f;
  if (openloop)      u = OPENLOOP_VOLT;
  else if (running)  u = fabsf(motor.voltage.q);
  float irms = (u / R_PHASE) / 1.4142f;
  float p    = 3.0f * irms * irms * R_PHASE;
  temp_est += ((AMBIENT_C + p * THERMAL_R_CW) - temp_est) * (dt / THERMAL_TAU_S);
  if (temp_est < AMBIENT_C) temp_est = AMBIENT_C;
}

// ------------------------------------------------------------- OPEN LOOP

void openLoopStart(float w) {
  ol_speed = w;
  if (!openloop) {
    running = false;
    motor.disable();
    driver.init();
    digitalWrite(EN_PIN, HIGH);
    ol_actual  = 0.0f;
    ol_last_us = micros();
    openloop   = true;
    Serial.println(F("[OPEN LOOP] sensor ignored. This is what alignment does."));
  }
  Serial.print(F("open loop target ")); Serial.print(w, 2);
  Serial.print(F(" rad/s = ")); Serial.print(w * 9.5493f, 1); Serial.println(F(" rpm"));
}

void openLoopStop() {
  if (!openloop) return;
  motor.setPhaseVoltage(0, 0, 0);
  digitalWrite(EN_PIN, LOW);
  openloop = false;
  ol_speed = ol_actual = 0;
  Serial.println(F("[OPEN LOOP OFF]"));
}

void openLoopTick() {
  uint32_t now = micros();
  float dt = (now - ol_last_us) * 1e-6f;
  ol_last_us = now;
  if (dt <= 0 || dt > 0.05f) dt = 1e-4f;

  float dv = ol_ramp * dt;
  if      (ol_actual < ol_speed - dv) ol_actual += dv;
  else if (ol_actual > ol_speed + dv) ol_actual -= dv;
  else                                ol_actual  = ol_speed;

  ol_theta += ol_actual * POLE_PAIRS * dt;
  while (ol_theta >  _2PI) ol_theta -= _2PI;
  while (ol_theta < -_2PI) ol_theta += _2PI;

  motor.setPhaseVoltage(OPENLOOP_VOLT, 0.0f, ol_theta);
}

// ------------------------------------------------------- SENSOR DIAGNOSTIC
// Rotate open loop and compare the COMMANDED electrical angle with the
// electrical angle the SENSOR reports. A healthy sensor keeps the difference
// roughly constant. Drift, jumps, or a reversed trend is the bug.
void sensorDiagnostic() {
  Serial.println(F("DIAGNOSTIC - 15 s of slow open-loop rotation."));
  Serial.println(F("cmd_e = commanded elec angle, sens_e = sensor elec angle"));
  Serial.println(F("Watch DIFF. Constant = good. Drifting/jumping = the bug."));
  Serial.println();

  driver.init();
  digitalWrite(EN_PIN, HIGH);
  float th = 0.0f, w = 1.0f;                 // 1 rad/s, slow and clear
  uint32_t t0 = millis(), tp = micros(), lastP = 0;
  float diff_min = 1e9, diff_max = -1e9;
  float first_sens = 0; bool have_first = false;

  while (millis() - t0 < 15000) {
    uint32_t now = micros();
    float dt = (now - tp) * 1e-6f; tp = now;
    if (dt <= 0 || dt > 0.05f) dt = 1e-4f;
    th += w * POLE_PAIRS * dt;
    while (th > _2PI) th -= _2PI;
    motor.setPhaseVoltage(OPENLOOP_VOLT, 0.0f, th);

    sensor.update();
    float mech  = sensor.getAngle();
    float sens_e = fmodf(mech * POLE_PAIRS, _2PI);
    if (sens_e < 0) sens_e += _2PI;
    if (!have_first) { first_sens = sens_e; have_first = true; }

    float d = th - sens_e;
    while (d >  PI) d -= _2PI;
    while (d < -PI) d += _2PI;
    if (d < diff_min) diff_min = d;
    if (d > diff_max) diff_max = d;

    if (millis() - lastP >= 400) {
      Serial.print(F("  cmd_e ")); Serial.print(th * 57.2958f, 0);
      Serial.print(F("  sens_e ")); Serial.print(sens_e * 57.2958f, 0);
      Serial.print(F("  DIFF ")); Serial.print(d * 57.2958f, 0);
      Serial.print(F("  mech ")); Serial.print(mech * 57.2958f, 0);
      Serial.println(F(" deg"));
      lastP = millis();
    }
  }
  motor.setPhaseVoltage(0, 0, 0);
  digitalWrite(EN_PIN, LOW);

  Serial.println();
  Serial.print(F("DIFF range: ")); Serial.print(diff_min * 57.2958f, 0);
  Serial.print(F(" .. "));         Serial.print(diff_max * 57.2958f, 0);
  Serial.println(F(" deg"));
  float spread = (diff_max - diff_min) * 57.2958f;
  if (spread < 40.0f)
    Serial.println(F("STABLE - sensor tracks the field. Feedback path is sane."));
  else if (spread < 120.0f)
    Serial.println(F("WOBBLY - noisy or laggy sensor. Increase Tf, lower P."));
  else {
    Serial.println(F("BROKEN - the sensor does not track the field."));
    Serial.println(F("  Likely: wrong pole pair count ('P') or SPI wiring."));
  }
}

void i2cScan();          // defined below, used by sensorCheck and setup
void magnetStatus();

// ------------------------------------------------------------- SENSOR CHECK
// Number of electrical turns used by the pole-pair measurement. One turn is
// dominated by cogging error - the rotor settles into the nearest detent, not
// exactly where commanded. Ten turns dilutes that tenfold.
#define PP_TURNS 10

// SPI needs no pulse calibration. This just proves the link and reports the
// chip's own magnet diagnostics.
void sensorCheck() {
  openLoopStop();
  magnetStatus();
  Serial.println(F("SENSOR CHECK - turn the rotor by hand for 8 s."));
  float lo = 1e9, hi = -1e9;
  uint32_t t0 = millis(), lastP = 0;
  while (millis() - t0 < 8000) {
    sensor.update();
    float a = sensor.getMechanicalAngle();
    if (a < lo) lo = a;
    if (a > hi) hi = a;
    if (millis() - lastP > 400) {
      Serial.print(F("  angle ")); Serial.print(a * 57.2958f, 1);
      Serial.print(F(" deg   cumulative "));
      Serial.print(sensor.getAngle() * 57.2958f, 1);
      Serial.println(F(" deg"));
      lastP = millis();
    }
    delay(5);
  }
  Serial.print(F("range seen ")); Serial.print((hi - lo) * 57.2958f, 1);
  Serial.println(F(" deg"));
  if ((hi - lo) < 0.05f)
    Serial.println(F("*** SENSOR NOT MOVING - check SPI wiring ***"));
  else
    Serial.println(F("Sensor tracks. Good."));
}

void measurePolePairs() {
  openLoopStop();
  Serial.print(F("POLE PAIRS - ")); Serial.print(PP_TURNS);
  Serial.println(F(" electrical turns. Rotor must be free. ~40 s."));
  delay(500);
  driver.init(); digitalWrite(EN_PIN, HIGH);

  for (int i = 0; i < 400; i++) { motor.setPhaseVoltage(ALIGN_VOLTAGE, 0, 0); delay(2); }
  sensor.update(); sensor.update();
  float a0 = sensor.getAngle();

  float th = 0.0f;
  for (int t = 0; t < PP_TURNS; t++) {
    for (int i = 0; i < 180; i++) {
      th += _2PI / 180.0f;
      motor.setPhaseVoltage(ALIGN_VOLTAGE, 0, th);
      delay(10);
      sensor.update();
    }
    Serial.print(F("  turn ")); Serial.print(t + 1);
    Serial.print(F("  shaft ")); Serial.print((sensor.getAngle() - a0) * 57.2958f, 1);
    Serial.println(F(" deg"));
  }
  for (int i = 0; i < 300; i++) { motor.setPhaseVoltage(ALIGN_VOLTAGE, 0, th); delay(2); sensor.update(); }
  float a1 = sensor.getAngle();
  motor.setPhaseVoltage(0, 0, 0); digitalWrite(EN_PIN, LOW);

  float mech = fabsf(a1 - a0);
  Serial.print(F("total shaft movement ")); Serial.print(mech, 4);
  Serial.print(F(" rad = ")); Serial.print(mech * 57.2958f, 1); Serial.println(F(" deg"));
  if (mech < 0.05f) { Serial.println(F("*** rotor did not move ***")); return; }

  float pp = (PP_TURNS * _2PI) / mech;
  Serial.print(F(">>> measured pole pairs ")); Serial.print(pp, 3);
  Serial.print(F("  nearest ")); Serial.println((int)(pp + 0.5f));
  Serial.print(F("    currently using ")); Serial.println(motor.pole_pairs);
  if ((int)(pp + 0.5f) != motor.pole_pairs) {
    Serial.print(F("*** MISMATCH - send  N")); Serial.print((int)(pp + 0.5f));
    Serial.println(F("  to switch, then realign with 'a' ***"));
  } else {
    Serial.println(F("    matches. Good."));
  }
  Serial.println(F("Read the per-turn shaft column: the steady increment"));
  Serial.println(F("should be about 26 deg, which is 360/14."));
}

// Measure the REAL shaft speed by timing raw sensor revolutions, completely
// independent of SimpleFOC's filtered velocity estimate. If motor.shaft_velocity
// disagrees with this, the PID is being lied to and will stop pushing long
// before the motor is actually at target - which looks exactly like a speed
// ceiling with cool windings.
void trueSpeedTest() {
  if (!running && !openloop) {
    Serial.println(F("start the motor first (e1 + t2, or o2)"));
    return;
  }
  Serial.println(F("TRUE SPEED - 6 s, comparing raw sensor against the PID's"));
  Serial.println(F("velocity estimate."));
  sensor.update();
  float a_start = sensor.getAngle();
  uint32_t t0 = millis(), lastP = 0;
  float est_sum = 0; int est_n = 0;
  while (millis() - t0 < 6000) {
    if (openloop) openLoopTick(); else { motor.loopFOC(); if (running) motor.move(target); }
    sensor.update();
    est_sum += motor.shaft_velocity; est_n++;
    if (millis() - lastP >= 1000) {
      Serial.print(F("  shaft ")); Serial.print(sensor.getAngle() * 57.2958f, 0);
      Serial.print(F(" deg  est vel ")); Serial.print(motor.shaft_velocity, 2);
      Serial.print(F(" rad/s  Uq ")); Serial.println(motor.voltage.q, 2);
      lastP = millis();
    }
  }
  sensor.update();
  float moved = sensor.getAngle() - a_start;
  float real = moved / 6.0f;
  float est  = (est_n > 0) ? est_sum / est_n : 0;
  Serial.println(F("--------------------------------"));
  Serial.print(F("TRUE speed      ")); Serial.print(real, 3);
  Serial.print(F(" rad/s = ")); Serial.print(real * 9.5493f, 1); Serial.println(F(" rpm"));
  Serial.print(F("PID's estimate  ")); Serial.print(est, 3); Serial.println(F(" rad/s"));
  Serial.print(F("commanded       ")); Serial.print(target, 3); Serial.println(F(" rad/s"));
  Serial.print(F("loop rate       ")); Serial.print(loop_hz); Serial.println(F(" Hz"));
  float err = fabsf(est - real);
  if (err > 0.3f * fabsf(real) + 0.3f) {
    Serial.println(F(">>> THE ESTIMATE IS WRONG <<<"));
    Serial.println(F("The PID thinks it is at target when it is not, so it"));
    Serial.println(F("stops pushing. Raise Tf (f0.02) to smooth the estimate."));
  } else if (fabsf(real - target) > 0.3f && fabsf(motor.voltage.q) > motor.voltage_limit * 0.95f) {
    Serial.println(F(">>> GENUINELY SATURATED <<<"));
    Serial.println(F("Full voltage applied and still short of target: this is a"));
    Serial.println(F("real torque limit. Mechanical drag, or you need 24 V."));
  } else if (fabsf(real - target) > 0.3f) {
    Serial.println(F(">>> NOT SATURATED, NOT AT TARGET <<<"));
    Serial.println(F("The controller is holding back. Raise I (i15) and P (p0.5)."));
  } else {
    Serial.println(F("Tracking the target correctly."));
  }
}

// ---------------------------------------------------------- I2C DIAGNOSTICS

void i2cScan() {
  Serial.println(F("I2C scan on SDA=19 SCL=18 ..."));
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    I2Cbus.beginTransmission(a);
    if (I2Cbus.endTransmission() == 0) {
      Serial.print(F("  device at 0x"));
      if (a < 16) Serial.print('0');
      Serial.print(a, HEX);
      if (a == AS5600_ADDR) Serial.print(F("   <- AS5600"));
      Serial.println();
      found++;
    }
  }
  if (found == 0) {
    Serial.println(F("  *** NOTHING FOUND ***"));
    Serial.println(F("  check SDA/SCL not swapped, VCC is 3.3 V, GND shared,"));
    Serial.println(F("  and that the module has I2C pull-up resistors."));
  }
}

uint8_t as5600Read8(uint8_t reg) {
  I2Cbus.beginTransmission(AS5600_ADDR);
  I2Cbus.write(reg);
  if (I2Cbus.endTransmission(false) != 0) return 0xFF;
  I2Cbus.requestFrom((int)AS5600_ADDR, 1);
  return I2Cbus.available() ? I2Cbus.read() : 0xFF;
}

uint16_t as5600Read16(uint8_t reg) {
  I2Cbus.beginTransmission(AS5600_ADDR);
  I2Cbus.write(reg);
  if (I2Cbus.endTransmission(false) != 0) return 0xFFFF;
  I2Cbus.requestFrom((int)AS5600_ADDR, 2);
  if (I2Cbus.available() < 2) return 0xFFFF;
  uint16_t hi = I2Cbus.read(), lo = I2Cbus.read();
  return ((hi << 8) | lo) & 0x0FFF;
}

// AS5600 STATUS register 0x0B: bit5 MH (too strong), bit4 ML (too weak),
// bit3 MD (magnet detected). AGC at 0x1A, magnitude at 0x1B.
void magnetStatus() {
  uint8_t st = as5600Read8(0x0B);
  if (st == 0xFF) { Serial.println(F("*** AS5600 not responding ***")); return; }
  uint8_t agc = as5600Read8(0x1A);
  uint16_t mag = as5600Read16(0x1B);
  uint16_t raw = as5600Read16(0x0C);

  bool md = st & (1 << 5);
  bool ml = st & (1 << 4);
  bool mh = st & (1 << 3);

  Serial.println(F("---------- AS5600 MAGNET ----------"));
  Serial.print(F("raw angle  ")); Serial.print(raw);
  Serial.print(F(" / 4095 = ")); Serial.print(raw * 360.0f / 4096.0f, 2);
  Serial.println(F(" deg"));
  Serial.print(F("AGC        ")); Serial.print(agc);
  Serial.println(F("   (at 3.3 V the range is 0-128, mid is ideal)"));
  Serial.print(F("magnitude  ")); Serial.println(mag);
  Serial.print(F("magnet     "));
  if (!md)      Serial.println(F("NOT DETECTED - too far, or missing"));
  else if (ml)  Serial.println(F("detected but TOO WEAK - move it closer"));
  else if (mh)  Serial.println(F("detected but TOO STRONG - increase the gap"));
  else          Serial.println(F("detected, strength GOOD"));
  Serial.println(F("The magnet must be DIAMETRICALLY magnetised, centred on"));
  Serial.println(F("the shaft axis, 0.5 to 3 mm from the chip."));
  Serial.println(F("-----------------------------------"));
}

// ============================================================================
// AUTO-COMMUTATION SWEEP
// Determines BOTH sensor_direction and zero_electric_angle by experiment,
// measuring actual shaft speed. It does not rely on initFOC's alignment, which
// on this motor is corrupted by static friction (zero angle came out 0.16,
// 2.22, 2.96 and 3.51 on four consecutive tries - it should repeat to ~0.1).
//
// Method: for each direction, step zero_electric_angle through a full
// electrical revolution at fixed q-axis voltage and record the speed. The
// combination that spins fastest is correct. Then refine around the winner.
// ============================================================================

float sweepSpeedAt(float zero, Direction dir, float uq, uint16_t settle_ms, uint16_t meas_ms) {
  motor.zero_electric_angle = zero;
  motor.sensor_direction    = dir;
  uint32_t t = millis();
  while (millis() - t < settle_ms) { motor.loopFOC(); motor.move(uq); }
  sensor.update();
  float a0 = sensor.getAngle();
  t = millis();
  while (millis() - t < meas_ms) { motor.loopFOC(); motor.move(uq); }
  sensor.update();
  float raw = (sensor.getAngle() - a0) / (meas_ms * 1e-3f);
  // CRITICAL: SimpleFOC reports shaft_velocity = sensor_direction * raw.
  // The PID regulates on THAT, not on the raw sensor. Measuring raw here
  // picked a commutation angle 180 deg out, which inverted the feedback and
  // made the loop run away instead of regulating.
  float dirsign = (dir == Direction::CW) ? 1.0f : -1.0f;
  return dirsign * raw;
}

void autoCommutation() {
  openLoopStop();

  if (temp_est > 60.0f) {
    Serial.print(F("Stator is at ~")); Serial.print(temp_est, 0);
    Serial.println(F(" C. Let it cool below 60 C first, then retry."));
    return;
  }

  Serial.println(F("AUTO-COMMUTATION - about 70 s. Rotor must be free."));
  Serial.println(F("It will spin at many different speeds. That is the point."));

  MotionControlType saved_mode = motor.controller;
  float saved_limit = motor.voltage_limit;
  float saved_target = target;

  motor.controller    = MotionControlType::torque;
  motor.voltage_limit = U_MAX;
  float uq = U_MAX * 0.7f;                 // 4.85 V, 1.7 A peak
  motor.enable();
  running  = false;

  const int COARSE = 16;                   // 22.5 electrical degrees per step
  // Pick the MOST POSITIVE speed, not the largest magnitude: positive Uq must
  // produce positive rotation or the feedback sign is inverted. And do not
  // compare against a -1e9 sentinel with fabsf() - that was the bug that threw
  // away a perfectly good sweep result.
  float best_speed = -1e30f, best_zero = 0;
  Direction best_dir = Direction::CCW;
  bool found_any = false;

  for (int d = 0; d < 2; d++) {
    Direction dir = (d == 0) ? Direction::CW : Direction::CCW;
    Serial.print(F("--- direction "));
    Serial.print(d == 0 ? F("CW") : F("CCW"));
    Serial.println(F(" ---"));
    for (int k = 0; k < COARSE; k++) {
      float z = (_2PI * k) / COARSE;
      float spd = sweepSpeedAt(z, dir, uq, 500, 600);
      Serial.print(F("  zero ")); Serial.print(z * 57.2958f, 0);
      Serial.print(F(" deg   speed ")); Serial.print(spd, 2);
      Serial.print(F(" rad/s = ")); Serial.print(spd * 9.5493f, 0);
      Serial.println(F(" rpm"));
      if (spd > best_speed) { best_speed = spd; best_zero = z; best_dir = dir; found_any = true; }
    }
  }

  Serial.println(F("--- refining around the winner ---"));
  float coarse_step = _2PI / COARSE;
  float refined_zero = best_zero, refined_speed = best_speed;
  for (int k = -3; k <= 3; k++) {
    if (k == 0) continue;
    float z = best_zero + (coarse_step * k) / 4.0f;
    while (z < 0)    z += _2PI;
    while (z > _2PI) z -= _2PI;
    float spd = sweepSpeedAt(z, best_dir, uq, 400, 600);
    Serial.print(F("  zero ")); Serial.print(z * 57.2958f, 1);
    Serial.print(F(" deg   speed ")); Serial.print(spd, 2); Serial.println(F(" rad/s"));
    if (spd > refined_speed) { refined_speed = spd; refined_zero = z; }
  }

  motor.move(0);
  motor.disable();
  motor.controller    = saved_mode;
  motor.voltage_limit = saved_limit;
  target = saved_target;

  Serial.println(F("======================================"));
  if (!found_any || refined_speed < 0.5f) {
    Serial.println(F("*** NO COMMUTATION OFFSET PRODUCED ROTATION ***"));
    Serial.println(F("Every angle in both directions gave near-zero speed."));
    Serial.println(F("This is NOT a tuning problem. Check, in order:"));
    Serial.println(F("  1. can you spin the rotor freely BY HAND, power off?"));
    Serial.println(F("  2. all three phases connected? measure 5.62 ohm across"));
    Serial.println(F("     each pair of motor leads with everything off"));
    Serial.println(F("  3. has the magnet moved relative to the rotor?"));
    Serial.println(F("  4. did the stator overheat earlier and lose magnetism?"));
    Serial.println(F("Commutation is NOT applied - previous values kept."));
    aligned = false;
    return;
  }

  motor.zero_electric_angle = refined_zero;
  motor.sensor_direction    = best_dir;
  aligned = true;

  Serial.print(F("zero_electric_angle ")); Serial.println(refined_zero, 4);
  Serial.print(F("sensor_direction    "));
  Serial.println(best_dir == Direction::CW ? F("CW") : F("CCW"));
  Serial.print(F("speed achieved      ")); Serial.print(refined_speed, 2);
  Serial.print(F(" rad/s = ")); Serial.print(refined_speed * 9.5493f, 0);
  Serial.println(F(" rpm at 4.85 V"));
  Serial.println(F("Applied. Send  e1  then  t2."));
  Serial.println(F("To keep these, hardcode them - see 'k'."));
}

// STEP RESPONSE - measures how the velocity loop actually behaves.
void stepTest(float lo, float hi) {
  if (!running) { Serial.println(F("enable first (e1)")); return; }
  motor.controller = MotionControlType::velocity;
  Serial.print(F("STEP TEST  ")); Serial.print(lo, 2);
  Serial.print(F(" -> ")); Serial.print(hi, 2); Serial.println(F(" rad/s"));

  target = lo;
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) { motor.loopFOC(); motor.move(target); }

  target = hi;
  t0 = millis();
  float peak = lo, settle_ms = -1;
  float band = fabsf(hi - lo) * 0.05f + 0.05f;
  uint32_t lastP = 0;
  while (millis() - t0 < 4000) {
    motor.loopFOC(); motor.move(target);
    float v = motor.shaft_velocity;
    if (fabsf(v - lo) > fabsf(peak - lo)) peak = v;
    if (settle_ms < 0 && fabsf(v - hi) < band) settle_ms = millis() - t0;
    if (millis() - lastP >= 200) {
      Serial.print(F("  t=")); Serial.print(millis() - t0);
      Serial.print(F(" ms  vel ")); Serial.print(v, 2);
      Serial.print(F("  Uq ")); Serial.println(motor.voltage.q, 2);
      lastP = millis();
    }
  }
  float overshoot = (fabsf(hi - lo) > 1e-3f)
                  ? (peak - hi) / fabsf(hi - lo) * 100.0f : 0.0f;
  Serial.println(F("--------------------------------------"));
  Serial.print(F("peak       ")); Serial.print(peak, 2); Serial.println(F(" rad/s"));
  Serial.print(F("overshoot  ")); Serial.print(overshoot, 0); Serial.println(F(" %"));
  Serial.print(F("settled in "));
  if (settle_ms < 0) Serial.println(F("NEVER - it did not reach the target"));
  else { Serial.print(settle_ms, 0); Serial.println(F(" ms")); }
  Serial.print(F("gains  P ")); Serial.print(motor.PID_velocity.P, 3);
  Serial.print(F("  I ")); Serial.print(motor.PID_velocity.I, 2);
  Serial.print(F("  D ")); Serial.print(motor.PID_velocity.D, 4);
  Serial.print(F("  Tf ")); Serial.println(motor.LPF_velocity.Tf, 4);
  Serial.println(F("overshoot > 25%  -> lower P or raise D"));
  Serial.println(F("slow to settle   -> raise I"));
  Serial.println(F("buzzy / noisy    -> raise Tf, lower P"));
  target = lo;
}

// COAST-DOWN: cut power at speed and time the deceleration. With rotor inertia
// J = 456 g.cm^2 = 4.56e-5 kg.m^2, friction torque = J * dw/dt. A free rotor
// coasts for seconds; a rubbing one stops almost instantly.
void coastDown() {
  if (!running) { Serial.println(F("enable first (e1)")); return; }
  Serial.println(F("COAST-DOWN - spinning up, then cutting power."));
  motor.controller = MotionControlType::velocity;
  target = 5.0f;
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) { motor.loopFOC(); motor.move(target); }

  sensor.update();
  float w0 = motor.shaft_velocity;
  if (fabsf(w0) < 1.0f) { Serial.println(F("did not reach speed - aborting")); target = 0; return; }

  motor.disable();
  uint32_t tstart = micros();
  float w = w0;
  float last_print = 0;
  Serial.print(F("cut at ")); Serial.print(w0, 2); Serial.println(F(" rad/s"));
  while (fabsf(w) > 0.15f * fabsf(w0) && (micros() - tstart) < 10000000UL) {
    motor.loopFOC();
    w = motor.shaft_velocity;
    float ms = (micros() - tstart) / 1000.0f;
    if (ms - last_print > 100) {
      Serial.print(F("  ")); Serial.print(ms, 0);
      Serial.print(F(" ms   ")); Serial.print(w, 2); Serial.println(F(" rad/s"));
      last_print = ms;
    }
  }
  float dt = (micros() - tstart) * 1e-6f;
  motor.enable();
  target = 0;

  const float J = 4.56e-5f;                 // kg.m^2, from the datasheet
  float decel = (fabsf(w0) - fabsf(w)) / dt;
  float tq    = J * decel;

  Serial.println(F("--------------------------------------"));
  Serial.print(F("coasted from ")); Serial.print(w0, 2);
  Serial.print(F(" to ")); Serial.print(w, 2);
  Serial.print(F(" rad/s in ")); Serial.print(dt, 3); Serial.println(F(" s"));
  Serial.print(F("deceleration ")); Serial.print(decel, 1); Serial.println(F(" rad/s^2"));
  Serial.print(F("friction torque ")); Serial.print(tq, 4); Serial.println(F(" N.m"));
  Serial.print(F("  = ")); Serial.print(tq / 0.43f * 1.414f, 2);
  Serial.println(F(" A peak just to overcome drag"));
  Serial.println();
  if (dt > 2.0f)      Serial.println(F("FREE - drag is negligible. Speed limit is voltage."));
  else if (dt > 0.5f) Serial.println(F("SOME DRAG - a 24 V supply is your main gain."));
  else {
    Serial.println(F("HEAVY DRAG - the rotor is being braked mechanically."));
    Serial.println(F("Check: is it concentric? rubbing the stator? bearing"));
    Serial.println(F("binding? A free frameless rotor coasts for seconds."));
    Serial.println(F("Fixing this is worth more than any tuning or supply change."));
  }
}

void showGains() {
  Serial.println(F("--------------- GAINS ---------------"));
  Serial.print(F("velocity  P  ")); Serial.println(motor.PID_velocity.P, 4);
  Serial.print(F("          I  ")); Serial.println(motor.PID_velocity.I, 4);
  Serial.print(F("          D  ")); Serial.println(motor.PID_velocity.D, 5);
  Serial.print(F("       ramp  ")); Serial.println(motor.PID_velocity.output_ramp, 0);
  Serial.print(F("         Tf  ")); Serial.println(motor.LPF_velocity.Tf, 5);
  Serial.print(F("angle     P  ")); Serial.println(motor.P_angle.P, 3);
  Serial.print(F("       LPF   ")); Serial.println(motor.LPF_angle.Tf, 5);
  Serial.print(F("vel limit    ")); Serial.println(motor.velocity_limit, 2);
  Serial.print(F("U limit      ")); Serial.println(motor.voltage_limit, 2);
  Serial.println(F("commands: p i d f R  (velocity)   g G (angle)"));
  Serial.println(F("-------------------------------------"));
}

void doAlign(int dirmode) {
  // dirmode: 0 = keep whatever is set, 1 = FLIP it, 2 = let SimpleFOC detect
  openLoopStop();
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  if (dirmode == 1) {
    motor.sensor_direction = (motor.sensor_direction == Direction::CCW)
                           ? Direction::CW : Direction::CCW;
    Serial.print(F("FORCING direction "));
    Serial.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
    Serial.println(F("(auto-detection is skipped when direction is preset)"));
  } else if (dirmode == 2) {
    motor.sensor_direction = Direction::UNKNOWN;
    Serial.println(F("clearing direction - SimpleFOC will auto-detect"));
  }
  Serial.println(F("initFOC..."));
  if (motor.initFOC()) {
    aligned = true;
    Serial.print(F("ALIGNED zero=")); Serial.print(motor.zero_electric_angle, 4);
    Serial.print(F(" dir=")); Serial.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
    Serial.println(F("e1 then t2 for closed loop."));
  } else {
    aligned = false;
    Serial.println(F("*** initFOC FAILED ***"));
  }
}

// Your motor needed ~1.5 A open loop just to break away from standstill. A
// velocity PID starting from zero error takes time to wind up to that. This
// gives it a short open-loop nudge first so the loop takes over a rotor that
// is already moving, which is much easier than starting one that is stuck.
bool  kick_enabled = true;
float kick_speed   = 1.5f;    // rad/s
uint16_t kick_ms   = 400;

void breakawayKick() {
  if (!kick_enabled) return;
  Serial.println(F("breakaway kick..."));
  driver.init(); digitalWrite(EN_PIN, HIGH);
  float th = 0; uint32_t t0 = millis(), tp = micros();
  while (millis() - t0 < kick_ms) {
    uint32_t now = micros();
    float dt = (now - tp) * 1e-6f; tp = now;
    if (dt <= 0 || dt > 0.05f) dt = 1e-4f;
    th += kick_speed * motor.pole_pairs * dt;
    while (th > _2PI) th -= _2PI;
    motor.setPhaseVoltage(OPENLOOP_VOLT, 0.0f, th);
    sensor.update();
  }
}

void motorRun(bool on) {
  if (on) {
    if (!aligned) { Serial.println(F("align first (a)")); return; }
    openLoopStop();
    motor.enable();
    running = true;
    Serial.println(F("[CLOSED LOOP ON]"));
  } else {
    running = false; target = 0; motor.disable();
    Serial.println(F("[OFF]"));
  }
}

void status() {
  Serial.println(F("============ STATUS ============"));
  Serial.print(F("open loop   ")); Serial.println(openloop ? F("RUNNING") : F("off"));
  if (openloop) {
    Serial.print(F("  ol speed  ")); Serial.print(ol_actual, 2);
    Serial.print(F(" / ")); Serial.print(ol_speed, 2); Serial.println(F(" rad/s"));
  }
  Serial.print(F("closed loop ")); Serial.println(running ? F("RUNNING") : F("off"));
  Serial.print(F("aligned     ")); Serial.println(aligned ? F("yes") : F("no"));
  Serial.print(F("direction   "));
  Serial.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
  Serial.print(F("zero angle  ")); Serial.println(motor.zero_electric_angle, 4);
  sensor.update();
  Serial.print(F("sensor mech ")); Serial.print(sensor.getAngle() * 57.2958f, 1); Serial.println(F(" deg"));
  Serial.print(F("shaft vel   ")); Serial.print(motor.shaft_velocity, 2); Serial.println(F(" rad/s"));
  Serial.print(F("Uq          ")); Serial.print(motor.voltage.q, 2);
  Serial.print(F(" of ")); Serial.print(motor.voltage_limit, 2); Serial.println(F(" V"));
  Serial.print(F("est temp    ")); Serial.print(temp_est, 0); Serial.println(F(" C"));
  Serial.print(F("I limit     ")); Serial.print(i_limit_peak, 2);
  Serial.print(F(" A peak = ")); Serial.print(i_limit_peak / 1.4142f, 2);
  Serial.print(F(" A RMS  limiter ")); Serial.println(current_limiter ? F("ON") : F("OFF"));
  Serial.print(F("Uq cap now  ")); Serial.print(motor.voltage_limit, 2);
  Serial.print(F(" of ")); Serial.print(U_MAX, 2); Serial.println(F(" V"));
  Serial.print(F("loop rate   ")); Serial.print(loop_hz);
  Serial.println(F(" Hz   (want > 1000 for good FOC)"));
  Serial.print(F("vel PID   P ")); Serial.print(motor.PID_velocity.P, 3);
  Serial.print(F(" I ")); Serial.print(motor.PID_velocity.I, 3);
  Serial.print(F(" Tf ")); Serial.println(motor.LPF_velocity.Tf, 4);
  Serial.println(F("================================"));
}

// ============================================================ SERIAL PARSER

String   rxline;
uint32_t rx_last_ms = 0;
bool     echo_bytes = false;      // 'B' toggles raw byte echo

void printHelp() {
  Serial.println(F("---------------- COMMANDS ----------------"));
  Serial.println(F("  o<n>  open loop rad/s      x     stop everything"));
  Serial.println(F("  c     sensor check         D     sensor diagnostic"));
  Serial.println(F("  P     measure pole pairs   N<n>  set pole pairs"));
  Serial.println(F("  a     align                U     align + auto direction"));
  Serial.println(F("  F     flip direction       Z<n>  nudge zero angle"));
  Serial.println(F("  k     print calibration    V     true speed vs estimate"));
  Serial.println(F("  e1/e0 enable / disable     t<n>  target"));
  Serial.println(F("  m<n>  mode 0 torq 1 vel 2 ang"));
  Serial.println(F("  p<n> P   i<n> I   d<n> D   f<n> Tf   g<n> angleP"));
  Serial.println(F("  l<n>  voltage limit V      A<n>  limit by amps peak"));
  Serial.println(F("  R<n>  PID output ramp      K<n>  kick ms, -1 toggles"));
  Serial.println(F("  S     AUTO-COMMUTATION: finds direction AND zero angle"));
  Serial.println(F("  W<n>  step response test to n rad/s   G  show all gains"));
  Serial.println(F("  y<n>  angle LPF Tf         L<n>  velocity limit"));
  Serial.println(F("  A<n>  current limit, amps PEAK   X1/X0 limiter on/off"));
  Serial.println(F("  Q1/Q0 stall guard on / off"));
  Serial.println(F("  M     coast-down: measures mechanical friction"));
  Serial.println(F("  I     I2C scan + AS5600 magnet status"));
  Serial.println(F("  w     apply measured commutation (CCW, 0 deg)"));
  Serial.println(F("  C<n>  set zero angle rad    j1/j0  direction CW/CCW"));
  Serial.println(F("  B     toggle raw byte echo  ?     status   h  this help"));
  Serial.println(F("-----------------------------------------"));
}

void dispatch(String line) {
  line.trim();
  Serial.print(F(">>> RECEIVED: \"")); Serial.print(line);
  Serial.print(F("\"  (")); Serial.print(line.length()); Serial.println(F(" chars)"));
  if (line.length() == 0) { Serial.println(F("    empty - ignored")); return; }

  char  c      = line.charAt(0);
  bool  hasnum = line.length() > 1;
  float v      = hasnum ? line.substring(1).toFloat() : 0.0f;
  if (hasnum) { Serial.print(F("    cmd '")); Serial.print(c);
                Serial.print(F("'  value ")); Serial.println(v, 4); }
  else        { Serial.print(F("    cmd '")); Serial.print(c); Serial.println(F("'  (no value)")); }

  switch (c) {
    case 'o':
      if (fabsf(v) < 1e-3f && openloop) { ol_speed = 0;
        Serial.println(F("open loop holding at 0 - 'x' cuts power")); }
      else openLoopStart(v);
      break;

    case 'x': openLoopStop(); motorRun(false); Serial.println(F("STOP")); break;
    case 'e': motorRun(v > 0.5f); break;

    case 't': {
      float old = target;
      target = v;
      Serial.print(F("target ")); Serial.print(target, 3);
      Serial.print(F(" rad/s = ")); Serial.print(target * 9.5493f, 1);
      Serial.println(F(" rpm"));
      if (running && motor.controller == MotionControlType::velocity
          && fabsf(old) < 0.05f && fabsf(target) > 0.05f
          && fabsf(motor.shaft_velocity) < 0.3f) {
        kick_speed = (target > 0) ? 1.5f : -1.5f;
        breakawayKick();
      }
      break;
    }

    case 'c': sensorCheck();        break;
    case 'D': openLoopStop(); sensorDiagnostic(); break;
    case 'P': measurePolePairs();   break;
    case 'V': trueSpeedTest();      break;
    case 'a': doAlign(0);           break;
    case 'F': doAlign(1);           break;
    case 'U': doAlign(2);           break;
    case '?': status();             break;
    case 'h': printHelp();          break;
    case 'S': autoCommutation();    break;
    case 'G': showGains();          break;
    case 'M': coastDown();          break;
    case 'I': i2cScan(); magnetStatus(); break;

    case 'C':   // set zero_electric_angle directly, in radians
      motor.zero_electric_angle = v;
      aligned = true;
      Serial.print(F("zero_electric_angle = ")); Serial.print(v, 4);
      Serial.print(F(" rad = ")); Serial.print(v * 57.2958f, 1);
      Serial.println(F(" deg"));
      break;

    case 'j':   // set sensor direction: j1 = CW, j0 = CCW
      motor.sensor_direction = (v > 0.5f) ? Direction::CW : Direction::CCW;
      aligned = true;
      Serial.print(F("sensor_direction = "));
      Serial.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
      break;

    case 'w':   // apply the values your sweep actually measured
      motor.sensor_direction    = Direction::CCW;
      motor.zero_electric_angle = 0.0f;
      aligned = true;
      Serial.println(F("WARNING: these values came from the AS5048A."));
      Serial.println(F("They are almost certainly wrong for the AS5600."));
      Serial.println(F("Run 'S' instead unless you have a reason not to."));
      Serial.println(F("  sensor_direction    CCW"));
      Serial.println(F("  zero_electric_angle 0.0 rad"));
      Serial.println(F("(180 deg was the same commutation with inverted sign -"));
      Serial.println(F(" it spun fine but the PID could not regulate it.)"));
      Serial.println(F("Send e1 then t2."));
      break;
    case 'Q': stall_guard = (v > 0.5f);
              Serial.print(F("stall guard "));
              Serial.println(stall_guard ? F("ON") : F("OFF")); break;
    case 'W': stepTest(hasnum ? v * 0.25f : 0.5f, hasnum ? v : 2.0f); break;
    case 'y': motor.LPF_angle.Tf = v;
              Serial.print(F("angle LPF Tf ")); Serial.println(motor.LPF_angle.Tf, 5); break;
    case 'L': motor.velocity_limit = v;
              Serial.print(F("velocity limit ")); Serial.println(motor.velocity_limit, 2); break;

    case 'B': echo_bytes = !echo_bytes;
              Serial.print(F("raw byte echo "));
              Serial.println(echo_bytes ? F("ON") : F("off")); break;

    case 'N': {
      int n = (int)(v + 0.5f);
      if (n < 1 || n > 30) { Serial.println(F("pole pairs must be 1..30")); break; }
      motor.pole_pairs = n; aligned = false;
      Serial.print(F("pole pairs now ")); Serial.println(n);
      Serial.println(F("realign with 'a'"));
      break;
    }

    case 'Z': motor.zero_electric_angle += v;
              Serial.print(F("zero angle now "));
              Serial.println(motor.zero_electric_angle, 4); break;

    case 'k': Serial.print(F("zero_electric_angle "));
              Serial.println(motor.zero_electric_angle, 6);
              Serial.print(F("sensor_direction    "));
              Serial.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
              Serial.print(F("pole_pairs          "));
              Serial.println(motor.pole_pairs); break;

    case 'm': target = 0;
              motor.controller = (v < 0.5f) ? MotionControlType::torque
                               : (v < 1.5f) ? MotionControlType::velocity
                                            : MotionControlType::angle;
              Serial.print(F("mode "));
              Serial.println((v < 0.5f) ? F("0 torque (Uq in volts)")
                           : (v < 1.5f) ? F("1 velocity (rad/s)")
                                        : F("2 angle (rad)"));
              Serial.println(F("target reset to 0")); break;

    case 'p': motor.PID_velocity.P = v;
              Serial.print(F("P ")); Serial.println(motor.PID_velocity.P, 4); break;
    case 'i': motor.PID_velocity.I = v;
              Serial.print(F("I ")); Serial.println(motor.PID_velocity.I, 4); break;
    case 'd': motor.PID_velocity.D = v;
              Serial.print(F("D ")); Serial.println(motor.PID_velocity.D, 5); break;
    case 'f': motor.LPF_velocity.Tf = v;
              Serial.print(F("Tf ")); Serial.println(motor.LPF_velocity.Tf, 5); break;
    case 'g': motor.P_angle.P = v;
              Serial.print(F("angle P ")); Serial.println(motor.P_angle.P, 3); break;
    case 'R': motor.PID_velocity.output_ramp = v;
              Serial.print(F("output ramp ")); Serial.println(motor.PID_velocity.output_ramp, 0); break;

    case 'l': motor.voltage_limit = v;
              if (motor.voltage_limit > U_MAX) { motor.voltage_limit = U_MAX;
                Serial.println(F("clamped to the SVPWM ceiling")); }
              Serial.print(F("Ulim ")); Serial.print(motor.voltage_limit, 2);
              Serial.print(F(" V = ")); Serial.print(motor.voltage_limit / R_PHASE, 2);
              Serial.println(F(" A peak")); break;

    case 'A': {
      float imax = U_MAX / R_PHASE;
      if (v > imax) { Serial.print(F("! ")); Serial.print(v, 2);
        Serial.print(F(" A exceeds the bus maximum ")); Serial.print(imax, 2);
        Serial.println(F(" A peak")); v = imax; }
      if (v < 0.2f) v = 0.2f;
      i_limit_peak = v;
      float w = 3.0f * powf(v / 1.4142f, 2) * R_PHASE;
      Serial.print(F("current limit ")); Serial.print(i_limit_peak, 2);
      Serial.print(F(" A peak = ")); Serial.print(i_limit_peak / 1.4142f, 2);
      Serial.println(F(" A RMS"));
      Serial.print(F("  stall power ")); Serial.print(w, 1);
      Serial.print(F(" W, supply draw ")); Serial.print(w / SUPPLY_VOLTAGE, 2);
      Serial.println(F(" A"));
      Serial.print(F("  torque ")); Serial.print(i_limit_peak / 1.4142f * 0.43f, 2);
      Serial.println(F(" N.m"));
      if (i_limit_peak > 2.3f)
        Serial.println(F("  ABOVE the 1.6 A RMS nominal - short bursts only"));
      break;
    }

    case 'X': current_limiter = (v > 0.5f);
      Serial.print(F("current limiter "));
      Serial.println(current_limiter ? F("ON (Uq = I*R + bemf)") : F("OFF (fixed Uq)"));
      if (!current_limiter) {
        motor.voltage_limit = U_MAX;
        Serial.println(F("  voltage limit set to the bus max - watch the heat"));
      }
      break;

    case 'K': if (v < 0) { kick_enabled = !kick_enabled;
                Serial.print(F("kick ")); Serial.println(kick_enabled ? F("on") : F("off")); }
              else { kick_ms = (uint16_t)v;
                Serial.print(F("kick ")); Serial.print(kick_ms); Serial.println(F(" ms")); }
              break;

    default:
      Serial.print(F("    *** UNKNOWN COMMAND '")); Serial.print(c);
      Serial.println(F("' - send h for the list ***"));
      break;
  }
}

// Accepts LF, CR, CRLF, or no terminator at all. If bytes arrive and then the
// line goes quiet for 100 ms, it dispatches anyway - so the Serial Monitor's
// line-ending setting cannot stop commands from working.
void serialPoll() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (echo_bytes) {
      Serial.print(F("[byte ")); Serial.print((int)(uint8_t)ch);
      if (ch >= 32 && ch < 127) { Serial.print(F(" '")); Serial.print(ch); Serial.print(F("'")); }
      Serial.println(F("]"));
    }
    if (ch == '\n' || ch == '\r') {
      if (rxline.length()) { String l = rxline; rxline = ""; dispatch(l); }
    } else {
      rxline += ch;
      rx_last_ms = millis();
      if (rxline.length() > 40) { rxline = ""; Serial.println(F("(line too long, discarded)")); }
    }
  }
  if (rxline.length() && millis() - rx_last_ms > 100) {
    String l = rxline; rxline = ""; dispatch(l);
  }
}

// ---------------------------------------------------------------- SETUP
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("=== WK5515 - OPEN LOOP + FOC ==="));

  SimpleFOCDebug::enable(&Serial);

  I2Cbus.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  delay(50);
  i2cScan();
  sensor.init(&I2Cbus);
  motor.linkSensor(&sensor);
  delay(200);
  sensor.update();
  Serial.print(F("sensor at boot ")); Serial.print(sensor.getAngle() * 57.2958f, 1);
  Serial.println(F(" deg"));

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit        = SUPPLY_VOLTAGE;
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation    = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller        = MotionControlType::velocity;
  motor.voltage_limit     = I_LIMIT_DEFAULT * R_PHASE;
  motor.velocity_limit    = 30.0f;
  // These were crippled to survive PWM sensor noise. With a clean 14-bit SPI
  // reading at 0.2 deg jitter, the loop can be far more aggressive - and it
  // has to be, because your rotor has enough static friction that a timid
  // integrator never builds the voltage needed to break away.
  // Reverted to 0.30. The step test showed 55% overshoot, but this motor is
  // friction-dominated (about 0.36 N.m of drag) and that drag damps the
  // overshoot on its own. Cutting P only made the loop slower to build the
  // torque needed to break away, which felt worse in practice.
  motor.PID_velocity.P           = 0.30f;
  motor.PID_velocity.I           = 8.0f;    // was 0.5 - this is the big one
  motor.PID_velocity.D           = 0.0f;
  motor.PID_velocity.output_ramp = 1000.0f; // V/s, was 200
  // 0.005 was too light: 0.2 deg of sensor jitter differentiated over a fast
  // loop becomes several rad/s of velocity noise, which is why your readings
  // bounce 0.2 to 3.4 rad/s at constant saturated voltage.
  // AS5600 is 12-bit, a quarter the resolution of the AS5048A, so differentiating
  // it produces more velocity noise. Start with heavier filtering; drop toward
  // 0.02 with 'f' if the response feels sluggish.
  motor.LPF_velocity.Tf          = 0.04f;
  motor.P_angle.P                = 20.0f;

  motor.useMonitoring(Serial);
  motor.init();

  // The old CCW / 0.0 rad commutation was measured against the AS5048A. The
  // AS5600 has a different zero reference and possibly the opposite direction
  // sign, so those values are meaningless now. Deliberately NOT applied.
  motor.sensor_direction    = Direction::CCW;
  motor.zero_electric_angle = 0.0f;
  aligned = false;

  motor.disable();


  Serial.println();
  Serial.print  (F("pole pairs in use: ")); Serial.println(motor.pole_pairs);
  Serial.println(F("ORDER:  c (sensor check)  ->  U (align)  ->  e1  ->  t2"));
  Serial.print  (F("bus ")); Serial.print(SUPPLY_VOLTAGE, 0);
  Serial.print  (F(" V -> Uq ceiling ")); Serial.print(U_MAX, 2); Serial.println(F(" V"));
  Serial.print  (F("current limit ")); Serial.print(i_limit_peak, 2);
  Serial.print  (F(" A peak = ")); Serial.print(i_limit_peak / 1.4142f, 2);
  Serial.println(F(" A RMS  (datasheet: 1.6 nominal, 3.2 stall)"));
  Serial.print  (F("stall power ")); Serial.print(3.0f*powf(i_limit_peak/1.4142f,2)*R_PHASE, 0);
  Serial.println(F(" W into an unmounted stator - watch the temperature"));
  Serial.println(F("Uq is capped at I*R + back-EMF, so current stays bounded"));
  Serial.println(F("while the ceiling rises with speed. 'A<n>' sets the amps."));
  Serial.println();
  Serial.println(F("SPEED TEST:  m0  then  t6  - torque mode, no PID."));
  Serial.println(F("  If that spins fast, your speed limit was PID tuning."));
  Serial.println(F("  If it also sticks near 2.6 rad/s, it is mechanical drag."));
  Serial.println(F("open loop:  o2      stop:  x"));
  Serial.println();
  Serial.println(F("Every command you send is echoed back as >>> RECEIVED."));
  Serial.println(F("If you see no echo, the bytes are not arriving at all:"));
  Serial.println(F("check the COM port and 115200 baud. Line ending does not"));
  Serial.println(F("matter - CR, LF, or none all work."));
  Serial.println(F("Send  h  for the command list."));
  Serial.println();
  Serial.println(F("SENSOR CHANGED to AS5600. The stored commutation is void."));
  Serial.println(F("ORDER:  I (check magnet) -> c (check tracking) -> S (sweep)"));
  Serial.println(F("        then  e1  and  t2."));
  last_msg = temp_last = millis();
}

// ----------------------------------------------------------------- LOOP
void loop() {
  if (openloop) {
    openLoopTick();
  } else {
    motor.loopFOC();
    if (running) {
      if (current_limiter) {
        // Uq_max = I*R + back-EMF. Full voltage at speed, safe current at rest.
        float ulim = i_limit_peak * R_PHASE + fabsf(motor.shaft_velocity) * KE;
        motor.voltage_limit = (ulim > U_MAX) ? U_MAX : ulim;
      }
      motor.move(target);
      // Runaway guard: if the loop has positive feedback the motor accelerates
      // away from the target with the voltage saturated the WRONG way. Catch it
      // instead of letting it hit 700 rpm. Not a thermal limit - this is a
      // controls fault detector.
      // Stall guard. Saturated voltage with no rotation means all 25 W goes
      // into the windings and nothing turns. Your stator reached 95 C this
      // way. Disable with 'Q0' if you really want it off.
      static uint32_t stall_since = 0;
      if (stall_guard && fabsf(motor.voltage.q) > motor.voltage_limit * 0.95f
          && fabsf(motor.shaft_velocity) < 0.15f && fabsf(target) > 0.05f) {
        if (stall_since == 0) stall_since = millis();
        else if (millis() - stall_since > 3000) {
          running = false; target = 0; motor.disable();
          stall_since = 0;
          Serial.println();
          Serial.println(F("*** STALL CUT OUT ***"));
          Serial.println(F("Full voltage, no rotation, 3 s. All of it became heat."));
          Serial.println(F("Run 'S' to fix commutation, or check the rotor turns"));
          Serial.println(F("freely by hand. 'Q0' disables this guard."));
          Serial.println();
        }
      } else stall_since = 0;

      float v = motor.shaft_velocity;
      if (fabsf(v) > 40.0f && fabsf(v) > fabsf(target) * 3.0f + 10.0f) {
        running = false; target = 0; motor.disable();
        Serial.println();
        Serial.print(F("*** RUNAWAY CUT OUT at ")); Serial.print(v, 1);
        Serial.println(F(" rad/s ***"));
        Serial.println(F("Positive feedback - the sensor direction is inverted."));
        Serial.println(F("Send  F  to flip it, then e1 and t2 again."));
        Serial.println();
      }
    }
  }
  serialPoll();
  updateThermal();

  // Loop rate. FOC needs a fast loop: below ~1 kHz the electrical angle is
  // updated too coarsely and torque production degrades badly at speed.
  loop_count++;
  if (millis() - loop_t0 >= 1000) { loop_hz = loop_count; loop_count = 0; loop_t0 = millis(); }

  if ((openloop || running) && millis() - last_msg >= 1000) {
    sensor.update();
    if (openloop) {
      Serial.print(F("OL "));      Serial.print(ol_actual, 2);
      Serial.print(F(" rad/s ("));  Serial.print(ol_actual * 9.5493f, 0);
      Serial.print(F(" rpm) | sensor ")); Serial.print(sensor.getAngle() * 57.2958f, 0);
      Serial.print(F(" deg | ~"));  Serial.print(temp_est, 0);
      Serial.println(F(" C"));
    } else {
      float uq  = motor.voltage.q;
      float ipk = fabsf(uq) / R_PHASE;
      Serial.print(F("CL tgt "));  Serial.print(target, 2);
      Serial.print(F(" | vel "));  Serial.print(motor.shaft_velocity, 2);
      Serial.print(F(" ("));       Serial.print(motor.shaft_velocity * 9.5493f, 0);
      Serial.print(F(" rpm) | Uq ")); Serial.print(uq, 2);
      Serial.print(F("/"));        Serial.print(motor.voltage_limit, 2);
      Serial.print(F(" V | "));    Serial.print(ipk, 2);
      Serial.print(F(" A | "));    Serial.print(3.0f*powf(ipk/1.4142f,2)*R_PHASE, 1);
      Serial.print(F(" W | ~"));   Serial.print(temp_est, 0);
      Serial.print(F(" C"));
      if (fabsf(uq) > motor.voltage_limit * 0.97f) Serial.print(F("  [SATURATED]"));
      Serial.println();
    }
    last_msg = millis();
  }
}
