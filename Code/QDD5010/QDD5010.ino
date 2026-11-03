#include <SimpleFOC.h>

// Hardware: Arduino Mega 2560 + SimpleFOC Shield v2.0.4 + MKS 5010 360KV + MKS AS5600
// (was an Uno until 2026-10-30; the sketch builds for both)
// Encoder: AS5600 over I2C. Mega I2C is pins 20/21 (and the SDA/SCL header next to AREF), NOT A4/A5 as on the Uno
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// MKS 5010: 7 pole pairs, 0.12 ohm phase resistance, 360 KV, 50 uH
// Giving R and KV makes SimpleFOC work in estimated amps instead of raw volts - essential with a 0.12 ohm motor
BLDCMotor motor = BLDCMotor(7, 0.12, 360, 0.00005);

// SimpleFOC Shield v2 default solder-jumper config: PWM A/B/C -> 9, 5, 6, Enable -> 8
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// Shield's INA240A2 sensing: 0.01 ohm shunts, gain 50 -> 0.5 V/A (about +-5 A range), on A0 / A2.
// With the driver on pins 9/5/6, A2 is phase A (inverted) and A0 is phase B - found 2026-10-30, see setup().
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, (int)A2, (int)A0);

Commander command = Commander(Serial);

const float SUPPLY_VOLTAGE = 12.0;

// Planetary gearbox: sun 12 / ring 60, carrier output -> 6:1
const float GEAR_RATIO = 6.0;

// SimpleFOC's default 3V alignment is far too much for a 0.12 ohm winding; 0.5V was too weak to move the rotor.
// Only needed when the encoder calibration below is commented out and re-run.
const float ALIGN_VOLTAGE = 1.0;
// The current-sense check also drives the phases at voltage_sensor_align. 1.0V is ~8 A in this winding, past the
// shield's +-5 A sensing range: readings clip and the check picks a different (wrong) phase mapping each boot.
const float CS_ALIGN_VOLTAGE = 0.3;

// Feedforward for the measured-current modes: apply R*I + Ke*w straight away (like the estimated mode) and leave the
// ~380 Hz current PI only the remaining error. Measured 2026-10-30 at 30-120 RPM: R = 0.69 ohm (winding 0.12 + the
// L6234's switches and wiring), Ke = 0.0113 V/(rad/s) (the library's KV-based estimate is 0.0153). 0 = off.
float ff_resistance = 0.69;
const float FF_KE = 0.0113;

unsigned long last_print_time = 0;
char tbuf[192];
char* tp;
int tlen = 0, tpos = 0;

void tadd(const char* label, float v, uint8_t prec) {
  strcpy(tp, label); tp += strlen(tp);
  dtostrf(v, 0, prec, tp); tp += strlen(tp);
}
bool telemetry_on = false;  // M turns it on; off at boot so the monitor stays readable for typing commands

void doRPM(char* cmd) {
  motor.controller = MotionControlType::velocity;
  float rpm = atof(cmd);
  motor.target = rpm * (2.0 * _PI / 60.0);
  Serial.println();
  Serial.print(F("Velocity mode, target: ")); Serial.print(rpm); Serial.println(F(" RPM"));
}

void doOutputRPM(char* cmd) {
  motor.controller = MotionControlType::velocity;
  float rpm = atof(cmd);
  motor.target = rpm * GEAR_RATIO * (2.0 * _PI / 60.0);
  Serial.println();
  Serial.print(F("Velocity mode, output target: ")); Serial.print(rpm); Serial.println(F(" RPM"));
}

void doAngle(char* cmd) {
  motor.controller = MotionControlType::angle;
  float deg = atof(cmd);
  motor.target = deg * (_PI / 180.0);
  Serial.println();
  Serial.print(F("Angle mode, target: ")); Serial.print(deg); Serial.println(F(" deg"));
}

void doOutputAngle(char* cmd) {
  motor.controller = MotionControlType::angle;
  float deg = atof(cmd);
  motor.target = deg * GEAR_RATIO * (_PI / 180.0);
  Serial.println();
  Serial.print(F("Angle mode, output target: ")); Serial.print(deg); Serial.println(F(" deg"));
}

// Make the current position read 0 (motor and output). sensor_offset shifts shaft_angle itself, so A, B and
// telemetry all follow. Lost on power-off: the encoder only knows the motor angle within one turn (60 deg output).
void doZero(char* cmd) {
  float a = motor.shaft_angle;
  motor.sensor_offset += a;
  if (motor.controller == MotionControlType::angle) motor.target -= a;
  Serial.println(F("\nOutput zero set here"));
}

void doTorque(char* cmd) {
  motor.controller = MotionControlType::torque;
  motor.target = atof(cmd);
  Serial.println();
  Serial.print(F("Torque mode, target current: ")); Serial.print(motor.target, 2); Serial.println(F(" A (est.)"));
}

// Ceilings for C and QV: the shield's current sensors read about +-5 A and the L6234 has no heatsink.
// 3 V across the measured 0.69 ohm is ~4.3 A at a stall - the most this shield should ever see.
const float MAX_CURRENT_LIMIT = 4.0;
const float MAX_VOLTAGE_LIMIT = 3.0;

void doCurrentLimit(char* cmd) {
  motor.updateCurrentLimit(_constrain(atof(cmd), 0, MAX_CURRENT_LIMIT));
  Serial.println();
  Serial.print(F("Current limit: ")); Serial.print(motor.current_limit, 2); Serial.println(F(" A"));
}

void doVelP(char* cmd) {
  motor.PID_velocity.P = atof(cmd);
  Serial.println(); Serial.print(F("Velocity P: ")); Serial.println(motor.PID_velocity.P, 4);
}
void doVelI(char* cmd) {
  motor.PID_velocity.I = atof(cmd);
  Serial.println(); Serial.print(F("Velocity I: ")); Serial.println(motor.PID_velocity.I, 4);
}
void doVelD(char* cmd) {
  motor.PID_velocity.D = atof(cmd);
  Serial.println(); Serial.print(F("Velocity D: ")); Serial.println(motor.PID_velocity.D, 4);
}
void doAngleP(char* cmd) {
  motor.P_angle.P = atof(cmd);
  Serial.println(); Serial.print(F("Angle P: ")); Serial.println(motor.P_angle.P, 4);
}
void doVelLimit(char* cmd) {
  motor.velocity_limit = atof(cmd);
  motor.P_angle.limit = motor.velocity_limit;
  Serial.println(); Serial.print(F("Velocity limit: ")); Serial.print(motor.velocity_limit, 2); Serial.println(F(" rad/s"));
}

void doEnable(char* cmd) {
  if (atoi(cmd)) { motor.enable(); Serial.println(F("\nMotor ENABLED")); }
  else           { motor.disable(); Serial.println(F("\nMotor DISABLED")); }
}

void doVelFilter(char* cmd) {
  motor.LPF_velocity.Tf = atof(cmd);
  Serial.println(); Serial.print(F("Vel filter Tf: ")); Serial.println(motor.LPF_velocity.Tf, 4);
}

void doRamp(char* cmd) {
  motor.PID_velocity.output_ramp = atof(cmd);
  Serial.println(); Serial.print(F("Vel ramp: ")); Serial.println(motor.PID_velocity.output_ramp);
}

void doKV(char* cmd) {
  float kv = atof(cmd);
  motor.KV_rating = kv > 0 ? kv : NOT_SET;
  Serial.println(); Serial.print(F("KV (0=off): ")); Serial.println(kv);
}

// Current loop, one command because Commander only holds 20 (a 21st silently corrupts the others):
// QP<v> / QI<v> PI gains (d and q), QF<v> filter Tf, QX<ohm> feedforward R (0=off), QV<volts> voltage cap,
// QL0 estimated current, QL1 dc_current (measured magnitude), QL2 foc_current (measured d/q, d held at 0)
void doCurrent(char* cmd) {
  float v = atof(cmd + 1);
  switch (cmd[0]) {
    case 'P': motor.PID_current_q.P = motor.PID_current_d.P = v; break;
    case 'I': motor.PID_current_q.I = motor.PID_current_d.I = v; break;
    case 'F': motor.LPF_current_q.Tf = motor.LPF_current_d.Tf = v; break;
    case 'X': ff_resistance = v; break;
    case 'V':
      motor.voltage_limit = _constrain(v, 0, MAX_VOLTAGE_LIMIT);
      motor.PID_current_q.limit = motor.PID_current_d.limit = motor.voltage_limit;
      break;
    case 'L':
      motor.torque_controller = v == 2 ? TorqueControlType::foc_current
                              : v == 1 ? TorqueControlType::dc_current : TorqueControlType::estimated_current;
      motor.PID_current_q.reset(); motor.PID_current_d.reset();
      break;
  }
  Serial.println();
  Serial.print(F("Current loop P=")); Serial.print(motor.PID_current_q.P, 4);
  Serial.print(F(" I=")); Serial.print(motor.PID_current_q.I, 4);
  Serial.print(F(" Tf=")); Serial.print(motor.LPF_current_q.Tf, 4);
  Serial.print(F(" ffR=")); Serial.print(ff_resistance, 4);
  Serial.print(F(" Vcap=")); Serial.print(motor.voltage_limit, 2);
  Serial.print(F(" mode=")); Serial.println((int)motor.torque_controller);
}

void doTelemetry(char* cmd) {
  telemetry_on = !telemetry_on;
}

void doGains(char* cmd) {
  Serial.println();
  Serial.print(F("VelP=")); Serial.print(motor.PID_velocity.P, 4);
  Serial.print(F(" VelI=")); Serial.print(motor.PID_velocity.I, 4);
  Serial.print(F(" VelD=")); Serial.print(motor.PID_velocity.D, 4);
  Serial.print(F(" AngleP=")); Serial.print(motor.P_angle.P, 4);
  Serial.print(F(" Ilim=")); Serial.print(motor.current_limit, 2);
  Serial.print(F(" Vlim(rad/s)=")); Serial.println(motor.velocity_limit, 2);
  doCurrent((char*)"?");
  Serial.print(F("zero_electric_angle=")); Serial.print(motor.zero_electric_angle, 4);
  Serial.print(F(" sensor_direction=")); Serial.println(motor.sensor_direction == Direction::CW ? F("CW") : F("CCW"));
}

void doHelp(char* cmd) {
  Serial.println(F("\n--- Commands (letter + number, then Enter; upper or lower case) ---"));
  Serial.println(F("Angle (go to and hold)"));
  Serial.println(F("  B<deg>    output angle                  B30  B-15  B0"));
  Serial.println(F("  A<deg>    motor angle (6x the output)   A90"));
  Serial.println(F("  Z         make this position zero"));
  Serial.println(F("Speed (spin continuously, 0 stops)"));
  Serial.println(F("  S<rpm>    output speed                  S10  S-5  S0"));
  Serial.println(F("  R<rpm>    motor speed (6x the output)   R60  R-30  R0"));
  Serial.println(F("Torque"));
  Serial.println(F("  T<amps>   push with a set current       T0.5  T0"));
  Serial.println(F("Power"));
  Serial.println(F("  E0 / E1   motor off (limp) / on         E0"));
  Serial.println(F("Limits"));
  Serial.println(F("  C<amps>   current cap (2, max 4)        C1.5"));
  Serial.println(F("  QV<volts> voltage cap (1.5, max 3)      QV2"));
  Serial.println(F("  V<rad/s>  angle-mode motor speed (20)   V10"));
  Serial.println(F("Tuning"));
  Serial.println(F("  P/I/D<v>  velocity gains (0.1/1/0)      P0.08"));
  Serial.println(F("  N<v>      angle gain (15)               N10"));
  Serial.println(F("  F<s>      velocity filter (0.02)        F0.01"));
  Serial.println(F("  O<A/s>    velocity output ramp (1000)   O1000"));
  Serial.println(F("  QP/QI<v>  current-loop gains (0/0=off)  QP0.2  QI100"));
  Serial.println(F("  QF<s>     current filter (0.001)        QF0.003"));
  Serial.println(F("  QX<ohm>   feedforward R (0.69, 0=off)   QX0.69"));
  Serial.println(F("  QL<0-2>   0 estimated / 1 dc / 2 foc    QL2"));
  Serial.println(F("  K<kv>     back-EMF KV, estimated mode   K360"));
  Serial.println(F("Info"));
  Serial.println(F("  M         telemetry on/off"));
  Serial.println(F("  G         show gains, limits, calibration"));
  Serial.println(F("  H or ?    this list"));
}

// Commands arrive here instead of through command.run() so that lower case works too: the command letter (and the
// Q sub-letter) is upper-cased before Commander sees the line. Commander can't just get lower-case aliases - it
// holds at most 20 commands. '?' is mapped to H because Commander's own '?' lists ids with empty labels.
char line_buf[32];
uint8_t line_len = 0;

void readCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (line_len < sizeof(line_buf) - 1) line_buf[line_len++] = c;
      continue;
    }
    line_buf[line_len] = 0;
    line_len = 0;
    if (!line_buf[0]) continue;
    line_buf[0] = toupper(line_buf[0]);
    if (line_buf[0] == 'Q') line_buf[1] = toupper(line_buf[1]);
    if (line_buf[0] == '?') line_buf[0] = 'H';
    command.run(line_buf);
  }
}

void setup() {
  Serial.begin(115200);
  // Opening the serial monitor resets the board, and the monitor needs a moment before it starts showing text -
  // without this pause the start of the boot log and command list scroll past unseen.
  delay(2000);
  SimpleFOCDebug::enable(&Serial);

  sensor.init();
  Wire.setClock(400000);
  #if defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(3000, true);
  #endif
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = SUPPLY_VOLTAGE / 2;
  driver.init();
  motor.linkDriver(&driver);

  current_sense.linkDriver(&driver);
  if (current_sense.init()) {
    // Fixed mapping instead of SimpleFOC's boot-time check: with the gearbox's friction that check picked a
    // different mapping on different boots, and the wrong ones made the motor swing or spin.
    current_sense.gain_a *= -1;
    current_sense.skip_align = true;
    motor.linkCurrentSense(&current_sense);
  } else {
    Serial.println(F("Current sense init FAILED"));
  }
  // The Mega's core leaves the ADC at /128 (~110 us per analogRead, two per loop). /16 = 1 MHz, ~15 us, still accurate.
  ADCSRA = (ADCSRA & ~0x07) | 0x04;

  // Encoder calibration is stored below, so the only alignment that runs at boot is the current-sense one
  motor.voltage_sensor_align = CS_ALIGN_VOLTAGE;
  // foc_current: torque from the measured d/q phase currents (d held at 0). Needs the current sense linked above.
  // L0 switches back to the old estimate (which turned out to match the measured current within ~10%).
  motor.torque_controller = TorqueControlType::foc_current;
  // Hard backstop independent of the current loop: 1.5V across the 0.12 ohm winding
  motor.voltage_limit = 1.5;
  motor.controller = MotionControlType::angle;

  // Real, measured amps. Shield sensing tops out near 5 A and the L6234 overheats above a few amps continuous.
  motor.current_limit = 2.0;
  motor.velocity_limit = 20.0;

  // Current PI off: the feedforward (R*I + Ke*w, see ff_resistance) alone sets the voltage, and since R was measured
  // the real current still tracks the setpoint (Iq meas in telemetry shows it). Closing the loop on the sensors fed
  // their +-0.3 A noise into the motor: holding jittered ~7 deg motor p-p with P 0.2 / I 100, vs ~0.1-1.3 deg without.
  // If re-enabled (QP/QI): loop runs at ~380 Hz and L/R (~0.07 ms) is far below one sample, so the plant is a pure gain
  // of 1/0.69 = 1.45 A/V with one sample of delay - per-sample loop gain P*1.45 + I*1.45/380 must stay well under 1.
  motor.PID_current_q.P = motor.PID_current_d.P = 0.0;
  motor.PID_current_q.I = motor.PID_current_d.I = 0.0;
  motor.PID_current_q.limit = motor.PID_current_d.limit = motor.voltage_limit;
  motor.LPF_current_q.Tf = motor.LPF_current_d.Tf = 0.001;

  // Tuned 2026-10-30 with the gearbox on (feedforward only, current PI off): 60 deg output steps settle in ~0.3 s,
  // no overshoot, <0.2 deg final error; holding 0.1-1.3 deg motor p-p. Amps here are real, so these equal the old
  // estimated-mode P 0.5 / I 8 scaled by 0.12/0.69. P 0.15 / I 1.5 doubles the holding jitter.
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  // 200 A/s lagged the current reversal when braking and caused +-50 deg limit cycles after large steps
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.02;

  motor.P_angle.P = 15.0;
  motor.LPF_angle.Tf = 0.0;

  // Calibrated 2026-10-29 (motor fixed to base). Comment both out to re-run calibration if the encoder magnet or motor wiring changes
  // (and set voltage_sensor_align back to ALIGN_VOLTAGE above).
  motor.sensor_direction = Direction::CCW;
  motor.zero_electric_angle = 1.0063;

  motor.init();
  motor.initFOC();

  motor.target = motor.shaft_angle;  // hold current position instead of snapping to 0

  command.add('R', doRPM, "");
  command.add('A', doAngle, "");
  command.add('B', doOutputAngle, "");
  command.add('Z', doZero, "");
  command.add('T', doTorque, "");
  command.add('C', doCurrentLimit, "");
  command.add('P', doVelP, "");
  command.add('I', doVelI, "");
  command.add('D', doVelD, "");
  command.add('N', doAngleP, "");
  command.add('V', doVelLimit, "");
  command.add('E', doEnable, "");
  command.add('M', doTelemetry, "");
  command.add('F', doVelFilter, "");
  command.add('K', doKV, "");
  command.add('O', doRamp, "");
  command.add('G', doGains, "");
  command.add('Q', doCurrent, "");
  command.add('H', doHelp, "");
  command.add('S', doOutputRPM, "");  // Commander holds at most 20 commands - this is the 20th, none left

  Serial.println(F("\n=== 5010 QDD Ready (angle mode, holding position) ==="));
  doHelp(nullptr);
  doGains(nullptr);
  _delay(1000);
}

unsigned long loop_count = 0;

void loop() {
  // current_sp is from the previous move(), one loop old - fine at this rate
  if (ff_resistance > 0 && motor.torque_controller != TorqueControlType::estimated_current) {
    float i_sp = _constrain(motor.current_sp, -motor.current_limit, motor.current_limit);
    motor.feed_forward_voltage.q = _constrain(ff_resistance * i_sp + FF_KE * motor.shaft_velocity,
                                              -motor.voltage_limit, motor.voltage_limit);
  } else {
    motor.feed_forward_voltage.q = 0;
  }
  motor.loopFOC();
  motor.move();
  readCommands();
  loop_count++;

  // Telemetry is built into tbuf once, then drained a few bytes per loop: a whole line through Serial.print
  // blocks ~12 ms at 115200 baud once the 64-byte TX buffer fills, which stalls the current loop.
  if (tpos < tlen) {
    int n = min(Serial.availableForWrite(), tlen - tpos);
    if (n > 0) { Serial.write(tbuf + tpos, n); tpos += n; }
  }
  // Must use _micros(): on the Uno, PWM on pins 5/6 makes SimpleFOC speed up Timer0, so millis()/micros() run 64x fast
  // (on the Mega pins 9/5/6 use Timers 2/3/4 and millis() is fine, but _micros() works on both)
  else if (telemetry_on && _micros() - last_print_time > 50000UL) {
    // shaft_angle only updates once FOC is ready; fall back to the raw encoder so a failed init still shows sensor data
    float ang = (motor.motor_status == FOCMotorStatus::motor_ready ? motor.shaft_angle : sensor.getAngle()) * (180.0 / _PI);
    tp = tbuf;
    tadd("Hz: ", loop_count * 20.0, 0);
    tadd("\tAngle: ", ang, 2);
    tadd(" deg\tVel: ", motor.shaft_velocity * (60.0 / (2.0 * _PI)), 1);
    tadd(" RPM\tIq set: ", motor.current_sp, 2);
    tadd(" A\tIq meas: ", motor.current.q, 2);
    tadd(" A\tId: ", motor.current.d, 2);
    tadd(" A\tUq: ", motor.voltage.q, 3);
    tadd("\tUd: ", motor.voltage.d, 3);
    tadd("\tOut: ", ang / GEAR_RATIO, 2);
    strcpy(tp, " deg\r\n");
    tlen = strlen(tbuf); tpos = 0;
    loop_count = 0;
    last_print_time = _micros();
  }
}
