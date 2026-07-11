#define BUFFER_LENGTH 64
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CANSAME5x.h>
#define MODE_SENT_FRAME_ID 0x401
#define MODE_RECV_FRAME_ID 0x402
#define SETP_SENT_FRAME_ID 0x403
#define VEL_RECV_FRAME_ID 0x404
#define TRACTION_SENT_FRAME_ID 0x405
#define TRACTION_RECV_FRAME_ID 0x406
#define STEER_SENT_FRAME_ID 0x407
#define STEER_RECV_FRAME_ID 0x408
#define MILLIS_RECV_FRAME_ID 0x409
#define STATE_SENT_FRAME_ID 0x410
#define PWM_COMMAND_RECV_FRAME_ID 0x411
#define VREF_RECV_FRAME_ID 0x412

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
CANSAME5x CAN;
#include <stdbool.h>

union Mgo1Char{ 
    char c;
    byte bytes[1];
};

union Mgo1Float{ 
    float f;
    byte bytes[4];
};

union Mgo1Int{ 
    int i;
    byte bytes[2];
};

union Mgo1Long{ 
    unsigned long L;
    byte bytes[4];
};

String getHexstring(byte data[], int length) {
  String hexstring = "";
  for(int i = 0; i < length; i++) {
    if(data[i] < 0x10) {
      hexstring += '0';
    }

    hexstring += String(data[i], HEX);
  }
  return hexstring;  
}
String HexString2ASCIIString(String hexstring) {
  String temp = "", sub = "";
  char buf[3];
  for(int i = 0; i < hexstring.length(); i += 2){
    sub = hexstring.substring(i, i+2);
    sub.toCharArray(buf, 3);
    char b = (char)strtol(buf, 0, 16);
    if(b == '\0')
      break;
    temp += b;
  }
  return temp;
}

char mode = 'n';
float setpoint = 0.0;
int traction = 1500;
int steer = 1400;
int state = 0;
int pwm_command = 1500;

bool maju = true;
bool mundurTriggered = true;
static int trigCount = 0;
unsigned long elapsed = 0;

// PID PARAMETERS (adjust these!)
float KP = 800.0;
float KI = 512.0;
float KD = 0.0;
float SETPOINT = 0.0;  // Target velocity (m/s)
volatile int NEUTRAL_PWM = 1522;

// Filtering
float vEMA = 0.0, vEMAneg = 0.0, vEMAkir = 0.0, vEMAfil = 0.0;
float EMA_ALPHA = 0.05;

const int analog_pin1 = A0;
const int analog_pin2 = A1;
#define PIN_RELAY       12
#define PWM_IN_TRACTION 9
#define PWM_IN_STEER    10

volatile int NEUTRAL_TRACT = 1522;
volatile int NEUTRAL_STEER = 1503;

volatile int pwm_steer_value = NEUTRAL_STEER;
volatile unsigned long prev_steer_time = 0;

volatile int pwm_tract_value = NEUTRAL_TRACT;
volatile unsigned long prev_tract_time = 0;

volatile unsigned long microtract = 0;
volatile unsigned long microsteer = 0;

void change_traction() {
  unsigned long t = micros();
  microtract = t;
  if (digitalRead(PWM_IN_TRACTION)) {
    prev_tract_time = t;             // rising edge
  } else {
    unsigned long wt = t - prev_tract_time; // falling edge
    if (wt > 1000 && wt < 2000) pwm_tract_value = (int)wt;
  }
}

void change_steer() {
  unsigned long s = micros();
  microsteer = s;
  if (digitalRead(PWM_IN_STEER)) {
    prev_steer_time = s;
  } else {
    unsigned long ws = s - prev_steer_time;
    if (ws > 1000 && ws < 2000) pwm_steer_value = (int)ws;
  }
}

// ----------------- PCA9685 helper (Adafruit) -----------------
// Convert a pulse width in microseconds (1000..2000) to PCA9685 ticks (0..4095)
// Period assumed 20 ms (50 Hz)
static uint16_t pulseUsToTicks(int pulse_us) {
  if (pulse_us < 750) pulse_us = 750;
  if (pulse_us > 2250) pulse_us = 2250;
  // rounding: (pulse_us * 4096) / 20000
  return (uint16_t)(((uint32_t)pulse_us * 4096UL + 10000UL) / 20000UL);
}

float accel = 0.0, vref = 0.0, vold = 0.0, pid_error = 0.0, pid_prev_error = 0.0, pid_integral = 0.0;
float decel = 0.0, stop2_start_vref = 0.0;
int prev_state = -1;
float pid_proportional = 0.0, pid_integral_term = 0.0, pid_derivative = 0.0;
float pid_output = 0.0;
bool logika_vref;

void setTraction(int traction_us) {
  uint16_t ticks = pulseUsToTicks(traction_us);
  pwm.setPWM(0, 0, ticks); // channel 0 for traction
}

void setSteer(int steer_us) {
  uint16_t ticks = pulseUsToTicks(steer_us);
  pwm.setPWM(1, 0, ticks); // channel 1 for steer
}

void setup() {
  Serial.begin(57600);
  while (!Serial) { delay(10); }
  Serial.println("M4 is alive");

  Serial.println("CAN Tranceiver");

  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster

  // start the CAN bus at 500 kbps
  if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    while (1) delay(10);
  }
  Serial.println("Starting CAN!");

  Wire.begin();         // initialize I2C
  // Initialize Adafruit PCA9685
//  pwm.reset();
  pwm.begin();
  pwm.setOscillatorFrequency(26100000);
  pwm.setPWMFreq(50);   // 50 Hz for servos (20 ms)

//  if (pin_value == HIGH){
//    mode = 'o';
//  } else {
//    mode = 'r';
//  }

  // RC inputs
  pinMode(PWM_IN_TRACTION, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_IN_TRACTION), change_traction, CHANGE);

  pinMode(PWM_IN_STEER, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_IN_STEER), change_steer, CHANGE);
  
//  // debug: print the interrupt numbers
//  Serial.print("int traction: ");
//  Serial.print(digitalPinToInterrupt(PWM_IN_TRACTION));
//  Serial.print(" | int steer  : ");
//  Serial.println(digitalPinToInterrupt(PWM_IN_STEER));
}

unsigned long previousMillis = 0;
const long interval = 20; // ms

void loop() {
  float gyr_x = 0.0, gyr_y = 0.0, gyr_z = 0.0, eul_head = 0.0, eul_roll = 0.0, eul_pitch = 0.0, ax = 0.0, ay = 0.0, az = 0.0;  
  unsigned long currentMillis = millis();
//  Serial.print("Millis: ");
//  Serial.println(currentMillis); 
  if (currentMillis - previousMillis >= interval) {
    int packetSize = CAN.parsePacket();
    if (packetSize) {
      // received a packet
      Serial.print("Received ");
      if (CAN.packetExtended()) {
        Serial.print("extended ");
      }
      if (CAN.packetRtr()) {
        Serial.print("RTR ");
      }
      Serial.print("packet with id 0x");
      int packetId = CAN.packetId();
      Serial.print(CAN.packetId(), HEX);
      if (CAN.packetRtr()) {
        Serial.print(" and requested length ");
        Serial.println(CAN.packetDlc());
      } else {
        Serial.print(" and length ");
        Serial.println(packetSize);
        byte data[8];
        // only print packet data for non-RTR packets
        int i = 0;
        while (CAN.available()) {
          data[i] = CAN.read();
          i++;
        }
        String result = "";
        result = getHexstring(data, packetSize);
        if(packetSize == 1 && packetId == MODE_SENT_FRAME_ID){
          mode = (char)data[0];
          Serial.print("Mode: ");
          Serial.println(mode);
        }
        if(packetSize == 4 && packetId == SETP_SENT_FRAME_ID){
          Mgo1Float setp;
          setp.bytes[3] = data[3]; setp.bytes[2] = data[2]; setp.bytes[1] = data[1]; setp.bytes[0] = data[0];
          setpoint = setp.f;
          Serial.print("Setpoint: ");
          Serial.println(setpoint);
        }
        if(packetSize == 2 && packetId == STEER_SENT_FRAME_ID){
          Mgo1Int sti2;
          sti2.bytes[1] = data[1]; sti2.bytes[0] = data[0];
          steer = sti2.i;
          Serial.print("Steer ");
          Serial.println(steer);
        }
        if(packetSize == 2 && packetId == STATE_SENT_FRAME_ID){
          Mgo1Int stt;
          stt.bytes[1] = data[1]; stt.bytes[0] = data[0];
          state = stt.i;
          Serial.print("State: ");
          Serial.println(state);
        }
      }
      Serial.println();
    }
    int pin_value = LOW; //low for rc and high for otonom
    if (mode == 'o'){
      pin_value = HIGH;
    } else if (mode == 'r'){
      pin_value = LOW;
    }
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, pin_value);
    delay(100);
//    Serial.print("Mode received: "); Serial.println(pin_value);
    int vel1 = analogRead(analog_pin1);
    int vel2 = analogRead(analog_pin2);
    int Vel1 = (vel1 - 6);
    int Vel2 = (vel2 - 6);
    float v1 = (Vel1 / 1023.0) * 4.0;
    float v2 = (Vel2 / 1023.0) * 4.0;
    float v = (v1 + v2) / 2.0;
    vEMA = EMA_ALPHA * v + (1.0 - EMA_ALPHA) * vEMA;
    if (vEMA > -0.012 && vEMA < 0.004) {
      vEMAfil = 0.0;
    } else {
      vEMAfil = vEMA;
    }
    mundurTriggered = true;
    kontrolotonom(currentMillis, vEMA);
//    Serial.print("State: ");
//    Serial.print(autonomyState); 
//    Serial.print(" | ADC1 before: ");
//    Serial.print(vel1);
//    Serial.print(" | ADC2 before: ");
//    Serial.print(vel2);
//    Serial.print(" | ADC1 after: ");
//    Serial.print(Vel);
//    Serial.print(" | Velocity kiri: ");
//    Serial.print(v1, 3);
//    Serial.print(" | Velocity kanan: ");
//    Serial.print(v2, 3);
//    Serial.print(" | Velocity: ");
//    Serial.println(v, 3);
//    Serial.print(" | vEMA: ");
//    Serial.println(vEMA, 3);
    calculatePID(vEMAkir, 0.02, interval, currentMillis);
    if (state == 2  && elapsed >= 147000 && mundurTriggered && vEMAkir <= 0.01) {
      setTraction(1320);
      delay(1000);
    }
    if (state == 6  && elapsed >= 147000 && mundurTriggered && vEMAkir <= 0.01) {
      setTraction(1320);
      delay(1000);
      mundurTriggered = false;
    }
    // Drive the PCA9685 channels with measured RC pulses
    // (use pwm_tract_value / pwm_steer_value which are microsecond pulse widths)
//    setSteer(1500);
//    setTraction(1549);

    // Debug print
//    Serial.print("uTract: ");
//    Serial.print(microtract);
//    Serial.print(" | uSteer: ");
//    Serial.print(microsteer);
//    Serial.print(" | PWM Tract: ");
//    Serial.print(pwm_tract_value);
//    Serial.print(" | PWM Steer: ");
//    Serial.println(pwm_steer_value);
//    //Sending Traction
//    CAN.beginPacket(MODE_RECV_FRAME_ID);
//    Mgo1Char trc;
//    trc.c = mode;
//    CAN.write(trc.bytes[0]);
//    CAN.endPacket();  
    //Sending Vel
    CAN.beginPacket(VEL_RECV_FRAME_ID);
    Mgo1Float vel3;
    vel3.f = vEMAkir;
    CAN.write(vel3.bytes[3]); CAN.write(vel3.bytes[2]); CAN.write(vel3.bytes[1]); CAN.write(vel3.bytes[0]);
    CAN.endPacket();  
    //Sending Traction
    CAN.beginPacket(TRACTION_RECV_FRAME_ID);
    Mgo1Int tri;
    tri.i = pwm_tract_value;
    CAN.write(tri.bytes[1]); CAN.write(tri.bytes[0]);
    CAN.endPacket();  
    //Sending Steer
    CAN.beginPacket(STEER_RECV_FRAME_ID);
    Mgo1Int sti;
    sti.i = pwm_steer_value;
    CAN.write(sti.bytes[1]); CAN.write(sti.bytes[0]);
    CAN.endPacket();
    //Sending Millis
    CAN.beginPacket(MILLIS_RECV_FRAME_ID);
    Mgo1Long mills;
    mills.L = currentMillis;  
    CAN.write(mills.bytes[3]); CAN.write(mills.bytes[2]); CAN.write(mills.bytes[1]); CAN.write(mills.bytes[0]);
    CAN.endPacket();
//    //Sending State
//    CAN.beginPacket(STATE_SENT_FRAME_ID);
//    Mgo1Int st;
//    st.i = autonomyState;
//    CAN.write(st.bytes[1]); CAN.write(st.bytes[0]);
//    CAN.endPacket(); 
    //Sending PWM Command
    CAN.beginPacket(PWM_COMMAND_RECV_FRAME_ID);
    Mgo1Int pw;
    pw.i = pwm_command;
    CAN.write(pw.bytes[1]); CAN.write(pw.bytes[0]);
    CAN.endPacket(); 
    //Sending Vref
    CAN.beginPacket(VREF_RECV_FRAME_ID);
    Mgo1Float vr;
    vr.f = vref;
    CAN.write(vr.bytes[3]); CAN.write(vr.bytes[2]); CAN.write(vr.bytes[1]); CAN.write(vr.bytes[0]);
    CAN.endPacket();
    previousMillis = currentMillis;
  }
}

void kontrolotonom(unsigned long currentMillis, float cur_vel) {
  int pwm_maju = 1540;
  int pwm_mundur = 1510; 
//  Serial.print(" | vEMA: ");
//  Serial.println(cur_vel, 3);
  if (state == 0) {
    mundurTriggered = true;
    SETPOINT = setpoint;
    vEMAkir = cur_vel;
    maju = false;
  } else if (state == 1) {
    NEUTRAL_PWM = pwm_maju;
    SETPOINT = setpoint;
    logika_vref = (vref > SETPOINT);
    vEMAkir = cur_vel;
    maju = true;
  } else if (state == 2) {
    SETPOINT = setpoint;
    logika_vref = (vref != SETPOINT);
    vEMAkir = cur_vel;
    maju = false;
  } //else if (autonomyState == AS_MUNDUR) {
//    NEUTRAL_PWM = pwm_mundur;
//    SETPOINT = -abs(SETPOINT);
//    vEMAneg = -abs(cur_vel);
//    vEMAkir = vEMAneg;
//    logika_vref = (vref < SETPOINT);
//    maju = false;
//  } else if (autonomyState == AS_STOP3) {
//    SETPOINT = -abs(SETPOINT);
//    vEMAneg = -abs(cur_vel);
//    vEMAkir = vEMAneg;
//    maju = false;
//  } else if (autonomyState == AS_MAJU2) {
//    NEUTRAL_PWM = pwm_maju;
//    SETPOINT = abs(SETPOINT);
//    logika_vref = (vref > SETPOINT);
//    vEMAkir = cur_vel;
//    maju = true;
//  } else if (autonomyState == AS_STOP4) {
//    SETPOINT = SETPOINT;
//    vEMAkir = cur_vel;
//    maju = false;
//  } else if (autonomyState == AS_MUNDUR2) {
//    NEUTRAL_PWM = pwm_mundur;
//    SETPOINT = -abs(SETPOINT);
//    vEMAneg = -abs(cur_vel);
//    vEMAkir = vEMAneg;
//    logika_vref = (vref < SETPOINT);
//    maju = false;
//  } else if (autonomyState == AS_STOP5) {
//    SETPOINT = -abs(SETPOINT);
//    vEMAneg = -abs(cur_vel);
//    vEMAkir = vEMAneg;
//    maju = false;
//  }
}

// PID calculation
void calculatePID(float current_velocity, float dt, int interval, unsigned long currentMillis) {
  //  static unsigned long trigStart = 0;
  if (state != prev_state) {
    if (state == 2) {
      stop2_start_vref = vold;
      int t_stop = 1000;
      decel = stop2_start_vref / t_stop;
    }
      prev_state = state;
    }
  if (state == 0 || state == 4 || state == 6 || state == 8) {
    vref = 0.0;
  } else if (state == 1) {
//    int x = 48;
//    accel = x * SETPOINT / 60000.0; //accelerator variable, change the x for adjustment
//    vref = vold + (accel * interval);
//    if (logika_vref) {
      vref = SETPOINT;
//    }
  } else if (state == 2) {
    vref = vold - (decel * interval);
    if (vref < SETPOINT) {
      vref = SETPOINT;
    }
  }

  pid_error = vref - current_velocity;
  pid_proportional = KP * pid_error;
  pid_integral += pid_error * dt;
  if (pid_integral > 600) pid_integral = 600.00;
  if (pid_integral < -600) pid_integral = -600.00;
  pid_integral_term = KI * pid_integral;
  pid_derivative = KD * (pid_error - pid_prev_error) / dt;
  pid_output = pid_proportional + pid_integral_term + pid_derivative;

  int pid_command = NEUTRAL_PWM + (int)pid_output;
  pwm_command = pid_command;
  
  pwm_command = constrain(pwm_command, 1000, 2000);

  setTraction(pwm_command);
  pid_prev_error = pid_error;
  vold = vref;
}
