#include <CAN.h>
#include <Arduino.h>
#include <Esp.h>
#include "ebike.h"
//#include "BluetoothSerial.h"

//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
//#endif

//BluetoothSerial SerialBT;

// VESC controller CAN Bus ID
#define VESC_CAN_ID 0 // VESC default is 0

// Time constants
#define LOOP_TIME_STEP 10 // milliseconds

// pins 34-39 do not have any pull-down/up functionality!
#define GAS_PIN 12
#define EBRAKE_PIN 4
#define PAS_PIN 17
#define FILTER_ALPHA 3 // do not use this to provide smoothing for ramp-control!

#define PAS_NUM_MAGNETS 8
#define PAS_MIN_RPM 10000 // 1000*rpm --> 5rpm
#define PAS_MAX_RPM 200000
#define GAS_ADC_LOWER 320
#define GAS_ADC_UPPER 750

#define EBRAKE_ADC_LOWER 320
#define EBRAKE_ADC_UPPER 750

#define MOTOR_POLE_PAIRS 23           // RH205 ebike motor
#define WHEEL_CIRCUMREFERENCE_MM 2234 // wheel circumreference in millimeters, 29 inch wheel

// speeds in mm/s
// 1km/h = 277.78 mm/s
#define STARTING_ASSISTANCE_SPEED 1666          // 6 km/h StVO
#define STARTING_ASSISTANCE_SPEED_CUTOFF 1108   // 5 km/h
#define STARTING_ASSISTANCE_MAX_THROTTLE 100000 // 100%
#define MAX_ASSISTANCE_SPEED 6944               // 25 km/h StVO
#define MAX_ASSISTANCE_SPEED_CUTOFF 6372        // 23 km/h
#define MAX_THROTTLE 100000                     //100%
#define THROTTLE_UPRAMP 100 // 100 thousandths of a percent per milliseconds
#define THROTTLE_DOWNRAMP 100000 // 100% per millisecond

volatile unsigned long pas_time = 0;    // last millis() timestamp of PAS interrupt
volatile unsigned long pas_dt = 100000; // difference between two last PAS interrupts, initialized to a high value = low pedaling activity
volatile int32_t erpm = 0;
uint32_t gas_percent_filtered = 0;
uint32_t ebrake_percent_filtered = 0;
uint32_t gas_control = 0; // ramp controlled

uint32_t loop_time = 0;
uint32_t loop_dt = LOOP_TIME_STEP;

void IRAM_ATTR pas_isr()
{
  unsigned long timestamp = millis();
  pas_dt = timestamp - pas_time;
  pas_time = timestamp;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("init...");
  //SerialBT.begin("EBike");
  analogReadResolution(10);
  pinMode(PAS_PIN, INPUT_PULLUP);
  pinMode(GAS_PIN, INPUT);
  pinMode(EBRAKE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PAS_PIN), pas_isr, FALLING);

  CAN.setPins(35, 5); // Olimex ESP32 EVB: RX 35, TX 5

  if (!CAN.begin(500E3))
  { //500kbit/s CAN bus
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }

  CAN.onReceive(onReceiveCAN); // handle VESC status messages
  Serial.println("Setup Complete!");
}

void onReceiveCAN(int packetSize)
{
  if (CAN.packetExtended() && !CAN.packetRtr())
  {
    long packetId = CAN.packetId();
    uint8_t vesc_id = packetId & 0xFF;
    CAN_PACKET_ID vesc_packet_id = static_cast<CAN_PACKET_ID>(packetId >> 8);
    if (vesc_id == VESC_CAN_ID)
    {
      if (vesc_packet_id == CAN_PACKET_STATUS)
      {                  // TODO: change to switch statement
        uint8_t data[8]; // maximum CAN frame size is 8 bytes
        int CAN_bytes = CAN.available();
        if (CAN_bytes > 8)
        {
          Serial.println("ERROR - Too many CAN bytes!!!");
        }
        else
        {
          CAN.readBytes(data, CAN_bytes);
          int index = 0;
          erpm = buffer_get_int32(data, &index);
          int32_t current = buffer_get_int16(data, &index);
          int32_t duty = buffer_get_int16(data, &index);
        }
      }
    }
  }
}

void loop()
{
  // stat_tmp->rpm = (float)buffer_get_int32(rxmsg.data8, &ind);
  //stat_tmp->current = (float)buffer_get_int16(rxmsg.data8, &ind) / 10.0;
  // stat_tmp->duty = (float)buffer_get_int16(rxmsg.data8, &ind) / 1000.0;

  uint32_t gas = median_3(analogRead(GAS_PIN), analogRead(GAS_PIN), analogRead(GAS_PIN));
  uint32_t ebrake = median_3(analogRead(EBRAKE_PIN), analogRead(EBRAKE_PIN), analogRead(EBRAKE_PIN));
  uint32_t pas_rpm = get_pas_rpm();
  int32_t speed = get_speed();

  // convert gas value range to 1/1000th of a percent
  uint32_t gas_percent = constrain(map(gas, GAS_ADC_LOWER, GAS_ADC_UPPER, 0, 100000), 0, 100000);
  // convert ebrake value range to 1/1000th of a percent
  uint32_t ebrake_percent = constrain(map(ebrake, EBRAKE_ADC_LOWER, EBRAKE_ADC_UPPER, 0, 100000), 0, 100000);

  exp_filter(&gas_percent, &gas_percent_filtered, FILTER_ALPHA);
  exp_filter(&ebrake_percent, &ebrake_percent_filtered, FILTER_ALPHA);

  Serial.print("Gas: ");
  Serial.print(gas);
  Serial.print(" - ");
  Serial.print(gas_percent);
  Serial.print(" - ");
  Serial.print(gas_percent_filtered);
  Serial.print(" -control: ");
  Serial.print(gas_control);
  Serial.print(" EBrake: ");
  Serial.print(ebrake);
  Serial.print(" - ");
  Serial.print(ebrake_percent);
  Serial.print(" - ");
  Serial.print(ebrake_percent_filtered);
  Serial.print(" - ");
  Serial.print(" PAS RPM:");
  Serial.print(pas_rpm);
  Serial.print(" PAS DT:");
  Serial.print(pas_dt);
  Serial.print(" PAS TIME:");
  Serial.print(pas_time);
  Serial.print(" ERPM: ");
  Serial.print(erpm);
  Serial.print(" Speed: ");
  Serial.print(speed);
  Serial.print(" Loop: ");
  Serial.print(loop_dt);
  Serial.println();

  int vesc_result;

  if (ebrake_percent_filtered > 0)
  { // BRAKING -> no gas allowed
    Serial.println("Braking!");
    gas_control = 0;
    vesc_result = vesc_command(VESC_CAN_ID, CAN_PACKET_SET_CURRENT_BRAKE_REL, ebrake_percent_filtered);
  }
  else
  { // not Braking
    if (pas_rpm > PAS_MIN_RPM)
    { // Pedaling
      uint32_t stvo_gas = linear_cutoff(speed, gas_percent_filtered, MAX_ASSISTANCE_SPEED, MAX_ASSISTANCE_SPEED_CUTOFF, MAX_THROTTLE);
      set_ramp_limited(&gas_control, stvo_gas, THROTTLE_UPRAMP*loop_dt, THROTTLE_DOWNRAMP*loop_dt);
      vesc_result = vesc_command(VESC_CAN_ID, CAN_PACKET_SET_CURRENT_REL, gas_control);
    }
    else
    { // NOT pedaling
      uint32_t starting_assistance_gas = linear_cutoff(speed, gas_percent_filtered, STARTING_ASSISTANCE_SPEED, STARTING_ASSISTANCE_SPEED_CUTOFF, STARTING_ASSISTANCE_MAX_THROTTLE);
      set_ramp_limited(&gas_control, starting_assistance_gas, THROTTLE_UPRAMP*loop_dt, THROTTLE_DOWNRAMP*loop_dt*10);
      vesc_result = vesc_command(VESC_CAN_ID, CAN_PACKET_SET_CURRENT_REL, gas_control);
    }
  }

  if (!vesc_result)
  {
    Serial.println("VESC Command CAN ERROR!");
  }

  loop_dt = millis() - loop_time;
  loop_time = millis();
  delay(LOOP_TIME_STEP);
}

uint32_t get_pas_rpm()
{
  uint32_t dt;
  dt = millis() - pas_time;

  if (dt < pas_dt)
  {
    dt = pas_dt;
    // time between last two pulses is smaller than between the last pulse and now
    // therefore, assume the lower PAS rpm as a "timeout"
  }

  if (pas_dt > 0)
  {                                           // check may be redundant?
    return 60000000 / (PAS_NUM_MAGNETS * dt); // convert milliseconds delta to rpm*1000
  }
  else
  {
    return PAS_MAX_RPM;
  }
}

void set_ramp_limited(uint32_t *output, uint32_t value, uint32_t up_ramp, uint32_t down_ramp)
{
  if(value < *output) {
    // down_ramp
    uint32_t delta = *output - value;
    *output -= min(delta, down_ramp);
  }
  if(value > *output) {
    // up_ramp
    uint32_t delta = value - *output;
    *output += min(delta, up_ramp);
  }
  // if value == output it is left untouched
}

int32_t get_speed()
{
  // returns the speed in millimeters / second
  int32_t speed = (erpm / MOTOR_POLE_PAIRS) * (WHEEL_CIRCUMREFERENCE_MM / 60);
  return speed;
}

int vesc_command(uint8_t controller_id, CAN_PACKET_ID command, int32_t value)
{
  CAN.beginExtendedPacket(controller_id | ((uint32_t)command << 8));
  uint8_t buffer[4];
  buffer[0] = value >> 24;
  buffer[1] = value >> 16;
  buffer[2] = value >> 8;
  buffer[3] = value;
  CAN.write(buffer, 4);
  return CAN.endPacket();
}

/*********************************************************************************************************
END FILE
*********************************************************************************************************/
