#include <SPI.h>
#include <mcp_can.h>

// #define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// CAN MCP Pins
#define MCP_INT 7 // PE6
#define MCP_CS 4  // PD4

// VESC controller CAN Bus ID
#define VESC_CAN_ID 1  // factory default is 0!

// LEDS
#define LED_RX 12 // PD6
#define LED_TX 8 // PB4
#define LED_F 6 // PD7

// Time constants
#define LOOP_TIME_STEP 10 // milliseconds

// Gas
// TODO: auto lower/upper range calibration on startup
#define GAS_PIN 4 // PF1
#define GAS_FILTER_ALPHA 3 // right shift x bits = division by 2^x
#define GAS_FILTER_NUM_READINGS 10
#define GAS_ADC_LOWER 165
#define GAS_ADC_UPPER 830
#define GAS_LAG 3
#define GAS_FACTOR_LAG 3

// Ebrake
#define EBRAKE_PIN 5 // PF0
#define EBRAKE_FILTER_ALPHA 3 // right shift x bits = division by 2^x
#define EBRAKE_FILTER_NUM_READINGS 10
#define EBRAKE_ADC_LOWER 165
#define EBRAKE_ADC_UPPER 830

// Brakes
#define BRAKE_L_PIN 9 // PB5
#define BRAKE_R_PIN 10 // PB6

// Switches
#define SWITCH_L_PIN 3 // PD0
#define SWITCH_R_PIN 2 // PD1

// Pedal Sensor
#define PED_PIN 1 // PD3

// ERPM to actual wheel rpm conversion
// #define ERPM_FACTOR 20 // motor pole pair count

// Limits for legal StVO
#define ERPM6_LOWER 824 // 6km/h-20%
#define ERPM6_UPPER 1030 // 6km/h -> 48rpm -> 48*20erpm
#define ERPM25_LOWER 3450 // 25km/h-20%
#define ERPM25_UPPER 4350 // 25km/h -> 200rpm -> 200*20erpm

// Initialize CAN controller
MCP_CAN CAN0(MCP_CS);

// Globals

// CAN (VESC)
int32_t raw_read = 0;
int32_t erpm = 0;
int32_t erpm_alt = 0;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];

// Gas
int32_t gas_filtered = 0;
int32_t gas_filtered_old = 0;
int32_t gas_percent; // 1/1000th of a percent
int32_t gas6_factor = 0;
int32_t gas6_factor_old = 0;
int32_t gas25_factor = 0;
int32_t gas25_factor_old = 0;

// EBrake
int32_t ebrake_filtered = 0;
int32_t ebrake_percent; // 1/1000th of a percent

// Loop Time calculation
int32_t loop_begin_micros;
int32_t loop_end_micros;
int32_t loop_time;

// Brake State
int BRAKE_L;
int BRAKE_R;

// Switch State
int SWITCH_L;
int SWITCH_R;

// Pedal Sensor
volatile int32_t prpm = 0;
volatile int32_t prpm_t1 = 0;
volatile int32_t prpm_t2 = 0;
int32_t prpm_t3 = 0;
int pedal = 0;

// Start-Up
int start_up = 0;

void setup() {
  Serial.begin(115200); // USB Debugging
  DEBUG_PRINTLN("INIT");

  // LEDs
  pinMode(LED_F, OUTPUT);
  pinMode(LED_RX, OUTPUT);
  pinMode(LED_TX, OUTPUT);

  // Brake Pullups
  pinMode(BRAKE_L_PIN, INPUT_PULLUP);
  pinMode(BRAKE_R_PIN, INPUT_PULLUP);

  // Switch Pullups
  pinMode(SWITCH_L_PIN, INPUT_PULLUP);
  pinMode(SWITCH_R_PIN, INPUT_PULLUP);

  // Pedal Sensor Interrupt
  pinMode(PED_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PED_PIN), rotational_speed, FALLING);
  
  // No masks, no filters, 500kb/s CAN
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    DEBUG_PRINTLN("MCP2515 Initialized Successfully!");
    led_flash_all(100, 3);
  } else { // halt
    while (1) {
      DEBUG_PRINTLN("Error Initializing MCP2515...");
      digitalWrite(LED_F, HIGH);
      delay(900);
      digitalWrite(LED_F, LOW);
      delay(100);
    }
  }

  // Change to normal mode to allow messages to be transmitted
  CAN0.setMode(MCP_NORMAL);
}

void loop() {
  loop_begin_micros = micros();
  DEBUG_PRINT("Loop Time: ");
  DEBUG_PRINT(loop_time);
  DEBUG_PRINTLN("us");

  update_switches();
  update_brakes();
  if (start_up == 0) {
    gas_filtered_old = GAS_ADC_LOWER;
    start_up = 1;
  } else {
    gas_filtered_old = gas_filtered;
  }
  read_filter_adc(GAS_PIN, GAS_FILTER_NUM_READINGS, GAS_FILTER_ALPHA, &gas_filtered);
  if (gas_filtered > gas_filtered_old + (2 << GAS_LAG)) {
    gas_filtered = gas_filtered_old + (2 << GAS_LAG);
  }
  read_filter_adc(EBRAKE_PIN, EBRAKE_FILTER_NUM_READINGS, EBRAKE_FILTER_ALPHA, &ebrake_filtered);
  DEBUG_PRINT("GAS, FILTERED: ");
  DEBUG_PRINTLN(gas_filtered);
  DEBUG_PRINT("EBRAKE, FILTERED: ");
  DEBUG_PRINTLN(ebrake_filtered);
  
  // convert gas value range to 1/1000th of a percent
  gas_percent = constrain(map(gas_filtered, GAS_ADC_LOWER, GAS_ADC_UPPER, 0, 100000), 0, 100000);
  // convert ebrake value range to 1/1000th of a percent
  ebrake_percent = constrain(map(ebrake_filtered, EBRAKE_ADC_LOWER, EBRAKE_ADC_UPPER, 0, 100000), 0, 100000);

  DEBUG_PRINT("GAS, PERCENT: ");
  DEBUG_PRINTLN(gas_percent);

  DEBUG_PRINT("EBRAKE, PERCENT: ");
  DEBUG_PRINTLN(ebrake_percent);

  DEBUG_PRINT("BRAKE L: ");
  DEBUG_PRINTLN(digitalRead(BRAKE_L_PIN));

  DEBUG_PRINT("BRAKE R: ");
  DEBUG_PRINTLN(digitalRead(BRAKE_R_PIN));
  
  byte sndStat;  //stores CAN transmission result, TODO: could be made global

  // TODO: this should be a state machine
  // TODO: EBrake should be reverse after coming to a complete stop (and releasing the brake once)
  if (SWITCH_L) { // StVO legal 
    DEBUG_PRINTLN("STVO LEGAL!");
    // Pedal RPM (<10rpm is detected as zero, respectively no pedaling)
    prpm_t3 = millis();
    if (prpm_t3 > (prpm_t2+300)) {
      prpm = 0;
    } 
    DEBUG_PRINT("prpm: ");
    DEBUG_PRINTLN(prpm);
    can_erpm();
    DEBUG_PRINT("erpm: ");
    DEBUG_PRINTLN(erpm);
    if (prpm < 10) { // Limit to 6km/h without pedaling (starting traction)
      if ((ebrake_percent == 0) && (BRAKE_L) && (BRAKE_R)) { // no brake, gas OK
        if (SWITCH_R) {          // Drive forwards
          pedal = 0;
          gas6_factor_old = gas6_factor;
          gas6_factor = constrain(map(erpm, ERPM6_LOWER, ERPM6_UPPER, 1023, 0), 0, 1023);
          if ((erpm <= erpm_alt) && (gas6_factor > (gas6_factor_old + (2 << GAS_FACTOR_LAG)))) { // maximum gas increase, full scale per loop
            gas6_factor = gas6_factor_old + (2 << GAS_FACTOR_LAG);
          }
          gas_filtered = (((gas_filtered - GAS_ADC_LOWER) * gas6_factor) >> 10) + GAS_ADC_LOWER;
          gas_percent = constrain(map(gas_filtered, GAS_ADC_LOWER, GAS_ADC_UPPER, 0, 100000), 0, 100000);
          erpm_alt = erpm;
          sndStat = comm_can_set_current_rel(VESC_CAN_ID, gas_percent);
          DEBUG_PRINTLN("GAS FORWARDS!");
        } else { // Drive backwards
          sndStat = comm_can_set_current_rel(VESC_CAN_ID, -1 * gas_percent);
          DEBUG_PRINTLN("GAS BACKWARDS!");
        }
      } else {  //EBrake is active, disable gas
        sndStat = comm_can_set_current_brake_rel(VESC_CAN_ID, ebrake_percent);
        start_up = 0;
        DEBUG_PRINTLN("BRAKE!");
      }
    } else { // Limit to 25km/h with pedaling
      if ((ebrake_percent == 0) && (BRAKE_L) && (BRAKE_R)) { // no brake, gas OK
        if (SWITCH_R) {          // Drive forwards
          if (pedal == 0) {
            gas_filtered = GAS_ADC_LOWER;
          }
          pedal = 1;
          gas25_factor_old = gas25_factor;
          gas25_factor = constrain(map(erpm, ERPM25_LOWER, ERPM25_UPPER, 1023, 0), 0, 1023);
          if ((erpm <= erpm_alt) && (gas25_factor > (gas25_factor_old + (2 << GAS_FACTOR_LAG)))) { // maximum gas increase, full scale per loop
            gas25_factor = gas25_factor_old + (2 << GAS_FACTOR_LAG);
          }
          gas_filtered = (((gas_filtered - GAS_ADC_LOWER) * gas25_factor) >> 10) + GAS_ADC_LOWER;
          gas_percent = constrain(map(gas_filtered, GAS_ADC_LOWER, GAS_ADC_UPPER, 0, 100000), 0, 100000);
          erpm_alt = erpm;
          sndStat = comm_can_set_current_rel(VESC_CAN_ID, gas_percent);
          DEBUG_PRINTLN("GAS FORWARDS!");
        } else { // Drive backwards
          sndStat = comm_can_set_current_rel(VESC_CAN_ID, -1 * gas_percent);
          DEBUG_PRINTLN("GAS BACKWARDS!");
        }
      } else {  //EBrake is active, disable gas
        sndStat = comm_can_set_current_brake_rel(VESC_CAN_ID, ebrake_percent);
        start_up = 0;
        DEBUG_PRINTLN("BRAKE!");
      }
    }   
  } else { // StVO illegal
    DEBUG_PRINTLN("STVO ILLEGAL!"); 
    if ((ebrake_percent == 0) && (BRAKE_L) && (BRAKE_R)) { // no brake, gas OK
      if (SWITCH_R) {          // Drive forwards
        sndStat = comm_can_set_current_rel(VESC_CAN_ID, gas_percent);
        DEBUG_PRINTLN("GAS FORWARDS!");
      } else { // Drive backwards
        sndStat = comm_can_set_current_rel(VESC_CAN_ID, -1 * gas_percent);
        DEBUG_PRINTLN("GAS BACKWARDS!");
      }
    } else {  //EBrake is active, disable gas
      sndStat = comm_can_set_current_brake_rel(VESC_CAN_ID, ebrake_percent);
      start_up = 0;
      DEBUG_PRINTLN("BRAKE!");
    }
  }
  if (sndStat == CAN_OK) {  //check if CAN transmission was received
    digitalWrite(LED_TX, HIGH);
    DEBUG_PRINTLN("Message Sent Successfully!");
    delay(5);
    digitalWrite(LED_TX, LOW);
  } else {
    digitalWrite(LED_F, HIGH);
    DEBUG_PRINTLN("Error Sending Message...");
    delay(5);
    digitalWrite(LED_F, LOW);
  }

  // loop time calculation
  loop_end_micros = micros();
  loop_time = loop_end_micros - loop_begin_micros;
  delay(LOOP_TIME_STEP);
}

// TODO: move to header file
// CAN commands
typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_FILL_RX_BUFFER,
  CAN_PACKET_FILL_RX_BUFFER_LONG,
  CAN_PACKET_PROCESS_RX_BUFFER,
  CAN_PACKET_PROCESS_SHORT_BUFFER,
  CAN_PACKET_STATUS,
  CAN_PACKET_SET_CURRENT_REL,
  CAN_PACKET_SET_CURRENT_BRAKE_REL,
  CAN_PACKET_SET_CURRENT_HANDBRAKE,
  CAN_PACKET_SET_CURRENT_HANDBRAKE_REL
} CAN_PACKET_ID;

// TODO: move to VESC specific file
byte comm_can_set_current_rel(uint8_t controller_id, int32_t current) {
  int32_t send_index = 0; // incremented by buffer_append()
  uint8_t send_buffer[4];
  buffer_append_int32(send_buffer, current, &send_index);
  return CAN0.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), 1, send_index,
                         send_buffer);
}

byte comm_can_set_current_brake_rel(uint8_t controller_id, int32_t current) {
  int32_t send_index = 0; // incremented by buffer_append()
  uint8_t send_buffer[4];
  buffer_append_int32(send_buffer, current, &send_index);
  return CAN0.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), 1,
						send_index, send_buffer);
}

byte comm_can_set_duty(uint8_t controller_id, int32_t duty) {
  int32_t send_index = 0; // incremented by buffer_append()
  uint8_t send_buffer[4];
  buffer_append_int32(send_buffer, duty, &send_index);
  // returns sendStat
  return CAN0.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), 1, send_index,
                         send_buffer);
}

void com_can_full_break(uint8_t controller_id) { comm_can_set_duty(controller_id, 0); }

void comm_can_release(uint8_t controller_id) { comm_can_set_current_rel(controller_id, 0); }

void can_erpm() {
  if (!digitalRead(MCP_INT)) { // If MCP_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    if ((rxId & 0x80000000) == 0x80000000) {    // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    } else {
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
      DEBUG_PRINTLN(msgString);
    }
    if ((rxId & 0x40000000) == 0x40000000) {    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      DEBUG_PRINTLN(msgString);
    } else {
      for (byte i = 0; i<len; i++) {
        if (i >= 0 && i <= 3) {
          erpm = erpm << 8 | rxBuf[i];
        }
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        DEBUG_PRINTLN(msgString);
      }
    }
  }
}

// utility functions

// TODO: ugly
void update_switches() {
  SWITCH_L = digitalRead(SWITCH_L_PIN);
  SWITCH_R = digitalRead(SWITCH_R_PIN);
}

void update_brakes() {
  BRAKE_L = digitalRead(BRAKE_L_PIN);
  BRAKE_R = digitalRead(BRAKE_R_PIN);
}

void rotational_speed() {
  if (SWITCH_L) { // StVO legal
    prpm_t2 = millis();
    prpm = (int32_t)(3000/(prpm_t2-prpm_t1));
    prpm_t1 = prpm_t2;
  }  
}

void read_filter_adc(int pin, int num_readings, int filter_alpha, int32_t *output) {
  for (int i = 0; i < num_readings; i++) {
    int32_t raw_read1 = analogRead(pin);
    int32_t raw_read2 = analogRead(pin);
    int32_t raw_read3 = analogRead(pin);
    median_3(raw_read1, raw_read2, raw_read3, &raw_read);
    *output = ((*output << filter_alpha)+(raw_read - *output)) >> filter_alpha;
  }
}

void led_flash_all(int on_time, int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_F, HIGH);
    digitalWrite(LED_RX, HIGH);
    digitalWrite(LED_TX, HIGH);
    delay(on_time);
    digitalWrite(LED_F, LOW);
    digitalWrite(LED_RX, LOW);
    digitalWrite(LED_TX, LOW);
    if (i < count - 1) {
      delay(on_time);
    }
  }
}

void median_3(int32_t a, int32_t b, int32_t c, int32_t *output) {
  if ((a <= b) && (a <= c)) {
    *output = (b <= c) ? b : c;
  } else if ((b <= a) && (b <= c)) {
    *output = (a <= c) ? a : c;
  } else {
    *output = (a <= b) ? a : b;
  }
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

//TODO: evaluate map2() vs standard map()
long map2(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

/*********************************************************************************************************
END FILE
*********************************************************************************************************/
