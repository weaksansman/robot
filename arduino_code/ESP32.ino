#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include <ddsm_ctrl.h>

DDSM_CTRL dc;

#define BATTERY_PIN 36
#define DDSM_RX 18
#define DDSM_TX 19

// ── Subscribers ───────────────────────────────────────────────────────────────
rcl_subscription_t LEDs_subscriber;
std_msgs__msg__Int8 LEDs_msg;

rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;

// /lock topic  →  data=1 : send raw RS485 brake packet (byte[7]=0xFF, speed-loop)
//                data=0 : release brake, back to speed-loop at 0 RPM
rcl_subscription_t lock_subscriber;
std_msgs__msg__Int8 lock_msg;

// ── Publishers ────────────────────────────────────────────────────────────────
rcl_publisher_t battery_publisher;
std_msgs__msg__Int8 battery_msg;

rcl_publisher_t left_encoder_publisher;
std_msgs__msg__Int32 left_encoder_msg;

rcl_publisher_t right_encoder_publisher;
std_msgs__msg__Int32 right_encoder_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LEFT_LED_PIN  17
#define RIGHT_LED_PIN 16

#define LeftEncoder_C1  23
#define LeftEncoder_C2  22
#define RightEncoder_C1 19   // shared with DDSM_TX pin — read-only here
#define RightEncoder_C2 18

#define IN1_PIN 33
#define IN2_PIN 25
#define PWMA    32
#define IN3_PIN 12
#define IN4_PIN 14
#define PWMB    13
#define LED_PIN  2

int motorSpeedLeft  = 0;
int motorSpeedRight = 0;
int LeftEncoderCount  = 0;
int RightEncoderCount = 0;

// Lock state — set by /lock callback
bool isLocked = false;

void LeftEncoderCallback();
void RightEncoderCallback();

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();} }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} }

#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis(); } \
  if (uxr_millis() - init > MS) { X; init = uxr_millis(); } \
} while (0)

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ─────────────────────────────────────────────────────────────────────────────
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

int limitToMaxValue(int value, int maxLimit) {
  return (value > maxLimit) ? maxLimit : value;
}

// ─────────────────────────────────────────────────────────────────────────────
// Timer — publish encoder + battery at 1 Hz
// ─────────────────────────────────────────────────────────────────────────────
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    right_encoder_msg.data =  RightEncoderCount;
    left_encoder_msg.data  = -LeftEncoderCount;
    RCSOFTCHECK(rcl_publish(&left_encoder_publisher,  &left_encoder_msg,  NULL));
    RCSOFTCHECK(rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL));

    battery_msg.data = get_battery_percentage();
    RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL)); 
  }
}

int8_t get_battery_percentage() {
  return static_cast<int8_t>(map_to_percentage(analogRead(BATTERY_PIN)));
}

int map_to_percentage(int raw_value) {
  return map(raw_value, 2400, 3720, 0, 100);
}

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────
void LEDs_subscription_callback(const void *msgin) {
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;
  switch (msg->data) {
    case 0: digitalWrite(LEFT_LED_PIN, LOW);  digitalWrite(RIGHT_LED_PIN, LOW);  break;
    case 1: digitalWrite(LEFT_LED_PIN, HIGH); digitalWrite(RIGHT_LED_PIN, LOW);  break;
    case 2: digitalWrite(LEFT_LED_PIN, LOW);  digitalWrite(RIGHT_LED_PIN, HIGH); break;
    case 3: digitalWrite(LEFT_LED_PIN, HIGH); digitalWrite(RIGHT_LED_PIN, HIGH); break;
  }
}

// ── Raw RS485 brake helpers ───────────────────────────────────────────────────
// Per Waveshare DDSM115 protocol docs:
//   Brake is NOT a mode number — it is a special byte (0xFF) in the speed-loop
//   packet at byte index 7.  The raw brake command for motor ID 0x01 is:
//     01 64 00 00 00 00 00 FF 00 D1
//   To brake a different motor ID, substitute byte[0] and recalculate CRC-8/MAXIM.
//
// CRC-8/MAXIM: polynomial x^8 + x^5 + x^4 + 1  (0x31), reflected
uint8_t crc8_maxim(uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x01) crc = (crc >> 1) ^ 0x8C;
      else            crc >>= 1;
    }
  }
  return crc;
}

// Send the brake packet for one motor ID over Serial1 (RS485 bus)
void send_brake(uint8_t motor_id) {
  // Packet template (9 data bytes + 1 CRC byte = 10 bytes total):
  //   [ID] [0x64] [0x00] [0x00] [0x00] [0x00] [0x00] [0xFF] [0x00] [CRC]
  // byte[7] = 0xFF is the brake flag; all other cmd bytes are 0.
  uint8_t pkt[10];
  pkt[0] = motor_id;
  pkt[1] = 0x64;
  pkt[2] = 0x00;
  pkt[3] = 0x00;
  pkt[4] = 0x00;
  pkt[5] = 0x00;
  pkt[6] = 0x00;
  pkt[7] = 0xFF;   // ← brake flag
  pkt[8] = 0x00;
  pkt[9] = crc8_maxim(pkt, 9);
  Serial1.write(pkt, 10);
  delay(4);  // same inter-command gap used by ddsm_ctrl library
}

// /lock callback
// data=1 → send raw RS485 brake packet to all 4 motors (valid in speed loop mode)
// data=0 → release brake by returning to speed loop at 0 RPM
void lock_callback(const void *msgin) {
  const std_msgs__msg__Int8 *msg = (const std_msgs__msg__Int8 *)msgin;

  if (msg->data == 1 && !isLocked) {
    isLocked = true;
    // Send brake packet to each motor (IDs 1-4)
    send_brake(1);
    send_brake(2);
    send_brake(3);
    send_brake(4);

  } else if (msg->data == 0 && isLocked) {
    isLocked = false;
    // Release: return to speed loop at 0 RPM, ready to accept cmd_vel again
    dc.ddsm_ctrl(1, 0, 2);
    dc.ddsm_ctrl(2, 0, 2);
    dc.ddsm_ctrl(3, 0, 2);
    dc.ddsm_ctrl(4, 0, 2);
  }
}

// cmd_vel callback — ignored while braked so we don't fight the brake
void cmd_vel_callback(const void *msgin) {
  if (isLocked) return;  // ← do NOT override brake mode

  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear  = msg->linear.x;
  float angular = msg->angular.z;

  motorSpeedLeft  = (int)((linear - angular / 2.0f) * 1000);
  motorSpeedRight = (int)((linear + angular / 2.0f) * 1000);
  setMotorSpeed(motorSpeedLeft, motorSpeedRight);
}

// ─────────────────────────────────────────────────────────────────────────────
void setMotorSpeed(int speedLeft, int speedRight) {
  // Left motors 1 & 3: positive = forward
  dc.ddsm_ctrl(1, speedLeft,         2);
  dc.ddsm_ctrl(3, speedLeft,         2);
  // Right motors 2 & 4: inverted
  dc.ddsm_ctrl(2, speedRight * (-1), 2);
  dc.ddsm_ctrl(4, speedRight * (-1), 2);
}

// ─────────────────────────────────────────────────────────────────────────────
// micro-ROS entity lifecycle
// ─────────────────────────────────────────────────────────────────────────────
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "RMRC_2025", "", &support));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(&LEDs_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "LEDs"));


  RCCHECK(rclc_subscription_init_default(&twist_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  // NEW: lock subscriber
  RCCHECK(rclc_subscription_init_default(&lock_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "/lock"));

  // Publishers
  RCCHECK(rclc_publisher_init_default(&battery_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "battery"));

  RCCHECK(rclc_publisher_init_default(&left_encoder_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "left_motor_ticks"));

  RCCHECK(rclc_publisher_init_default(&right_encoder_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "right_motor_ticks"));

  // Timer (1 Hz)
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(&timer, &support,
    RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Executor — now 5 handles (1 timer + 4 subscribers)
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber,  &twist_msg,  &cmd_vel_callback,           ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &lock_subscriber,   &lock_msg,   &lock_callback,              ON_NEW_DATA));  // NEW

  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&battery_publisher,       &node);
  rcl_publisher_fini(&left_encoder_publisher,  &node);
  rcl_publisher_fini(&right_encoder_publisher, &node);

  rcl_subscription_fini(&LEDs_subscriber,  &node);
  rcl_subscription_fini(&twist_subscriber, &node);
  rcl_subscription_fini(&lock_subscriber,  &node);  // NEW

  rcl_node_fini(&node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  pinMode(LEFT_LED_PIN,  OUTPUT); digitalWrite(LEFT_LED_PIN,  HIGH);
  pinMode(RIGHT_LED_PIN, OUTPUT); digitalWrite(RIGHT_LED_PIN, HIGH);
  pinMode(BATTERY_PIN, INPUT);

  pinMode(LeftEncoder_C1,  INPUT_PULLUP);
  pinMode(LeftEncoder_C2,  INPUT_PULLUP);
  pinMode(RightEncoder_C1, INPUT_PULLUP);
  pinMode(RightEncoder_C2, INPUT_PULLUP);

  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
  pinMode(PWMA,    OUTPUT); pinMode(PWMB,    OUTPUT);
  digitalWrite(PWMA, HIGH); digitalWrite(PWMB, HIGH);

  attachInterrupt(digitalPinToInterrupt(LeftEncoder_C1),  LeftEncoderCallback,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightEncoder_C1), RightEncoderCallback, CHANGE);

  delay(2000);

  Serial1.begin(DDSM_BAUDRATE, SERIAL_8N1, DDSM_RX, DDSM_TX);
  dc.pSerial = &Serial1;
  dc.set_ddsm_type(115);
  dc.clear_ddsm_buffer();

  left_encoder_msg.data  = 0;
  right_encoder_msg.data = 0;

  state = WAITING_AGENT;
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(100,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      );
      break;

    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroy_entities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(100,
        state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      );
      if (state == AGENT_CONNECTED)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }

  digitalWrite(LED_PIN, (state == AGENT_CONNECTED) ? HIGH : LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
void LeftEncoderCallback() {
  LeftEncoderCount += (digitalRead(LeftEncoder_C1) == digitalRead(LeftEncoder_C2)) ? 1 : -1;
}

void RightEncoderCallback() {
  RightEncoderCount += (digitalRead(RightEncoder_C1) == digitalRead(RightEncoder_C2)) ? 1 : -1;
}
