#include <Wire.h>
#include <MPU6050.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#include <WiFi.h>

char ssid[] = "Redmi 12 5G";
char password[] = "123456789";
char agent_ip[] = "10.19.181.92";

#define TRIGGER_PIN 19   // SPDT NO → GPIO19

/* Motor (TB6612FNG PWMA) */
#define MOTOR_PIN        25
#define MOTOR_CHANNEL    0      // only used on Arduino-ESP32 core 2.x
#define MOTOR_PWM_FREQ   1000
#define MOTOR_PWM_BITS   8
#define MOTOR_MAX        200    // hard limit on the duty value
#define MOTOR_TIMEOUT_MS 500    // stop motor if no /motor message arrives

MPU6050 mpu;

/* ROS objects */
rcl_publisher_t yaw_publisher;
rcl_publisher_t trigger_publisher;
rcl_subscription_t motor_subscriber;

std_msgs__msg__Float32 yaw_msg;
std_msgs__msg__Int32 trigger_msg;
std_msgs__msg__Int32 motor_msg;

rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;

/* IMU variables */
float yaw = 0.0;
unsigned long lastTime = 0;
int16_t gx, gy, gz;

/* Motor state */
unsigned long lastMotorMsg = 0;

void motorSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(MOTOR_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
#else
  ledcSetup(MOTOR_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcAttachPin(MOTOR_PIN, MOTOR_CHANNEL);
#endif
}

void motorWrite(int duty) {
  duty = constrain(duty, 0, MOTOR_MAX);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PIN, duty);
#else
  ledcWrite(MOTOR_CHANNEL, duty);
#endif
}

/* Called whenever the game publishes on /motor (0 = off, up to 200 = strongest) */
void motor_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  motorWrite(msg->data);
  lastMotorMsg = millis();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  motorSetup();
  motorWrite(0);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(100);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  mpu.initialize();
  delay(100);

  int16_t ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.println("MPU OK | Gz=" + String(gz));

  WiFi.begin(ssid, password);
  Serial.print("WiFi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  Serial.println("\nIP: " + WiFi.localIP().toString());

  set_microros_wifi_transports(ssid, password, agent_ip, 8888);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "imu_node", "", &support);

  rclc_publisher_init_default(
    &yaw_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "yaw_angle"
  );

  rclc_publisher_init_default(
    &trigger_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "limit_switch"
  );

  rclc_subscription_init_default(
    &motor_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "motor"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &motor_subscriber,
    &motor_msg,
    &motor_callback,
    ON_NEW_DATA
  );

  lastTime = millis();
  Serial.println("micro-ROS IMU + Trigger + Motor READY");
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  int16_t ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // float gyroX = (float)gx / 131.0;   // deg/sec
  // yaw += gyroX * dt;
  yaw += (gz / 131.0) * dt;

  if (yaw >= 360) yaw -= 360;
  if (yaw < 0) yaw += 360;

  yaw_msg.data = yaw;
  rcl_publish(&yaw_publisher, &yaw_msg, NULL);

  int trigger_state = digitalRead(TRIGGER_PIN);

  if (trigger_state == LOW) {
    trigger_msg.data = 0;   // FIRE
  } else {
    trigger_msg.data = 1;   // IDLE
  }

  rcl_publish(&trigger_publisher, &trigger_msg, NULL);

  Serial.print("Yaw: ");
  Serial.print(yaw, 1);
  Serial.print(" | Trigger: ");
  Serial.println(trigger_state);

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));

  // Failsafe: if the game stops sending, switch the motor off
  if (millis() - lastMotorMsg > MOTOR_TIMEOUT_MS) {
    motorWrite(0);
  }

  delay(20);   // ~50 Hz
}