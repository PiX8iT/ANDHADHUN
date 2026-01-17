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
char password[] = "0123987654";
char agent_ip[] = "10.57.191.92";

#define TRIGGER_PIN 19   // SPDT NO → GPIO19

MPU6050 mpu;

/* ROS objects */
rcl_publisher_t yaw_publisher;
rcl_publisher_t trigger_publisher;

std_msgs__msg__Float32 yaw_msg;
std_msgs__msg__Int32 trigger_msg;

rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;

/* IMU variables */
float yaw = 0.0;
unsigned long lastTime = 0;
int16_t gx, gy, gz;

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(TRIGGER_PIN, INPUT_PULLUP);

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

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  lastTime = millis();
  Serial.println("micro-ROS IMU + Trigger READY");
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  int16_t ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float gyroX = (float)gx / 131.0;   // deg/sec
  yaw += gyroX * dt;

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
  delay(20);   // ~50 Hz
}
