#include <Wire.h>
#include <MPU6050.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <WiFi.h>

// YOUR VALUES
char ssid[] = "Redmi 12 5G";        
char password[] = "0123987654";    
char agent_ip[] = "10.57.191.92";   

MPU6050 mpu;
rcl_publisher_t yaw_publisher;
std_msgs__msg__Float32 yaw_msg;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;

float yaw = 0;
unsigned long lastTime = 0;
int16_t gx, gy, gz;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(100);
  
  // Manual MPU wake-up
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  
  mpu.initialize();
  delay(100);
  
  // ✅ FIXED: getMotion6() = 6 arguments only
  int16_t ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.println("✅ MPU6050 RAW: Gx=" + String(gx) + " Gy=" + String(gy) + " Gz=" + String(gz));
  
  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("📶 WiFi");
  int wifi_tries = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_tries < 20) {
    delay(500);
    Serial.print(".");
    wifi_tries++;
  }
  Serial.println("\n✅ WiFi: " + WiFi.localIP().toString());
  
  // micro-ROS
  set_microros_wifi_transports(ssid, password, agent_ip, 8888);
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "imu_node", "", &support);
  
  rclc_publisher_init_default(&yaw_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "yaw_angle");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  
  lastTime = millis();
  Serial.println("🎉 micro-ROS IMU LIVE!");
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  
  // ✅ FIXED: Only gyro Z for yaw
  int16_t ax, ay, az;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float gyroZ = (float)gz / 131.0;
  yaw += gyroZ * dt;
  
  if (yaw >= 360) yaw -= 360;
  if (yaw < 0) yaw += 360;
  
  yaw_msg.data = yaw;
  rcl_publish(&yaw_publisher, &yaw_msg, NULL);
  
  Serial.print("Yaw: "); 
  Serial.print(yaw, 1);
  Serial.print("° | Gz: "); 
  Serial.println(gz);
  
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
  delay(20);
}
