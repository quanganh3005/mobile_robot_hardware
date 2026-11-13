// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <geometry_msgs/msg/pose2_d.h>

#include "config.h"
#include "syslog.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#include "encoder.h"
#include <WiFi.h>
// #include "wifis.h"
// #include "ota.h"

#ifdef WDT_TIMEOUT
#include <esp_task_wdt.h>
#endif
#ifdef USE_WIFI_TRANSPORT
// remove wifi initialization code from wifi transport
static inline void set_microros_net_transports(IPAddress agent_ip, uint16_t agent_port)
{
    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}
#endif

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

#ifndef NODE_NAME
#define NODE_NAME "linorobot_base_node"
#endif
#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;
rcl_subscription_t initial_pose_sub;
rcl_subscription_t set_pose_sub;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
geometry_msgs__msg__PoseWithCovarianceStamped initial_pose_msg;
geometry_msgs__msg__PoseWithCovarianceStamped set_pose_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P_1, K_I_1, K_D_1);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P_2, K_I_2, K_D_2);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;

void setup() 
{
#ifdef BOARD_INIT // board specific setup
    BOARD_INIT;
#endif

    Serial.begin(BAUDRATE);

    Serial.println(); 
    Serial.flush();
    delay(100);

    // Always turn off WiFi and Bluetooth first
    // to ensure nothing is running in the background
    WiFi.mode(WIFI_OFF);
    btStop();

    pinMode(LED_PIN, OUTPUT);
#ifdef SDA_PIN // specify I2C pins
#ifdef ESP32
    Wire.begin(SDA_PIN, SCL_PIN);
#else // teensy
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
#endif
#endif
#ifdef WDT_TIMEOUT
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
#endif
    // initWifis();
    // initOta();

    bool imu_ok = imu.init();
    if(!imu_ok)
    {
        flashLED(3);
    }

#ifdef USE_WIFI_TRANSPORT
    set_microros_net_transports(AGENT_IP, AGENT_PORT);
#else
    set_microros_serial_transports(Serial);
#endif
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
}

void loop() {
    switch (state) 
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            syslog(LOG_INFO, "%s agent available %lu", __FUNCTION__, millis());
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) 
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) 
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            syslog(LOG_INFO, "%s agent disconnected %lu", __FUNCTION__, millis());
            motor1_pid.reset();
            motor2_pid.reset();
            fullStop();
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
    // runWifis();
    // runOta();
#ifdef WDT_TIMEOUT
    esp_task_wdt_reset();
#endif
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       moveBase();
       publishData();
    }
}

void twistCallback(const void * msgin) 
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

void initialPoseCallback(const void * msgin)
{
    const geometry_msgs__msg__PoseWithCovarianceStamped * msg = (const geometry_msgs__msg__PoseWithCovarianceStamped *)msgin;

    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    // Calculate yaw (theta) from quaternion (z, w)
    // Robot 2D only rotates around Z axis
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    
    // Formula for converting Quaternion to Euler
    double siny_cosp = 2.0 * (qw * qz);
    double cosy_cosp = 1.0 - 2.0 * (qz * qz);
    float yaw = atan2(siny_cosp, cosy_cosp);

    // Set pose
    odometry.setPose(x, y, yaw);

    // Reset PID
    motor1_pid.reset();
    motor2_pid.reset();
    fullStop();

    // Update prev_odom_update to prevent div by 0
    prev_odom_update = millis();

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    syslog(LOG_INFO, "Initial Pose: x=%.2f, y=%.2f, theta=%.2f", x, y, yaw);
}

void setPoseCallback(const void * msgin)
{
    const geometry_msgs__msg__Pose2D * msg = (const geometry_msgs__msg__Pose2D *)msgin;

    odometry.setPose(msg->x, msg->y, msg->theta);

    motor1_pid.reset();
    motor2_pid.reset();
    fullStop();

    prev_odom_update = millis();

    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    syslog(LOG_INFO, "Set Pose: x=%.2f, y=%.2f, theta=%.2f", msg->x, msg->y, msg->theta);
}

bool createEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_PREFIX "odom/unfiltered"
    ));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        TOPIC_PREFIX "imu/data"
    ));
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_PREFIX "cmd_vel"
    ));
    // create initial pose subscriber
    RCCHECK(rclc_subscription_init_default(
        &initial_pose_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseWithCovarianceStamped),
        TOPIC_PREFIX "initialpose"
    ));
    // creat set pose subscriber
    RCCHECK(rclc_subscription_init_default(
        &set_pose_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
        TOPIC_PREFIX "set_pose"
    ));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    // 1 timer (control) + 3 sub (twist, initialpose, setpose) = 4
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &initial_pose_sub,
        &initial_pose_msg,
        &initialPoseCallback,
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &set_pose_sub,
        &set_pose_msg,
        &setPoseCallback,
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);
    prev_odom_update = millis();
    
    return true;
}

bool destroyEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_subscription_fini(&initial_pose_sub, &node);
    rcl_subscription_fini(&set_pose_sub, &node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);
    
    return true;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
}

void moveBase()
{
    unsigned long now = millis();

    // ------------------------------------------------------------------
    // 1. READ SENSORS & UPDATE ODOMETRY
    // ------------------------------------------------------------------
    
    // Get current RPM
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();

    // Calculate elapsed time
    float vel_dt = (now - prev_odom_update) / 1000.0;

    if (prev_odom_update == 0)
    {
        vel_dt = 0.0;
    }

    // Update prev_odom_update
    prev_odom_update = now;

    // Update Odom
    if (vel_dt > 0.0f)
    {
        Kinematics::velocities current_vel = kinematics.getVelocities(
            current_rpm1, 
            current_rpm2,
            0.0,
            0.0
        );
        odometry.update(
            vel_dt, 
            current_vel.linear_x, 
            current_vel.linear_y, 
            current_vel.angular_z
        );
    }

    // ------------------------------------------------------------------
    // 2. WATCHDOG
    // ------------------------------------------------------------------
    static bool watchdog_stopped = false;

    // brake if there's no command received
    if (millis() - prev_cmd_time >= 600)
    {
        if (!watchdog_stopped)
        {
            watchdog_stopped = true;

            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.z = 0.0;

            motor1_pid.reset();
            motor2_pid.reset();
            fullStop();

            digitalWrite(LED_PIN, HIGH);
        }
        
        return; // Stop processing here
    }
    else
    {
        watchdog_stopped = false; // Re-arm watchdog
    }

    // ------------------------------------------------------------------
    // 3. MOTOR CONTROL (PID)
    // ------------------------------------------------------------------

    // Calculate required RPM
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );

    // STOP_EPS + Active braking
    float angular_to_linear = twist_msg.angular.z * (LR_WHEELS_DISTANCE * 0.5f);
    float cmd_mag = hypot(twist_msg.linear.x, angular_to_linear);
    const float STOP_EPS = 0.01f; // 1 cm/s
    if (cmd_mag < STOP_EPS)
    {
        req_rpm.motor1 = 0.0;
        req_rpm.motor2 = 0.0;
    }

    // Spin motors
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
}

void publishData()
{
    odom_msg = odometry.getData();
    imu_msg = imu.getData();

    // Calculate Euler angles
    double roll, pitch, yaw;
    roll = atan2(imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
    pitch = atan2(-imu_msg.linear_acceleration.y,
                (sqrt(imu_msg.linear_acceleration.y * imu_msg.linear_acceleration.y +
                      imu_msg.linear_acceleration.z * imu_msg.linear_acceleration.z)));
    // Note: Since Mag is not used, Yaw will drift over time if only using Accelerometer.
    // Yaw is temporarily set to 0
    yaw = 0.0;
    // Convert to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    imu_msg.orientation.x = cy * cp * sr - sy * sp * cr;
    imu_msg.orientation.y = sy * cp * sr + cy * sp * cr;
    imu_msg.orientation.z = sy * cp * cr - cy * sp * sr;
    imu_msg.orientation.w = cy * cp * cr + sy * sp * sr;

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        // Get time in milliseconds or nanoseconds
        int64_t time_ns = rmw_uros_epoch_nanos();
	timespec tp;
	tp.tv_sec = time_ns / 1000000000;
	tp.tv_nsec = time_ns % 1000000000;
	clock_settime(CLOCK_REALTIME, &tp);
#else
	unsigned long long ros_time_ms = rmw_uros_epoch_millis();
	// now we can find the difference between ROS time and uC time
	time_offset = ros_time_ms - millis();
#endif
	return true;
    }
    return false;
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

void rclErrorLoop() 
{
    while(true)
    {
        flashLED(2);
	    // runOta();
    }
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}
