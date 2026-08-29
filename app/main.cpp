#include <atomic>
#include <cmath>
#include <cstdio>
#include <numbers>

#include <halx/core.hpp>
#include <halx/driver/gpio.hpp>
#include <halx/driver/uart_dma.hpp>
#include <halx/driver/uart_it.hpp>
#include <halx/peripheral.hpp>

#include "bno055.hpp"
#include "encoder.hpp"
#include "feetech_position_control.hpp"
#include "main.h"
#include "motor.hpp"
#include "pid_controller.hpp"
#include "ps3.hpp"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim20;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef hlpuart1;
extern I2C_HandleTypeDef hi2c3;

using halx::driver::GPIO;
using halx::driver::UART_DMA;
using halx::driver::UART_IT;
using halx::peripheral::ST_TIM;

constexpr float CONTROL_DT = 0.01f;

constexpr float ROBOT_RADIUS = 0.177f;
constexpr float DRIVE_WHEEL_RADIUS = 0.160f;
constexpr float ODOMETRY_WHEEL_RADIUS = 0.03f;

constexpr float DRIVE_WHEEL_THETA_1 = 90 * std::numbers::pi / 180.0f;
constexpr float DRIVE_WHEEL_THETA_2 = 199 * std::numbers::pi / 180.0f;
constexpr float DRIVE_WHEEL_THETA_3 = 340 * std::numbers::pi / 180.0f;

constexpr PIDParameters DRIVE_WHEEL_PID_PARAMS{
    .kp = 0.01f,
    .ki = 0.7f,
    .kd = 0.0f,
    .output_upper_limit = 0.2f,
    .integral_upper_limit = 1.0f,
};

constexpr PIDParameters P2P_X_PID_PARAMS{
    .kp = 1.0f,
    .output_upper_limit = 0.3f,
};
constexpr PIDParameters P2P_Y_PID_PARAMS{
    .kp = 1.0f,
    .output_upper_limit = 0.3f,
};
constexpr PIDParameters P2P_YAW_PID_PARAMS{
    .kp = 1.0f,
    .output_upper_limit = std::numbers::pi / 2.0f,
};

UART_IT<&hlpuart1> lpuart1;
uint8_t uart4_tx_buf[512];
uint8_t uart4_rx_buf[512];
UART_DMA<&huart4> uart4(uart4_tx_buf, sizeof(uart4_tx_buf), uart4_rx_buf, sizeof(uart4_rx_buf));
uint8_t uart5_tx_buf[512];
uint8_t uart5_rx_buf[512];
UART_DMA<&huart5> uart5(uart5_tx_buf, sizeof(uart5_tx_buf), uart5_rx_buf, sizeof(uart5_rx_buf));

GPIO motor1_pin(Motor8_GPIO_Port, Motor8_Pin);
GPIO motor2_pin(Motor5_GPIO_Port, Motor5_Pin);
GPIO motor3_pin(Motor4_GPIO_Port, Motor4_Pin);
// GPIO motor4_pin(Motor3_GPIO_Port, Motor3_Pin);

Encoder<&htim8> motor1_encoder(2048, 2.0f, CONTROL_DT);
Encoder<&htim5> motor2_encoder(2048, 2.0f, CONTROL_DT);
Encoder<&htim4> motor3_encoder(2048, 2.0f, CONTROL_DT);
// Encoder<&htim1> motor4_encoder(2048, 1.0f, CONTROL_DT);

// 実機検証結果: motor2はencoder3、motor3は反転したencoder2を使用する。
float get_motor1_feedback_rps() { return motor1_encoder.get_rps(); }
float get_motor2_feedback_rps() { return motor3_encoder.get_rps(); }
float get_motor3_feedback_rps() { return -motor2_encoder.get_rps(); }

Motor<&htim15> motor1(TIM_CHANNEL_1, motor1_pin);
Motor<&htim20> motor2(TIM_CHANNEL_2, motor2_pin);
Motor<&htim20> motor3(TIM_CHANNEL_1, motor3_pin);
// Motor<&htim3> motor4(TIM_CHANNEL_4, motor4_pin);

PIDController motor1_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
PIDController motor2_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
PIDController motor3_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);

PS3 ps3(uart4);
BNO055<&hi2c3> imu;

constexpr int BLOCK_HOLDER_OPEN_POSITION = 521;
constexpr int BLOCK_HOLDER_CLOSED_POSITION = 2028;
constexpr int WATERING_CAN_RELEASE_POSITION = 1015;
constexpr int WATERING_CAN_COLLECT_POSITION = 1560;

FeetechPositionControl block_holder_servo(uart5, 1, 521);  // 521-3353   2028でブロックを回収する
FeetechPositionControl watering_can_servo(uart5, 2, 1015); // 1015-1560

std::atomic<float> imu_yaw = 0.0f;

std::atomic<float> debug_pose_x = 0.0f;
std::atomic<float> debug_pose_y = 0.0f;
std::atomic<float> debug_pose_yaw = 0.0f;

struct Velocity {
  float x;   // [m/s]
  float y;   // [m/s]
  float yaw; // [rad/s]
};

struct Pose {
  float x;   // [m]
  float y;   // [m]
  float yaw; // [rad]
};

constexpr float SEQUENCE_POSITION_TOLERANCE = 0.05f; // [m]　許容誤差
constexpr float SEQUENCE_YAW_TOLERANCE = 0.05f;      // [rad]
constexpr uint32_t WATERING_START_TICKS = 500; // [1/100秒]倉庫Bから白ブロックを運んでから何秒待って水やりを開始するか
uint32_t competition_ticks = 0;                // 競技時間を計測
uint32_t waiting_ticks = 0;                    // どんくらい待ってるか
bool competition_running = false;              // 計測のトリガー的な

// R2スタートゾーンの中心を原点、右を+x、上を+y
constexpr Pose R2_START_POSE{0.0f, 0.0f, 0.5f * std::numbers::pi};

// 目標角度

constexpr float mae_theta = 0.5f * std::numbers::pi;
constexpr float migi_theta = 0.0f * std::numbers::pi;
constexpr float hidari_theta = 1.0f * std::numbers::pi;
constexpr float ushiro_theta = 1.5f * std::numbers::pi;

// コントロールモード一覧
enum class AutoControlMode {
  EMERGENCY_STOP,
  MANUAL,
};

Pose robot_pose = R2_START_POSE;
float target_yaw = R2_START_POSE.yaw;
AutoControlMode auto_control_mode = AutoControlMode::EMERGENCY_STOP;

void timer_callback(void *);
void update_localization();
Velocity calculate_velocity(const Pose &now_pose, const Pose &target_pose);
void drive_wheels(const Velocity &cmd_vel);
void stop_drive_wheels();
void set_auto_control_mode(AutoControlMode mode);
void move_to_pose(const Pose &target_pose, AutoControlMode next_mode);
void move_servo(FeetechPositionControl &servo, float target_position);
void collect_block_and_watering_can();
extern "C" void app_main() {
  halx::driver::enable_stdout(lpuart1);

  uart4.start();
  uart5.start();
  lpuart1.start();

  motor1_encoder.start();
  motor2_encoder.start();
  motor3_encoder.start();
  // motor4_encoder.start();

  motor1.start();
  motor2.start();
  motor3.start();
  // motor4.start();

  imu.start();

  // block_holder_servo.start();
  // watering_can_servo.start();

  ST_TIM<&htim6>::register_period_elapsed_callback(timer_callback, nullptr);
  ST_TIM<&htim6>::start_base_it();

  while (true) {
    if (auto euler = imu.get_euler()) {
      imu_yaw = std::get<0>(*euler);
    }

    // block_holder_servo.update();
    // watering_can_servo.update();

    // printf("x: %f, y: %f, yaw: %f, block_pos: %f, watering_pos: %f\n\r", debug_pose_x.load(), debug_pose_y.load(),
    //        debug_pose_yaw.load(), block_holder_servo.get_position(), watering_can_servo.get_position());
    // printf("block_holder_pos %d\n\r", static_cast<int>(block_holder_servo.get_position()));
    // printf("yaw %f\n\r", debug_pose_yaw.load());
    halx::core::delay(10);
  }
}

void timer_callback(void *) {
  motor1_encoder.update();
  motor2_encoder.update();
  motor3_encoder.update();
  // motor4_encoder.update();
  ps3.update();
  update_localization();

  if (ps3.get_key_down(PS3Key::SELECT)) {
    competition_running = false;
    set_auto_control_mode(AutoControlMode::EMERGENCY_STOP);
  }

  switch (auto_control_mode) {
  case AutoControlMode::EMERGENCY_STOP:
    stop_drive_wheels();
    if (ps3.get_key(PS3Key::L1) && ps3.get_key(PS3Key::R1)) {
      set_auto_control_mode(AutoControlMode::MANUAL);
    }
    break;

  case AutoControlMode::MANUAL: {
    if (ps3.get_key_down(PS3Key::START)) {
      robot_pose = R2_START_POSE;
      target_yaw = R2_START_POSE.yaw;
      competition_ticks = 0;
      competition_running = true;
      // block_holder_servo.set_position(BLOCK_HOLDER_OPEN_POSITION);
      // watering_can_servo.set_position(WATERING_CAN_RELEASE_POSITION);
      stop_drive_wheels();
      break;
    }
    // メモ　デバッグするときは下のコメントアウトを外してset_auto_control_modeをコメントアウトする
    if (ps3.get_key_down(PS3Key::LEFT)) {
      target_yaw = hidari_theta;
    }
    if (ps3.get_key_down(PS3Key::RIGHT)) {
      target_yaw = migi_theta;
    }
    if (ps3.get_key_down(PS3Key::UP)) {
      target_yaw = mae_theta;
    }
    if (ps3.get_key_down(PS3Key::DOWN)) {
      target_yaw = ushiro_theta;
    }

    // 並進は左スティック、旋回はIMUのyaw角を使った目標角度制御にする。
    const Pose yaw_target_pose{robot_pose.x, robot_pose.y, target_yaw};
    Velocity velocity = calculate_velocity(robot_pose, yaw_target_pose);
    velocity.x = 0.5f * ps3.get_axis(PS3Axis::LEFT_X);
    velocity.y = 0.5f * ps3.get_axis(PS3Axis::LEFT_Y);
    drive_wheels(velocity);
    break;
  }
  }
  // move_to_pose(行く場所, 次の動作)
  // move_servo(動かすサーボ, set_position)

  if (competition_running) {
    ++competition_ticks;
  }
  debug_pose_x = robot_pose.x;
  debug_pose_y = robot_pose.y;
  debug_pose_yaw = robot_pose.yaw;
}

void set_auto_control_mode(AutoControlMode mode) { auto_control_mode = mode; }

void update_localization() {
  robot_pose.yaw = std::remainder(imu_yaw.load(), 2.0f * std::numbers::pi);
}

void move_servo(FeetechPositionControl &servo, float target_position) {
  stop_drive_wheels();
  servo.set_position(target_position);
}

void collect_block_and_watering_can() { // ブロックとじょうろが同時に取れる前提で書いた
  stop_drive_wheels();
  block_holder_servo.set_position(BLOCK_HOLDER_CLOSED_POSITION);
  watering_can_servo.set_position(WATERING_CAN_COLLECT_POSITION);
}

Velocity calculate_velocity(const Pose &now_pose, const Pose &target_pose) {
  static PIDController p2p_x_pid(P2P_X_PID_PARAMS, CONTROL_DT);
  static PIDController p2p_y_pid(P2P_Y_PID_PARAMS, CONTROL_DT);
  static PIDController p2p_yaw_pid(P2P_YAW_PID_PARAMS, CONTROL_DT);

  Velocity world_velocity;
  world_velocity.x = p2p_x_pid.solve(target_pose.x - now_pose.x);
  world_velocity.y = p2p_y_pid.solve(target_pose.y - now_pose.y);
  const float yaw_error = std::remainder(target_pose.yaw - now_pose.yaw, 2.0f * std::numbers::pi);
  world_velocity.yaw = p2p_yaw_pid.solve(yaw_error);

  Velocity robot_velocity;
  robot_velocity.x = world_velocity.x * std::cos(now_pose.yaw) + world_velocity.y * std::sin(now_pose.yaw);
  robot_velocity.y = world_velocity.y * std::cos(now_pose.yaw) - world_velocity.x * std::sin(now_pose.yaw);
  robot_velocity.yaw = world_velocity.yaw;

  return robot_velocity;
}

void drive_wheels(const Velocity &velocity) {
  constexpr float VEL2RPS = 1.0f / (2.0f * std::numbers::pi * DRIVE_WHEEL_RADIUS);

  float motor1_target_rps = (-velocity.x * std::sin(DRIVE_WHEEL_THETA_1) + velocity.y * std::cos(DRIVE_WHEEL_THETA_1) +
                             ROBOT_RADIUS * velocity.yaw) *
                            VEL2RPS;
  float motor2_target_rps = (-velocity.x * std::sin(DRIVE_WHEEL_THETA_2) + velocity.y * std::cos(DRIVE_WHEEL_THETA_2) +
                             ROBOT_RADIUS * velocity.yaw) *
                            VEL2RPS;
  float motor3_target_rps = (-velocity.x * std::sin(DRIVE_WHEEL_THETA_3) + velocity.y * std::cos(DRIVE_WHEEL_THETA_3) +
                             ROBOT_RADIUS * velocity.yaw) *
                            VEL2RPS;

  motor2_target_rps = -motor2_target_rps;
  motor3_target_rps = -motor3_target_rps;

  float motor1_output = motor1_pid.solve(motor1_target_rps - get_motor1_feedback_rps());
  float motor2_output = motor2_pid.solve(motor2_target_rps - get_motor2_feedback_rps());
  float motor3_output = motor3_pid.solve(motor3_target_rps - get_motor3_feedback_rps());

  motor1.set_output(motor1_output);
  motor2.set_output(motor2_output);
  motor3.set_output(motor3_output);
}

void stop_drive_wheels() {
  motor1_pid = PIDController(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  motor2_pid = PIDController(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  motor3_pid = PIDController(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
  motor1.set_output(0.0f);
  motor2.set_output(0.0f);
  motor3.set_output(0.0f);
}
