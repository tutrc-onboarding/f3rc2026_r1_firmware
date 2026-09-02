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
constexpr float SPEED_SCALE = 1.25f;
constexpr float MANUAL_TARGET_YAW_RATE = SPEED_SCALE * std::numbers::pi / 2.0f; // [rad/s]

constexpr float ROBOT_RADIUS = 0.177f;
constexpr float DRIVE_WHEEL_RADIUS = 0.050f;
constexpr float ODOMETRY_WHEEL_RADIUS = 0.03f;

constexpr float DRIVE_WHEEL_THETA_1 = 90 * std::numbers::pi / 180.0f;
constexpr float DRIVE_WHEEL_THETA_2 = 199 * std::numbers::pi / 180.0f;
constexpr float DRIVE_WHEEL_THETA_3 = 340 * std::numbers::pi / 180.0f;

constexpr PIDParameters DRIVE_WHEEL_PID_PARAMS{
    .kp = 0.01f,
    .ki = 0.7f,
    .kd = 0.0f,
    .output_upper_limit = 0.6f,
    .integral_upper_limit = 1.0f,
};
constexpr float MOTOR4_VELOCITY_KP = 0.1f;
constexpr float MOTOR4_TARGET_RPS_SCALE = 3.0f;
constexpr float MOTOR4_STICK_DEAD_ZONE = 0.08f;

constexpr PIDParameters P2P_X_PID_PARAMS{
    .kp = 1.0f,
    .output_upper_limit = SPEED_SCALE * 0.3f,
};
constexpr PIDParameters P2P_Y_PID_PARAMS{
    .kp = 1.0f,
    .output_upper_limit = SPEED_SCALE * 0.3f,
};
constexpr PIDParameters P2P_YAW_PID_PARAMS{
    .kp = 1.4f,
    .output_upper_limit = SPEED_SCALE * std::numbers::pi / 1.5f,
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
GPIO motor4_pin(Motor3_GPIO_Port, Motor3_Pin);

Encoder<&htim8> motor1_encoder(2048, 2.0f, CONTROL_DT);
Encoder<&htim5> motor2_encoder(2048, 2.0f, CONTROL_DT);
Encoder<&htim4> motor3_encoder(2048, 2.0f, CONTROL_DT);
Encoder<&htim1> motor4_encoder(2048, 1.0f, CONTROL_DT);

// otor2はencoder3、motor3は反転したencoder2を使用する。
float get_motor1_feedback_rps() { return motor1_encoder.get_rps(); }
float get_motor2_feedback_rps() { return motor3_encoder.get_rps(); }
float get_motor3_feedback_rps() { return -motor2_encoder.get_rps(); }

Motor<&htim15> motor1(TIM_CHANNEL_1, motor1_pin);
Motor<&htim20> motor2(TIM_CHANNEL_2, motor2_pin);
Motor<&htim20> motor3(TIM_CHANNEL_1, motor3_pin);
Motor<&htim3> motor4(TIM_CHANNEL_4, motor4_pin);

PIDController motor1_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
PIDController motor2_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);
PIDController motor3_pid(DRIVE_WHEEL_PID_PARAMS, CONTROL_DT);

PS3 ps3(uart4);
BNO055<&hi2c3> imu;

constexpr int SERVO2_ARM_RAISED_POSITION = 3010;
constexpr int SERVO2_ARM_LOWERED_POSITION = 868;

FeetechPositionControl BLOCK_HOLDER_6(uart5, 6, 2834);                     // 2250-3236
FeetechPositionControl BLOCK_HOLDER_5(uart5, 5, 949);                      // 542-1680
FeetechPositionControl BLOCK_LIFTER_4(uart5, 4, 4095);                     // 523-4000
FeetechPositionControl PLANT_HOLDER_3(uart5, 3, 3261);                     // 1123-3261
FeetechPositionControl RAIL_REVO_2(uart5, 2, SERVO2_ARM_LOWERED_POSITION); // 525-3272
FeetechPositionControl BLOCK_PUTTER_1(uart5, 1, 3242);                     // 579-3022

std::atomic<float> imu_yaw = 0.0f;
std::atomic<float> debug_world_velocity_yaw = 0.0f;

std::atomic<float> debug_pose_x = 0.0f;
std::atomic<float> debug_pose_y = 0.0f;
std::atomic<float> debug_pose_yaw = 0.0f;
std::atomic<float> debug_belt_position = 0.0f;
std::atomic<float> debug_belt_velocity = 0.0f;

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
// 直進中は、この範囲を超えた向きずれをIMUで検出して旋回速度にフィードバックする。
constexpr float SEQUENCE_YAW_TOLERANCE = 0.08f; // [rad] yawの許容誤差（約5度）
constexpr uint32_t WATERING_START_TICKS = 500;  // [1/100秒]倉庫Bから白ブロックを運んでから何秒待って水やりを開始するか
uint32_t competition_ticks = 0;                 // 競技時間を計測
uint32_t waiting_ticks = 0;                     // どんくらい待ってるか
bool competition_running = false;               // 計測のトリガー的な

// R2スタートゾーンの中心を原点、右を+x、上を+y
constexpr Pose R2_START_POSE{0.0f, 0.0f, 0.5f * std::numbers::pi};
float target_yaw = 0.0f;
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

enum class MechaCommand {
  NONE,
  PLANT_HOLD,             // プラント把持
  PLANT_RELEASE,          // プラント解放
  BLOCK_HOLD_AND_LIFT_UP, // ブロック全把持・上昇
  F_BLOCK_RELEASE,        // 前方ブロックの解放
  R_BLOCK_RELEASE,        // 後方ブロックの解放
  BLOCK_LIFT_DOWN,        // 全ブロック下降
  UP_ARM,                 // アーム上昇
  DOWN_ARM,               // アーム下降
  RISE_ARM,               // アームを起き上がらせる
  CLOSE_ARM,              // アームを閉じる
  OPEN_ARM,               // アームを開ける
};

struct Position {
  int open;
  int close;
};

constexpr Position servo_pos[] = {
    {3242, 2264},                                              // S1
    {SERVO2_ARM_RAISED_POSITION, SERVO2_ARM_LOWERED_POSITION}, // S2
    {3261, 1858},                                              // S3
    {0, 4095},                                                 // S4
    {949, 1681},                                               // S5
    {2834, 2102},                                              // S6
};

constexpr int servo1_plant_close = 1017;
constexpr int servo1_block_close = 2264;

enum class ARM_INFO {
  STORED,
  RAISED,
};

enum class LIFT_INFO {
  LOWERED,
  RAISED,
};

ARM_INFO arm_info = ARM_INFO::STORED;
LIFT_INFO lift_info = LIFT_INFO::LOWERED;
uint32_t block_lift_delay_ticks = 0;

enum class ArmHandState {
  OPEN,
  BLOCK_CLOSED,
  PLANT_CLOSED,
};

ArmHandState next_arm_hand_close_state(ArmHandState state) {
  return state == ArmHandState::BLOCK_CLOSED ? ArmHandState::PLANT_CLOSED : ArmHandState::BLOCK_CLOSED;
}

ArmHandState arm_hand_state = ArmHandState::OPEN;

std::atomic<float> circle_counter = 0.0f;

Pose robot_pose = R2_START_POSE;

AutoControlMode auto_control_mode = AutoControlMode::EMERGENCY_STOP;
MechaCommand mecha_command = MechaCommand::NONE;

std::atomic<float> motor4_target_rps = 0.0f;

std::atomic<float> motor4_vel_error = 0.0f;

void timer_callback(void *);
void update_localization();
Velocity calculate_velocity(const Pose &now_pose, const Pose &target_pose);
void drive_wheels(const Velocity &cmd_vel);
void stop_drive_wheels();
void set_auto_control_mode(AutoControlMode mode);
void move_to_pose(const Pose &target_pose, AutoControlMode next_mode);
void move_servo(FeetechPositionControl &servo, float target_position);
void collect_block_and_watering_can();
void set_mecha_command(MechaCommand command);
void stop_motor4();
void control_motor4_manually();

extern "C" void app_main() {
  halx::driver::enable_stdout(lpuart1);

  uart4.start();
  lpuart1.start();

  printf("starting uart5...\r\n");
  const bool uart5_started = uart5.start();
  printf("uart5.start(): %s\r\n", uart5_started ? "OK" : "FAILED");

  motor1_encoder.start();
  motor2_encoder.start();
  motor3_encoder.start();
  motor4_encoder.start();

  motor1.start();
  motor2.start();
  motor3.start();
  motor4.start();

  imu.start();
  printf("ping servo ID 1...\r\n");
  BLOCK_PUTTER_1.start();
  printf("ping servo ID 1 OK\r\n");

  printf("ping servo ID 2...\r\n");
  RAIL_REVO_2.start();
  printf("ping servo ID 2 OK\r\n");

  printf("ping servo ID 3...\r\n");
  PLANT_HOLDER_3.start();
  printf("ping servo ID 3 OK\r\n");

  printf("ping servo ID 4...\r\n");
  BLOCK_LIFTER_4.start();
  printf("ping servo ID 4 OK\r\n");

  printf("ping servo ID 5...\r\n");
  BLOCK_HOLDER_5.start();
  printf("ping servo ID 5 OK\r\n");

  printf("ping servo ID 6...\r\n");
  BLOCK_HOLDER_6.start();
  printf("ping servo ID 6 OK\r\n");

  const auto mae_theta = std::get<0>(*imu.get_euler());
  const auto migi_theta = mae_theta + 1.5f * static_cast<float>(std::numbers::pi);
  const auto hidari_theta = mae_theta + 0.5f * static_cast<float>(std::numbers::pi);
  const auto ushiro_theta = mae_theta + 1.0f * static_cast<float>(std::numbers::pi);
  target_yaw = mae_theta;
  BLOCK_LIFTER_4.set_position(servo_pos[4 - 1].open);
  BLOCK_HOLDER_5.set_position(servo_pos[5 - 1].close);
  BLOCK_HOLDER_6.set_position(servo_pos[6 - 1].close);

  ST_TIM<&htim6>::register_period_elapsed_callback(timer_callback, nullptr);
  ST_TIM<&htim6>::start_base_it();

  uint32_t belt_debug_print_count = 0;
  while (true) {
    if (auto euler = imu.get_euler()) {
      imu_yaw = std::get<0>(*euler);
    }
    BLOCK_HOLDER_6.update();
    BLOCK_HOLDER_5.update();
    BLOCK_LIFTER_4.update();
    PLANT_HOLDER_3.update();
    RAIL_REVO_2.update();
    BLOCK_PUTTER_1.update();

    // printf("x: %f, y: %f, yaw: %f, block_pos: %f, watering_pos: %f\n\r", debug_pose_x.load(), debug_pose_y.load(),
    //        debug_pose_yaw.load(), block_holder_servo.get_position(), watering_can_servo.get_position());
    // printf("block_holder_pos %d\n\r", static_cast<int>(block_holder_servo.get_position()));
    // printf("yaw %f\n\r", debug_pose_yaw.load());
    // printf("world_velocity.yaw: %f rad/s, imu_yaw: %f rad, target_yaw: %f rad, servo6_pos: %f \r\n",
    //        debug_world_velocity_yaw.load(), imu_yaw.load(), target_yaw, BLOCK_HOLDER_6.get_position());

    // printf("target : %F, error : %f, output : %f, vel %f\r\n", motor4_target_rps.load(), motor4_vel_error.load(),
    //        std::clamp(MOTOR4_VELOCITY_KP * motor4_vel_error.load(), -0.20f, 0.20f), motor4_encoder.get_rps());
    halx::core::delay(10);
  }
}

void timer_callback(void *) {
  motor1_encoder.update();
  motor2_encoder.update();
  motor3_encoder.update();
  motor4_encoder.update();
  debug_belt_position = motor4_encoder.get_position();
  debug_belt_velocity = motor4_encoder.get_rps();
  ps3.update();

  // BLOCK_HOLDER_6.update();

  update_localization();

  if (ps3.get_key_down(PS3Key::SELECT)) {
    competition_running = false;
    set_auto_control_mode(AutoControlMode::EMERGENCY_STOP);
  }

  switch (auto_control_mode) {
  case AutoControlMode::EMERGENCY_STOP:
    stop_drive_wheels();
    stop_motor4();
    if (ps3.get_key(PS3Key::L1) && ps3.get_key(PS3Key::R1)) {
      BLOCK_HOLDER_5.set_position(servo_pos[5 - 1].open);
      BLOCK_HOLDER_6.set_position(servo_pos[6 - 1].open);
      set_auto_control_mode(AutoControlMode::MANUAL);
    }
    break;

  case AutoControlMode::MANUAL: {

    control_motor4_manually();

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
    if (ps3.get_key_down(PS3Key::CIRCLE)) {
      circle_counter = circle_counter + 1.0;
      if (static_cast<int>(circle_counter.load()) % 2 == 1) {
        PLANT_HOLDER_3.set_position(servo_pos[3 - 1].open);
      } else {
        PLANT_HOLDER_3.set_position(servo_pos[3 - 1].close);
      }
    }
    if (ps3.get_key_down(PS3Key::TRIANGLE)) {
      if (ps3.get_key(PS3Key::R1)) {
        set_mecha_command(MechaCommand::BLOCK_LIFT_DOWN);
      } else if (arm_info == ARM_INFO::RAISED) {
        block_lift_delay_ticks = 0;
        set_mecha_command(MechaCommand::BLOCK_HOLD_AND_LIFT_UP);
      }
    }
    if (ps3.get_key_down(PS3Key::SQUARE)) {
      set_mecha_command(MechaCommand::F_BLOCK_RELEASE);
    }
    if (ps3.get_key_down(PS3Key::CROSS)) {
      set_mecha_command(MechaCommand::R_BLOCK_RELEASE);
    }
    if (ps3.get_key_down(PS3Key::L1)) {
      set_mecha_command(MechaCommand::RISE_ARM);
    }
    if (ps3.get_key_down(PS3Key::L2) && arm_info == ARM_INFO::RAISED) {
      set_mecha_command(MechaCommand::CLOSE_ARM);
    }
    if (ps3.get_key_down(PS3Key::R2) && arm_info == ARM_INFO::RAISED) {
      set_mecha_command(MechaCommand::OPEN_ARM);
    }
    if (ps3.get_key_down(PS3Key::DOWN) && ps3.get_key(PS3Key::R1)) {
      target_yaw = hidari_theta;
    }
    if (ps3.get_key_down(PS3Key::UP) && ps3.get_key(PS3Key::R1)) {
      target_yaw = migi_theta;
    }
    if (ps3.get_key_down(PS3Key::RIGHT) && ps3.get_key(PS3Key::R1)) {
      target_yaw = mae_theta;
    }
    if (ps3.get_key_down(PS3Key::LEFT) && ps3.get_key(PS3Key::R1)) {
      target_yaw = ushiro_theta;
    }
    switch (mecha_command) {
    case MechaCommand::NONE: {
      break;
    }
    case MechaCommand::PLANT_HOLD: {
      PLANT_HOLDER_3.set_position(servo_pos[3 - 1].close);
      break;
    }
    case MechaCommand::PLANT_RELEASE: {
      PLANT_HOLDER_3.set_position(servo_pos[3 - 1].open);
      break;
    }
    case MechaCommand::BLOCK_HOLD_AND_LIFT_UP: {
      if (arm_info != ARM_INFO::RAISED) {
        set_mecha_command(MechaCommand::NONE);
        break;
      }

      BLOCK_HOLDER_5.set_position(servo_pos[5 - 1].close);
      BLOCK_HOLDER_6.set_position(servo_pos[6 - 1].close);
      block_lift_delay_ticks++;
      if (block_lift_delay_ticks >= 10) {
        BLOCK_LIFTER_4.set_position(servo_pos[4 - 1].close);
        lift_info = LIFT_INFO::RAISED;
        set_mecha_command(MechaCommand::NONE);
      }

      break;
    }
    case MechaCommand::F_BLOCK_RELEASE: {
      BLOCK_HOLDER_6.set_position(servo_pos[6 - 1].open);
      break;
    }
    case MechaCommand::R_BLOCK_RELEASE: {
      BLOCK_HOLDER_5.set_position(servo_pos[5 - 1].open);
      break;
    }
    case MechaCommand::BLOCK_LIFT_DOWN: {
      BLOCK_LIFTER_4.set_position(servo_pos[4 - 1].open);
      lift_info = LIFT_INFO::LOWERED;
      set_mecha_command(MechaCommand::NONE);
      break;
    }
    case MechaCommand::UP_ARM: {
      break;
    }
    case MechaCommand::DOWN_ARM: {
      break;
    }
    case MechaCommand::RISE_ARM: {
      RAIL_REVO_2.set_position(SERVO2_ARM_RAISED_POSITION);
      arm_info = ARM_INFO::RAISED;
      set_mecha_command(MechaCommand::NONE);
      break;
    }
    case MechaCommand::CLOSE_ARM: {
      if (arm_info != ARM_INFO::RAISED) {
        set_mecha_command(MechaCommand::NONE);
        break;
      }

      arm_hand_state = next_arm_hand_close_state(arm_hand_state);
      if (arm_hand_state == ArmHandState::PLANT_CLOSED) {
        BLOCK_PUTTER_1.set_position(servo1_plant_close);
      } else {
        BLOCK_PUTTER_1.set_position(servo1_block_close);
      }
      set_mecha_command(MechaCommand::NONE);
      break;
    }
    case MechaCommand::OPEN_ARM: {
      if (arm_info != ARM_INFO::RAISED) {
        set_mecha_command(MechaCommand::NONE);
        break;
      }

      BLOCK_PUTTER_1.set_position(servo_pos[1 - 1].open);
      arm_hand_state = ArmHandState::OPEN;
      set_mecha_command(MechaCommand::NONE);
      break;
    }
    }
    const Pose yaw_target_pose{robot_pose.x, robot_pose.y, target_yaw};
    Velocity velocity = calculate_velocity(robot_pose, yaw_target_pose);
    const bool cross_button_pressed =
        ps3.get_key(PS3Key::UP) || ps3.get_key(PS3Key::DOWN) || ps3.get_key(PS3Key::RIGHT) || ps3.get_key(PS3Key::LEFT);
    if (!ps3.get_key(PS3Key::R1) && cross_button_pressed) {
      velocity.x = 0.0f;
      velocity.y = 0.0f;
      if (ps3.get_key(PS3Key::UP)) {
        velocity.y = -0.15f;
      }
      if (ps3.get_key(PS3Key::DOWN)) {
        velocity.y = 0.15f;
      }
      if (ps3.get_key(PS3Key::RIGHT)) {
        velocity.x = 0.2f;
      }
      if (ps3.get_key(PS3Key::LEFT)) {
        velocity.x = -0.2f;
      }
    } else if (!ps3.get_key(PS3Key::R1)) {
      velocity.x = SPEED_SCALE * 0.5f * ps3.get_axis(PS3Axis::LEFT_X);
      velocity.y = -SPEED_SCALE * 0.5f * ps3.get_axis(PS3Axis::LEFT_Y);
      target_yaw = std::remainder(target_yaw + MANUAL_TARGET_YAW_RATE * ps3.get_axis(PS3Axis::RIGHT_X) * CONTROL_DT,
                                  2.0f * std::numbers::pi);
    } else {
      velocity.x = SPEED_SCALE * 0.5f * ps3.get_axis(PS3Axis::LEFT_X);
      velocity.y = -SPEED_SCALE * 0.5f * ps3.get_axis(PS3Axis::LEFT_Y);
    }
    if (velocity.x == 0.0f && velocity.y == 0.0f && velocity.yaw == 0.0f) {
      stop_drive_wheels();
    } else {
      drive_wheels(velocity);
    }
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
void set_mecha_command(MechaCommand command) { mecha_command = command; }

void update_localization() { robot_pose.yaw = std::remainder(imu_yaw.load(), 2.0f * std::numbers::pi); }

void move_servo(FeetechPositionControl &servo, float target_position) {
  stop_drive_wheels();
  servo.set_position(target_position);
}

Velocity calculate_velocity(const Pose &now_pose, const Pose &target_pose) {
  static PIDController p2p_x_pid(P2P_X_PID_PARAMS, CONTROL_DT);
  static PIDController p2p_y_pid(P2P_Y_PID_PARAMS, CONTROL_DT);
  static PIDController p2p_yaw_pid(P2P_YAW_PID_PARAMS, CONTROL_DT);

  Velocity world_velocity;
  world_velocity.x = p2p_x_pid.solve(target_pose.x - now_pose.x);
  world_velocity.y = p2p_y_pid.solve(target_pose.y - now_pose.y);
  const float yaw_error = std::remainder(-target_pose.yaw + now_pose.yaw, 2.0f * std::numbers::pi);
  // world_velocity.yaw = std::abs(yaw_error) <= SEQUENCE_YAW_TOLERANCE ? 0.0f : p2p_yaw_pid.solve(yaw_error);
  if (std::abs(yaw_error) <= SEQUENCE_YAW_TOLERANCE) {
    world_velocity.yaw = 0.0f;

  } else {
    world_velocity.yaw = p2p_yaw_pid.solve(yaw_error);
  }

  debug_world_velocity_yaw = world_velocity.yaw;

  Velocity robot_velocity;
  robot_velocity.x = world_velocity.x * std::cos(now_pose.yaw) + world_velocity.y * std::sin(now_pose.yaw);
  robot_velocity.y = world_velocity.y * std::cos(now_pose.yaw) - world_velocity.x * std::sin(now_pose.yaw);
  robot_velocity.yaw = world_velocity.yaw;
  // robot_velocity.yaw = 0;
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

void stop_motor4() { motor4.set_output(0.0f); }

void control_motor4_manually() {
  if (arm_info != ARM_INFO::RAISED || !ps3.get_key(PS3Key::R1)) {
    stop_motor4();
    return;
  }

  const float stick_y = -0.5 * ps3.get_axis(PS3Axis::RIGHT_Y);
  if (std::abs(stick_y) <= MOTOR4_STICK_DEAD_ZONE) {
    stop_motor4();
    return;
  }

  const float target_rps = MOTOR4_TARGET_RPS_SCALE * stick_y;
  const float velocity_error = target_rps - motor4_encoder.get_rps();
  motor4_target_rps = target_rps;
  motor4_vel_error = velocity_error;
  if (motor4_encoder.get_position() <= -2.0f) {
    motor4.set_output(std::clamp(MOTOR4_VELOCITY_KP * velocity_error, -0.4f, 0.0f));
  } else if (motor4_encoder.get_position() >= 2.0f) {
    motor4.set_output(std::clamp(MOTOR4_VELOCITY_KP * velocity_error, 0.0f, 0.4f));
  } else {
    motor4.set_output(std::clamp(MOTOR4_VELOCITY_KP * velocity_error, -0.4f, 0.4f));
  }
}
