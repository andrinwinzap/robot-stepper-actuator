#ifndef JOINT_2_H
#define JOINT_2_H

const char *robot_name = "robot";
const char *joint_name = "joint_2";

#define CONTROL_LOOP_FREQUENCY 100

#define AS5600_I2C_PORT I2C_NUM_0
#define AS5600_I2C_SDA 18
#define AS5600_I2C_SCL 17
#define I2C_FREQ_HZ 400000
#define INVERT_AS5600 true

#define STEP_PIN GPIO_NUM_4
#define DIR_PIN GPIO_NUM_5
#define EN_PIN GPIO_NUM_6
#define STEPS_PER_REVOLUTION 200
#define MICROSTEPS 8
#define GEAR_RATIO 15.0f
#define INVERT_STEPPER true

#define ENDSTOP_PIN GPIO_NUM_8

#define ACTUATOR_MAX M_PI
#define ACTUATOR_MIN -M_PI
#define ACTUATOR_MAX_VELOCITY M_PI * 5
#define ACTUATOR_MAX_ACCELERATION M_PI * 5

#define AS5600_VELOCITY_FILTER_ALPHA 0.1f
#define ACCELERATION_FILTER_ALPHA 0.1f

#define KP 2.0f
#define KI 0.0f
#define KD 0.0f
#define KF 1.0f

#endif // JOINT_2_H
