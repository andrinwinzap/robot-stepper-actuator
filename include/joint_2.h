#ifndef JOINT_2_H
#define JOINT_2_H

#define I2C_SDA_GPIO 18
#define I2C_SCL_GPIO 17
#define I2C_PORT I2C_NUM_0
#define I2C_FREQ_HZ 400000

#define STEP_PIN GPIO_NUM_4
#define DIR_PIN GPIO_NUM_5
#define EN_PIN GPIO_NUM_6
#define INVERT_STEPPER true

#define GEAR_RATIO 15.0f
#define STEPS_PER_REV 200
#define MICROSTEPS 8

#define ACTUATOR_MAX 3.14
#define ACTUATOR_MIN -3.14
#define MAX_SPEED 6.0
#define MAX_ACCELERATION 3.14

#define INVERT_AS5600 true
#define SPEED_FILTER_ALPHA 0.1f
#define SPEED_DEADBAND 0.01f

#define HALL_EFFECT_SENSOR_PIN 8

#define KP 1.0f
#define KI 0.0f
#define KD 0.0f
#define KF 1.0f

#define CONTROL_LOOP_FREQUENCY 100

const char *robot_name = "robot";
const char *joint_name = "joint_2";

#endif // JOINT_2_H