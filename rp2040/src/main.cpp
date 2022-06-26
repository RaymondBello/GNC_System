#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

// #include "FreeRTOS.h"
// #include "FreeRTOSConfig.h"
// #include "task.h"
#include "mpu6050_dmp.h"
#include "pwm_driver.h"

#include "standard.h"
#include "config.h"
#include "radio.h"
#include "FC.h"

// Defines and variable declarations
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gy;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

const uint RED_LED_PIN = 21;
const uint GREEN_LED_PIN = 22;
const uint BLUE_LED_PIN = 23;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

float *imu_ptr;

// Class Instantiation
FC flightController;
MPU6050 mpu;

void dmpDataReady()
{
    mpuInterrupt = true;
}

int main()
{
    stdio_init_all();

    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(MPU9250_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MPU9250_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(MPU9250_SDA);
    gpio_pull_up(MPU9250_SCL);
    gpio_init(RED_LED_PIN);
    gpio_init(GREEN_LED_PIN);
    gpio_init(BLUE_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);
    gpio_set_dir(BLUE_LED_PIN, GPIO_OUT);

    gpio_put(RED_LED_PIN, 1);
    gpio_put(BLUE_LED_PIN, 0);
    gpio_put(GREEN_LED_PIN, 0);

    sleep_ms(TERMINAL_WAIT_DELAY);

    flightController.setup_mcu();
    flightController.setup_radio_receiver();

    float pitch_calib_err = 0.0;
    float roll_calib_err = 0.0;
    float calibration_counter = 0;
    float calibration_loop_count = 2000;
    bool calibration_done = false;
    

    //Set radio channels to default (safe) values before entering main loop
    flightController.channel_1_pwm = flightController.channel_1_fs;
    flightController.channel_2_pwm = flightController.channel_2_fs;
    flightController.channel_3_pwm = flightController.channel_3_fs;
    flightController.channel_4_pwm = flightController.channel_4_fs;
    flightController.channel_5_pwm = flightController.channel_5_fs;
    flightController.channel_6_pwm = flightController.channel_6_fs;
    flightController.channel_7_pwm = flightController.channel_7_fs;
    flightController.channel_8_pwm = flightController.channel_8_fs;

    float motor_init_pwm_value = MOTOR_INIT_PWM;
    float servo1_init_pwm_value = SERVO1_INIT_PWM;
    float servo2_init_pwm_value = SERVO2_INIT_PWM;

    // Setup Servos
    PWM Servo1{MOTOR3_PIN, servo1_init_pwm_value, MIN_PWM, MAX_PWM};
    PWM Servo2{MOTOR4_PIN, servo2_init_pwm_value, MIN_PWM, MAX_PWM};
    Servo1.write_pwm(SERVO1_INIT_PWM);
    Servo2.write_pwm(SERVO2_INIT_PWM);

    // Calibrate ESC before going into main loop
    gpio_put(BLUE_LED_PIN, 1);
    printf("[INFO] Calibrating Motors...\n");
    PWM Motor1{MOTOR1_PIN, motor_init_pwm_value, MIN_PWM, MAX_PWM};
    PWM Motor2{MOTOR2_PIN, motor_init_pwm_value, MIN_PWM, MAX_PWM};
    Motor1.write_pwm(MIN_PWM);
    Motor2.write_pwm(MIN_PWM);
    sleep_ms(1500);
    Motor1.write_pwm(MAX_PWM);
    Motor2.write_pwm(MAX_PWM);
    sleep_ms(1500);
    Motor1.write_pwm(CHAN3_FS);
    Motor2.write_pwm(CHAN3_FS);
    printf("[INFO] Calibrated\n");


    // // TEST MOTORS
    // sleep_ms(1500);
    // Motor1.write_pwm(1050);
    // Motor2.write_pwm(1050);
    // sleep_ms(1500);
    // Motor1.write_pwm(CHAN3_FS);
    // Motor2.write_pwm(CHAN3_FS);

    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true); // turn on the DMP, now that it's ready
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;                         // set our DMP Ready flag so the main loop() function knows it's okay to use it
        packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
    }
    else
    { // ERROR!        1 = initial memory load failed         2 = DMP configuration updates failed        (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)", devStatus);
        sleep_ms(2000);
    }
    gpio_put(RED_LED_PIN, 0);
    gpio_put(BLUE_LED_PIN, 0);
    gpio_put(GREEN_LED_PIN, 1);

    while (1)
    {
        if (!dmpReady)
            ; // if programming failed, don't try to do anything
        mpuInterrupt = true;
        fifoCount = mpu.getFIFOCount();                 // get current FIFO count
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) // check for overflow (this should never happen unless our code is too inefficient)
        {
            mpu.resetFIFO(); // reset so we can continue cleanly
            printf("FIFO overflow!");
        }
        else if (mpuIntStatus & 0x01) // otherwise, check for DMP data ready interrupt (this should happen frequently)
        {
            while (fifoCount < packetSize)
                fifoCount = mpu.getFIFOCount();       // wait for correct available data length, should be a VERY short wait
            mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO
            fifoCount -= packetSize;                  // track FIFO count here in case there is > 1 packet available
            
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            yaw = ypr[0] * 180 / PI;
            pitch = ypr[1] * 180 / PI;
            roll = ypr[2] * 180 / PI;
            // printf("ypr: %f, %f, %f\n", yaw, pitch, roll);
            // printf("%f, %f, %f\n", yaw, pitch, roll);

            
        }

        // Update Time variables
        flightController.prev_time = flightController.current_time;
        flightController.current_time = time_us_32();
        flightController.dt = (flightController.current_time - flightController.prev_time) / 1000000.0; // conversion to seconds

        // Correct sensor measurement with calculated error
        // pitch = pitch - pitch_calib_err;
        // roll = roll - roll_calib_err;

        flightController.pitch_IMU = pitch;
        flightController.roll_IMU = roll;
        flightController.yaw_IMU = yaw;

        // Computed Desired State
        flightController.update_desired_state();

        printf("yaw, %.1f, pitch, %.1f, roll, %.1f, Hz, %.1f, ", flightController.yaw_IMU, flightController.pitch_IMU, flightController.roll_IMU, 1 / flightController.dt);
        printf("roll_des, %.1f, pitch_des, %.1f, thro_des, %.1f, yaw_des, %.1f, CH1, %d, CH2, %d, CH3, %d, CH4, %d, CH5, %d, CH6, %d, CH7, %d, CH8, %d,  ", flightController.roll_des, flightController.pitch_des, flightController.thro_des, flightController.yaw_des, flightController.channel_1_pwm, flightController.channel_2_pwm, flightController.channel_3_pwm, flightController.channel_4_pwm, flightController.channel_5_pwm, flightController.channel_6_pwm, flightController.channel_7_pwm, flightController.channel_8_pwm);
        printf("m1_comm, %d, m2_comm, %d, m3_comm, %d, m4_comm, %d\n", flightController.m1_command_PWM, flightController.m2_command_PWM, flightController.m3_command_PWM, flightController.m4_command_PWM);
        printf("\n");

        // PID Controller
        flightController.controlANGLE(); // Stabilize on angle setpoint

        // Actuator Mixing
        flightController.controlMixer(); //mixes PID outputs to scaled actuator commands -- custom mixing assignments done here

        // Write out commands to all actuators
        if (flightController.channel_6_pwm >= 1850)
        {
            Motor1.write_pwm(flightController.m1_command_PWM);
            Motor2.write_pwm(flightController.m2_command_PWM);
            // Servo1.write_pwm(flightController.m3_command_PWM);
            // Servo2.write_pwm(flightController.m4_command_PWM);
            printf("ARMED\n");
        }
        else
        {
            Motor1.write_pwm(CHAN3_FS);
            Motor2.write_pwm(CHAN3_FS);
            Servo1.write_pwm(SERVO1_INIT_PWM);
            Servo2.write_pwm(SERVO2_INIT_PWM);
        }

        // Get vehicle comands for next iteration
        flightController.get_radio_commands();
        
        // flightController.loop();
    }

    return 0;
}
