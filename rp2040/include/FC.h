#pragma once
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"


enum SetpointControl
{
    SETPOINT_RC_RECEIVER,
    SETPOINT_ACS,
};

enum Mode
{
  Error = 0,
  Uninitialized = 1 << 0,
  Initialization = 1 << 1,
  Active = 1 << 2,
  Idle = 1 << 3
};

class FC
{
    private:
        /* Servo Pins */
        const int servo1Pin = SERVO1_PIN;
        const int servo2Pin = SERVO2_PIN;
        const int servo3Pin = SERVO3_PIN;
        const int servo4Pin = SERVO4_PIN;
        const int servo5Pin = SERVO5_PIN;
        const int servo6Pin = SERVO6_PIN;
        const int servo7Pin = SERVO7_PIN;

        


        //Filter parameters - Defaults tuned for 2kHz loop rate
        float B_madgwick = 0.04; //madgwick filter parameter
        float B_accel = 0.2;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
        float B_gyro = 0.17;     //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
        float B_mag = 1.0;       //Magnetometer LP filter parameter

        //Controller parameters (take note of defaults before modifying!):
        float i_limit = 25.0;  //Integrator saturation level, mostly for safety (default 25.0)
        float maxRoll = 30.0;  //Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
        float maxPitch = 30.0; //Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
        float maxYaw = 160.0;  //Max yaw rate in deg/sec

        float Kp_roll_angle = 0.2;   //Roll P-gain - angle mode
        float Ki_roll_angle = 0.3;   //Roll I-gain - angle mode
        float Kd_roll_angle = 0.05;  //Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0. Use B_loop_roll)
        float B_loop_roll = 0.9;     //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
        float Kp_pitch_angle = 0.2;  //Pitch P-gain - angle mode
        float Ki_pitch_angle = 0.3;  //Pitch I-gain - angle mode
        float Kd_pitch_angle = 0.05; //Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0. Use B_loop_pitch)
        float B_loop_pitch = 0.9;    //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

        float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
        float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
        float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
        float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
        float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
        float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

        float Kp_yaw = 0.3;     //Yaw P-gain
        float Ki_yaw = 0.05;    //Yaw I-gain
        float Kd_yaw = 0.00015; //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

    public:
        unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm, channel_7_pwm, channel_8_pwm;
        unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev, channel_5_pwm_prev, channel_6_pwm_prev, channel_7_pwm_prev, channel_8_pwm_prev;
        float AccX, AccY, AccZ;
        float GyroX, GyroY, GyroZ;
        float MagX, MagY, MagZ;
        float AccX_prev, AccY_prev, AccZ_prev;
        float GyroX_prev, GyroY_prev, GyroZ_prev;
        float MagX_prev, MagY_prev, MagZ_prev;
        float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
        float roll_IMU, pitch_IMU, yaw_IMU;
        float roll_IMU_prev, pitch_IMU_prev, yaw_IMU_prev;

        //Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
        unsigned long channel_1_fs = 1500; //roll
        unsigned long channel_2_fs = 1500; //pitch
        unsigned long channel_3_fs = 1000; //thro
        unsigned long channel_4_fs = 1500; //rudd
        unsigned long channel_5_fs = 1000; //gear,
        unsigned long channel_6_fs = 1000; //aux1
        unsigned long channel_7_fs = 1000; //aux1
        unsigned long channel_8_fs = 1000; //aux1

        //Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
        float MagErrorX = 0.0;
        float MagErrorY = 0.0;
        float MagErrorZ = 0.0;
        float MagScaleX = 1.0;
        float MagScaleY = 1.0;
        float MagScaleZ = 1.0;
        //initialize quaternion for madgwick filter
        float q0 = 1.0f;
        float q1 = 0.0f;
        float q2 = 0.0f;
        float q3 = 0.0f;

        //Barometric pressure sensor
        float internal_temp, pressure, altitude;

        uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
        bool dmpReady = false; // set true if DMP init was successful
        uint16_t packetSize;   // expected DMP packet size (default is 42 bytes)
        float dt;
        uint32_t current_time, prev_time;
        unsigned long blink_counter, blink_delay;
        unsigned long beep_counter, beep_delay;
        bool blinkAlternate;
        bool beepAlternate;

        //Normalized desired state:
        float thro_des, roll_des, pitch_des, yaw_des;
        float roll_passthru, pitch_passthru, yaw_passthru;

        //Controller:
        float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
        float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
        float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

        //Mixer
        float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
        int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
        float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
        int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

        #ifdef AIRFRAME_FIXEDWING
            // TBD
        #endif
        #ifdef AIRFRAME_QUADCOPTER
            float motor_init_pwm_value = MOTOR_INIT_PWM;

            // PWM Motor1{MOTOR1_PIN, motor_init_pwm_value, MIN_PWM, MAX_PWM};
            // PWM Motor2{MOTOR2_PIN, motor_init_pwm_value, MIN_PWM, MAX_PWM};
            // PWM Motor3{MOTOR3_PIN, motor_init_pwm_value, MIN_PWM, MAX_PWM};
            // PWM Motor4{MOTOR4_PIN, motor_init_pwm_value, MIN_PWM, MAX_PWM};
        #endif

        // Declare Mode Enum
        Mode mode; // current autopilot mode
        // Declare IMU class
        // Declare Baro class

        FC();                           // Constructor
        ~FC();
        void setup_mcu(); 
        void loop();        
        void setup_radio_receiver() { init_radio(); };    
        void get_radio_commands();
        void update_desired_state();
        void controlANGLE();
        void controlMixer();
};

FC::FC()
{
    mode = Uninitialized;
}

void FC::setup_mcu()
{
  // Turn on LED to indicate boot
  gpio_init(LED_BUILTIN);
  gpio_set_dir(LED_BUILTIN, GPIO_OUT);
  gpio_put(LED_BUILTIN, GPIO_ON);

  // Add MCU setup logic >>>> HERE <<<<
}


void FC::loop()
{
  switch (this->mode)
  {
    case Uninitialized:
    {
      // sleep_ms(1000);
      printf("[MODE] Uninitialized\n");
      setup_mcu();
      mode = Initialization;
      break;
    }
    case Initialization:
    {
        printf("[MODE] Initialization\n");
        

        //   this->setup_radio_receiver();
        //   BoolInt boolint = this->setup_flightcontroller();
        //   boolint.flag ? (0) : (this->mode = Mode::Error);
        //   BoolInt boolint1 = this->calibrate_flightcontroller();
        //   boolint1.flag ? (this->mode = Mode::Active) : (this->mode = Mode::Error);
        mode = Active;
        printf("[INFO] Boot sequence complete.\n");
        // sleep_ms(1000);
        break;
    }
    case Active:
    {
      // flightController.loop_blink();
      printf("[MODE] Active\n");

      // Update all timing related variables
      // update_flightcontroller_time();

      // Update the state variables related to aircraft attitude
      // update_flightcontroller_orientation();

      // Get Desired Setpoints from Radio or CLI
      // update_flightcontroller_desired_state();

      // Update PID Controller variables
      // update_flightcontroller_pid_loop();


      // Print out any debug messages
      // print_debug_msg();

      // sleep_ms(10);

      // handle_serial();
      // printf("%.2f \n", current_usec);

    //   flightController.loop_rate(1);
      break;
    }
    case Idle:
    {
      printf("[INFO]: ACS-Mode: Idle\n");
      break;
    }
    case Error:
    {
      printf("[INFO] ACS-Mode: Error\n");
      // flightController.loop_beep();
      break;
    }
    default:
    {
      printf("[FATAL]; FATAL ACS-Mode Error! Rebooting...\n");
      // this->setup_beep(4, 160, 70);
      sleep_ms(50);
      // SCB_AIRCR = 0x05FA0004; reboot command
      break;
    }
  }
}

void FC::get_radio_commands()
{
    #ifdef USE_PPM_RX

    if (PPM_PACKET[0] + PPM_OFFSET > MIN_PWM )
    {
        channel_1_pwm = PPM_PACKET[0] + PPM_OFFSET;
    }
    if (PPM_PACKET[1] + PPM_OFFSET > MIN_PWM)
    {
        channel_2_pwm = PPM_PACKET[1] + PPM_OFFSET;
    }
    if (PPM_PACKET[2] + PPM_OFFSET > MIN_PWM)
    {
        channel_3_pwm = PPM_PACKET[2] + PPM_OFFSET;
    }
    if (PPM_PACKET[3] + PPM_OFFSET > MIN_PWM)
    {
        channel_4_pwm = PPM_PACKET[3] + PPM_OFFSET;
    }
    if (PPM_PACKET[4] + PPM_OFFSET > MIN_PWM)
    {
        channel_5_pwm = PPM_PACKET[4] + PPM_OFFSET;
    }
    if (PPM_PACKET[5] + PPM_OFFSET > MIN_PWM)
    {
        channel_6_pwm = PPM_PACKET[5] + PPM_OFFSET;
    }
    if (PPM_PACKET[6] + PPM_OFFSET > MIN_PWM)
    {
        channel_7_pwm = PPM_PACKET[6] + PPM_OFFSET;
    }
    if (PPM_PACKET[7] + PPM_OFFSET > MIN_PWM)
    {
        channel_8_pwm = PPM_PACKET[7] + PPM_OFFSET;
    }

    //Low-pass the critical commands and update previous values
    float b = 0.2; //lower=slower, higher=noisier

    channel_1_pwm = (channel_1_pwm > MIN_PWM && channel_1_pwm < MAX_PWM) ? (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm : channel_1_fs;
    channel_2_pwm = (channel_2_pwm > MIN_PWM && channel_2_pwm < MAX_PWM) ? (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm : channel_2_fs;
    channel_3_pwm = (channel_3_pwm > MIN_PWM && channel_3_pwm < MAX_PWM) ? (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm : channel_3_fs;
    channel_4_pwm = (channel_4_pwm > MIN_PWM && channel_4_pwm < MAX_PWM) ? (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm : channel_4_fs;
    channel_5_pwm = (channel_5_pwm > MIN_PWM && channel_5_pwm < MAX_PWM) ? channel_5_pwm : channel_5_fs;
    channel_6_pwm = (channel_6_pwm > MIN_PWM && channel_6_pwm < MAX_PWM) ? channel_6_pwm : channel_6_fs;
    channel_7_pwm = (channel_7_pwm > MIN_PWM && channel_7_pwm < MAX_PWM) ? channel_7_pwm : channel_7_fs;
    channel_8_pwm = (channel_8_pwm > MIN_PWM && channel_8_pwm < MAX_PWM) ? channel_8_pwm : channel_8_fs;

    channel_1_pwm_prev = channel_1_pwm;
    channel_2_pwm_prev = channel_2_pwm;
    channel_3_pwm_prev = channel_3_pwm;
    channel_4_pwm_prev = channel_4_pwm;
    channel_5_pwm_prev = channel_5_pwm;
    channel_6_pwm_prev = channel_6_pwm;
    channel_7_pwm_prev = channel_7_pwm;
    channel_8_pwm_prev = channel_8_pwm;


#endif
}

void FC::update_desired_state()
{
    /*
    Normalizes desired control values to appropriate values
    Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des.
    These are computed by using the raw RC pwm commands and scaling them to be within our limits defined in config.
    roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
    (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
    yaw_passthru variables, to be used in commanding motors/servos with direct un-stabilized commands in controlMixer().
    */

    roll_des = (channel_1_pwm - 1500.0) / 500.0;  //between -1 and 1
    pitch_des = (channel_2_pwm - 1500.0) / 500.0; //between -1 and 1
    thro_des = (channel_3_pwm - 1000.0) / 1000.0; //between 0 and 1
    yaw_des = (channel_4_pwm - 1500.0) / 500.0;   //between -1 and 1

    //Constrain within normalized bounds
    thro_des = constrain(thro_des, 0.0, 1.0);               //between 0 and 1
    roll_des = constrain(roll_des, -1.0, 1.0) * maxRoll;    //between -maxRoll and +maxRoll
    pitch_des = constrain(pitch_des, -1.0, 1.0) * maxPitch * -1; //between -maxPitch and +maxPitch
    yaw_des = constrain(yaw_des, -1.0, 1.0) * maxYaw;       //between -maxYaw and +maxYaw

    roll_passthru = roll_des / (2 * maxRoll);
    pitch_passthru = pitch_des / (2 * maxPitch);
    yaw_passthru = yaw_des / (2 * maxYaw);
}

void FC::controlANGLE()
{
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * dt;

  if (channel_3_pwm < 1060)
  { //don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_roll = (roll_IMU - roll_IMU_prev)/ dt;
  roll_PID = 0.01 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll); //scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_3_pwm < 1060)
  { //don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
  pitch_PID = .01 * (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch - Kd_pitch_angle * derivative_pitch); //scaled by .01 to bring within -1 to 1 range

  //Yaw, stabilize on rate from GyroZ
  error_yaw = yaw_des - ((yaw_IMU-yaw_IMU_prev)/dt);
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_3_pwm < 1060)
  { //don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw); //scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  roll_IMU_prev = roll_IMU;

  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  pitch_IMU_prev = pitch_IMU;

  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
  yaw_IMU_prev = yaw_IMU;

  // DEBUG
  // u_int32_t ts = time_us_32();
  // float delay;
  // printf("PITCH, Error, %.2f, Integral, %.2f, Deriv, %.2f, PID, %.2f\n", error_pitch, integral_pitch, derivative_pitch, pitch_PID);
  // printf("ROLL: Error=%.3f, Integral=%.3f, Deriv=%.3f, PID=%f\n", error_roll, integral_roll, derivative_roll, roll_PID);
  // printf("YAW: Error=%.3f, Integral=%.3f, Deriv=%.3f, PID=%f\n", error_yaw, integral_yaw, derivative_yaw, yaw_PID);
  // printf("RATE: %.2fHz\n", 1/dt);
  // delay = (time_us_32() - ts) * 1000000;
  // printf("PRINT TOOK %.6f secs", delay);
}

void FC::controlMixer()
{
  ////////////////////////////////
  // Quad mixing
  ////////////////////////////////
  //m1 = front left, m2 = front right, m3 = back right, m4 = back left
  // m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID;
  // m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID;
  // m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID;
  // m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID;

  // scale commands to 1000 - 2000us range for motors
  // m1_command_PWM = m1_command_scaled * 1000 + 1000;
  // m2_command_PWM = m2_command_scaled * 1000 + 1000;
  // m3_command_PWM = m3_command_scaled * 1000 + 1000;
  // m4_command_PWM = m4_command_scaled * 1000 + 1000;
  ////////////////////////////////

  ////////////////////////////////
  // Bicopter Mixing
  ////////////////////////////////
  m1_command_scaled = thro_des + roll_PID;
  m2_command_scaled = thro_des - roll_PID;
  m3_command_scaled = pitch_PID + yaw_PID;
  m4_command_scaled = pitch_PID - yaw_PID;

  // scale commands to 1000 - 2000us range for motors
  m1_command_PWM = m1_command_scaled * 1000 + 1000;
  m2_command_PWM = m2_command_scaled * 1000 + 1000;
  m3_command_PWM = m3_command_scaled * 1000 + 1000;
  m4_command_PWM = m4_command_scaled * 1000 + 1000;
  ////////////////////////////////


  // printf("%f\n", pitch_PID);
}