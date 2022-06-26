
/**************************************************************/
/************* VEHICLE CONFIGURATION  *************************/
/**************************************************************/
#define AIRFRAME_QUADCOPTER             
// #define AIRFRAME_FIXEDWING
// #define USE_DIFFERENTIAL_THRUST

/*** @brief Timings and Conversions
 */
#define RAD_TO_DEG  57.2957795131f
#define G_MPS2      9.81000000000f
#define GPIO_ON 1
#define GPIO_OFF 0

/*** @brief Complementary Filter
 */
#define COMP_FILT_ALPHA     0.05000000000f


/*** @brief USB Serial Data rate (bps)
 */
#define SERIAL_BAUD 115200              // Default Serial Monitor Speed
// #define SERIAL_BAUD 9600             // Alternative Monitor Speed
#define TERMINAL_WAIT_DELAY  2000


/*** @brief Signal Encoding method from Receiver. Can be PWM, PPM or SBUS
 */
// #define USE_PWM_RX                   // PWM Encoding from Receiver
#define USE_PPM_RX                      // PPM Encoding from Receiver
#define NUM_PPM_CHANNELS 8                  // 8 Channels for FS-iA10B PPM-Output Setting.
#define PPM_OFFSET 400
//#define USE_SBUS_RX                   // SBUS Communication from receiver


/*** @brief Select which IMU sensor and communication protocol to use
 */
#define USE_MPU9250_I2C                 // MPU9250 I2C
// #define USE_MPU9250_SPI              // MPU9250 SPI 
// #define USE_MPU9250_I2C_1            // 


/*** @brief Select Full Scale Range for Gyro and Accelerometer
 */
#define GYRO_250DPS                  //default full scale range
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS
#define ACCEL_2G                        //default full scale range
// #define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G


/*** @brief Select which Barometer sensor and communication protocol to use
 */
#define USE_BMP280_I2C  


/*** @brief Assign Indicators to Pins
 */
#define BUZZER_PIN 14
#define LED_BUILTIN 25
// #define BMP280_SS 4
// #define MPU_INTERRUPT_PIN 2
// #define MPU9250_SS 10
// #define LED_BLUE 25
// #define LED_GREEN 33



/*** @brief Actuator Configuration Defines
 */
#define MAX_SERVO_ANGLE 120
#define MIN_SERVO_ANGLE 60
#define MAX_PWM 2050
#define MIN_PWM 950


/*** @brief GPS Recever Pin Defines
 */
#define BN220_TX 29
#define BN220_RX 28

/*** @brief IMU I2C PINS
 */
#define I2C_BAUDRATE 1000000  // Nominal = 400,000Hz = 400KHz
#define MPU9250_SDA 4
#define MPU9250_SCL 5
#define CALIB_VERBOSE 0
#define IMU_VERBOSE_CONFIG 
// #define IMU_VERBOSE_READ 
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
// #define OUTPUT_READABLE_CUSTOM


/*** @brief Radio Receiver Channels
 */
#define CH1_PIN 0                      // Roll 
#define CH2_PIN 0                      // Pitch
#define CH3_PIN 0                      // Throttle
#define CH4_PIN 0                      // Rudder
#define CH5_PIN 0                      // Flaps
#define CH6_PIN 0
#define CH7_PIN 0
#define CH8_PIN 0
#define PPM_PIN 6

// Motor Configuration
#define MOTOR1_PIN 10
#define MOTOR2_PIN 11
#define MOTOR3_PIN 12
#define MOTOR4_PIN 13
#define AUX1_PIN 14
#define AUX2_PIN 13

#define MOTOR_INIT_PWM 1000
#define SERVO1_INIT_PWM 1575
#define SERVO2_INIT_PWM 1600

/*** @brief Servo Actuator Configuration
 */
#define SERVO1_PIN 0                    // Ailerons/    
#define SERVO2_PIN 0                    // Elevators/   
#define SERVO3_PIN 0                    // Rudder/      
#define SERVO4_PIN 0                    // Flaps/       
#define SERVO5_PIN 0
#define SERVO6_PIN 0
#define SERVO7_PIN 0

/** @brief Failsafe Configuration
 */
#define CHAN1_FS 1500
#define CHAN2_FS 1500
#define CHAN3_FS 1000
#define CHAN4_FS 1500
#define CHAN5_FS 1500
#define CHAN6_FS 1500
#define CHAN7_FS 1000
#define CHAN8_FS 1000



#if defined USE_MPU9250_I2C
#define GYRO_FS_SEL_250 MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500 MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000 MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000 MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2 MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4 MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8 MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16 MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
#define GYRO_FS_SEL_250 mpu9250.GYRO_RANGE_250DPS
#define GYRO_FS_SEL_500 mpu9250.GYRO_RANGE_500DPS
#define GYRO_FS_SEL_1000 mpu9250.GYRO_RANGE_1000DPS
#define GYRO_FS_SEL_2000 mpu9250.GYRO_RANGE_2000DPS
#define ACCEL_FS_SEL_2 mpu9250.ACCEL_RANGE_2G
#define ACCEL_FS_SEL_4 mpu9250.ACCEL_RANGE_4G
#define ACCEL_FS_SEL_8 mpu9250.ACCEL_RANGE_8G
#define ACCEL_FS_SEL_16 mpu9250.ACCEL_RANGE_16G
#endif

#if defined GYRO_250DPS
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE GYRO_FS_SEL_500
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE GYRO_FS_SEL_1000
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE GYRO_FS_SEL_2000
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE ACCEL_FS_SEL_4
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE ACCEL_FS_SEL_8
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE ACCEL_FS_SEL_16
#define ACCEL_SCALE_FACTOR 2048.0
#endif




