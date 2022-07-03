#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "mpu6050_dmp.h"

#include "standard.h"
#include "config.h"
#include "FC.h"

// Defines and variable declarations
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


// Class Instantiation
FC flight_ctrl;
MPU6050 mpu;


// Function definitions
void StateMgmtTask(void *param)
{
    while (1)
    {
        switch (flight_ctrl.mode)
        {
        case Uninitialized:
            vTaskDelay(1000);
            printf("[MODE] Uninitialized\n");
            flight_ctrl.setup_mcu();
            flight_ctrl.mode = Initialization;
            break;

        case Initialization:
            printf("[MODE] Initialization\n");
            // this->setup_radio_receiver();
            // BoolInt boolint = this->setup_flightcontroller();
            // boolint.flag ? (0) : (this->mode = Mode::Error);
            // BoolInt boolint1 = this->calibrate_flightcontroller();
            // boolint1.flag ? (this->mode = Mode::Active) : (this->mode = Mode::Error);
            flight_ctrl.mode = Active;
            printf("[INFO] Boot sequence complete.\n");
            vTaskDelay(1000);
            break;

        case Active:
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

            vTaskDelay(1000);
            break;

        default:
            break;
        }
    }
}

void StatusLEDTask(void *param)
{
    while (1)
    {
        printf("%d\n", flight_ctrl.mode);
        vTaskDelay(1000);

        if (flight_ctrl.mode == Active)
        {
            gpio_put(LED_BUILTIN, GPIO_ON);
            vTaskDelay(1000);
            gpio_put(LED_BUILTIN, GPIO_OFF);
            vTaskDelay(500);
            printf("Blinked\n");
        }
        
    }
}

void dmpDataReady() {
    mpuInterrupt = true;
}


int main()
{
    stdio_init_all();
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(MPU9250_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MPU9250_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(MPU9250_SDA);
    gpio_pull_up(MPU9250_SCL);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(MPU9250_SDA, MPU9250_SCL, GPIO_FUNC_I2C));

    sleep_ms(TERMINAL_WAIT_DELAY);

    // ================================================================
    // ===                      INITIAL SETUP                       ===
    // ================================================================

    // Task Handles
    TaskHandle_t state_mgmt_task = NULL;
    TaskHandle_t status_indicator_task = NULL;

    // uint32_t state_mgmt_task_status = xTaskCreate(
    //     StateMgmtTask,
    //     "State Management Task",
    //     1024,
    //     NULL,
    //     tskIDLE_PRIORITY,
    //     &state_mgmt_task);

    // uint32_t status = xTaskCreate(
    //     StatusLEDTask,
    //     "Built-in LED Status Indicator",
    //     1024,
    //     NULL,
    //     tskIDLE_PRIORITY,
    //     &status_indicator_task);

    // vTaskStartScheduler();

    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) 
    {
        mpu.setDMPEnabled(true);                // turn on the DMP, now that it's ready
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;                        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        packetSize = mpu.dmpGetFIFOPacketSize();      // get expected DMP packet size for later comparison
    } 
    else 
    {                                          // ERROR!        1 = initial memory load failed         2 = DMP configuration updates failed        (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)", devStatus);
        sleep_ms(2000);
    }
    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;

    for (;;)
    {
        //should never get here

        if (!dmpReady);                    // if programming failed, don't try to do anything
        mpuInterrupt = true;
        fifoCount = mpu.getFIFOCount();                                           // get current FIFO count
        if ((mpuIntStatus & 0x10) || fifoCount == 1024)                           // check for overflow (this should never happen unless our code is too inefficient)
        {
            mpu.resetFIFO();                                                      // reset so we can continue cleanly
            printf("FIFO overflow!");
        } 
        else if (mpuIntStatus & 0x01)                                             // otherwise, check for DMP data ready interrupt (this should happen frequently)
        {    
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();        // wait for correct available data length, should be a VERY short wait
            mpu.getFIFOBytes(fifoBuffer, packetSize);                             // read a packet from FIFO
            fifoCount -= packetSize;                                              // track FIFO count here in case there is > 1 packet available
            #ifdef OUTPUT_READABLE_YAWPITCHROLL                                               // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                yaw = ypr[0] * 180 / PI;
                pitch = ypr[1] * 180 / PI;
                roll = ypr[2] * 180 / PI;
                // printf("ypr: %f, %f, %f\n", yaw, pitch, roll);
                printf("%f, %f, %f\n", yaw, pitch, roll);

            #endif
            #ifdef OUTPUT_READABLE_REALACCEL
                // display real acceleration, adjusted to remove gravity
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                printf("areal: %d,\t %d,\t %d\n", aaReal.x, aaReal.y, aaReal.z);
            #endif
            #ifdef OUTPUT_READABLE_WORLDACCEL
                // display initial world-frame acceleration, adjusted to remove gravity
                // and rotated based on known orientation from quaternion
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                printf("aworld: %d,\t %d,\t %d\n", aaWorld.x, aaWorld.y, aaWorld.z);
            #endif
            #ifdef OUTPUT_READABLE_CUSTOM
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                printf("W: %f\t X: %f\t Y: %f\t Z: %f\n", q.w, q.x, q.y, q.z);
            #endif
        }

    }
}