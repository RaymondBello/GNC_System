

# RP2040 UAV Flight Controller

This is a brief outline of tasks and functions executed in the flight control firmware

## Flight controller modes

1. Uninitialized
   - First mode on boot
   - Sets up the mcu for initialization

2. Initialization
   - Sets up radio receiver
   - Sets up flight controller
   - Calibrates flight controller measurements

3. Active
   - Update flight controller time
   - Update flight controller orientation
   - Update flight controller desired state
   - Update flight controller PID
   - Mix controls based on configuarion and transitions
   - Command actuators with mixed output
   - Print debug/status messages/ handle serial
   - Encode debug data in Video/Audio Signals for transmission

4. Idle

5. Error

## Flight Control Loop

1. Update flight controller time variables used in PID and control mixing
   - previous_time - time stamp from last control loop
   - current_time - time stamp for current control loop
   - delta_t - time difference between current and previous timestamps
2. Update flight controller orientation 
   - get_imu_quart() - gets the current quarternion representations from the IMU
   - get_baro_alt() - get current barometric pressure reading, calculate the AGL altitude and lowpass filter the results 
3. Update flight controller desired state
   - Decide whether the desired state is set by a serial command or by radio transmission
     - Radio Transmission
       - Update each channel 1-8 based on the radio command received 
       - Low pass filter each channel
       - Roll, Pitch, Thro, Yaw are set by normalizing the each channel
     - Serial Command
       - Desired State can be set by a series of serial commands following the outlined format
       - {*command_identifier*} {*parameter*} | {*value*}
         - Eg. **set** thro_des **50** (sets the desired throttle to 50%)
         - Eg. **get** pitch_curr (returns the current pitch value)
       - Roll, Pitch, Thro, Yaw are normalized 

## FREERTOS Tasks

1. state_mgmt_task - manages current state and state transitions

