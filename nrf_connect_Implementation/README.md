# Info

This algorithm is designed to detect irregularities in walking patterns by analyzing accelerometer data from Caretronic wristbands. It relies on the Zephyr operating system and the capabilities of the LSM6DSL sensor.

Key features of this algorithm include efficient data processing, regular operation cycles, and the application of advanced techniques such as the Mahalanobis distance calculation to accurately identify unusual walking patterns. This algorithm represents a seamless integration of hardware and software, offering a valuable tool for precise and personalized tracking and analysis of human movement patterns.

# Algorithm files

**board** folder contains the hardware description of caretronic device. The description is nedded for the nrf sdk to work.
All the program code is in **lsm6dsl** folder


**src/main.c** 
Contains the main program initialzization functions
    - Init Bluetooth
    - Init i2c

**remote_service/remote.c**
Contains the bluetooth connection handling

**features/features.c**
Functions for calculating the features from raw data

**features/mahalanobis_calculation.c**
Calculating the mahalanobis distance from two matrices of features

**lsm6dsl_custom/lsm6dsl_reg.c** 
Interface with LSM6DSL chip

**lsm6dsl_custom/sensor_api.c**
Main logic for the program code
    - Setting registers
    - Initializing sensors
    - Collecting data
    - Handling interrupts
    - ...

**lsm6dsl_custom/gpio_interrupt_setup.c**
Setting up the interrrupts at the beginning


# Algorithm Execution

1. **Operating System**: The algorithm is powered by the Zephyr operating system, which supports the use of threads (or tasks). Using threads enables the separation of data collection, data processing, machine learning operations, and BLE communication, preventing interference between these processes.

2. **Sensor**: The LSM6DSL sensor is utilized for various functions. Some operations are carried out directly within the sensor, reducing the need for direct communication with the leading SoC (system-on-chip) nrf52840. Functions offloaded to LSM6DSL include:
   - Data collection into a FIFO buffer
   - Bandpass filtering of accelerometer output with frequency limits of 0.6 Hz and 13 Hz
   - Step counting and motion detection

3. **Periodic Execution**: For individuals wearing the wristband while walking, the entire program runs periodically every 10 minutes. During the first 10 minutes, 118 reference samples are stored. After the subsequent 10 minutes (new 118 samples), the Mahalanobis distance is calculated, and classification using a threshold is performed.

4. **Operation Diagram**: The operation of the code:

   - When the code starts, it initializes BLE technology. Once BLE is set up, the device becomes visible to smartphones. After establishing a connection between the smartphone and the device, the LSM6DSL sensor is set to power-saving mode. This mode uses a sampling frequency of 12.5 Hz and a measurement range of ±2 g. Angular acceleration measurement is disabled during the device's entire operation.
   - Upon detecting significant motion, the IMU sensor switches to step counting mode and begins storing data in an internal buffer (FIFO). The sampling rate is set to 52 Hz at this point.
   - If at least three steps are detected within a five-second window, a new interrupt is triggered. The main processor reads 256 samples of accelerometer data for each axis (x, y, and z) upon interrupt. This process of buffering and data reading continues periodically as long as steps are detected in each subsequent 5-second window.
   - In each cycle (5-second window) where steps are detected, data processing and feature calculation are initiated. The first 118 samples are saved in memory as reference values for calculating the threshold. For each subsequent group of 118 new samples measured by the system, the deviation of these samples from the reference samples is calculated. (The number 118 is chosen to balance the ratio between the number of features and the number of samples, avoiding excessive computational complexity in calculating the Mahalanobis distance.)
   - The program then transmits the result over the BLE connection to the connected smartphone.
   - If no steps are detected by the accelerometer within five seconds, the LSM6DSL switches back to power-saving mode and waits for motion events. The entire process is then repeated upon detecting movement.
   - In the event of a BLE connection interruption, the device resets and waits for reconnection with the smartphone.

This algorithm is designed to periodically detect deviations in walking patterns using the Zephyr operating system and the LSM6DSL sensor. It leverages BLE for data communication with a connected smartphone.
