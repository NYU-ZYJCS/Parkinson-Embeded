**Overview**

Our Tremor Challenge project focuses on creating a wearable Parkinsonian tremor and its intensity detector using the STM32F429 Discovery board. The device utilizes the onboard gyroscope to measure angular velocities, aiding in the detection and assessment of the intensity of resting tremors, characteristic of Parkinson's disease.

**Objective**

To develop a reliable tremor detection system that measures the gyroscope's real-time rotation to spot the occurrence and intensity of a resting tremor.

**Requirements**

- STM32F429 Discovery Board with built-in gyroscope
- Power supply/USB power bank
- PlatformIO IDE

**Setup and Configuration**

- Begin with an SPI object instance to communicate with the gyroscope.
- Configure the gyroscope control registers (CTRL_REG1 and CTRL_REG4).
- Implement the main loop to read XYZ values from the gyroscope.
- Apply Kalman filtering to the gyroscope data.
- Use the provided utilities to analyze the time-domain features of the filtered data.

**Output and Visualization**

STM32F429 Discovery board displays whether a tremor has been detected or not, and if detected, the display updates to show the tremor’s intensity measured over the 10-second period. This allows users to immediately detect the status of tremor detection and its severity.

![Horizontal_tremor](https://github.com/NYU-ZYJCS/Parkinson-Embeded/blob/main/img/horizontal_tremor.png)

![Vertical_tremor](https://github.com/NYU-ZYJCS/Parkinson-Embeded/blob/main/img/vertical_tremor.png)
