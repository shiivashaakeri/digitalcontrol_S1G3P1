# Digital Control Lab - Session 1
## Description
This project is the first session job of the Digital Control Lab, which aims to turn on and off a LED on the FPGA board, make it blinking, and use a push button as an external interrupt to control the LED blinking frequency. The project is implemented using the STM32CubeIDE, and the STM32F407ZGT6 microcontroller, and requires two packages: stm32cube_fw_f4_v1270 and stm32cube_fw_f4_v1271.

## How to Run the Project
### Hardware Requirements
- STM32F407ZGT6 microcontroller
- FPGA board
- LED
- Push button

### Software Requirements
- STM32CubeIDE
- stm32cube_fw_f4_v1270 and stm32cube_fw_f4_v1271 packages
### Setup

- Connect the STM32F407ZGT6 microcontroller to the FPGA board, and connect the LED to a GPIO pin.
- Connect the push button to another GPIO pin of the microcontroller.
- Install STM32CubeIDE and the required packages.
- Open the project in STM32CubeIDE.
- Configure the pin settings in the .ioc file for the LED and push button

### Running the Program
- Build and run the program on the microcontroller.
- The LED should turn on and off periodically.
- Press the push button to turn the LED on and off.
- Press the push button multiple times to increase the LED blinking frequency.
- After pressing the button four times, the frequency resets to the initial blinking frequency.

## Code Explanation
The main.c file contains the code for the project. The code initializes the microcontroller, configures the system clock, initializes the GPIO pins, and enters an infinite loop.

The frequency of the LED blinking is controlled by the variable "x". The delay time is calculated based on the value of "x". When the push button is pressed, the interrupt function HAL_GPIO_EXTI_Callback is called, which increments the value of "x". If the value of "x" reaches 5, it is reset to 1.
