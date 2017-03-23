# Dynamic Performance Scaling in FreeRTOS
### UCLA EEM202B Course Project
**Authors**: Gautham Kumar Reddy \<<gautham03@ucla.edu>\> and Vishesh Dokania \<<vdokania@ucla.edu>\><br />
**Advisor**: Prof. Mani Srivastava<br />
Winter 2017

### Website
More details and implementation approach can be found at<br />
https://sites.google.com/g.ucla.edu/freertospowermangement

### Fork
Project forked from original STM32 FreeRTOS Port : [CORTEX_STM32L152_Discovery_IAR](https://github.com/cjlano/freertos/tree/master/FreeRTOS/Demo/CORTEX_STM32L152_Discovery_IAR)<br />
See [here](http://www.freertos.org/Free-RTOS-for-Cortex-M3-STM32-STM32L152-EVAL.html) for more details.

### Directory Structure
| Directory                              | Description                                     |
| -------------------------------------- | ----------------------------------------------- |
| `Source/`                              | FreeRTOS kernel source code                     |
| `Demo/CORTEX_STM32L152_Discovery_IAR/` | Application and Drivers for the specific board  |
| `Demo/Common/`                         | Common drivers/methods for all FreeRTOS ports   |
| `Power Measurement/`                   | Power consumption data for different modes      |                    

### Usage :

1. Open the project in IAR Embedded Workbench for Cortex-M.
2. Define the required frequency governor policy in FreeRTOSConfig.h
3. Specify the min. and max. CPU usage range for OnDemand CPU governor policy in FreeRTOSConfig.h
4. CPU usage can be obtained through the function vCpuStats() function
