GitHub Repository for 202B project : Dynamic Performance Scaling in FreeRtos

FreeRtosPort : CORTEX_STM32L152_Discovery_IAR

Source/ contains FreeRtos source 

Demo/CORTEX_STM32L152_Discovery_IAR/ contains the application for the specific board

Power Measurement contains the results and power consumption for different modes

Usage :

1.Open the project from IAR Embedded Workbench.

2.Define the required frequency governor policy in FreeRtosconfig.h

3.Specify the min and max CPU usage range for OnDemand CPU governor policy in FreeRtosConfig.h .

4.CPU usage can be obtained through the function vCpuStats() function.

More details and implementation approach can be found at https://sites.google.com/g.ucla.edu/freertospowermangement



