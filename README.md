GitHub Repository for 202B project : Dynamic Performance Scaling in FreeRtos

Source contains FreeRtos source 

Demo/Stm32 contains the application for the specific board

Power Measurement contains the results and power consumption for different modes

Usage :

1.Define the required frequency governor policy in FreeRtosconfig.h

2.Specify the min and max CPU usage range for OnDemand CPU governor policy.

3. CPU usage can be obtained through the function vCpuStats() function.
