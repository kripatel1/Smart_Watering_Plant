# Smart_Watering_Plant
Theory of Operation
EEPROM (Data Storage)
The smart water pot has the capability to log data such as moisture, water, and light level along with a timestamp for each entry. Since there are only 16 words each 4 bytes wide, I broke up 32 bits into different segments to encode the values. I designated moisture, water, and light levels each with 6 bits which leaves 14 bits for a time stamp. Since a percentage ranges from 0-100, this requires at least 7 bits, however, we are able to multiple the percentage by a value of 63 which allows us to encode 6-bit values. This means that creates loss of precision when going back to look at the data. This loss of error for the 3 values is +- 3 percent. Since a similar algorithm is implemented for the time stamp, there is a loss of precision of +- 5seconds. The required number of bits for the number of seconds in a day needs 17 bits but we can encode the value with the remaining 14 bits. Diagram below shows how the values are encoded.  
These values are stored by shifting moisture, water, light to right by 26, 20, 14, respectively. These values are extracted by applying a mask and shift the values back to the left when history command is made.
During initialization of the pot, a funciton is called to set the offset for EEPROM. It checks if there is previous data in EEPROM by looking for non-zero data. The block offset pointer is a global pointer set to the next available word (first zeroed entry). If all words are filled in the block then offset is set to 0.
An erase command can be called to clear EEPROM by writing 0’s to all the words in the block. Only one block is used for the duration of the project, block 2.
A record command can be used to manual log a data entry.
A history command can be used to recall all valid entries.
During initialization, the RTC module is set for keeping time. However, during boot up, timer is set to 0 and needs to run time command to set correct time. The green and red LED’s are also initialized to indicate running status. Green LED indicate normal sensor checking. RED LED shows that a command is being processed. The ADC, comparators, 3 timers, UART, and EEPROM are also initialized. The integrator is turned off by driving a high to the transistor to allow current to from collector to emitter rather than through the capacitor. The reason I did this because if the transistor is off and allows the capacitor to charge, an interrupt will keep firing preventing the program from doing any meaningful work.
ADC is used for light, moisture, and battery voltage measurement. Sequence Sampler 3 is used for the analog input. Based on the voltage drop a raw 12  bit value is generated. Since each measurement was on a different analog pin, we change channels using the setAdc0Ss3Mux(uint8_t ANx) based on the required pin. The data was sampled multiple times before being returned to the main program by the internal hardware averager.  
The comparator uses an interrupt that is generated when the capacitor is charged. Timer is set to 0. DEINT pin is set to low (turn off transistor), then when it is charged, an interrupt is fired by the comparator which turns the timer off, records the time, and DEINT pin is set to HIGH (turn on transistor). 
Commands are interpreted by parsing the strings in the buffer inputed by the user. They are then compared with the hardcoded command to see if they are valid.
There are three timers that are initailized. The first timer was used to calculated the time it takes for the water resevoir which is a capacitor to charge. The second timer is used to set the tone that will be used and changed to create a melody. The third timer is used to make a data entry in the EEPROM every 2 mins. We can change how often we log data by changing the value in TIMER4_TAILE_R. Since this is board is clocked at 40 MHz, we can multiple by the number seconds we want to log data. Example: For every 10, we can take (10min * 60 sec) * 40E6 and place in the register.
The start time for watering and end time for watering are stored in a gloabal variable. Default values on boot up is zero. Minimum water level, light intensity, battery voltage, and moisture are also stored in a global variable and are set to 0 when  boot except for battery voltage which is set to 50 percent.
PIN	FUNCTION
3.3v	voltage
GND	GND
PB1	DEINT (TRANSITOR BASE CONNECTION)
PC7	COMPARATOR (CAP CHARGED?)
PE1 (AN2)	LIGHT SENSOR
PE2 (AN1) 	MOISTURE SENSOR
PE3 (AN0)	BATTERY VOLTAGE
PB2 	MOTOR CONTROL (ON|OFF)
PB7	SPEAKER
	
	
  VALID COMMANDS
    1.  [time H M]  -> SET TIME [24 HR FORMAT] USING COMMAND 
    2.  [water H1 M1 H2 M2] -> SET WHEN TO WATER USING COMMAND 
    3.  [level val] -> SET MIN MOISTURE LEVEL USING COMMAND 
    4.  [SET_MIN_BATTERY_VOLTAGE val] -> SET MIN BATTERY LEVEL USING COMMAND 
    5.  [alert val] -> SET light Intensity(TIME OF DAY) FOR ALERTS USING COMMAND 
    6.  [status]    -> shows the current readings vs. Required values 
    7.  [light]     -> gets current light level 
    8.  [battery]   -> gets current voltage of battery 
    9.  [moisture]  -> gets current moisture level 
    10. [pump ON]   -> Turns water pump ON 
    11. [pump OFF]  -> Turns water pump OFF 
    12. [getCurrentTime]  -> get Current Time 
    13. [history]   -> show the past entries 
    14. [record]    -> manually log an entry 
    15. [erase]     -> erase all entries 

