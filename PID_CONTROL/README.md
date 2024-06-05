* This example Implements a Simple PID CONTROL Using ADC1 And PWM
* Also Sends Some Debug Data To the MAX7219 8x8 Led Matrix	 
* The PID Output is Calculated  every 100 ms.
* The setpoint gets incremented by 10 units every 7 secs.


* Tested With An RC Filter. Needs Some Tuning Though
* Tested With Two Pots For Disturbing The process    		

* PA0 -- R1K -- 100uF -- R1K -- PA1(PWM) 
* ---------------GND	

