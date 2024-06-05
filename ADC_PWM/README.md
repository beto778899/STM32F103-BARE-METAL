Reads the ADC1 Value Every 250ms.

The On Board LED Blinks Proportionally To the ADC1 Value
The PWM Duty Changes    Proportionally To the ADC1 Value

To make the values compatible with each other
There is a Function That Maps A Value to a Different Value.

Connect a 10K Potenciometer or similar between VCC and GND. The middle pin of the Pot goes to PA0.
Read the PWM value with a Multimiter. Or just put a LED and a 1KR between PA1 and GND.

As you move the Potenciometer the LED Blinks Proportionally to the Potenciometer Position.
Also the LED on PA1 Fades Proportionally to the Potenciometer Position.

ADC1 = PA0		PWM  = PA1		LED = PC13
