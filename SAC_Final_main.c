#include <msp430.h>
#include <math.h>

volatile signed int tempAve = 0;
volatile signed int numSamplesTaken = 0;
volatile unsigned int bits = 0;
volatile signed int PWM_Duty = 0;
volatile signed int error = 0;
volatile signed int integral = 0;
volatile signed int derivative = 0;
volatile signed int previous_error = 0;
volatile unsigned int setpoint = 0;
volatile float temp = 0;
volatile float temp_0 = 0;
volatile float temp_1 = 0;
volatile float temp_2 = 0;
volatile float temp_3 = 0;
volatile float temp_4 = 0;
volatile unsigned char tempF = 0;
volatile unsigned int poscounter = 0;
volatile unsigned int negcounter = 0;


int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  //Set up Button for PWM Test
  P2REN |= BIT1;                            // Enables the resistor in P2.1
  P2OUT |= BIT1;                            // Sets the pull up resistor
  P2IE |= BIT1;                             // Enables button to cause interrupt
  P2IES |= BIT1;                            // Sets the edge for the interrupt to falling edge
  P2IFG &= ~BIT1;                           // P2.1 IFG cleared

  //Clock for the PWM
  P1SEL |= BIT2;                            //Select P1.2 to output the PWM
  P1DIR |= BIT2;
  TA0CCTL1 = OUTMOD_2;                      //Toggle / Reset output mode

  //Set the PWM clock parameters
  TA0CCR0 = 1000;
  TA0CCR1 = 1000;

  TA0CTL = TACLR;
  TA0CTL = TASSEL_2 + MC_1 + ID_0;          // Timer is set in Up mode, uses SMCLK and no division

  //Set up the data collection clock
  TB0CTL = TBSSEL_2 + MC_2 + ID_2;          // Timer is in continuous mode, uses SMCLK and divide by 4
  TB0CCTL0 = CCIE;                          // enable interrupt for the clock

  //ADC Initialize
  ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
  ADC12IE = 0x01;                           // Enable interrupt
  ADC12CTL0 |= ADC12ENC;
  ADC12MCTL0 = ADC12INCH_0;
  P6SEL |= 0x01;                            // P6.0 ADC option select
  P1DIR |= 0x01;                            // P1.0 output

  //Set up UART
  P3SEL |= BIT3 + BIT4;
  P4SEL |= BIT4 + BIT5;
  UCA1CTL1 = UCSWRST;                       // initialize USCI
  UCA1CTL1 |= UCSSEL_2;                     // set to use SMCLK (UCSSEL_2)
  UCA1BR0 = 104;                            // Baud Rate is 9600
  UCA1BR1 = 0;                              // set to 0
  UCA1MCTL = UCBRS_1;                       // set modulation pattern to high on bit 1 & 5
  UCA1CTL1 &= ~UCSWRST;                     // initialize USCI
  UCA1IE |= UCRXIE;                         // enable USCI_A1 RX interrupt

  while (1)
  {
    ADC12CTL0 |= ADC12SC;                   // Start sampling/conversion

    __bis_SR_register(GIE);                 // LPM0, ADC12_ISR will force exit
  }
}

//Overall Timer Interrupt Vector
#pragma vector = TIMER0_B0_VECTOR
__interrupt void DATA(void)
{
    //Set up the PID Equation
        error = setpoint - tempAve;            // Calculate error
        integral = integral + error;        // Calculate integral
        derivative = error - previous_error;// Calculate derivative
        if(error > 5)                      // If the temp is 15 degrees Fahrenheit colder then setpoint
        {
            TA0CCR1 = 1000;                 // Turn off the fan
            integral = 0;                   // Set integral to zero
        }

        else if(error < -5)                // If the temp is 15 degrees Fahrenheit warmer then the set point
        {
            TA0CCR1 = 0;                    // Fully turn on the fan
            integral = 0;                   // Set integral to zero
        }

        else                                // If temp is in range to be manipulated
        {
            PWM_Duty = 120 * error + 3 * integral + 10 * derivative;    // Our PID equations (Proportion, Integral,
            // Setting limits/correction factors
            if(PWM_Duty > 1000)
                TA0CCR1 = 1000;
            else if (PWM_Duty < 0)
                TA0CCR1 = 0;
            else
                TA0CCR1 = PWM_Duty;
        }
        if(PWM_Duty < 0)                    // if the PWM is less then zero
            negcounter = negcounter + 1;    // Add up a counter

        if(PWM_Duty > 1000)                 // if the pwm is more then 1500
            poscounter = poscounter + 1;    // Add up a counter

        // When the counters hit 60, reset the PWM, Integral and each counter, so values don't get too far out of range
        if (negcounter > 15)
        {
            integral = 0;
            PWM_Duty = 0;
            negcounter = 0;
        }

        if (poscounter > 15)
        {
            integral = 0;
            poscounter = 0;
        }

        //Set Previous error
        previous_error = error;

        //Store averaged temp in UART TxBUF
        numSamplesTaken = numSamplesTaken + 1;

        //make shift average filter
        if(numSamplesTaken == 0)
            temp_0 = temp;
        else if(numSamplesTaken == 1)
            temp_1 = temp;
        else if(numSamplesTaken == 2)
            temp_2 = temp;
        else if(numSamplesTaken == 3)
            temp_3 = temp;
        else if(numSamplesTaken == 4)
            temp_4 = temp;
        else
        {
            tempAve = (temp_0 + temp_1 + temp_2 + temp_3 + temp_4)/5;
            UCA1TXBUF = tempAve;
            numSamplesTaken = -1;
        }


        //UCA1TXBUF = (temp_0 + temp );

}

//UART Interrupt Vector
#pragma vector=USCI_A1_VECTOR
__interrupt void Temp_control(void)
{
  switch(__even_in_range(UCA1IV,4))
  {
      case 0:break;                             // Vector 0 - no interrupt
      case 2:                                   // Vector 2 - RXIFG, recieve the set point
          setpoint = UCA1RXBUF;
          break;
      case 4:break;                             // Vector 4 - TXIFG
      default: break;
  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) A++++DC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
            bits = ADC12MEM0;               // Store bit value into variable bits

            // Linearization of the temperature equations
            temp = bits / 12.8;

//            // Convert the temp to fahrenheit
//                temp = temp -273.15;
//                temp = temp * 1.8;
//                temp = temp + 32;
//                temp = round(temp);
//
//            //Convert to character to transmit over UART
//                tempF = (char)temp;

    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}


