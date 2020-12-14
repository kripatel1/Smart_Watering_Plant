

/**
 * NAME: KRISHNA PATEL
 * 
 */


//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//  Configured to 115,200 baud, 8N1

/* Additonal Pins
* 3.3v  voltage
* GND GND
* PB1 DEINT (TRANSITOR BASE CONNECTION)
* PC7 COMPARATOR (CAP CHARGED?)
* PE1 (AN2)   LIGHT SENSOR
* PE2 (AN1)   MOISTURE SENSOR
* PE3 (AN0)   BATTERY VOLTAGE
* PB2     MOTOR CONTROL (ON|OFF)
* PB7 SPEAKER

 *
 *
 */

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"



#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;


// Bitband aliases
#define RED_LED         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

#define DEINT           (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4)))
#define MOTOR_CONTROL   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 2*4)))
#define SPEAKER_CONTROL (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 7*4)))

// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK 2
// PORT B1
#define DEINT_MASK 2 // PIN B1
#define COMPARATOR_MASK 0x80 // PIN C7


#define LIGHT_SENSOR_MASK      0x02 // PIN E1
#define MOISTURE_SENSOR_MASK   0x02 // PIN E2
#define MEASURING_VOLTAGE_MASK 0x08 // PIN E3

#define MOTOR_CONTROL_MASK     0x04 // PIN B2
#define SPEAKER_MASK           0x80 // PIN B7

#define seconds_in_a_day    86400
#define MAX_WATER_VOLUME    240

#define MAX_VOLTS           6.4

// Block Number for EEPROM to RW
#define BLOCK_NUMBER    2
#define MAX_OFFSET      16

uint32_t timerValue = 0;
uint16_t interruptRunning = 0;
uint32_t start_seconds = 0;
uint32_t end_seconds = 0;
uint32_t minWaterLevel_ML = 60;
uint32_t moistureLevel = 0;
uint32_t lightThreashold = 0;
float minBatteryLevel = 3.2; //perentage

uint8_t offsetIdx_global=0;
uint8_t FORCE_LOG_ENTRY =0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void wait1ms()                              // <---
{
    __asm("             B NEXT");           // runs 2 clocks
    __asm("values       .field 7");         // 0 clocks ----> Change N based on clock/values
    __asm("NEXT:        LDR R1, values");   // 1 clock
    __asm("DECREASE:    SUB R1, #1");       // N times
    __asm("             CBZ R1, DONE");     // N times + 2 clocks for the last taking B
    __asm("             NOP");              // N TIMES
    __asm("             B DECREASE");       // 2 CLOCKS PER BRANCH = 2*N TIMES
    __asm("DONE:        NOP");              // 1 CLOCK
}

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;


    // Enable clocks // PORT F
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs
}
void initEEPROM_Module()
{

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    SYSCTL_RCGCEEPROM_R = SYSCTL_RCGCEEPROM_R0; // Turn on EEPROM Run Mode Clock Gating
    _delay_cycles(7); // Must wait 6 cycles
    while(EEPROM_EEDONE_R  & EEPROM_EEDONE_WORKING); // DONE working 0x1;
    while((EEPROM_EESUPP_R & 0x8) || (EEPROM_EESUPP_R & 0x4));

    SYSCTL_SREEPROM_R |= 0x1; // Reset EEPROM
    SYSCTL_SREEPROM_R &= ~SYSCTL_SREEPROM_R0; // Complete Reset by clearing bit
    _delay_cycles(7); // Must wait 6 cycles
    while(EEPROM_EEDONE_R  & EEPROM_EEDONE_WORKING); // DONE working 0x1;
    while((EEPROM_EESUPP_R & 0x8) || (EEPROM_EESUPP_R & 0x4));

}
void initRTC_Module()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // TURN ON Hibernation Module
    SYSCTL_RCGCHIB_R |= SYSCTL_RCGCHIB_R0; // Hibernation Module 0x01;
    _delay_cycles(3);



    HIB_IM_R |= 0x10; // ENABLE WC INTERRUPT
    HIB_CTL_R = 0x0; // ENABLE OSCILLATOR INPUT |HIB_CTL_CLK32EN
    while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_CTL_R = 0x41; // ENABLE OSCILLATOR INPUT |HIB_CTL_CLK32EN
    while(!(HIB_CTL_R & HIB_CTL_WRC)); // WAIT FOR WRC TO BE SET BEFORE MOVING FOWWARD
//    HIB_CTL_R |= 0x01;

    HIB_RTCLD_R = 1;
    HIB_IC_R  |= 0x10; // Clear WC interrupt;
}
void hibernateISR()
{
//    GREEN_LED ^= 1;
    waitMicrosecond(10000);
}
void initHW_Comparator0()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
   SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

   // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
   // Note UART on port A must use APB
   SYSCTL_GPIOHBCTL_R = 0;

   // TURN ON RCGCACMP - FOR CO(NEGATIVE INPUT)
   SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2; // PORT C to use CO-
   SYSCTL_RCGCACMP_R = 1; // TURN ON ANALOG COMPARATOR
   _delay_cycles(3);


   GPIO_PORTC_DIR_R  &= ~(COMPARATOR_MASK);      // C7 must be INPUT
   GPIO_PORTC_DEN_R  &= ~(COMPARATOR_MASK);      // TURN OFF DIGITAL FUNCTIONS

   GPIO_PORTC_AMSEL_R |= COMPARATOR_MASK;        // ENABLE ANALOG FUNCTIONS
   GPIO_PORTC_AFSEL_R |= COMPARATOR_MASK;        // TURN ON ALTERNATE FUNCTION
   GPIO_PORTC_PCTL_R  &= ~(GPIO_PCTL_PC7_M);     // CLEAR ALTERNATE FUNCTION SIGNAL FOR C7

   COMP_ACREFCTL_R    |= COMP_ACREFCTL_EN;       // REGISTER LADDER ENABLE bit 9-> 0x00000200
   COMP_ACREFCTL_R    &= ~(COMP_ACREFCTL_RNG);   // REGISTER HIGH RANGE SO clear                  bit 8 | MASK -> 0x00000100
   COMP_ACREFCTL_R    |= COMP_ACREFCTL_VREF_M;   // VREF set to 2.469 -> 0x0000000F;


   COMP_ACCTL0_R   &= ~(COMP_ACCTL0_TOEN);          // TURN OFF ADC TRIGGER while set up          bit 11| MASK -> 0x00000800
   COMP_ACCTL0_R   |= COMP_ACCTL0_CINV;            // VIN > VREF = HIGH                          bit 1 | MASK -> 0x00000002
   COMP_ACCTL0_R   |= COMP_ACCTL0_ASRCP_REF;       // USING INTERNAL VOLTAGE REF             bits 10:11| MASK -> 0x00000400
   COMP_ACCTL0_R   |= COMP_ACCTL0_ISLVAL;          // INTERRUPT TRIGGERD IF OUTPUT IS HIGH       bit 4 | MASK -> 0x00000010
   COMP_ACCTL0_R   &= ~(COMP_ACCTL0_ISEN_M);       // SETTING TO LEVEL SENSE                   bits 3:2| MASK -> 0x0000000C
   COMP_ACCTL0_R   |= COMP_ACCTL0_TOEN;             // ENABLE ADC TRIGGER
   _delay_cycles(401); // delay 10 us;




  // SET UP INTERRUPT ------ These lines are causing a fault to occur
     COMP_ACMIS_R   |= COMP_ACMIS_IN0;               // Reset INTERRUPT C0
     NVIC_EN0_R     |= 0x02000000;                   // ANALOG COMPARATOR0 INTERRUPT VECTOR NUMBER = 25
     _delay_cycles(500);
     COMP_ACINTEN_R |= COMP_ACINTEN_IN0;             // ENABLE C0 INTERRUPT                       bit 0 | MASK -> 0x00000001




//-----------------THESE LINES ARE DIFFERNT VARIATIONS OF THE INTERRUPT CODE BUT STILL FAULTS

   // SET UP INTERRUPT ------ These lines are causing a fault to occur
   //   COMP_ACRIS_R   &= ~(COMP_ACRIS_IN0);            // Reset INTERRUPT C0
   //   COMP_ACRIS_R   &= ~(COMP_ACRIS_IN1);            // Reset Intterrupt c1
   //   COMP_ACINTEN_R |= COMP_ACINTEN_IN0;             // ENABLE C0 INTERRUPT                       bit 0 | MASK -> 0x00000001
   //   COMP_ACMIS_R   |= COMP_ACMIS_IN0;               // Again trying to clear interrupt
   //   NVIC_EN0_R     |= 0x02000000;                   // ANALOG COMPARATOR0 INTERRUPT VECTOR NUMBER = 25
   //   _delay_cycles(500);
}
void initHW_DEINT()
{
     // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks // PORT B-PIN1
    // TURN ON RCGCACMP - FOR CO(NEGATIVE INPUT)
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2; // PORT C to use CO-
    SYSCTL_RCGCACMP_R = 1; // TURN ON ANALOG COMPARATOR
    _delay_cycles(3);

    GPIO_PORTB_DIR_R  |= DEINT_MASK; // OUTPUT ->1
    GPIO_PORTB_DR2R_R |= DEINT_MASK; // 2ma rive
    GPIO_PORTB_DEN_R  |= DEINT_MASK;
}

void initTimer1HW()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;


   TIMER1_CTL_R  &= ~TIMER_CTL_TAEN;                  // turn-off timer before reconfiguring --TAEN=0x01
   TIMER1_CFG_R   = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B) -->32_BIT_TIMER= 0x00
   TIMER1_TAMR_R |= TIMER_TAMR_TACDIR;                // COUNTS UP starts at 0       bit4 --> 0x10

}

void lightSensor_moisture_voltage_Init()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure AIN0, AIN1, AIN2 as an analog input
    GPIO_PORTE_AFSEL_R |= LIGHT_SENSOR_MASK | MOISTURE_SENSOR_MASK | MEASURING_VOLTAGE_MASK;                 // select alternative functions for AN0-2 PE1-PE3
    GPIO_PORTE_DEN_R &= ~(LIGHT_SENSOR_MASK | MOISTURE_SENSOR_MASK | MEASURING_VOLTAGE_MASK);                // turn off digital operation on AN0-2 PE1-PE3
    GPIO_PORTE_AMSEL_R |= LIGHT_SENSOR_MASK | MOISTURE_SENSOR_MASK | MEASURING_VOLTAGE_MASK;                 // turn on analog operation on AN0-2 PE1-PE3
    GPIO_PORTE_PCTL_R  &= ~(GPIO_PCTL_PC1_M | GPIO_PCTL_PC2_M | GPIO_PCTL_PC3_M);                            // Clear digital functions PE1-PE3



    // Initializing ADC0
    // Enable clocks
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    _delay_cycles(16);

    // Configure ADC
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_PC_R = ADC_PC_SR_1M;                        // select 1Msps rate
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}
// Sets up motor Control PIN PB2
void motor_Init()
{
// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks // PORT B-PIN1
    // TURN ON RCGCACMP - FOR CO(NEGATIVE INPUT)
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    _delay_cycles(3);

    GPIO_PORTB_DIR_R  |= MOTOR_CONTROL_MASK; // OUTPUT ->1
    GPIO_PORTB_DR2R_R |= MOTOR_CONTROL_MASK; // 2ma rive
    GPIO_PORTB_DEN_R  |= MOTOR_CONTROL_MASK;

}
// Sets up Speaker Control PIN PB3
void speaker_Init()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks // PORT B-PIN1
    // TURN ON RCGCACMP - FOR CO(NEGATIVE INPUT)
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    _delay_cycles(3);

    GPIO_PORTB_DIR_R  |= SPEAKER_MASK; // OUTPUT ->1
    GPIO_PORTB_DR2R_R |= SPEAKER_MASK; // 2ma rive
    GPIO_PORTB_DEN_R  |= SPEAKER_MASK;

}
void initTimer2Hw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
   SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

   // Enable clocks
   SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;      // MASK IS  0x04
   _delay_cycles(3);

   TIMER2_CTL_R   &= ~0x01;
   TIMER2_CFG_R   = TIMER_CFG_32_BIT_TIMER;
   TIMER2_TAMR_R  = TIMER_TAMR_TAMR_PERIOD;          // 0x02
   TIMER2_TAILR_R = 0;                           // 1 SECOND INTERRUPT -> CLOCK*N
   TIMER2_IMR_R   = TIMER_IMR_TATOIM;                // 0x01 turn on intterupts
   NVIC_EN0_R    |= 1<< 23;                           // interrupt 70 bit 7
//   TIMER2_CTL_R  |= TIMER_CTL_TAEN;                  // turn-on timer 0x01

}
void timer2ISR()
{
//    static int melodyIdx =0;
//   TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
//   TIMER2_TAILR_R = 45454;
   SPEAKER_CONTROL ^= 1;
   TIMER2_ICR_R = TIMER_ICR_TATOCINT;

//   waitMicrosecond(1000000);
//   TIMER2_CTL_R  |= TIMER_CTL_TAEN;                  // turn-on timer 0x01          // turn-on timer 0x01

}
void initTimer4Hw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
   SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

   // Enable clocks
   SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R4;      // MASK IS  0x10
   _delay_cycles(3);

   TIMER4_CTL_R &= ~0x01;
   TIMER4_CFG_R = TIMER_CFG_32_BIT_TIMER;
   TIMER4_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // 0x02
   TIMER4_TAILR_R = 40000000*30;                    // 1 SECOND INTERRUPT -> CLOCK*N
   TIMER4_IMR_R  = TIMER_IMR_TATOIM;                // 0x01 turn on intterupts
   NVIC_EN2_R    |= 0x40;                           // interrupt 70 bit 7
   TIMER4_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer 0x01

}

void playBatteryLowAlert()
{
    TIMER2_TAILR_R = (40e6)/(2*(466)); // Its
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(100000);
    RED_LED ^= 1;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(294)); // D been
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(200000);
    RED_LED ^= 1;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(349)); // F a
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(50000);
    RED_LED ^= 1;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(392)); // G long
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(1000000);
    RED_LED ^= 1;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(349)); // F
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(2000);
    RED_LED ^= 1;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(392)); //G
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(500000);
    RED_LED ^= 1;
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
}
void playWaterLowAlert()
{
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(262)); // G long
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(20000);
    RED_LED ^= 1;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(523)); // F
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(200000);
    RED_LED ^= 1;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(262)); //G
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(100000);
    RED_LED ^= 1;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(523)); // G long
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(200000);
    RED_LED ^= 1;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(262)); // F
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(100000);
    RED_LED ^= 1;

    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer
    TIMER2_TAILR_R = (40e6)/(2*(523)); //G
    TIMER2_CTL_R  |= TIMER_CTL_TAEN;
    waitMicrosecond(200000);
    RED_LED ^= 1;
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; // turn off timer

}

// Set SS3 input sample average count
void setAdc0Ss3Log2AverageCount(uint8_t log2AverageCount)
{
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_SAC_R = log2AverageCount;                   // sample HW averaging
    if (log2AverageCount == 0)
        ADC0_CTL_R &= ~ADC_CTL_DITHER;               // turn-off dithering if no averaging
    else
        ADC0_CTL_R |= ADC_CTL_DITHER;                // turn-on dithering if averaging
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}

// Set SS3 analog input
void setAdc0Ss3Mux(uint8_t input)
{
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_SSMUX3_R = input;                           // Set analog input for single sample
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}

// Request and read one sample from SS3
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    while (ADC0_SSFSTAT3_R & ADC_SSFSTAT3_EMPTY);
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}


// Timer1 interrupu Service Routine for measuring liquid
void comparatorIsr()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // IMMEDIATELY TURN OFF TIMER
    timerValue = TIMER1_TAV_R;
    GREEN_LED = 1;
    RED_LED = 0;
    DEINT =1;                                       // TURN OFF DEINT otherwise interrupt will keep triggering
    COMP_ACMIS_R   |= COMP_ACMIS_IN0;               // writing a 1 to clear the interrupt
    interruptRunning =0;                            // to let the while loop end ; indirect polling but using hardware
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
void parseFields(USER_DATA* data)
{
    int count =0;
    data->fieldCount =0;
    int i =0;
    int startedField = 0;

    while(1)
    {
        if(i > MAX_CHARS){return;}
        char currentChar = data->buffer[i];
        //processing field
        if(!startedField)
        {   // recording the startIDX of the field
            // recording the type
            if( ((currentChar >= 'A') && (currentChar <='Z')) ||
                ((currentChar >= 'a') && (currentChar <='z')))
            {
                data->fieldPosition[count] = i;
                data->fieldType[count] = 'A';
                data->fieldCount++;
                startedField = 1;
            }
            else if ((( currentChar >= '0') && (currentChar <= '9')) ||
                    (currentChar == '.') || (currentChar == '-') || (currentChar == ','))
            {
                data->fieldPosition[count] = i;
                data->fieldType[count] = 'N';
                data->fieldCount++;
                startedField = 1;
            }
            else
            {
                data->buffer[i]= NULL; // placing Null everywhere else
            }
        }
        else
        {
            if( data->fieldType[count] == 'A')
            {
                if( ((currentChar >= 'A') && (currentChar <='Z')) ||
                    ((currentChar >= 'a') && (currentChar <='z')))
                {
                    // verify it is the same type
                }
                else
                {
                    startedField =0; // parsing has ended because types changed
                    count++;

                    data->buffer[i]= NULL;
                    if(count == MAX_FIELDS)
                    {
                        return;
                    }
                }
            }
            else if(data->fieldType[count] == 'N')
            {
                if ((currentChar >= '0') && (currentChar <= '9') ||
                    (currentChar == '.') || (currentChar == '-') || (currentChar == ','))
                {
                    // verify it is the same type
                }
                else
                {
                    startedField =0; // parsing has ended because types changed
                    count++;
                    data->buffer[i]= NULL;
                    if(count == MAX_FIELDS)
                    {
                        return;
                    }
                }
            }
        }
        i++;
    }

}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    // since idxs are stored in increasing order check against
    // last stored idx for OUT OF RANGE condition

    int i =0;
    int startIdx;
    int size =0;
    // check fieldNumber against the last index
    if(fieldNumber > data->fieldPosition[(data->fieldCount) -1 ] )
    {
        return NULL; // field Number is out of range;
    }
   i =0;
    while(i< (data->fieldCount) )
    {
        if( data->fieldPosition[i] == fieldNumber)
        {
            break;
        }
        i++;
    }
    // position 5 does not exist so the fieldNumber does not exist
    if (i == data->fieldCount)
    {
        return NULL;
    }
    startIdx =data->fieldPosition[i];
    size =0;


    // determining the size for allocating the field
    while(data->buffer[startIdx] != 0)
    {
        size++;
        startIdx++;
    }

    startIdx =data->fieldPosition[i];

    i =0;
    char *ptr = &(data->buffer[startIdx]);
    return ptr;
}
/*
 * Converst str -> integer
 * fieldNumber -> index of what argument it will convert
 * ALSO THIS FUNCTION DOES NOT MAKE ANY SENSE
 * LOOK AT VERSION2 OF THIS FUNCITON
 * UPDATED: 05/05/2020
 */
int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{

    int startIdx;
    int value = 0;
    // check fieldNumber against the last index
    if(fieldNumber > data->fieldPosition[(data->fieldCount) -1 ] )
    {
        return 0; // field Number is out of range;
    }
    int i =0;
    while(i< (data->fieldCount) )
    {
        if( data->fieldPosition[i] == fieldNumber)
        {
            break;
        }
        i++;
    }
    // position 5 does not exist so the fieldNumber does not exist
    if (i == data->fieldCount)
    {
        return 0;
    }
    if( data->fieldType[i] != 'N')
    {
        return 0;
    }
    startIdx = data->fieldPosition[i];
    int multipler  = 1;
    int endIdx = startIdx;
    // to convert properly- need to find ones place and go from there
    while(data->buffer[endIdx] != 0)
    {
        endIdx++;
    }
    endIdx--;

    while(endIdx != startIdx)
    {
        value = value + ((data->buffer[endIdx] - '0') * multipler);
        multipler = multipler *10;
        endIdx--;
    }
    return value;
}
/*
 * CREATED 5/5/2020
 * New function created
 */
uint32_t getFieldIntegerV2(USER_DATA* data, uint8_t fieldNumber)
{
    int startIdx;
    int value = 0;
    // check fieldNumber against the last index
    if(fieldNumber > (data->fieldCount - 1)   )
    {
        return 0; // field Number is out of range;
    }
    // Verifies the type at that field
    if( data->fieldType[fieldNumber] != 'N')
    {
        return 0; // Not a number
    }

    startIdx = data->fieldPosition[fieldNumber];
    int multipler  = 1;
    int endIdx = startIdx;
    // to convert properly- need to find ones place and go from there
    while(data->buffer[endIdx] != 0)
    {
        endIdx++;
    }
    endIdx--;
    while(endIdx != (startIdx-1))
   {
       value = value + ((data->buffer[endIdx] - '0') * multipler);
       multipler = multipler *10;
       endIdx--;
   }
    return value;
}
bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
// minArguments does not count the CMD as an argument
   if(((data->fieldCount) -1) < minArguments)
   {
       return false;
   }

    int i =0;
    char tempData[MAX_CHARS];

    while(strCommand[i] != 0)
    {
        tempData[i] = strCommand[i];
        i++;
    }
    while(i< MAX_CHARS)
    {
        tempData[i] = 0;
        i++;
    }
    tempData[i] = 0;


    // field Position at idx 0 contains the cmd
    // due to parsing; it does not always start at INDEX 0 of the BUFFER
    // so we use fieldPosition
    int startIdx = data->fieldPosition[0];
    i =0;
    while( (data->buffer[startIdx] == strCommand[i]) && (strCommand[i] != 0))
    {
        i++;
        startIdx++;
    }
    if(strCommand[i] != 0)
    {
        return false;
    }

    // uses strcmp function // Not need though because the previous 10 lines take care of this
    // this just proves I can use strcmp
    char *storedCMD = getFieldString(data,data->fieldPosition[0]);
    if(strcmp(storedCMD,tempData) == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void getsUart0(USER_DATA* data)
{
    // clearing buffer
    int i = 0;
    for(i =0; i < (MAX_CHARS + 1); i++ )
    {
        data->buffer[i] = 0;
    }
    int count  =0;
    while(1)
    {
        char c = getcUart0();
        // Backspace or delete
        if((c == 8) || (c == 127))
        {
            // empty?
            if(count > 0)
            {
                count--;
            }
            else
            {
                continue;
            }
        }
        // [ENTER] key is pressed
        else if(c == 13)
        {
            data->buffer[count] = 0;
            return;
        }
        else if (c >= 32)
        {

            data->buffer[count]=c;
            count++;
            if(count == MAX_CHARS)
            {
                data->buffer[count] =0;
                return;
            }
        }
    }

}
float getLightPercentage()
{
    setAdc0Ss3Mux(2);
    setAdc0Ss3Log2AverageCount(2);
    uint16_t raw;
    float voltage;
    float voltage_percentage =0;
//    char str[100];


    raw = readAdc0Ss3();
//    sprintf(str, ">Raw ADC:                                         %4u\n\r", raw);
//    putsUart0(str);
    // error = +- 0.03 Volts
    voltage = (((float)(raw)) *0.0008014) + ((float)(-0.00065257));
    voltage_percentage  = (voltage / 3.3)*100.00;
//    sprintf(str, ">Calculated Voltage:                             %0.4f  \n\r", voltage);
//    putsUart0(str);
//    sprintf(str, ">Percentage of light:                              %0.2f  \n\n\r", voltage_percentage);
//    putsUart0(str);

    return voltage_percentage;

}
float getBatteryVoltage()
{
   setAdc0Ss3Mux(0); // change channels
   setAdc0Ss3Log2AverageCount(2);
   uint16_t raw;
   float voltage;
   float voltage_percentage;
//   char str[100];



   raw = readAdc0Ss3();
//   sprintf(str, ">Raw ADC:                                          %4u\n\r", raw);
//   putsUart0(str);
   // error = +- 0.03 Volts
   voltage = (( (float)(raw) ) /4095.00) * 3.3;
   voltage_percentage  = (voltage / 3.3)*100.00;
//   sprintf(str, ">Calculated Voltage:                             %0.4f  \n\r", voltage);
//   putsUart0(str);
//   sprintf(str, ">Percentage of Battery:                            %0.2f  \n\n\r", voltage_percentage);
//   putsUart0(str);

   return voltage;

}
float getMoisturePercentage()
{
   setAdc0Ss3Mux(1); // change channels SOIL IS AN1-PE2
   setAdc0Ss3Log2AverageCount(2);
   uint16_t raw;
   float voltage;
   float voltage_percentage;
//   char str[100];

   raw = readAdc0Ss3();
//   sprintf(str, ">Raw ADC:                                           %4u\n\r", raw);
//   putsUart0(str);
   // error = +- 0.03 Volts
   voltage = (((float)(raw)) *0.00080205)+ ((float)(0.0025));
   voltage_percentage  = (voltage / 3.3)*100.00;
//   sprintf(str, ">Calculated Voltage:                               %0.4f  \n\r", voltage);
//   putsUart0(str);
//   sprintf(str, ">Moisture Percentage:                              %0.2f  \n\n\r", voltage_percentage);
//   putsUart0(str);

   return voltage;
}
uint32_t getVolume()
{

  uint32_t mL =0;

  DEINT = 1;
  wait1ms();
  wait1ms();
  while(COMP_ACSTAT0_R && 0x00000002);            // making sure comparator is 0
  TIMER1_TAV_R =0;
  DEINT =0;                                       // TURN OFF DEINT
  TIMER1_CTL_R |= TIMER_CTL_TAEN;                 // Turn on timer
  interruptRunning = 1;                           // global variable
  while(interruptRunning);                        // keep the function from exiting without completing read ; AN INTERRUPT trigger will exit loop
  mL = (((float) timerValue) * (0.3455)) + -91;    // calculated using a base line and MATLab Polyfit -> y = mx + b

return mL;
}
void setCurrentSeconds(uint32_t seconds)
{
  HIB_RTCLD_R = seconds;
}
// Get current Time of day function
uint32_t getCurrentSeconds()
{
    return (HIB_RTCC_R % seconds_in_a_day);
}
bool isWateringAllowed()
{
   uint32_t timeofDay = getCurrentSeconds();
   if( (timeofDay >= start_seconds) && (timeofDay <= end_seconds) )
   {
       return true;
   }
   return false;
}
void enablePump()
{
    MOTOR_CONTROL = 1;
}
void disablePump()
{
    MOTOR_CONTROL = 0;
}
void check_MoistureLevel()
{
    char str[100];
    float currentlevel = getMoisturePercentage();
    currentlevel  = (currentlevel / 3.3)*100.00;
    currentlevel = 100 - currentlevel;

    bool keepwatering = false;
    // soil is dry
    if(currentlevel < moistureLevel)
    {
        enablePump();
        waitMicrosecond(5000000); // run for 5 seconds
        disablePump();
        waitMicrosecond(30000000); // wait 30 seoconds to let water saturate
        currentlevel = getMoisturePercentage();
        currentlevel  = (currentlevel / 3.3)*100.00;
        currentlevel = 100 - currentlevel;
        sprintf(str, "\r\n**NEED TO WATER PLANT*** \n\r");
        putsUart0(str);

    }
}
void check_batteryLevel()
{
    float currentLevel = getBatteryVoltage();
    currentLevel = (currentLevel / 2.07)* MAX_VOLTS;
    if(currentLevel < minBatteryLevel)
    {
        if (getLightPercentage() >= lightThreashold)
        {
            playBatteryLowAlert();
        }
    }
}
void check_resevoirLevel()
{
 if (getVolume() <minWaterLevel_ML)
 {
    playWaterLowAlert();
 }
}
void getStatus()
{
    char str[100];

    GREEN_LED = 0;
    uint32_t getCurrent_Seconds = getCurrentSeconds();
    uint32_t hours = getCurrent_Seconds / (60*60);
    uint32_t mins = (getCurrent_Seconds - (hours *(60*60))) /(60);
    sprintf(str, "\r\nCurrent time is %d:%d \n\r", hours,mins);
    putsUart0(str);

    uint32_t volume = getVolume();
    sprintf(str,"CURRENT WATER LEVEL: %d milimeters \n\r",volume);
    putsUart0(str);
    sprintf(str,"REQUIRED WATER LEVEL: %d milimeters \n\n\r",minWaterLevel_ML);
    putsUart0(str);

    uint32_t intensity = getLightPercentage();
    sprintf(str, "CURRENT:Light Percentage: %d  \n\r", intensity);
    putsUart0(str);
    sprintf(str, "REQUIRED:Light Percentage: %d  \n\n\r", lightThreashold);
    putsUart0(str);

    float volts = getBatteryVoltage();
    volts  = ((volts / 2.07)*MAX_VOLTS);
    sprintf(str, "CURRENT:Battery Voltage: %2.2f  \n\r", volts);
    putsUart0(str);
    sprintf(str, "REQUIRED:Battery Voltage: %2.2f  \n\n\r", (minBatteryLevel));
    putsUart0(str);

    float voltageADC = getMoisturePercentage();
    voltageADC  = (voltageADC / 3.3)*100.00;
    voltageADC = 100 - voltageADC;
    sprintf(str, "CURRENT:Moisture Percentage: %3.2f  \n\r", voltageADC);
    putsUart0(str);
    sprintf(str, "REQUIRED:Moisture Percentage: %d  \n\n\r", moistureLevel);
    putsUart0(str);
}
void list_of_valid_commands_for_user()
{
    putsUart0("                    VALID COMMANDS \r\n");
    putsUart0("1.  [time H M]  -> SET TIME [24 HR FORMAT] USING COMMAND \r\n");
    putsUart0("2.  [water H1 M1 H2 M2] -> SET WHEN TO WATER USING COMMAND \r\n");
    putsUart0("3.  [level val] -> SET MIN MOISTURE LEVEL USING COMMAND \r\n");
    putsUart0("4.  [SET_MIN_BATTERY_VOLTAGE val] -> SET MIN BATTERY LEVEL USING COMMAND \r\n");
    putsUart0("5.  [alert val] -> SET light Intensity(TIME OF DAY) FOR ALERTS USING COMMAND \r\n");
    putsUart0("6.  [status]    -> shows the current readings vs. Required values \r\n");
    putsUart0("7.  [light]     -> gets current light level \r\n");
    putsUart0("8.  [battery]   -> gets current voltage of battery \r\n");
    putsUart0("9.  [moisture]  -> gets current moisture level \r\n");
    putsUart0("10. [pump ON]   -> Turns water pump ON \r\n");
    putsUart0("11. [pump OFF]  -> Turns water pump OFF \r\n");
    putsUart0("12. [getCurrentTime]  -> get Current Time \r\n");
    putsUart0("13. [history]   -> show the past entries \r\n");
    putsUart0("14. [record]    -> manually log an entry \r\n");
    putsUart0("15. [erase]     -> erase all entries \r\n");

}
void storeData()
{

     char str[100];
     // ONLY HAVE 6 BITS FOR EACH VARIABLE SO MAX IS 64;
     // Trimming is required to stored data in a single word
     /*
     *  6 bits for moisture
     *  6 bits for water
     *  6 bits for light
     *  14 bits for time
     */
     // SETS DATA UP FOR STORING
     float voltageADC = getMoisturePercentage();
     voltageADC  = (voltageADC / 3.3)*100.00;
     voltageADC = 100 - voltageADC;
     uint8_t moisturePercent = (uint8_t) (voltageADC);
     moisturePercent = ((uint8_t) ( (moisturePercent/100.00)*63));

     // SETS DATA UP FOR STORING
     uint8_t waterLevel = getVolume();
     waterLevel = (  ((float) waterLevel)/MAX_WATER_VOLUME) *63;

     // SETS DATA UP FOR STORING
     uint8_t lightLevel = getLightPercentage();
     lightLevel =(  ((float) lightLevel)/100.00) *63;

     uint32_t timeStamp = getCurrentSeconds();
     uint16_t percentDay = ( ((float)timeStamp)/ seconds_in_a_day) * 16383; // percentage of day that has passed * 2^14 bits

     // Need to shift variables to their corresponding section
     // bits 31-26 -> moisture level
     uint32_t moisture6bits = moisturePercent << 26;
     // bits 25-20 -> water level
     uint32_t water6bits = waterLevel << 20;
     // bits 19-14 -> light level
     uint32_t light6bits = lightLevel << 14;
     // bits 13-0 -> timeStamp
     // MASKS LOWER 14 bits
     uint32_t time14bits = percentDay & (0x3FFF);

     uint32_t storingWord = ( moisture6bits | water6bits | light6bits | time14bits);
     EEPROM_EEBLOCK_R = BLOCK_NUMBER;
     EEPROM_EEOFFSET_R = offsetIdx_global;
     EEPROM_EERDWRINC_R = storingWord;
     offsetIdx_global = ((offsetIdx_global+1) % MAX_OFFSET); // values wraps around
}
void getHistory()
{
    char str[100];
    uint32_t storedWord;
    EEPROM_EEBLOCK_R = BLOCK_NUMBER;
    EEPROM_EEOFFSET_R = 0;
    uint8_t idx = 0;
    for(idx =0; ((idx < MAX_OFFSET)&& (idx < offsetIdx_global)); idx++)
    {
        EEPROM_EEOFFSET_R = idx;
        storedWord= EEPROM_EERDWRINC_R;
        // Masking each 6 bits of the 32 bit word will get the stored data
        // lower 14 bits for time
        uint32_t moisture6bitsEXTRACTION = (storedWord & (0xFC000000)) >> 26;
        uint32_t water6bitsEXTRACTION = (storedWord &    (0x03F00000)) >> 20;
        uint32_t light6bitsEXTRACTION = (storedWord &    (0x000FC000)) >> 14;
        uint32_t time14bitsEXTRACTION =  (storedWord &    (0x00003FFF));
         // SETS DATA UP FOR EXTRACTING
        moisture6bitsEXTRACTION = (moisture6bitsEXTRACTION/63.00) *100.00; // ONLY HAVE 6 BITS FOR EACH VARIABLE SO MAX IS 64;
        water6bitsEXTRACTION = (  ((float) water6bitsEXTRACTION)/63) *MAX_WATER_VOLUME;
        light6bitsEXTRACTION =(  ((float) light6bitsEXTRACTION)/63) *100;
        time14bitsEXTRACTION = ((float)(time14bitsEXTRACTION)/16383) * seconds_in_a_day;

        uint32_t hours = time14bitsEXTRACTION / (60*60);
        uint32_t mins = (time14bitsEXTRACTION - (hours *(60*60))) /(60);

        sprintf(str, "\r\nEntry #%d at [%d:%d]: \n\r",idx,hours,mins);
        putsUart0(str);
        sprintf(str, "RECORDED: Moisture Percentage: %d \n\r", moisture6bitsEXTRACTION);
        putsUart0(str);
        sprintf(str, "RECORDED: WATER LEVEL %d  \n\r", water6bitsEXTRACTION);
        putsUart0(str);
        sprintf(str, "RECORDED: LIGHT LEVEL %d  \n\r", light6bitsEXTRACTION);
        putsUart0(str);
    }
}
void eraseEEPROM()
{
    EEPROM_EEBLOCK_R = BLOCK_NUMBER;
    EEPROM_EEOFFSET_R = 0;
    uint8_t idx = 0;
    for(idx =0; idx < MAX_OFFSET; idx++)
    {
        EEPROM_EEBLOCK_R = BLOCK_NUMBER;
        EEPROM_EEOFFSET_R = idx;
        EEPROM_EERDWR_R = 0;
    }
    offsetIdx_global = 0;
}
void setOffsetPointerEEPROM()
{
    EEPROM_EEBLOCK_R = BLOCK_NUMBER;
    EEPROM_EEOFFSET_R = 0;
    uint8_t idx = 0;
    uint32_t storedData = EEPROM_EERDWRINC_R;
    // goes looking for the next available entry
    for(idx =0; idx < MAX_OFFSET; idx++)
    {
        EEPROM_EEOFFSET_R = idx;
        storedData = EEPROM_EERDWRINC_R;
        if(storedData == 0)
        {
            offsetIdx_global = idx;
            return;
        }
    }
    offsetIdx_global =0;

}
void timer4ISR()
{
    RED_LED = 1;
    FORCE_LOG_ENTRY =1;
    RED_LED =0;
    TIMER4_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}
int main(void)
{
    initRTC_Module();
    // Initialize hardware
    char str[100];
    initHw();
    initEEPROM_Module();
    initHW_DEINT();
    DEINT = 1;
    initHW_Comparator0();
    initUart0();
    initTimer1HW();
    lightSensor_moisture_voltage_Init();
    motor_Init();
    speaker_Init();
    initTimer2Hw();
    initTimer4Hw();
    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    setOffsetPointerEEPROM();


    // Display greeting
    putcUart0('\n');
    putsUart0("\r**********************************************************************\n\r");
    putsUart0("Welcome to world of flowers\r\n");
    putsUart0("BOOT UP SEQUENCE REQUIRES THE FOLLOWING\r\n");
    putsUart0("1. SET TIME [24 HR FORMAT] USING COMMAND: [time H M]\r\n");
    putsUart0("2. SET WHEN TO WATER USING COMMAND: [water H1 M1 H2 M2]\r\n");
    putsUart0("3. SET MIN MOISTURE LEVEL USING COMMAND: [level val]\r\n");
    putsUart0("4. SET MIN BATTERY LEVEL USING COMMAND: [SET_MIN_BATTERY_VOLTAGE val]\r\n");
    putsUart0("5. SET light Intensity(TIME OF DAY) FOR ALERTS USING COMMAND: [alert val]\r\n");
    putsUart0("\nCURRENT SETTINGS SHOWN BELOW\r");
    putsUart0("\n-----------------------------\r");
    getStatus();// outputs the current settings of the device
    list_of_valid_commands_for_user();
    putsUart0("\r**********************************************************************\n\r");
    putcUart0('>');

    //USER_DATA data;
    USER_DATA info;
    // enter command
    while(1)
    {
        bool valid = false;
        if(kbhitUart0())
        {
           RED_LED =1;
           getsUart0(&info);
           putsUart0(info.buffer);
           putcUart0('\n');
           putsUart0("\r");
           parseFields(&info);


           if (isCommand(&info, "set", 2))
           {
               int32_t add = getFieldInteger(&info, 1);
               int32_t data = getFieldInteger(&info, 2);
               valid = true;
               putsUart0("set command was set\n\r>");

           }
           else if(isCommand(&info,"time",2))
           {
               uint32_t hour = getFieldIntegerV2(&info,1);
               uint32_t minute =  getFieldIntegerV2(&info,2);
               uint32_t seconds =  (hour * 60* 60) + (minute * 60);
               setCurrentSeconds(seconds);
               sprintf(str,"Value you entered was H=%d M=%d\n\r>",hour,minute);
               putsUart0(str);
               valid = true;

           }
           // When to water
           // This sets the time of day
           else if(isCommand(&info,"water",4))
           {
               uint32_t H1 = getFieldIntegerV2(&info,1);
               uint32_t M1 = getFieldIntegerV2(&info,2);
               uint32_t H2 = getFieldIntegerV2(&info,3);
               uint32_t M2 = getFieldIntegerV2(&info,4);
               start_seconds = (H1 * 60* 60) + (M1 * 60);
               end_seconds = (H2 * 60* 60) + (M2 * 60);
               sprintf(str,"Value you entered was H1=%d M1=%d H2=%d M2=%d\n\r>",H1,M1,H2,M2);
               putsUart0(str);
               valid = true;

           }
           // sets the minimum moisture level
           // before water needs to happen
           else if(isCommand(&info,"level",1))
           {
               moistureLevel = getFieldIntegerV2(&info,1);
               sprintf(str,"Value you entered was percentage of moisture=%d\n\r>",moistureLevel);
               putsUart0(str);
               valid = true;
           }
           // set light level required for watering
           else if (isCommand(&info, "alert", 1))
           {
               lightThreashold = getFieldIntegerV2(&info, 1);
               playBatteryLowAlert();
               playWaterLowAlert();
               valid = true;
               putsUart0("ALERT COMMAND WAS SET\n\r>");

           }
           //sets required min voltage
           else if(isCommand(&info,"SET_MIN_BATTERY_VOLTAGE",1))
           {
               minBatteryLevel = (getFieldIntegerV2(&info,1) / 100.00) * 6.4;
           }

           else if(isCommand(&info,"status",0))
           {

               getStatus();
               valid = true;
           }
           else if(isCommand(&info,"light",0))
           {

               uint32_t intensity = getLightPercentage();
               sprintf(str, "CURRENT:Light Percentage: %d  \n\r", intensity);
               putsUart0(str);
               sprintf(str, "REQUIRED:Light Percentage: %d  \n\n\r", lightThreashold);
               putsUart0(str);
               valid = true;
           }
           else if(isCommand(&info,"battery",0))
           {
               float volts = getBatteryVoltage();
               volts  = ((volts / 2.07)*MAX_VOLTS);
               sprintf(str, "CURRENT:Battery Voltage: %2.2f  \n\r", volts);
               putsUart0(str);
               sprintf(str, "REQUIRED:Battery Voltage: %2.2f  \n\n\r", (minBatteryLevel));
               putsUart0(str);
           }
           else if(isCommand(&info,"moisture",0))
           {
               float voltageADC = getMoisturePercentage();
               voltageADC  = (voltageADC / 3.3)*100.00;
               voltageADC = 100 - voltageADC;
               sprintf(str, "CURRENT:Moisture Percentage: %3.2f  \n\r", voltageADC);
               putsUart0(str);
               sprintf(str, "REQUIRED:Moisture Percentage: %d  \n\n\r", moistureLevel);
               putsUart0(str);
               valid = true;
           }

           else if((isCommand(&info,"pump",1)))
           {
               char *secondArg = getFieldString(&info,info.fieldPosition[1]);
               if((strcmp(secondArg,"ON")) == 0)
               {
                   enablePump();
                   putsUart0("PUMP WAS TURNED ON\n\r>");
                   valid = true;
               }
               else if((strcmp(secondArg,"OFF"))  == 0)
               {
                   disablePump();
                   putsUart0("PUMP WAS TURNED OFF\n\r>");
                   valid = true;
               }
               else
               {
                   valid = false;
               }
           }
           else if (isCommand(&info,"getCurrentTime",0))
           {
               uint32_t getCurrent_Seconds = getCurrentSeconds();
               uint32_t hours = getCurrent_Seconds / (60*60);
               uint32_t mins = (getCurrent_Seconds - (hours *(60*60))) /(60);

               sprintf(str, ">Current time is %d : %2d \n\r", hours,mins);
               putsUart0(str);
               valid = true;
           }
           else if (isCommand(&info,"history",0))
           {

               getHistory();
               valid = true;

           }
           else if (isCommand(&info,"record",0))
           {

               storeData();
               valid = true;

           }
           else if (isCommand(&info,"erase",0))
           {
               eraseEEPROM();
               valid = true;
           }
           else if(!valid)
           {
               putsUart0("Invalid command\n\r>");
           }
           list_of_valid_commands_for_user();
           RED_LED = 0;
        }
        GREEN_LED ^=1;
        if(FORCE_LOG_ENTRY)
        {
            storeData();
            putsUart0("FORCED BACKUP HAPPEND\n\r>");
            FORCE_LOG_ENTRY =0;
        }
        check_MoistureLevel();
        check_batteryLevel();
        check_resevoirLevel();
        waitMicrosecond(50000);

    }

}
