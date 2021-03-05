#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"

/*
TM4C Pin      Function      Connection
PA0           TX0           PC(Rx line)
PA1           RX0           PC(Tx line)
PE3           AIN0          MQ-135 gas sensor
PE2           AIN1          LM35 temperature sensor
PF2           Interrupt     HR-SC501 motion sensor
PF3           Digital Out   Buzzer
*/

// Function prototypes

// UART0 functions
void UART0_init(void);
void UART0_Tx(char);
char UART0_Rx(void);
void UART0_sendString(char *);
void UART0_sensorStatus(void);

// ADC0 functions
void ADC0_init(void);
void ADC0_readSensors(void);

// interrupt service routines
void UART0_Handler(void);
void GPIOPortF_Handler(void);

// general functions
void GPIOF_init(void);
void checkAlarm(void);
void delayMs(void);
void delay(int);
void buzz(void);

// Global variables
float temp;                  // current reading value of temperature
float gasQuality;            // current reading value of gas quality
bool alarm = 0;              // is there reason for alarm to be sounded
bool toggleAlarm = 1;        // does the user want the alarm enabled or not
float temp_min = 0;          // threshold for minimum temperature
float temp_max = 60;         // threshold for maximum temperature
float gasQuality_max = 500;  // threshold for gas quality index

int main(void) {
  delay(20000);            // Warmup routine for MQ-135
  GPIOF_init();            // motion sensor handled through GPIO interrupt
  UART0_init();            // user control handled through UART interrupt
  ADC0_init();             // for gas and temperature sensors
  while (1) {              // do forever
    ADC0_readSensors();    // read sensor values and store in value_
    UART0_sensorStatus();  // show values on PC
    checkAlarm();          // do sensor values
    delay(1000);           // delay for ease of reading
  }
}

void UART0_init(void) {
  SYSCTL->RCGCUART |= 1;        // provide clock to UART0
  SYSCTL->RCGCGPIO |= 1;        // enable clock to PORTA
  UART0->CTL = 0;               // disable UART0
  UART0->IBRD = 8;              // 115200 baud rate -> 1MHz/115200 = 8.68055...
  UART0->FBRD = 44;             // fraction part -> 0.68055 * 64 + 0.5 = 44.0555...
  UART0->CC = 0;                // use system clock
  UART0->LCRH = 0x60;           // 8-bit, no parity, 1-stop bit, no FIFO
  UART0->IM |= 0x0010;          // enable UART0 RX interrupt
  UART0->CTL = 0x301;           // enable UART0, TXE, RXE
  GPIOA->DEN = 0x03;            // Make PA0(TX0) and PA1(RX0) as digital
  GPIOA->AFSEL = 0x03;          // Use PA0,PA1 alternate function
  GPIOA->PCTL = 0x11;           // configure PA0 and PA1 for UART
  NVIC->IP[5] = 3 << 5;         // set priority to 3
  NVIC->ISER[0] |= 0x00000020;  // enable IRQ5
}

void UART0_Tx(char c) {
  while ((UART0->FR & 0x0020) != 0)
    ;             // wait until Tx buffer is full
  UART0->DR = c;  // before giving it another byte
}

char UART0_Rx(void) {
  while ((UART0->FR & 0x0010) != 0)
    ;
  return ((char)(UART0->DR & 0xFF));
}

void UART0_sendString(char *ptr) {
  while (*ptr) {     // while there are characters in ptr
    UART0_Tx(*ptr);  // send character over Tx
    ptr++;           // get next character
  }
}

void UART0_sensorStatus(void) {
  char str[50];                                                   // dummy string
  sprintf(str, "Temp: %2fC\tGas: %2fppm\n\r", temp, gasQuality);  // format string
  UART0_sendString(str);                                          // sending over Tx
}

void ADC0_init(void) {
  SYSCTL->RCGCGPIO |= 0x10;  // enable clock for port E
  SYSCTL->RCGCADC |= 1;      // enable clock for ADC0
  GPIOE->AFSEL |= 0x0C;      // enable alternate function for AIN0 (PE3) and AIN1 (PE2)
  GPIOE->DEN &= ~0x0C;       // disable digital function for PE3 and PE2
  GPIOE->AMSEL |= 0x0C;      // enable analog function for PE3 and PE2
  ADC0->ACTSS &= ~8;         // diable ADC0SS3
  ADC0->EMUX &= ~0xF000;     // software trigger conversion
  ADC0->SSCTL3 |= 6;         // one sample at a time, flag on first complete
  ADC0->ACTSS |= 8;          // enable ADC0SS3
}

void ADC0_readSensors(void) {
  ADC0->ACTSS &= ~8;                  // diable ADC0SS3
  ADC0->SSMUX3 = 0;                   // MQ135(gas sensor) on PE3, AIN0
  ADC0->ACTSS |= 8;                   // enable ADC0SS3
  ADC0->PSSI |= 8;                    // start conversion
  while ((ADC0->RIS & 8) == 0)        //
    ;                                 //
  gasQuality = ADC0->SSFIFO3;         //
  ADC0->ISC = 8;                      // clear completion flag
  ADC0->ACTSS &= ~8;                  // diable ADC0SS3
  ADC0->SSMUX3 = 1;                   // LM35(temperature sensor) on PE2, AIN1
  ADC0->ACTSS |= 8;                   // enable ADC0SS3
  ADC0->PSSI |= 8;                    // start conversion
  while ((ADC0->RIS & 8) == 0)        //
    ;                                 //
  temp = ADC0->SSFIFO3 * 330 / 4096;  //
  ADC0->ISC = 8;                      // clear completion flag
}

void GPIOF_init(void) {
  SYSCTL->RCGCGPIO |= 0x20;     // enable port F clock
  GPIOF->DIR &= ~0x04;          // input for motion sensor interrupt
  GPIOF->DIR |= 0x08;           // output for buzzer
  GPIOF->DEN |= 0x04;           // digital enable for motion sensor input
  GPIOF->DEN |= 0x08;           // digital enable for buzzer output
  GPIOF->IS &= ~0x04;           // edge triggered interrupt
  GPIOF->IBE &= ~0x04;          // trigger is controlled by IEV
  GPIOF->IEV &= ~0x04;          // falling edge trigger
  GPIOF->ICR |= 0x04;           // clear any prior interrupt
  GPIOF->IM |= 0x04;            // unmask interrupt
  NVIC->IP[30] = 3 << 5;        // set priority to 3
  NVIC->ISER[0] |= 0x40000000;  // enable IRQ30
  __enable_irq();               // global IRQ enable
}

void delay(int x) {
  for (int i = 0; i < x; i++) {
    delayMs();
  }
}

void delayMs(void) {
  SysTick->LOAD = 15999;                  // 1ms/62.5ns - 1
  SysTick->VAL = 0;                       // set current value to minimum
  SysTick->CTRL = 5;                      // start timer
  while ((SysTick->CTRL & 0x10000) == 0)  // wait for flag
    ;                                     //
  SysTick->CTRL = 0;                      // disable timer
}

void buzz(void) {
  __disable_irq();                  // no interrupt during alarm
  for (int i = 0; i < 5000; i++) {  // 5 seconds loop
    GPIOF->DATA ^= 0x08;            // toggle buzzer pin
    delayMs();                      // 1kHz frequency for 3 seconds
  }                                 //
  __enable_irq();                   // re-enable interrup
}

void checkAlarm(void) {
  if (toggleAlarm == 0) return;
  if (gasQuality > gasQuality_max) alarm = 1;
  if (temp > temp_max || temp < temp_min) alarm = 1;
  if (alarm == 1) buzz();
  alarm = 0;
}

void GPIOPortF_Handler(void) {
  volatile int readback;
  while (GPIOF->MIS != 0) {
    if (GPIOF->MIS & 0x04)    // confirm if motion sensor interrupt
    {                         //
      buzz();                 // sound the alarm
      GPIOF->ICR |= 0x10;     // clear the interrupt flag
      readback = GPIOF->ICR;  // a read to force clearing of interruptflag
    }
  }
}

void UART0_Handler(void) {
  volatile int readback;
  if (UART0->MIS & 0x0010) {  // Check if Rx interrupt
    toggleAlarm ^= 1;         // toggle the alarm when user enters key on PC
    UART0->ICR = 0x0010;      // clear the interrupt flag
    readback = UART0->ICR;    // a read to force clearing of interruptflag
  }
}
