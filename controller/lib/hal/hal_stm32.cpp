/* Copyright 2020, RespiraWorks

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#if defined(BARE_STM32)

#include "hal.h"
#include "stm32.h"

// This is the main stack 
uint32_t mainStack[ 500 ];

// local static functions.  I don't want to add any private 
// functions to the Hal class to avoid complexity with other
// builds
static void EnableClock( uint32_t base );

// For now, the main function in main.cpp is called setup
// rather then main.  If we adopt this HAL then we can 
// just rename it main and get rid of the following function.
extern void setup( void );
int main( void )
{
   setup();
}

/*
 * One time init of HAL.
 * This does some basic setup of the STM32 processor such as
 * turning on the PLL to bring the clock rate up to the max
 * value (80MHz), enabling floating point, etc.
 */
void HalApi::init() {

   // Enable the FPU
   SysCtrl_Reg *sysCtl = (SysCtrl_Reg *)SYSCTL_BASE;
   sysCtl->cpac = 0x00F00000;

   // Reset caches and set latency for 80MHz opperation
   EnableClock( FLASH_BASE );
   FlashReg *flash = (FlashReg *)FLASH_BASE;
   flash->access = 0x00000004;
   flash->access = 0x00001804;
   flash->access = 0x00001804;
   flash->access = 0x00000604;

   // Enable the PLL.
   // We use the MSI clock as the source for the PLL
   // The MSI clock is running at it's default frequency of 
   // 4MHz.
   //
   // The PLL can generate several clocks with somewhat
   // less then descriptive names in the reference manual.
   // These clocks are:
   //   P clock - Used for the SAI peripherial.  Not used here
   //   Q clock - 48MHz output clock used for USB.  Not used here.
   //   R clock - This is the main system clock.  We care about this one.
   //
   // When configuring the PLL there are several constants programmed
   // into the PLL register to set the frequency of the internal VCO 
   // These constants are called N and M in the reference manual:
   //
   // Fin = 4MHz
   // Fvco = Fin * (N/M)
   //
   // Legal range for Fvco is 96MHz to 344MHz according to the 
   // data sheet.  I'll use 160MHz for Fvco and divide by 2
   // to get an 80MHz output clock
   //
   int N = 40;
   int M = 1;
   RCC_Regs *rcc = (RCC_Regs *)RCC_BASE;
   rcc->pllCfg = 0x01000001 | (N<<8) | ((M-1)<<4);

   // Turn on the PLL 
   rcc->clkCtrl |= 0x01000000;

   // Wait for the PLL ready indication
   while( !(rcc->clkCtrl & 0x02000000) ){}

   // Set PLL as system clock
   rcc->clkCfg = 0x00000003;

   // Enable the GPIO modules that we're using and 
   // configure the GPIO pins.


   // Enable interrupts
   IntEnable();
}

// Reset the processor
[[noreturn]] void HalApi::reset_device() {

   // Note that the system control registers are a standard ARM peripherial
   // they aren't documented in the normal STM32 reference manual, rather
   // they're in the processor programming manual.
   // The register we use to reset the system is called the 
   // "Application interrupt and reset control register (AIRCR)"
   SysCtrl_Reg *sysCtl = (SysCtrl_Reg *)SYSCTL_BASE;
   sysCtl->apInt = 0x05FA0004;
}

uint32_t HalApi::millis() { return 0; }

void HalApi::delay(uint32_t ms) {}

int HalApi::analogRead(AnalogPin pin){
  return 0;
}

void HalApi::analogWrite(PwmPin pin, int value) {
}

void HalApi::digitalWrite(BinaryPin pin, VoltageLevel value) {
}

uint16_t HalApi::serialRead(char *buf, uint16_t len) {
  return 0;
}

uint16_t HalApi::serialBytesAvailableForRead() {
  return 0;
}

uint16_t HalApi::serialWrite(const char *buf, uint16_t len) {
  return 0;
}

uint16_t HalApi::serialBytesAvailableForWrite() {
  return 0;
}

void HalApi::watchdog_init() {
}

void HalApi::watchdog_handler() {
}

// Enable clocks to a specific peripherial.
// On the STM32 the clocks going to various peripherials on the chip
// are individually selectable and for the most part disabled on startup.
// Clocks to the specific peripherials need to be enabled through the 
// RCC (Reset and Clock Controller) module before the peripherial can be
// used.
// Pass in the base address of the peripherial to enable it's clock
static void EnableClock( uint32_t base )
{
   RCC_Regs *rcc = (RCC_Regs *)RCC_BASE;

   // I don't include all the peripherials here, just the ones
   // that we currently use or seem likely to be used in the 
   // future.  To add more peripherials, just look up the appropriate
   // bit in the reference manual.
   //
   // This big case statement finds the index of the register in the 
   // array of clock enable registers, and the bit number used to enable
   // the clock for the specified peripherial.
   int ndx = -1;
   int bit = 0;
   switch( base )
   {
      case DMA1_BASE:    ndx = 0; bit =  0; break;
      case DMA2_BASE:    ndx = 0; bit =  1; break;
      case FLASH_BASE:   ndx = 0; bit =  8; break;

      case DIGIO_A_BASE: ndx = 1; bit =  0; break;
      case DIGIO_B_BASE: ndx = 1; bit =  1; break;
      case DIGIO_C_BASE: ndx = 1; bit =  2; break;
      case DIGIO_D_BASE: ndx = 1; bit =  3; break;
      case DIGIO_E_BASE: ndx = 1; bit =  4; break;
      case DIGIO_H_BASE: ndx = 1; bit =  7; break;
      case ADC_BASE:     ndx = 1; bit = 13; break;

      case TIMER2_BASE:  ndx = 4; bit =  0; break;
      case TIMER3_BASE:  ndx = 4; bit =  1; break;
      case TIMER6_BASE:  ndx = 4; bit =  4; break;
      case TIMER7_BASE:  ndx = 4; bit =  7; break;
      case SPI2_BASE:    ndx = 4; bit = 14; break;
      case SPI3_BASE:    ndx = 4; bit = 15; break;
      case UART2_BASE:   ndx = 4; bit = 17; break;
      case UART3_BASE:   ndx = 4; bit = 18; break;
      case UART4_BASE:   ndx = 4; bit = 19; break;
      case I2C1_BASE:    ndx = 4; bit = 21; break;
      case I2C2_BASE:    ndx = 4; bit = 22; break;
      case I2C3_BASE:    ndx = 4; bit = 23; break;

      case I2C4_BASE:    ndx = 5; bit =  1; break;

      case TIMER1_BASE:  ndx = 6; bit = 11; break;
      case SPI1_BASE:    ndx = 6; bit = 12; break;
      case UART1_BASE:   ndx = 6; bit = 13; break;
      case TIMER15_BASE: ndx = 6; bit = 16; break;
      case TIMER16_BASE: ndx = 6; bit = 17; break;
   }

   // Not sure what to do if the input address isn't found.
   // For now I just return.  Maybe I should force a fault?
   if( ndx < 0 ) return;

   // Enable the clock of the requested peripherial
   RCC_Regs *rcc = (RCC_Regs *)RCC_BASE;
   rcc->periphClkEna[ndx] |= (1<<bit);
}



// Fault handlers
static void fault( void )
{
   while(1){}
}

extern "C" void _init( void ){}

static void NMI( void )           { fault(); }
static void FaultISR( void )      { fault(); }
static void MPUFaultISR( void )   { fault(); }
static void BusFaultISR( void )   { fault(); }
static void UsageFaultISR( void ) { fault(); }
static void BadISR( void )        { fault(); }

// Interrupt vector table.
extern "C" void Reset_Handler( void );
__attribute__ ((section(".isr_vector")))
void (* const vectors[])(void) =
{
   (void (*)(void))((uint32_t)mainStack + sizeof(mainStack)),
   (void(*)(void))((uint32_t)Reset_Handler+1), //   1 - 0x004 The reset handler
   NMI,                                    //   2 - 0x008 The NMI handler
   FaultISR,                               //   3 - 0x00C The hard fault handler
   MPUFaultISR,                            //   4 - 0x010 The MPU fault handler
   BusFaultISR,                            //   5 - 0x014 The bus fault handler
   UsageFaultISR,                          //   6 - 0x018 The usage fault handler
   BadISR,                                 //   7 - 0x01C Reserved
   BadISR,                                 //   8 - 0x020 Reserved
   BadISR,                                 //   9 - 0x024 Reserved
   BadISR,                                 //  10 - 0x028 Reserved
   BadISR,                                 //  11 - 0x02C SVCall handler
   BadISR,                                 //  12 - 0x030 Debug monitor handler
   BadISR,                                 //  13 - 0x034 Reserved
   BadISR,                                 //  14 - 0x038 The PendSV handler
   BadISR,                                 //  15 - 0x03C 
   BadISR,                                 //  16 - 0x040 
   BadISR,                                 //  17 - 0x044 
   BadISR,                                 //  18 - 0x048 
   BadISR,                                 //  19 - 0x04C 
   BadISR,                                 //  20 - 0x050 
   BadISR,                                 //  21 - 0x054 
   BadISR,                                 //  22 - 0x058 
   BadISR,                                 //  23 - 0x05C 
   BadISR,                                 //  24 - 0x060 
   BadISR,                                 //  25 - 0x064 
   BadISR,                                 //  26 - 0x068 
   BadISR,                                 //  27 - 0x06C 
   BadISR,                                 //  28 - 0x070 
   BadISR,                                 //  29 - 0x074 
   BadISR,                                 //  30 - 0x078 
   BadISR,                                 //  31 - 0x07C 
   BadISR,                                 //  32 - 0x080 
   BadISR,                                 //  33 - 0x084 
   BadISR,                                 //  34 - 0x088 
   BadISR,                                 //  35 - 0x08C 
   BadISR,                                 //  36 - 0x090 
   BadISR,                                 //  37 - 0x094 
   BadISR,                                 //  38 - 0x098 
   BadISR,                                 //  39 - 0x09C 
   BadISR,                                 //  40 - 0x0A0 
   BadISR,                                 //  41 - 0x0A4 
   BadISR,                                 //  42 - 0x0A8 
   BadISR,                                 //  43 - 0x0AC 
   BadISR,                                 //  44 - 0x0B0 
   BadISR,                                 //  45 - 0x0B4 
   BadISR,                                 //  46 - 0x0B8 
   BadISR,                                 //  47 - 0x0BC 
   BadISR,                                 //  48 - 0x0C0 
   BadISR,                                 //  49 - 0x0C4 
   BadISR,                                 //  50 - 0x0C8 
   BadISR,                                 //  51 - 0x0CC
   BadISR,                                 //  52 - 0x0D0 
   BadISR,                                 //  53 - 0x0D4 
   BadISR,                                 //  54 - 0x0D8 
   BadISR,                                 //  55 - 0x0DC 
   BadISR,                                 //  56 - 0x0E0 
   BadISR,                                 //  57 - 0x0E4 
   BadISR,                                 //  58 - 0x0E8 
   BadISR,                                 //  59 - 0x0EC 
   BadISR,                                 //  60 - 0x0F0 
   BadISR,                                 //  61 - 0x0F4 
   BadISR,                                 //  62 - 0x0F8 
   BadISR,                                 //  63 - 0x0FC 
};

// NOTE - this never actually gets called.  It's just here
// to prevent the linker from remove the vector array
const void *GetVectorAddr( void ){ return &vectors; }

#endif