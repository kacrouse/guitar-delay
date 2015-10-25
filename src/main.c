#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"

#define SAMPLE_RATE 	13000	// approximately 16kHz (by experimentation)
#define DAC_BIT_RATE 	15000000 // DAC baud of 15MHz (TLV5616 max is 20MHz)

//
// Configures Timer0 module by setting Timer0A to time out at a frequency of
// SAMPLE_RATE. Set to trigger ADC0 on time out. Timer enabled upon completion.
// CAVEAT: SAMPLE_RATE frequency is approximate. Tends to be higher than value.
// NOTE: Must be called before ConfigureADC0.
//
void ConfigureTimer0();

//
// Configures ADC0 module to retrieve a single sample triggered by the time out
// of Timer0A. Set to receive input from PE3 (AIN0). Interrupt flag set and
// ADC0IntHandler called after conversion is complete. ADC enabled upon
// completion.
//
void ConfigureADC0();

//
// Configures SSI0 module to master mode, Motorola format, 16-bit transfer.
// Bit rate set by DAC_BIT_RATE. Polarity = 1, phase = 1. SSI enabled upon
// completion.
// NOTE: Only SCLK, FS, and TX pins are enabled. RX pin is not.
//
void ConfigureSSI();

//
// Initializes g_block_address variable by setting 0th element equal to 0 and
// each following component (up to DECAY - 1 components) a distance of
// block_space apart.
//
void InitBlockAddress(uint16_t block_space);

//
// Returns the delayed output for the particular point in time. Gets each sample
// signified by elements of g_block_address, multiplies by appropriate factor,
// and sums. Finally, the sum is multiplied by a factor based on DECAY to scale
// the sum down to 12 bits.
//
uint16_t GetDelayOutput();

//
// Returns real-time output added to delayed output with MIX factored in.
//
uint16_t GetMixOutput(volatile uint16_t delay_output);

//
// Adds control bits to sample pointed to by data, then sends sample to DAC at
// baud rate of DAC_BIT_RATE. Function completed after all bits have been sent.
// NOTE: *data must hold data in 12 LSBs only. DAC may not function correctly
// otherwise.
//
void SendSampleToDAC(volatile uint16_t *data);

//
// Called when ADC0 conversion is completed. Stores most recent sample in
// g_input_buffer[0], clears interrupt flag, and sets g_sample_complete to
// trigger other operations.
//
void ADC0IntHandler(void);

//
// Returns 16-bit int obtained from LSBs of value. 16 MSBs are discarded.
//
uint16_t Convert32to16(uint32_t value);

//
// Increments each value contained in elements of g_block_address array.
// If value exceeds top_mem_address, value is set to 0.
//
void IncBlockAddress(uint16_t top_mem_address);

// The sample rate in kHz. Used to compute block_space.
const uint16_t SAMPLE_RATE_kHz = 16;

// Used to compute the number of echoes as well as the amplitude of echoes.
// The number of echoes is equal to DECAY - 1
const uint8_t DECAY = 3;

// The length of each echo in milliseconds.
const uint16_t DELAY_TIME_ms = 25;

// Balance of real-time signal with delayed signal. A value between 0 and 10.
// Low values mean more of the delayed signal is heard.
const uint8_t MIX = 4;

// Stores sample value retrieved from ADC0.
uint32_t g_input_buffer[1];

// Stores retrieved samples cyclically.
// Not all elements may be used depending upon the parameters.
volatile uint16_t g_data[16000] = { 0 };

// Stores addresses (indices) of current sample and delayed samples.
volatile uint16_t g_block_address[6];

// Set by ADC0IntHandler. Triggers other operations to take place in main.
volatile uint8_t g_sample_complete = 0;




void main(void)
{
	// 80MHz Clock
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
	                   SYSCTL_XTAL_16MHZ);
	ConfigureADC0();
	ConfigureTimer0();
	ConfigureSSI();

	uint32_t block_space = SAMPLE_RATE_kHz * DELAY_TIME_ms;
	uint16_t top_mem_address = DECAY * block_space - 1;
	uint16_t current_sample = 0;
	volatile uint16_t delay_output, mix_output;

    InitBlockAddress(block_space);
	IntMasterEnable(); // Enable processor interrupts

    while(1)
    {
		switch(g_sample_complete)
		{
			case 1:
			{
				// convert most recent sample to 16 bit int
				current_sample = Convert32to16(g_input_buffer[0]);

				// write most recent sample to current mem address
				g_data[g_block_address[0]] = current_sample;

				// calculate delay output
				delay_output = GetDelayOutput();

				// calculate delay output mixed with real-time output
				mix_output = GetMixOutput(delay_output);

				// send to DAC
				SendSampleToDAC(&mix_output);

				// increment addresses
				IncBlockAddress(top_mem_address);

				g_sample_complete = 0;
				break;
			}
			default:
				break;
		}
    }
}

void ConfigureTimer0()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Value loaded in causes time out at a frequency of SAMPLE_RATE
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/SAMPLE_RATE);
	TimerEnable(TIMER0_BASE, TIMER_A);
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);    // Trigger ADC0
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void ConfigureADC0()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlDelay(10);                               // Allow clock to reach ADC0
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);   // PE3 -> AIN0
    IntDisable(INT_ADC0SS3);                       // Disabled for configuration
    ADCIntDisable(ADC0_BASE, 3);
    ADCSequenceDisable(ADC0_BASE, 3);	           // Disabled for configuration
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);

    // Single sample, AIN0, interrupt flag set upon completion
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 |
    		ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
	ADCIntEnable(ADC0_BASE, 3);
	IntEnable(INT_ADC0SS3);
}

void ConfigureSSI()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    // PA2 -> CLK, PA3 -> FS, PA5 -> TX
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5);

    // Polarity = 1, Phase = 1, Master mode, Baud rate = DAC_BIT_RATE,
    // 16-bit transfers
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,
			SSI_MODE_MASTER, DAC_BIT_RATE, 16);
	SSIEnable(SSI0_BASE);
}

void InitBlockAddress(uint16_t block_space)
{
	uint8_t n;

	g_block_address[0] = 0;	// real-time signal

	for(n = 1; n < DECAY; n++)
		g_block_address[n] = block_space * n - 1;
}


uint16_t GetDelayOutput()
{
	uint16_t output = 0;
	uint8_t n;

    // Each delayed sample is multiplied by a factor then added to the total.
    // More recent samples receive a larger multiplying factor than older
    // samples.
	for(n = 1; n < DECAY; n++)
		output = output + ((g_data[g_block_address[n]] * n) / DECAY);

    // Normalize output
	switch(DECAY)
	{
	case 4:
		output *= 0.6;
		break;
	case 5:
		output *= 0.5;
		break;
	case 6:
		output *= 0.4;
		break;
	case 7:
		output *= 0.3;
		break;
	case 8:
		output *= 0.28;
		break;
	case 9:
		output *= 0.25;
		break;
	case 10:
		output *= 0.22;
		break;
	default:
		break;
	}

	return output;
}

uint16_t GetMixOutput(volatile uint16_t delay_output)
{
	uint16_t output;

    // Real-time signal and delay_output weighted based on MIX.
	output = ((MIX * g_data[g_block_address[0]]) / 10) +
        (((10 - MIX) * delay_output) / 10);

	return output;
}


void SendSampleToDAC(volatile uint16_t *data)
{
	uint32_t send_data;

	// Sets control bits for DAC & converts to 32-bit int.
	// 0x4000 for fast mode
	// 0x0000 for slow mode
	send_data = (*data | 0x4000) & 0xFFFF;

	SSIDataPut(SSI0_BASE, send_data);   // Transmit send_data over SSI.
	while(SSIBusy(SSI0_BASE));	        // Wait until SSI is done transmitting.
}

void ADC0IntHandler(void)
{
    ADCIntClear(ADC0_BASE, 3);  	// Clear the interrupt status flag.

    // Place sample value into g_input_buffer.
    ADCSequenceDataGet(ADC0_BASE, 3, g_input_buffer);
    g_sample_complete = 1;          // Trigger other operations to begin.
}

uint16_t Convert32to16(uint32_t value)
{
	uint16_t converted;
	converted = value & 0xFFFF;
	return converted;
}

void IncBlockAddress(uint16_t top_mem_address)
{
	uint8_t n;

	for(n = 0; n < DECAY; n++)
	{
		g_block_address[n]++;

		if(g_block_address[n] > top_mem_address) // Cyclic
			g_block_address[n] = 0;
	}
}
