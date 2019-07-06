///////////////////////////////////////////////////////////////////////////////
// Altura MIDI Theremin
//	STM32F103C8T6 version by Dewtronics/Donna Whisnant
//	Written July 2019
//	Copyright(c)2019 by Donna Whisnant (a.k.a. Dewtronics), and is released under
//	the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 Unported License:
//	https://creativecommons.org/licenses/by-nc-sa/4.0/
//
//
//	Based on original design by Zeppelin Design Labs:
//		ALTURA MIDI Theremin by Zeppelin Design Labs LLC, 2017
//		by Thomas K Wray & Glen van Alkemade. Code inspirations included:
//		thereThing
//		MiniWI
//
///////////////////////////////////////////////////////////////////////////////
//
//	Uses off-the-shelf STM32F103C8T6 BluePill PCB
//
///////////////////////////////////////////////////////////////////////////////
//
//	Interfaces:
//	-----------
//	USART1 (RX/TX) - Used for both System Bootloader and for Serial MIDI output
//	I2C1 (SCL/SDA) - Used for CAT24C32WI 4Kx8 EEPROM
//	I2C2 (SCL/SDA) - Used for TM1637, 5631BH, Three-Digit Seven Segment LED
//	PB3 - Used for Left Ultrasonic Distance Sensor Trigger
//	TIM2/1 (PA15) - Used for Left Ultrasonic Distance Sensor Echo
//	PB5 - Used for Right Ultrasonic Distance Sensor Trigger
//	TIM3/1 (PB4) - Used for Right Ultrasonic Distance Sensor Echo
//	SPI2 (PB12-PB15, PA8) - Reserved for SD Card
//	USB (USBDM/USBDP) - USB Device - MIDI and optionally: Serial and Mass Storage
//	JTAG (JTMS-SWDIO, JTCK-SWCLK, JTDI, JTDO, NJTRST) - JTAG Debugging and Programming
//	Analog (PA0-PA7) - Eight Input Potentiometers (See Below)
//	LED (PC13) - Status LED on Blue Pill
//
// Potentiometer layout with their Analog Channel Number:
//
//                          7
//                        ++o
//
//  3   0   1   2   4   5   6
// +-+ +-+ +-+ +-+ +-+ +-+ +-+
// |o| |o| |o| |o| |o| |o| |o|
// +-+ +-+ +-+ +-+ +-+ +-+ +-+
//
// PA3 = Data Far
// PA0 = Data Near
// PA1 = Function
// PA2 = Key
// PA4 = Scale
// PA5 = Octave Near(*)
// PA6 = Octave Far(*)
// PA7 = Articulation
//
//	(*)Note: Octave Near/Octave Far are swapped relative to the Zeppelin Design Labs
//			analog multiplexer assignment.  This is because they swapped VR7 and VR6
//			on the X5 and X6 inputs of the CD4051BCN chip in their design.
//
///////////////////////////////////////////////////////////////////////////////
/*

Clock: 72.0 MHz
µC: STM32F103C8T6 (LQFP-48)

STM32F103Cxxx Pin I/O Definitions: (48 pin parts)
----------------------------------
PA0  - Data Near Pot      PA0/WKUP/USART2_CTS/ADC12_IN0/TIM2_CH1_ETR (pin 10)
PA1  - Function Pot       PA1/USART2_RTS/ADC12_IN1/TIM2_CH2 (pin 11)
PA2  - Key Pot            PA2/USART2_TX/ADC12_IN2/TIM2_CH3 (pin 12)
PA3  - Data Far Pot       PA3/USART2_RX/ADC12_IN3/TIM2_CH4 (pin 13)
PA4  - Scale Pot          PA4/SPI1_NSS/USART2_CK/ADC12_IN4 (pin 14)
PA5  - Octave Near Pot    PA5/SPI1_SCK/ADC12_IN5 (pin 15)
PA6  - Octave Far Pot     PA6/SPI1_MISO/ADC12_IN6/TIM3_CH1/TIM1_BKIN (pin 16)
PA7  - Articulation Pot   PA7/SPI1_MOSI/ADC12_IN7/TIM3_CH2/TIM1_CH1N (pin 17)
PA8  - ~SD_Card_Det       PA8/USART1_CK/TIM1_CH1/MCO (pin 29)
PA9  - USART1_TX          PA9/USART1_TX/TIM1_CH2 (pin 30)
PA10 - USART1_RX          PA10/USART1_RX/TIM1_CH3 (pin 31)
PA11 - USBDM              PA11/USART1_CTS/USBDM/CAN_RX/TIM1_CH4 (pin 32)
PA12 - USBDP              PA12/USART1_RTS/USBDP/CAN_TX/TIM1_ETR (pin 33)
PA13 - JTMS-SWDIO         JTMS-SWDIO/PA13 (pin 34)
PA14 - JTCK-SWCLK         JTCK-SWCLK/PA14 (pin 37)
PA15 - L_Echo             JTDI/TIM2_CH1_ETR/PA15/SPI1_NSS (pin 38)
PB0  - unused             PB0/ADC12_IN8/TIM3_CH3/TIM1_CH2N (pin 18)
PB1  - unused             PB1/ADC12_IN9/TIM3_CH4/TIM1_CH3N (pin 19)
PB2  - BOOT1              PB2/BOOT1 (pin 20)
PB3  - L_Trigger          JTDO/PB3/TRACESWO/TIM2_CH2/SPI1_SCK (pin 39)
PB4  - R_Echo             NJTRST/PB4/TIM3_CH1/SPI1_MISO (pin 40)
PB5  - R_Trigger          PB5/I2C1_SMBA/TIM3_CH2/SPI1_MOSI (pin 41)
PB6  - I2C1_SCL           PB6/I2C1_SCL/TIM4_CH1/USART1_TX (pin 42)
PB7  - I2C1_SDA           PB7/I2C1_SDA/TIM4_CH2/USART1_RX (pin 43)
PB8  - unused             PB8/TIM4_CH3/I2C1_SCL/CAN_RX (pin 45)
PB9  - unused             PB9/TIM4_CH4/I2C1_SDA/CAN_TX (pin 46)
PB10 - I2C2_SCL           PB10/I2C2_SCL/USART3_TX/TIM2_CH3 (pin 21)
PB11 - I2C2_SDA           PB11/I2C2_SDA/USART3_RX/TIM2_CH4 (pin 22)
PB12 - SPI2_NSS           PB12/SPI2_NSS/I2C2_SMBA/USART3_CK/TIM1_BKIN (pin 25)
PB13 - SPI2_SCK           PB13/SPI2_SCK/USART3_CTS/TIM1_CH1N (pin 26)
PB14 - SPI2_MISO          PB14/SPI2_MISO/TIM1_CH2N/USART3_RTS (pin 27)
PB15 - SPI2_MOSI          PB15/SPI2_MOSI/TIM1_CH3N (pin 28)
PC13 - LED1               PC13/TAMPER-RTC  (Low-Drive Pin!) (pin 2)
PC14 - OSC32_IN           PC14/OSC32_IN  (Low-Drive Pin!) (pin 3)
PC15 - OSC32_OUT          PC15/OSC32_OUT (Low-Drive Pin!) (pin 4)
PD0  - OSC_IN             OSC_IN/PD0 (pin 5)
PD1  - OSC_OUT            OSC_OUT/PD1 (pin 6)
----
BOOT0 - Pin 44
VBAT - Pin 1
VDD1 - Pin 24
VDD2 - Pin 36
VDD3 - Pin 48
VDDA - Pin 9
VSS1 - Pin 23
VSS2 - Pin 35
VSS3 - Pin 47
VSSA - Pin 8
NRST - Pin 7
*/


#include <Arduino.h>
#include <util/atomic.h>
#include <MIDI.h>
#include <TM1637.h>
#include <USBComposite.h>
#include <usb_midi_device.h>
#include <USBCompositeSerial.h>
#include <HardwareTimer.h>
#include <libmaple/timer.h>
#include <series/timer.h>
#include <HardwareSerial.h>
#include <libmaple/usart.h>
#include <SPI.h>
#include <Wire.h>
#include <libmaple/gpio.h>

#include "timebase.h"
#include "timeout.h"

#define Q(text) #text
#define QUOTE(text) Q(text)

// ============================================================================

#define GIT_VERSION_STR QUOTE(GIT_VERSION)
#pragma message("Version=" GIT_VERSION_STR)
static const char g_constrVersion[] PROGMEM = GIT_VERSION_STR;
static const char g_constrDateTime[] PROGMEM = __DATE__ " " __TIME__;

// ============================================================================

// STM32 Arduino Framework Library Example:
#define USBMIDIID_ARD_EXAMPLE			0
#define USBMIDIID_ARD_EXAMPLE_VID		0x1EAF				// Maple LeafLabs
#define USBMIDIID_ARD_EXAMPLE_PID		0x0003
// USB Descriptor for the LUFA (Lightweight USB Framework for AVRs) Project
//              http://www.fourwalledcubicle.com/LUFA.php
//              http://www.fourwalledcubicle.com/files/LUFA/Doc/120730/html/_page__v_i_d_p_i_d.html
#define USBMIDIID_LUFA					1
#define USBMIDIID_LUFA_VID				0x03EB				// LUFA Project
#define USBMIDIID_LUFA_PID				0x2048				// MIDI Demo Application
// QinHeng Electronics CH345 MIDI adapter (cheap Chinese adapter used for most Music Tesla Coils)
#define USBMIDIID_QINHENG				2
#define USBMIDIID_QINHENG_VID			0x1a86				// QinHeng Electronics
#define USBMIDIID_QINHENG_PID			0x752d				// CH345 MIDI adapter
// v-usb textual name discrimination
// https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
#define USBMIDIID_VUSB_TEXTUAL			3
#define USBMIDIID_VUSB_TEXTUAL_VID		0x16c0
#define USBMIDIID_VUSB_TEXTUAL_PID		0x05e4
// v-usb serial number discrimination
// https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
#define USBMIDIID_VUSB_SERIAL			4
#define USBMIDIID_VUSB_SERIAL_VID		0x16c0
#define USBMIDIID_VUSB_SERIAL_PID		0x27de


#define		USB_MIDI_ID_TO_USE			LUFA

#define USBMIDIID_NAME_INDEX(x)	USBMIDIID_ ## x
#define USBMIDIID_NAME_VID(x)	USBMIDIID_ ## x ## _VID
#define USBMIDIID_NAME_PID(x)	USBMIDIID_ ## x ## _PID
#define USBMIDIID_INDEX(x)		USBMIDIID_NAME_INDEX(x)
#define USBMIDIID_VID(x)		USBMIDIID_NAME_VID(x)
#define USBMIDIID_PID(x)		USBMIDIID_NAME_PID(x)

constexpr uint16_t USBMIDI_VID = USBMIDIID_VID(USB_MIDI_ID_TO_USE);				// MIDI USB Vender ID
constexpr uint16_t USBMIDI_PID = USBMIDIID_PID(USB_MIDI_ID_TO_USE);				// MIDI USB Product ID


// ============================================================================

// USART definitions
#define USART_BAUDRATE 31250

// ============================================================================

// Pin definitions:

#define SWAP_SENSORS		true		// If 'true' the left and right sensors will be swapped

#define LeftTrigger_pin		PB3
#define LeftEcho_pin		PA15
#define RightTrigger_pin	PB5
#define RightEcho_pin		PB4

#define DataFarPot_pin		PA3
#define DataNearPot_pin		PA0
#define FunctionPot_pin		PA1
#define KeyPot_pin			PA2
#define ScalePot_pin		PA4
#define OctaveNearPot_pin	PA5
#define OctaveFarPot_pin	PA6
#define ArticulationPot_pin	PA7			//This channel is a special case and should be left alone.

#define TM1637_CLK			PB10		// I2C2_SCL
#define TM1637_DIO			PB11		// I2C2_SDA

#define SD_CS_pin			PB12
#define SD_Card_Ins_pin		PA8
#define SD_Card_Ins_Active	false		// Level of active state of Card Inserted pin


#define LED1_pin			PC13		// General status LED (active low)

#define USART_tx_pin		PA9
#define USART_rx_pin		PA10

SPIClass g_SPI2(2);

const adc_prescaler ADC_Clock_Prescaler = ADC_PRE_PCLK2_DIV_6;
const adc_smp_rate ADC_Sample_Rate = ADC_SMPR_55_5;

static inline __always_inline void setOutputPin(uint8 nPin) { PIN_MAP[nPin].gpio_device->regs->BSRR = (1U << PIN_MAP[nPin].gpio_bit); }
static inline __always_inline void clrOutputPin(uint8 nPin) { PIN_MAP[nPin].gpio_device->regs->BSRR = (1U << PIN_MAP[nPin].gpio_bit) << 16; }
static inline __always_inline bool pinInputState(uint8 nPin) { return ((PIN_MAP[nPin].gpio_device->regs->IDR & (1U << PIN_MAP[nPin].gpio_bit)) != 0); }
static inline __always_inline bool pinOutputState(uint8 nPin) { return ((PIN_MAP[nPin].gpio_device->regs->ODR & (1U << PIN_MAP[nPin].gpio_bit)) != 0); }
//static inline __always_inline void toggleOutputPin(uint8 nPin) { PIN_MAP[nPin].gpio_device->regs->ODR = PIN_MAP[nPin].gpio_device->regs->ODR ^ (1U << PIN_MAP[nPin].gpio_bit); }
static inline __always_inline void toggleOutputPin(uint8 nPin) { pinOutputState(nPin) ? clrOutputPin(nPin) : setOutputPin(nPin); }

// ============================================================================

static inline __always_inline adc_dev * const getADC()
{
	return &adc1;
}

// ============================================================================

static inline __always_inline void enableStatusLED()
{
	// Light status LED
	clrOutputPin(LED1_pin);		// Output is active low
}

static inline __always_inline void disableStatusLED()
{
	// Turn off status LED
	setOutputPin(LED1_pin);		// Output is active low
}

void flashStatusLED(uint8_t nCount, uint16_t nMillseconds = 250)
{
	do {
		enableStatusLED();
		delay(nMillseconds);
		disableStatusLED();
		delay(nMillseconds);
		disableStatusLED();
	} while(--nCount);
}

// ============================================================================

class MyUSARTMIDI : public HardwareSerial
{
public:
	MyUSARTMIDI()
		:	HardwareSerial(USART1, USART_tx_pin, USART_rx_pin)
	{ }

	inline void init() { begin(USART_BAUDRATE, SERIAL_8N1);  }
	inline void clearBufferData() { usart_reset_rx(this->c_dev()); }
	inline bool haveBufferData() { return (available() > 0); }
	inline uint8_t popFromBuffer() { return (read()); }
} g_USARTMIDI;
MIDI_CREATE_INSTANCE(HardwareSerial, g_USARTMIDI, MIDI)

// ============================================================================

class MyUSBMIDI
{
private:
	uint32 m_txPacketSize = 64;
	uint32 m_rxPacketSize = 64;

	static bool init(MyUSBMIDI* me) {
		usb_midi_setTXEPSize(me->m_txPacketSize);
		usb_midi_setRXEPSize(me->m_rxPacketSize);
		return true;
	}

public:
	bool registerComponent()
	{
		return USBComposite.add(&usbMIDIPart, this, (USBPartInitializer)&MyUSBMIDI::init);
	}
};

// ============================================================================

static void pumpSerialEvents(bool bDispatch = true)
{
	// MIDI Mode : Check for new data in the USART buffer
	while (g_USARTMIDI.haveBufferData()) {
		uint8_t nData = g_USARTMIDI.popFromBuffer();
		// Work on the data
//		if (bDispatch) g_MIDIFileProcessor.handleMIDIdata(nData);
	}

	// MIDI Mode : Check for new data in the USB buffer
	while (usb_midi_data_available()) {
		// Read the packet and unpack it:
		union EVENT_t {
			uint32_t i;
			uint8_t b[4];
			MIDI_EVENT_PACKET_t p;
		} ev;
		if (usb_midi_rx(&ev.i, 1)) {
			if (bDispatch) {
				switch (ev.p.cin) {
					case CIN_1BYTE:				// 1 Byte messages:
					case CIN_SYSEX_ENDS_IN_1:
						// handleMIDIdata(ev.p.midi0);
						break;
					case CIN_2BYTE_SYS_COMMON:	// 2 Byte messages:
					case CIN_SYSEX_ENDS_IN_2:
					case CIN_PROGRAM_CHANGE:
					case CIN_CHANNEL_PRESSURE:
						// handleMIDIdata(ev.p.midi0);
						// handleMIDIdata(ev.p.midi1);
						break;
					case CIN_3BYTE_SYS_COMMON:	// 3 Byte messages:
					case CIN_SYSEX:
					case CIN_SYSEX_ENDS_IN_3:
					case CIN_NOTE_OFF:
					case CIN_NOTE_ON:
					case CIN_AFTER_TOUCH:
					case CIN_CONTROL_CHANGE:
					case CIN_PITCH_WHEEL:
						// handleMIDIdata(ev.p.midi0);
						// handleMIDIdata(ev.p.midi1);
						// handleMIDIdata(ev.p.midi2);
						break;
					case CIN_MISC_FUNCTION:		// Reserved for future expansion
					case CIN_CABLE_EVENT:
						break;
				}
			}
		}
	}
}

// ============================================================================

// Initialize GPIO Pins
void GPIO_Init()
{
	// Disable JTAG port so we can use the JTDI, JTDO, and NJTRST pins as
	//	general GPIO.  This leaves the JTAG_SW ports enabled, which may come
	//	in handy for reprogramming and/or emergency device recovery:
	afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

	pinMode(LeftEcho_pin, INPUT_PULLDOWN);
	pinMode(LeftTrigger_pin, OUTPUT);	digitalWrite(LeftTrigger_pin, LOW);
	pinMode(RightEcho_pin, INPUT_PULLDOWN);
	pinMode(RightTrigger_pin, OUTPUT);	digitalWrite(RightTrigger_pin, LOW);

	pinMode(LED1_pin, OUTPUT);	disableStatusLED();

	pinMode(DataFarPot_pin, INPUT_ANALOG);
	pinMode(DataNearPot_pin, INPUT_ANALOG);
	pinMode(FunctionPot_pin, INPUT_ANALOG);
	pinMode(KeyPot_pin, INPUT_ANALOG);
	pinMode(ScalePot_pin, INPUT_ANALOG);
	pinMode(OctaveNearPot_pin, INPUT_ANALOG);
	pinMode(OctaveFarPot_pin, INPUT_ANALOG);
	pinMode(ArticulationPot_pin, INPUT_ANALOG);
}

// ============================================================================

// Initialize ADC Modules
void ADC_Init()
{
	// Value will be Left-Aligned to facilitate optimal 8-bit reading

	adc_set_prescaler(ADC_Clock_Prescaler);

	adc_init(getADC());						// Enable RCC clock
	getADC()->regs->SQR1 = 0;				// 1 Conversion
	getADC()->regs->CR1 = 0;				// Independent mode, interrupts disabled, analog watchdog disabled
	getADC()->regs->CR2 = ADC_CR2_CONT | ADC_CR2_JEXTSEL_JSWSTART | ADC_CR2_EXTSEL_SWSTART;		// AD on
	do {
		// According to 11.3.5, for continuous mode, this must be set after ADC_CR2_CONT is set to '1'
		getADC()->regs->CR2 |= ADC_CR2_ADON;	// Initial turn on of ADC (11.3.1 in Reference says it must be set twice)
		// From 11.12.3 in Reference:
		//	Note: If any other bit in this register apart from ADON is changed at the same time, then
		//	conversion is not triggered. This is to prevent triggering an erroneous conversion.
	} while ((getADC()->regs->CR2 & ADC_CR2_ADON) == 0);
	getADC()->regs->CR2 |= ADC_CR2_ADON;		// Initial turn on of ADC (11.3.1 in Reference says it must be set twice)
	getADC()->regs->SMPR1 = 0x00249249 * (static_cast<uint32_t>(ADC_Sample_Rate) & 0x07);
	getADC()->regs->SMPR2 = 0x09249249 * (static_cast<uint32_t>(ADC_Sample_Rate) & 0x07);
	getADC()->regs->SR = 0;						// Clear Flags
	getADC()->regs->SQR3 = PIN_MAP[ArticulationPot_pin].adc_channel;	// Select correct channel (lower 4 bits is channel)
	adc_calibrate(getADC());					// Calibrate the ADC
	getADC()->regs->CR2 |=  ADC_CR2_SWSTART;	// Start conversion
}

uint16_t readADC(uint8_t nPin)
{
	adc_reg_map * pADC = getADC()->regs;
	uint8_t nChannel = PIN_MAP[nPin].adc_channel;

	uint32_t nCurrent = pADC->DR;	// clears EOC flag

	// If in continuous mode, see if this is the channel already selected
	//	and if so, return it:
	if (pADC->CR2 & ADC_CR2_CONT) {
		if (((pADC->SQR3 & 0x1F) == nChannel) &&
			(pADC->CR2 & ADC_CR2_SWSTART)) {
			// If the input is the same and conversions are started, just return our continuously monitored value:
			return (nCurrent & 0xFFFF);
		}
		pADC->CR2 &= ~ADC_CR2_SWSTART;		// Disable conversion in progress
		pADC->DR;		// Clear any current conversion that might have completed before we disabled the ADC
	}

	pADC->SQR3 = nChannel;					// select new input
	pADC->SR &= ~ADC_SR_EOC;				// Make certain the EOC is clear so we read the new ADC and not the old
	pADC->CR2 |= ADC_CR2_SWSTART;			// Trigger a conversion
	while (!(pADC->SR & ADC_SR_EOC)) { }	// wait for completion

	return (pADC->DR & 0xFFFF);
}

// ============================================================================

// Timer Interrupts

volatile bool g_bLeftSensorRead = false;			// Set when the left sensor reading is acquired
volatile uint32_t g_nLeftSensorEchoTime = 0;		// Echo time for the Left Ultrasonic sensor in microseconds
volatile bool g_bRightSensorRead = false;			// Set when the right sensor reading is acquired
volatile uint32_t g_nRightSensorEchoTime = 0;		// Echo time for the Right Ultrasonic sensor in microseconds

extern "C" void __irq_tim2(void) {
	uint32_t dsr = (TIMER2->regs.gen->DIER & TIMER2->regs.gen->SR);
	if (dsr & TIMER_SR_CC1IF) {

	}
	if (dsr & TIMER_SR_CC2IF) {
		uint32_t nValue = TIMER2->regs.gen->CCR2;
		#if (SWAP_SENSORS)
		if (nValue && !g_bRightSensorRead) {
			g_nRightSensorEchoTime = nValue;
			g_bRightSensorRead = true;
		}
		#else
		if (nValue && !g_bLeftSensorRead) {
			g_nLeftSensorEchoTime = nValue;
			g_bLeftSensorRead = true;
		}
		#endif
	}
	// ----
	if (dsr & TIMER_SR_UIF) {
		// General Timebase:
		handleSystemTimeTick();
	}
	TIMER2->regs.gen->SR = ~(dsr & (TIMER_SR_CC1IF | TIMER_SR_CC2IF | TIMER_SR_CC1OF | TIMER_SR_CC2OF | TIMER_SR_UIF));
}

extern "C" void __irq_tim3(void) {
	uint32_t dsr = (TIMER3->regs.gen->DIER & TIMER3->regs.gen->SR);
	if (dsr & TIMER_SR_CC1IF) {

	}
	if (dsr & TIMER_SR_CC2IF) {
		uint32_t nValue = TIMER2->regs.gen->CCR2;
		#if (SWAP_SENSORS)
		if (nValue && !g_bLeftSensorRead) {
			g_nLeftSensorEchoTime = nValue;
			g_bLeftSensorRead = true;
		}
		#else
		if (nValue && !g_bRightSensorRead) {
			g_nRightSensorEchoTime = nValue;
			g_bRightSensorRead = true;
		}
		#endif
	}
	// ----
	if (dsr & TIMER_SR_UIF) {
		// General Timebase:
		handleSystemTimeTick();
	}
	TIMER3->regs.gen->SR = ~(dsr & (TIMER_SR_CC1IF | TIMER_SR_CC2IF | TIMER_SR_CC1OF | TIMER_SR_CC2OF | TIMER_SR_UIF));
}

extern "C" void __irq_tim4(void) {
	uint32_t dsr = (TIMER4->regs.gen->DIER & TIMER4->regs.gen->SR);
	if (dsr & TIMER_SR_UIF) {
		// General Timebase:
		handleSystemTimeTick();
	}
	TIMER4->regs.gen->SR = ~(dsr & (TIMER_SR_UIF));
}

// ============================================================================

// Initialize Timer Modules
void Timer_Init()
{
	// Enable RCC clocks:
	timer_init(TIMER2);				// Timer2 is the Left Ultrasonic Echo
	timer_init(TIMER3);				// Timer3 is the Right Ultrasonic Echo
	timer_init(TIMER4);				// Timer4 is the System Timebase

	// Setup Timer Prescalers:
	(TIMER2->regs).gen->PSC = (TIMER_PRESCALER - 1);
	(TIMER3->regs).gen->PSC = (TIMER_PRESCALER - 1);
	(TIMER4->regs).gen->PSC = (TIMER_PRESCALER - 1);

	// Disable Output Compare Output generation setup by default configuration:
	(TIMER2->regs).gen->CCMR1 = 0;
	(TIMER2->regs).gen->CCMR2 = 0;
	(TIMER2->regs).gen->CCER = 0;
	(TIMER2->regs).gen->SMCR = 0;
	(TIMER3->regs).gen->CCMR1 = 0;
	(TIMER3->regs).gen->CCMR2 = 0;
	(TIMER3->regs).gen->CCER = 0;
	(TIMER3->regs).gen->SMCR = 0;
	(TIMER4->regs).gen->CCMR1 = 0;
	(TIMER4->regs).gen->CCMR2 = 0;
	(TIMER4->regs).gen->CCER = 0;
	(TIMER4->regs).gen->SMCR = 0;

	// Remap TIM2 CH1->PA15, CH2->PB3, CH3->PB10, CH4->PB11
	afio_remap(AFIO_REMAP_TIM2_FULL);

	// Remap TIM3 CH1->PB4, CH2->PB5, CH3->PB0, CH4->PB1
	afio_remap(AFIO_REMAP_TIM3_PARTIAL);

	// Configure Timer2 as PWM Input mode (CCR1=Entire Pulse Time, CCR2=High Pulse Width):
	//	CCR2 will be the ultrasonic sensor echo time in microseconds
	(TIMER2->regs).gen->CCMR1 |= TIMER_CCMR1_CC1S_INPUT_TI1;		// Select TI1 as Channel 1 source
	(TIMER2->regs).gen->CCER &= ~TIMER_CCER_CC1P;					// Select Rising Edge on TI1 Channel 1
	(TIMER2->regs).gen->CCMR1 |= TIMER_CCMR1_CC2S_INPUT_TI1;		// Select TI1 as Channel 2 source
	(TIMER2->regs).gen->CCER |= TIMER_CCER_CC2P;					// Select Falling Edge on TI1 Channel 2
	(TIMER2->regs).gen->SMCR |= TIMER_SMCR_TS_TI1FP1;				// Select Slave source as TI1FP1 (must be selected while SMS==0)
	(TIMER2->regs).gen->SMCR |= TIMER_SMCR_SMS_RESET;				// Select TI1 Rising edge to reset counter
	(TIMER2->regs).gen->CCER |= TIMER_CCER_CC1E | TIMER_CCER_CC2E;	// Enable Input Capture

	// Configure Timer3 as PWM Input mode (CCR1=Entire Pulse Time, CCR2=High Pulse Width):
	//	CCR2 will be the ultrasonic sensor echo time in microseconds
	(TIMER3->regs).gen->CCMR1 |= TIMER_CCMR1_CC1S_INPUT_TI1;		// Select TI1 as Channel 1 source
	(TIMER3->regs).gen->CCER &= ~TIMER_CCER_CC1P;					// Select Rising Edge on TI1 Channel 1
	(TIMER3->regs).gen->CCMR1 |= TIMER_CCMR1_CC2S_INPUT_TI1;		// Select TI1 as Channel 2 source
	(TIMER3->regs).gen->CCER |= TIMER_CCER_CC2P;					// Select Falling Edge on TI1 Channel 2
	(TIMER3->regs).gen->SMCR |= TIMER_SMCR_TS_TI1FP1;				// Select Slave source as TI1FP1 (must be selected while SMS==0)
	(TIMER3->regs).gen->SMCR |= TIMER_SMCR_SMS_RESET;				// Select TI1 Rising edge to reset counter
	(TIMER3->regs).gen->CCER |= TIMER_CCER_CC1E | TIMER_CCER_CC2E;	// Enable Input Capture

	// Disable ARPE, Dir=Up, Counter Enable:
	(TIMER2->regs).gen->CR1 = TIMER_CR1_CEN;
	(TIMER3->regs).gen->CR1 = TIMER_CR1_CEN;
	(TIMER4->regs).gen->CR1 = TIMER_CR1_CEN;

	// General Timers:
	// ---------------
	// Timer2:
	(TIMER2->regs).gen->DIER = 0;
	(TIMER2->regs).gen->SR = 0;							// Clear any current IRQs
	nvic_irq_enable(NVIC_TIMER2);						// Enable IRQ callback in NVIC
	(TIMER2->regs).gen->DIER |= TIMER_DIER_CC2IE;		// Enable IRQ on CCR2 capture (pulse width)
	if (SYSTEM_TIME_TIMER_BASE == TIMER2->regs.gen) {
		(TIMER2->regs).gen->DIER |= TIMER_DIER_UIE;		// Start Timer 2 update IRQ for timing purposes
	}

	// Timer3:
	(TIMER3->regs).gen->DIER = 0;
	(TIMER3->regs).gen->SR = 0;							// Clear any current IRQs
	nvic_irq_enable(NVIC_TIMER3);						// Enable IRQ callback in NVIC
	(TIMER3->regs).gen->DIER |= TIMER_DIER_CC2IE;		// Enable IRQ on CCR2 capture (pulse width)
	if (SYSTEM_TIME_TIMER_BASE == TIMER3->regs.gen) {
		(TIMER3->regs).gen->DIER |= TIMER_DIER_UIE;		// Start Timer 3 update IRQ for timing purposes
	}

	// Timer4:
	(TIMER4->regs).gen->DIER = 0;
	(TIMER4->regs).gen->SR = 0;							// Clear any current IRQs
	nvic_irq_enable(NVIC_TIMER4);						// Enable IRQ callback in NVIC
	if (SYSTEM_TIME_TIMER_BASE == TIMER4->regs.gen) {
		(TIMER4->regs).gen->DIER |= TIMER_DIER_UIE;		// Start Timer 4 update IRQ for timing purposes
	}
}

// ============================================================================

// Initialize the I2C Port:
void I2C_Init()
{
	Wire.begin();			// I2C1 is EEPROM
}

// ============================================================================

// Initialize the SPI Port:
void SPI_Init()
{
	 // SD Card:
	pinMode(SD_CS_pin, OUTPUT);
	digitalWrite(SD_CS_pin, HIGH);
	pinMode(SD_Card_Ins_pin, INPUT_PULLUP);

	g_SPI2.begin();
	g_SPI2.setBitOrder(MSBFIRST);			// ??
// 	SPI Mode 	CPOL 	CPHA 	Shift SCK-edge 	Capture SCK-edge
// 0 			0 		0 		Falling 		Rising
// 1 			0 		1 		Rising 			Falling
// 2 			1 		0 		Rising 			Falling
// 3 			1 		1 		Falling 		Rising
	g_SPI2.setDataMode(SPI_MODE0);
	g_SPI2.setClockDivider(SPI_CLOCK_DIV16);		// Slow Speed (72 / 16 = 4.5MHz)
	g_SPI2.end();
}

// ============================================================================

class MyUSBComposite
{
public:
	MyUSBComposite()
		:	m_bUSBRegistered(false)
	{
	}

	void init()
	{
		//
		// Setup USB:
		//	Note: The total packet size of all devices
		//	is 320:
		//		USBMIDI : 128 (64 Rx, 64 Tx)
		//		Serial  : 144 (64 Rx, 64 Tx, 16 Control), Change to: 64 (24 Rx, 24 Tx, 16 Control)
		//		Storage : 128 (64 Rx, 64 Tx)
		//	Max endpoints is 7
		//		USBMIDI : 2
		//		Serial  : 3
		//		Storage : 2
		//
		USBComposite.setVendorId(USBMIDI_VID);
		USBComposite.setProductId(USBMIDI_PID);
		USBComposite.setManufacturerString("Dewtronics/Donna Whisnant");
		USBComposite.setProductString("Altura-MIDI-Theremin");
		USBComposite.setSerialString(g_constrVersion);
//		m_CompositeSerial.setRXPacketSize(24);
//		m_CompositeSerial.setTXPacketSize(24);

		registerComponents();
	}

protected:
	void registerComponents()
	{
		if (m_bUSBRegistered) {
			USBComposite.end();
			m_bUSBRegistered = false;
		}
		USBComposite.clear();
		m_USBMIDI.registerComponent();
//		m_CompositeSerial.registerComponent();
		USBComposite.begin();
		m_bUSBRegistered = true;
	}

private:
	bool m_bUSBRegistered;

	static MyUSBMIDI m_USBMIDI;
//	static USBCompositeSerial m_CompositeSerial;
//	static USBMassStorage m_MassStorage;
} g_MyUSBComposite;
MyUSBMIDI MyUSBComposite::m_USBMIDI;
//USBCompositeSerial MyUSBComposite::m_CompositeSerial;
//USBMassStorage MyUSBComposite::m_MassStorage;

// ============================================================================

class myTM1637 : public TM1637
{
public:
	myTM1637()
		:	TM1637(TM1637_CLK, TM1637_DIO)
	{ }

	void displayRaw(uint8_t nDigit, uint8_t nSegData)
	{
		start();					// Start signal sent to TM1637 from MCU
		writeByte(ADDR_FIXED);		// Command1: Set data
		stop();
		start();
		writeByte(nDigit | 0xc0);	// Command2: Set data (fixed address)
		writeByte(nSegData);		// Transfer display data 8 bits
		stop();
		start();
		writeByte(cmd_disp_ctrl);	// Control display
		stop();
	}

	void displayRaw(uint8_t nCount, uint8_t nSegData[])
	{
		start();				// Start signal sent to TM1637 from MCU
		writeByte(ADDR_AUTO);	// Command1: Set data
		stop();
		start();
		writeByte(cmd_set_addr);	// Command2: Set address (automatic address adding)

		for (uint8_t i = 0; i < nCount; i++) {
			writeByte(nSegData[i]);		// Transfer display data (8 bits x num_of_digits)
		}

		stop();
		start();
		writeByte(cmd_disp_ctrl);		// Control display
		stop();
	}


} g_TM1637;

void Display_Init()
{
	pinMode(TM1637_CLK, OUTPUT);
	pinMode(TM1637_DIO, OUTPUT);

	g_TM1637.init();
	g_TM1637.set(BRIGHT_TYPICAL);
}

// ============================================================================
// ============================================================================


// ============================================================================
// Begin Altura MIDI Theremin Specific Code
// ============================================================================

// FORMAT: D,DP,C,G,B,F,A,E
const byte g_condisplayableCharacters[50] = {
		0b10101111, 0b00101000, 0b10011011, 0b10111010, 0b00111100,
		0b10110110, 0b10110111, 0b00101010, 0b10111111, 0b10111110,
		0b10000111, 0b11000111, 0b10111001, 0b11111001, 0b10010111,
		0b00010111, 0b01010111, 0b10110111, 0b11110111, 0b00111111,
		0b01111111, 0b10110101, 0b00000000, 0b11101111, 0b10101111,
		0b10101110, 0b10101010, 0b10101000, 0b10100000, 0b10000000,
		0b10000001, 0b10000101, 0b10000111, 0b10001111, 0b10101111,
		0b11101111, 0b11111110, 0b11111101, 0b11111011, 0b11110111,
		0b11101111, 0b11011111, 0b10111111, 0b01111111, 0b11111111,
		0b11011011, 0b01101000, 0b00010000, 0b00010101, 0b00111000
	};

static uint8_t digitBitRemap(uint8_t nData)
{
	// Altura is D,DP,C,G,B,F,A,E
	// TM1637 is DP,G,F,E,D,C,B,A
	return (((nData & 0x02) >> 1) |			// A
			((nData & 0x08) >> 2) |			// B
			((nData & 0x20) >> 3) |			// C
			((nData & 0x80) >> 4) |			// D
			((nData & 0x01) << 4) |			// E
			((nData & 0x04) << 3) |			// F
			((nData & 0x10) << 2) |			// G
			((nData & 0x40) << 1));			// DP
}

constexpr int32_t g_conADCScale = 4096;				// ADC Range reading (STM32F103 has 12-bit ADCs)
constexpr int32_t g_conADCThreshold = 15;			// ADC Value Change Threshold (Use 3 for 10-bit ADC, 15 for 12-bit ADC)

// Map ADC binpoint number to scaling
//		binpoint is the number of bits on the ADC
static inline uint32_t mapADCvalue(uint32_t nValue, uint32_t nOutScale)
{
	nValue *= nOutScale;
	nValue /= g_conADCScale;
	return nValue;
}

class CAltura
{
private:
	// Sensor Setup -----------------------------------------
	static constexpr byte g_conLeftHandBufferAmount = 16;		// try to keep as a power of 2
	static constexpr byte g_conRightHandBufferAmount = 16;		// try to keep as a power of 2

	static constexpr int g_conMinimumDistance = 350;
	static constexpr int g_conMaximumDistance = 3000;
	static constexpr int g_conSensorTimeOut = 4000;
	static constexpr int g_conNoteBufferSpace = 720;

	unsigned long m_nLeftSensorProcessed=0;
	unsigned long m_nRightSensorProcessed=0;

	byte m_nNotesInCurrentScale = 15;
	byte m_nScaleCurrent;
	byte m_nKeyCurrent;

	int m_nNoteBuffer = g_conNoteBufferSpace / m_nNotesInCurrentScale;

	int m_nLastNote = 0;

	const byte m_conOctaveMax = 8;
	int m_nOctaveNearCurrent = 5;
	int m_nOctaveFarCurrent = 4;
	int m_nNumberOfOctavesCurrent = 2;
	bool m_bDescending = true;

	static const byte g_scales[12][13];

	//Pot Functionality Setup -----------------------------
	byte m_nFunctionSelectCurrent = 1;

	int m_nDataFar = 0;
	int m_nDataNear = 0;

	int m_nDataFarOld  = -1;
	int m_nDataNearOld  = -1;

	int m_nxyDataFarRight = 0;
	int m_nxyDataNearRight = 0;

	int m_nxyLeftControlChange = 85;
	int m_nxyRightControlChange = 86;

	bool m_bxyMode = false;

	bool m_bArticulationMode = true;

	byte m_nFastActionRatio = 1;

	//MIDI Packet Data -------------------------------------
	int m_nPitchBendNeutralZone = 10;
	int m_nPitchBendUp = 1700;
	int m_nPitchBendDown = 1700;

	bool m_bPortamentoOn = false;
	byte m_nPortamentoTime = 0;

	byte m_nNoteVelocity = 127;

	byte m_nMIDIChannel = 1;

	//volitile handle with care------------------------------------
	volatile byte m_arrMIDINotes[109];


	enum POT_CHANNEL_ENUM {
		POT_dataFarPot        = 0,
		POT_dataNearPot       = 1,
		POT_functionSelectPot = 2,
		POT_keyPot            = 3,
		POT_scalePot          = 4,
		POT_octaveNearPot     = 5,
		POT_octaveFarPot      = 6,
		POT_articulationPot   = 7,
		POT_END_OF_LIST
	};

	static constexpr byte g_nTotalChannels = 8;
	static_assert(g_nTotalChannels == POT_END_OF_LIST, "Pot Channel List is invalid");

	int m_arrPot[g_nTotalChannels];

	byte m_ledLeftDigit = 0;
	byte m_ledMiddleDigit = 0;
	byte m_ledRightDigit = 0;

	int m_nDisplayTimeout = 2000;
	unsigned long m_nCurrentMillis;
	unsigned long m_nShortTimeout;
	unsigned long m_nLongTimeout;
	byte m_nDisplayPriority = 0;

	CTimeoutUS m_timeoutLeftSensor;
	CTimeoutUS m_timeoutRightSensor;

protected:
	void readPotentiometers()
	{
		m_arrPot[POT_dataFarPot] = readADC(DataFarPot_pin);
		m_arrPot[POT_dataNearPot] = readADC(DataNearPot_pin);
		m_arrPot[POT_functionSelectPot] = readADC(FunctionPot_pin);
		m_arrPot[POT_keyPot] = readADC(KeyPot_pin);
		m_arrPot[POT_scalePot] = readADC(ScalePot_pin);
		m_arrPot[POT_octaveNearPot] = readADC(OctaveNearPot_pin);
		m_arrPot[POT_octaveFarPot] = readADC(OctaveFarPot_pin);
		m_arrPot[POT_articulationPot] = readADC(ArticulationPot_pin);
	}

	void setScale()
	{
		if(m_bDescending){
			m_arrMIDINotes[m_nNotesInCurrentScale] = m_nKeyCurrent + (m_nOctaveFarCurrent * 12);
			for (int note = m_nNotesInCurrentScale; note >= 0; note--) {
				m_arrMIDINotes[note - 1] = m_arrMIDINotes[note] + g_scales[m_nScaleCurrent][(m_nNotesInCurrentScale - note) % g_scales[m_nScaleCurrent][12]];
			}
		} else {
			m_arrMIDINotes[0] = m_nKeyCurrent + (m_nOctaveNearCurrent * 12);
			for (int note = 0; note <= m_nNotesInCurrentScale; note++) {
				m_arrMIDINotes[note + 1] = m_arrMIDINotes[note] + g_scales[m_nScaleCurrent][note % g_scales[m_nScaleCurrent][12]];
			}
		}
	}

	void setDisplay()
	{
		uint8_t arrDisplay[3];
		arrDisplay[0] = digitBitRemap(g_condisplayableCharacters[m_ledLeftDigit]);
		arrDisplay[1] = digitBitRemap(g_condisplayableCharacters[m_ledMiddleDigit]);
		arrDisplay[2] = digitBitRemap(g_condisplayableCharacters[m_ledRightDigit]);
		g_TM1637.displayRaw(sizeof(arrDisplay), arrDisplay);
	}

	void digitSplit2(int number)
	{
		m_ledMiddleDigit = (number < 10 ? 22 : (number/10) % 10);
		m_ledRightDigit = number % 10;
	}

	void digitSplit(int number)
	{
		m_ledLeftDigit = (number < 100 ? 22 : (number / 100) % 10 );
		digitSplit2(number);
	}

	void startTimerWithPriority(byte priority)
	{
		m_nCurrentMillis = millis();
		m_nShortTimeout = m_nCurrentMillis + m_nDisplayTimeout;
		m_nLongTimeout = m_nCurrentMillis + 60000;
		m_nDisplayPriority = priority;		//used to prevent certain sections from running until reverting to the default display
	}

	bool outsidePotBuffer(int oldValue, int newValue)
	{
		return ((oldValue >= (newValue + g_conADCThreshold)) || ((oldValue <= (newValue - g_conADCThreshold)) && (oldValue >= 0)));
	}

	void startupDisplay()
	{
		for (int k = 0; k < 3; k++) {
			pingRightSensor();
			for (int j = 36; j < 45; j++) {
				m_ledLeftDigit = j;
				m_ledMiddleDigit = j;
				m_ledRightDigit = j;
				setDisplay();
				delay(45);
			}
			m_nRightSensorProcessed = readRightSensor();
		}
	}

	void displayKeyAndMode()
	{
		digitSplit2(m_nScaleCurrent + 1);
		m_ledLeftDigit = m_nKeyCurrent + 10;
	}

	void checkScalePot()
	{
		static int nScaleOld = -1;
		if (m_arrPot[POT_scalePot] != nScaleOld) {
			m_nScaleCurrent = mapADCvalue(m_arrPot[POT_scalePot], 12);
			if (m_nScaleCurrent > 11) {
				m_nScaleCurrent = 11;
			}
			m_nNotesInCurrentScale = m_nNumberOfOctavesCurrent * g_scales[m_nScaleCurrent][12] ;
			m_nNoteBuffer =  g_conNoteBufferSpace / m_nNotesInCurrentScale;
			setScale();
			if (outsidePotBuffer(nScaleOld, m_arrPot[POT_scalePot]) && (m_nDisplayPriority < 3)) {
				startTimerWithPriority(2);
				displayKeyAndMode();
			}
			nScaleOld = m_arrPot[POT_scalePot];
		}
	}

	void checkKeyPot()
	{
		static int nKeyOld = -1;
		if (m_arrPot[POT_keyPot] != nKeyOld) {
			m_nKeyCurrent = mapADCvalue(m_arrPot[POT_keyPot], 12);
			if (m_nKeyCurrent > 11) {
				m_nKeyCurrent = 11;
			}
			setScale();

			if (outsidePotBuffer(nKeyOld, m_arrPot[POT_keyPot]) && (m_nDisplayPriority < 3)) {
				startTimerWithPriority(2);
				displayKeyAndMode();
			}
			nKeyOld = m_arrPot[POT_keyPot];
		}
	}

	void checkOctavePots()
	{
		static int nOctaveNearOld = -1;
		static int nOctaveFarOld = -1;
		static int nSlopeCurrent = 1;
		static int nSlopeOld = 1;
		if ((m_arrPot[POT_octaveNearPot] != nOctaveNearOld) ||
			(m_arrPot[POT_octaveFarPot] != nOctaveFarOld)) {
			if (m_arrPot[POT_octaveNearPot] != nOctaveNearOld) {
				m_nOctaveNearCurrent = mapADCvalue(m_arrPot[POT_octaveNearPot], m_conOctaveMax)+1;
				if (m_nOctaveNearCurrent > m_conOctaveMax) {
					m_nOctaveNearCurrent = m_conOctaveMax;
				}
			}
			if (m_arrPot[POT_octaveFarPot] != nOctaveFarOld){
				m_nOctaveFarCurrent = mapADCvalue(m_arrPot[POT_octaveFarPot], m_conOctaveMax)+1;
				if (m_nOctaveFarCurrent > m_conOctaveMax) {
					m_nOctaveFarCurrent = m_conOctaveMax;
				}
			}

			nSlopeCurrent = (m_nOctaveFarCurrent + m_conOctaveMax) - m_nOctaveNearCurrent;
			if (nSlopeCurrent == m_conOctaveMax) {
				nSlopeCurrent = nSlopeOld;
			}
			if (nSlopeCurrent < m_conOctaveMax) {
				m_bDescending = true;
			}
			if (nSlopeCurrent > m_conOctaveMax) {
				m_bDescending = false;
			}

			m_nNumberOfOctavesCurrent = max(m_nOctaveFarCurrent, m_nOctaveNearCurrent) -
										min(m_nOctaveFarCurrent, m_nOctaveNearCurrent) + 1;

			m_nNotesInCurrentScale = m_nNumberOfOctavesCurrent * g_scales[m_nScaleCurrent][12] ;
			m_nNoteBuffer = g_conNoteBufferSpace / m_nNotesInCurrentScale;
			setScale();
			if ((outsidePotBuffer(nOctaveFarOld, m_arrPot[POT_octaveFarPot]) ||
				outsidePotBuffer(nOctaveNearOld, m_arrPot[POT_octaveNearPot])) && (m_nDisplayPriority < 3) ) {
				startTimerWithPriority(2);
				m_ledLeftDigit = m_nOctaveNearCurrent;
				m_ledRightDigit = m_nOctaveFarCurrent;
				m_ledMiddleDigit = 22;
			}
			nOctaveNearOld = m_arrPot[POT_octaveNearPot];
			nOctaveFarOld = m_arrPot[POT_octaveFarPot];
			nSlopeOld = nSlopeCurrent;
		}
	}

	void checkFunctionPot()
	{
		static int nFunctionSelectOld = -1;
		if (m_arrPot[POT_functionSelectPot] != nFunctionSelectOld) {
			m_nFunctionSelectCurrent = mapADCvalue(m_arrPot[POT_functionSelectPot], 7)+1;
			if (m_nFunctionSelectCurrent > 7) {
				m_nFunctionSelectCurrent = 7;
			}
			if (outsidePotBuffer(nFunctionSelectOld, m_arrPot[POT_functionSelectPot])) {
				startTimerWithPriority(3);
				m_ledRightDigit = 22;
				m_ledMiddleDigit = 22;
				m_ledLeftDigit = m_nFunctionSelectCurrent;

				// Reset data to force it to adjust to the new setting
				m_nDataNearOld  = -1;
				m_nDataFarOld  = -1;
			}
			if ((m_nFunctionSelectCurrent == 6) && !m_bxyMode) {
				xyModeStart();
			}
			if ((m_nFunctionSelectCurrent != 6) && m_bxyMode) {
				xyModeStop();
			}
			nFunctionSelectOld = m_arrPot[POT_functionSelectPot];
		}
	}

	void checkDataPots()
	{
		if (m_arrPot[POT_dataNearPot] != m_nDataNearOld) {
			switch (m_nFunctionSelectCurrent) {
				case 1:
					m_nPitchBendNeutralZone = mapADCvalue(m_arrPot[POT_dataNearPot], 128);
					if (outsidePotBuffer(m_nDataNearOld, m_arrPot[POT_dataNearPot]) && (m_nDisplayPriority < 3)) {
						startTimerWithPriority(2);
						digitSplit(m_nPitchBendNeutralZone);
					}
					m_nPitchBendUp = 1700 + m_nPitchBendNeutralZone * 4;
					m_nPitchBendDown = 1700 - m_nPitchBendNeutralZone * 4;
					break;

				case 7:
					break;

				default:
					m_nDataNear = mapADCvalue(m_arrPot[POT_dataNearPot], 128);
					if (outsidePotBuffer(m_nDataNearOld, m_arrPot[POT_dataNearPot]) && (m_nDisplayPriority < 3)) {
						startTimerWithPriority(2);
						digitSplit(m_nDataNear);
					}
					break;
			}
			m_nDataNearOld = m_arrPot[POT_dataNearPot];
		}

		if (m_arrPot[POT_dataFarPot] != m_nDataFarOld) {
			switch (m_nFunctionSelectCurrent) {
				case 1:
					m_nDataFar = mapADCvalue(m_arrPot[POT_dataFarPot], 13);
					if (m_nDataFar > 12) {
						m_nDataFar = 12;
					}
					if (outsidePotBuffer(m_nDataFarOld, m_arrPot[POT_dataFarPot]) && (m_nDisplayPriority < 3)) {
						startTimerWithPriority(2);
						digitSplit(m_nDataFar);
					}
					MIDI.sendControlChange(20, m_nDataFar, m_nMIDIChannel);
					break;

				case 7:
					if (m_nDisplayPriority <= 1) {
						m_nMIDIChannel = mapADCvalue(m_arrPot[POT_dataFarPot], 16)+1;
						if (m_nMIDIChannel > 16) {
							m_nMIDIChannel = 16;
						}

						startTimerWithPriority(1);
						digitSplit(m_nMIDIChannel);
					}
					break;

				default:
					m_nDataFar = mapADCvalue(m_arrPot[POT_dataFarPot], 128);
					if (outsidePotBuffer(m_nDataFarOld, m_arrPot[POT_dataFarPot]) && (m_nDisplayPriority < 3)) {
						startTimerWithPriority(2);
						digitSplit(m_nDataFar);

					}
					break;
			}
			m_nDataFarOld  = m_arrPot[POT_dataFarPot];
		}
	}

	void xyModeStart()
	{
		m_bxyMode = true;
		m_nDisplayTimeout = 800;
	}

	void xyModeStop()
	{
		m_bxyMode = false;
		m_nDisplayTimeout = 2000;
	}

	void xyCheckControlPots(int nLeftControlPot, int nRightControlPot)
	{
		static int nLeftControlOld;
		static int nRightControlOld;
		if (nLeftControlOld != m_arrPot[nLeftControlPot]) {
			m_nxyLeftControlChange = mapADCvalue(m_arrPot[nLeftControlPot], 128);
			if (outsidePotBuffer(nLeftControlOld, m_arrPot[nLeftControlPot]) && (m_nDisplayPriority < 3)) {
				startTimerWithPriority(2);
				digitSplit(m_nxyLeftControlChange);
			}
		}
		if (nRightControlOld != m_arrPot[nRightControlPot]) {
			m_nxyRightControlChange = mapADCvalue(m_arrPot[nRightControlPot], 128);
			if (outsidePotBuffer(nRightControlOld, m_arrPot[nRightControlPot]) && (m_nDisplayPriority < 3)) {
				startTimerWithPriority(2);
				digitSplit(m_nxyRightControlChange);
			}
		}
		nLeftControlOld = m_arrPot[nLeftControlPot];
		nRightControlOld = m_arrPot[nRightControlPot];
	}

	void xyCheckRightDataPots(int nRightDataNearPot, int nRightDataFarPot)
	{
		static int nRightDataFarOld = -1;
		static int nRightDataNearOld = -1;
		if (nRightDataFarOld != m_arrPot[nRightDataFarPot]) {
			m_nxyDataFarRight = mapADCvalue(m_arrPot[nRightDataFarPot], 128);
			if (outsidePotBuffer(nRightDataFarOld , m_arrPot[nRightDataFarPot]) && (m_nDisplayPriority < 3)) {
				startTimerWithPriority(2);
				digitSplit(m_nxyDataFarRight);
			}
		}
		if (nRightDataNearOld != m_arrPot[nRightDataNearPot]) {
			m_nxyDataNearRight = mapADCvalue(m_arrPot[nRightDataNearPot], 128);
			if (outsidePotBuffer(nRightDataNearOld , m_arrPot[nRightDataNearPot]) && (m_nDisplayPriority < 3)) {
				startTimerWithPriority(2);
				digitSplit(m_nxyDataNearRight);
			}
		}
		nRightDataFarOld  = m_arrPot[nRightDataFarPot];
		nRightDataNearOld = m_arrPot[nRightDataNearPot];
	}

	void checkPots()
	{
		checkScalePot();
		checkKeyPot();
		checkOctavePots();
		checkFunctionPot();
		checkDataPots();
	}

	void xyCheckPots()
	{
		xyCheckControlPots(POT_keyPot, POT_scalePot);
		checkDataPots();
		xyCheckRightDataPots(POT_octaveNearPot, POT_octaveFarPot);
		checkFunctionPot();
	}

	void defaultDisplay()
	{
		digitSplit2(m_nScaleCurrent + 1);
		m_ledLeftDigit = m_nKeyCurrent + 10;
		m_nDisplayPriority = 0;
	}

	void wipeDisplay()
	{
		m_ledLeftDigit = 22;
		m_ledMiddleDigit = 22;
		m_ledRightDigit = 22;
	}

	void checkTimeouts()
	{
		m_nCurrentMillis = millis();
		if (m_nLongTimeout < m_nCurrentMillis) {
			wipeDisplay();
		} else if (m_nShortTimeout < m_nCurrentMillis) {
			defaultDisplay();
			if (m_bxyMode) {
				m_ledLeftDigit = 47;
				m_ledMiddleDigit = 47;
				m_ledRightDigit = 47;
			}
		}
	}

	void checkArticulation()
	{
		static int nFastActionRatioOld;
		if (m_bArticulationMode) {
			if (m_arrPot[POT_articulationPot] != nFastActionRatioOld) {
				m_nFastActionRatio = mapADCvalue(m_arrPot[POT_articulationPot], 16)+1;
				nFastActionRatioOld = m_arrPot[POT_articulationPot];
				digitSplit(m_nFastActionRatio * 15);
				startTimerWithPriority(3);
			}
		}
	}

	long sensorConstrain(long nReading)
	{
		if (nReading == 0) return 0;
		if (nReading <= g_conMinimumDistance) return g_conMinimumDistance;
		if (nReading >= g_conMaximumDistance) return g_conMaximumDistance;
		return nReading;
	}

	void pingLeftSensor()
	{
		g_bLeftSensorRead = false;
		#if (SWAP_SENSORS)
		uint8_t nTrigger = RightTrigger_pin;
		#else
		uint8_t nTrigger = LeftTrigger_pin;
		#endif
		digitalWrite(nTrigger, LOW);
		delayMicroseconds(2);
		digitalWrite(nTrigger, HIGH);
		delayMicroseconds(10);
		digitalWrite(nTrigger, LOW);
		m_timeoutLeftSensor.restart();
	}

	void pingRightSensor()
	{
		g_bRightSensorRead = false;
		#if (SWAP_SENSORS)
		uint8_t nTrigger = LeftTrigger_pin;
		#else
		uint8_t nTrigger = RightTrigger_pin;
		#endif
		digitalWrite(nTrigger, LOW);
		delayMicroseconds(2);
		digitalWrite(nTrigger, HIGH);
		delayMicroseconds(10);
		digitalWrite(nTrigger, LOW);
		m_timeoutRightSensor.restart();
	}

	long stabilizeLeftReadings(long nReading)
	{
		static byte nPointer = 0;
		static int arrLeftReadings[g_conLeftHandBufferAmount] = { };

		if (nReading == 0) return nReading;

		arrLeftReadings[nPointer] = nReading;
		++nPointer;
		if (nPointer >= g_conLeftHandBufferAmount) nPointer = 0;

		int nReadingsTotal = 0;
		for (int j = 0; j < g_conLeftHandBufferAmount; j++) {
			nReadingsTotal = nReadingsTotal + arrLeftReadings[j];
		}
		return nReadingsTotal / g_conLeftHandBufferAmount;
	}

	long stabilizeRightReadings(long nReading)
	{
		static byte nPointer = 0;
		static int arrRightReadings[g_conRightHandBufferAmount] = { };

		if (nReading == 0) return nReading;

		arrRightReadings[nPointer] = nReading;
		++nPointer;
		if (nPointer >= g_conRightHandBufferAmount) nPointer = 0;

		int nReadingsTotal = 0;
		for (int j = 0; j < g_conRightHandBufferAmount; j++) {
			nReadingsTotal = nReadingsTotal + arrRightReadings[j];
		}
		return nReadingsTotal / g_conRightHandBufferAmount;
	}

	void handleVelocity()
	{
		if (m_nLeftSensorProcessed != 0) {
			m_nNoteVelocity = map(m_nLeftSensorProcessed, g_conMinimumDistance, g_conMaximumDistance, m_nDataNear, m_nDataFar);

			digitSplit(m_nNoteVelocity);
			startTimerWithPriority(1);
		}
	}

	void handlePitchBend()
	{
		static int  nPitchBendOld = 0;
		static byte nOutOfRangeL = 0;
		static byte nSpinDial = 29;
		int nPitchBend = 0;
		if (m_nPortamentoTime != 0) {
			m_bPortamentoOn = false;
			m_nPortamentoTime = 0;
			MIDI.sendControlChange(5, m_nPortamentoTime, m_nMIDIChannel);
			MIDI.sendControlChange (65, 0, m_nMIDIChannel);
		}
		if (m_nLeftSensorProcessed > m_nPitchBendUp) {
			nPitchBend = map(m_nLeftSensorProcessed, m_nPitchBendUp, g_conMaximumDistance, 0, -1023);
		} else if (m_nLeftSensorProcessed < m_nPitchBendDown) {
			nPitchBend = map(m_nLeftSensorProcessed, g_conMinimumDistance, m_nPitchBendDown, 1023, 0);
		} else {
			nPitchBend = 0;
		}

		if (m_nLeftSensorProcessed == 0) {
			if (nOutOfRangeL < 16) {
				nOutOfRangeL++;
			}
			if (nOutOfRangeL == 15) {
				m_ledRightDigit = 29;
				m_ledMiddleDigit = 22;
				m_ledLeftDigit = 22;
				startTimerWithPriority(1);

				MIDI.sendPitchBend(0, m_nMIDIChannel);
			}
		} else {
			if (nPitchBend != nPitchBendOld) {
				if (nPitchBend > 0) {
					nSpinDial = map(constrain(nPitchBend, 0, 1023), 1023, 0 , 34, 29);
					if (nPitchBend == 1023) {
						++nSpinDial;
					}
				} else if (nPitchBend < 0) {
					nSpinDial = map(constrain(nPitchBend, -1023, 0), -1023, 0, 24, 29);
					if (nPitchBend == -1023) {
						--nSpinDial;
					}
				} else {
					nSpinDial = 29;
				}
				m_ledRightDigit = nSpinDial;
				m_ledMiddleDigit = 22;
				m_ledLeftDigit = 22;
				startTimerWithPriority(1);

				nPitchBendOld = nPitchBend;
				nPitchBend = nPitchBend * 8;
				MIDI.sendPitchBend(nPitchBend, m_nMIDIChannel);

				nOutOfRangeL = 0;
			}
		}
	}

	void handleVolume()
	{
		static byte nChannelVolumeOld = 127;
		if (m_nLeftSensorProcessed != 0) {
			byte nChannelVolume = map(m_nLeftSensorProcessed, g_conMinimumDistance, g_conMaximumDistance, m_nDataNear, m_nDataFar);
			if (nChannelVolume != nChannelVolumeOld) {
				digitSplit(nChannelVolume);
				startTimerWithPriority(1);
				MIDI.sendControlChange(7, nChannelVolume, m_nMIDIChannel);
				nChannelVolumeOld = nChannelVolume;
			}
		}
	}

	void handleModulation()
	{
		static byte nModulationOld = 0;
		if (m_nLeftSensorProcessed != 0) {
			byte nModulation = map(m_nLeftSensorProcessed, g_conMinimumDistance, g_conMaximumDistance, m_nDataNear, m_nDataFar);
			if (nModulation != nModulationOld) {
				digitSplit(nModulation);
				startTimerWithPriority(1);
				MIDI.sendControlChange(1, nModulation, m_nMIDIChannel);
				nModulationOld = nModulation;
			}
		}
	}

	void handlePortamento()
	{
		static byte nPortamentoTimeOld = 0;
		if (m_nLeftSensorProcessed !=0) {
			m_nPortamentoTime = map(m_nLeftSensorProcessed, g_conMinimumDistance, g_conMaximumDistance, m_nDataNear, m_nDataFar);
			if (m_nPortamentoTime != nPortamentoTimeOld) {
				digitSplit(m_nPortamentoTime);
				startTimerWithPriority(1);
				MIDI.sendControlChange(5, m_nPortamentoTime, m_nMIDIChannel);
				if (m_nPortamentoTime == 0) {
					MIDI.sendControlChange(65, 0, m_nMIDIChannel);
					if (m_bPortamentoOn) {
						m_bPortamentoOn = false;
					}
				}
				if (!m_bPortamentoOn && (m_nPortamentoTime != 0)) {
					MIDI.sendControlChange(65, 127, m_nMIDIChannel);
					m_bPortamentoOn=true;
				}
				nPortamentoTimeOld = m_nPortamentoTime;
			}
		}
	}

	void handleLeftSensor()
	{
		if (m_nDisplayPriority < 2) {
			switch (m_nFunctionSelectCurrent) {
				case 1:
					handlePitchBend();
					break;
				case 2:
					handleModulation();
					break;
				case 3:
					handleVelocity();
					break;
				case 4:
					handleVolume();
					break;
				case 5:
					handlePortamento();
					break;
				case 6:
					xyHandleLeftSensor();
					break;
				default:
					break;
			}
		}
	}

	void xyHandleLeftSensor()
	{
		static byte nLastValue = -1;
		if (m_nLeftSensorProcessed > 0) {
			byte nDataLeft = map(m_nLeftSensorProcessed, g_conMinimumDistance, g_conMaximumDistance, m_nDataNear, m_nDataFar);
			if ((nDataLeft != nLastValue) && (m_nDisplayPriority < 1)) {
				digitSplit(nDataLeft);
				startTimerWithPriority(0);
			}
			MIDI.sendControlChange(m_nxyLeftControlChange, nDataLeft, m_nMIDIChannel);
			nLastValue = nDataLeft;
		}
	}

	void xyHandleRightSensor()
	{
		static byte nLastValue = -1;
		if (m_nDisplayPriority < 3) {
			if (m_nRightSensorProcessed > 0) {
				byte nDataRight = map(m_nRightSensorProcessed, g_conMinimumDistance, g_conMaximumDistance, m_nxyDataNearRight, m_nxyDataFarRight);
				if ((nDataRight != nLastValue) && (m_nDisplayPriority < 1)) {
					digitSplit(nDataRight);
					startTimerWithPriority(0);
				}
				MIDI.sendControlChange(m_nxyRightControlChange, nDataRight, m_nMIDIChannel);
				nLastValue = nDataRight;
				if ((m_nLeftSensorProcessed > 0) && (m_nRightSensorProcessed > 0)) {
					m_nDisplayTimeout = 200;
					m_ledLeftDigit = 48;
					m_ledMiddleDigit = 47;
					m_ledRightDigit = 49;
					startTimerWithPriority(1);
					m_nDisplayTimeout = 800;
				}
			}
		}
	}

	void handleRightSensor(long sensorReading)
	{
		static bool bNotePlaying = false;
		static byte nCurrentNote;
		static byte nOldNote;
		static byte nOutOfRange = 0;

		if (sensorReading == 0) {
			m_nLastNote = -10;

			if (nOutOfRange <= 5) {
				++nOutOfRange;
			}
			if ((nOutOfRange > 5) && bNotePlaying) {
				MIDI.sendNoteOn(nOldNote, 0, m_nMIDIChannel);
				nOutOfRange = 0;
				bNotePlaying = false;
			}
		} else {
			nOutOfRange = 0;
			m_nCurrentMillis = millis();
			m_nLongTimeout = m_nCurrentMillis + 60000;
			byte newNote = map(sensorReading, g_conMinimumDistance, g_conMaximumDistance, 0, m_nNotesInCurrentScale + 1);
			if (newNote > m_nNotesInCurrentScale) newNote=m_nNotesInCurrentScale;
			m_nLastNote = newNote;
			nCurrentNote = m_arrMIDINotes[newNote];

			if (!bNotePlaying) {
				bNotePlaying = true;
				MIDI.sendNoteOn(nCurrentNote, m_nNoteVelocity, m_nMIDIChannel);
				nOldNote = nCurrentNote;
			} else {
				if (nCurrentNote != nOldNote) {
					MIDI.sendNoteOn(nOldNote, 0, m_nMIDIChannel);
					MIDI.sendNoteOn(nCurrentNote, m_nNoteVelocity, m_nMIDIChannel);
					bNotePlaying = true;
					nOldNote = nCurrentNote;
				}
			}
		}
	}

	long checkNoteBuffer(long reading)
	{
		if ((reading > (map(m_nLastNote + 1, 0, m_nNotesInCurrentScale + 1, g_conMinimumDistance, g_conMaximumDistance) + m_nNoteBuffer)) ||
			(reading < (map(m_nLastNote, 0, m_nNotesInCurrentScale + 1, g_conMinimumDistance, g_conMaximumDistance) - m_nNoteBuffer))) {
			m_nRightSensorProcessed = reading;
		}
		return m_nRightSensorProcessed;
	}

	long readLeftSensor()
	{
		while (!m_timeoutLeftSensor.hasExpired() && !g_bLeftSensorRead) { }
		if (g_bLeftSensorRead) {
			return sensorConstrain(g_nLeftSensorEchoTime);
		}
		return 0;
	}

	long readRightSensor()
	{
		while (!m_timeoutRightSensor.hasExpired() && !g_bRightSensorRead) { }
		if (g_bRightSensorRead) {
			return checkNoteBuffer(sensorConstrain(g_nRightSensorEchoTime));
		}
		return 0;
	}

	void initializeArticulation()
	{
		for (int i = 0; i < 7; ++i) {
			if (m_arrPot[i] >= 10) {
				m_bArticulationMode = false;
			}
		}
		m_nFastActionRatio = mapADCvalue(m_arrPot[POT_articulationPot], 16)+1;
	}

public:
	CAltura()
		:	m_timeoutLeftSensor(g_conSensorTimeOut),
			m_timeoutRightSensor(g_conSensorTimeOut)
	{ }

	void init()
	{
		//Clear MIDI buffer upon startup
		byte startupBuffer[2] = {0,0};

		MIDI.sendSysEx(2, startupBuffer, false);
		MIDI.sendProgramChange(81, m_nMIDIChannel);		// Typically a square lead, appropriate to a theremin

		readPotentiometers();
		initializeArticulation();
		if (m_bArticulationMode) {	// Display the software version (2.1.2)
			m_ledLeftDigit = 45;
			m_ledMiddleDigit = 46;
			m_ledRightDigit = 2;
			setDisplay();
			delay(1500);
		} else {
			startupDisplay();
		}
		startTimerWithPriority(2);
	}

	void runFastActions()
	{
		for (byte i = m_nFastActionRatio; i > 0; --i) {
			checkArticulation();
			setDisplay();
			delay(8);		// This ,combined with the above "for loop", is the main driving point for timing
			pingLeftSensor();
			delay(8);
			pingRightSensor();
			delay(8);
			m_nLeftSensorProcessed = stabilizeLeftReadings(readLeftSensor());
			handleLeftSensor();

			if (m_bxyMode) {
				m_nRightSensorProcessed = stabilizeRightReadings(readRightSensor());
				xyHandleRightSensor();
				xyCheckPots();
			}
		}
	}

	void runSlowActions()
	{
		readPotentiometers();
		checkTimeouts();
		if (!m_bxyMode) {
			pingRightSensor();
			checkPots();
			handleRightSensor(readRightSensor());
		}
	}
} g_Altura;

const byte CAltura::g_scales[12][13] = {       //The value in the last column indicates the number of notes in the scale.
	{ 2, 2, 1, 2, 2, 2, 1, 0, 0, 0, 0, 0, 7 }, // Ionian Mode (Major)
	{ 2, 1, 2, 2, 2, 1, 2, 0, 0, 0, 0, 0, 7 }, // Dorian Mode
	{ 1, 2, 2, 2, 1, 2, 2, 0, 0, 0, 0, 0, 7 }, // Phrygian Mode
	{ 2, 2, 2, 1, 2, 2, 1, 0, 0, 0, 0, 0, 7 }, // Lydian Mode
	{ 2, 2, 1, 2, 2, 1, 2, 0, 0, 0, 0, 0, 7 }, // Mixolydian
	{ 2, 1, 2, 2, 1, 2, 2, 0, 0, 0, 0, 0, 7 }, // Aeolian Mode (Natural Minor)
	{ 1, 2, 2, 1, 2, 2, 2, 0, 0, 0, 0, 0, 7 }, // Locrian Mode
	{ 2, 1, 2, 2, 1, 3, 1, 0, 0, 0, 0, 0, 7 }, // Harmonic Minor
	{ 2, 2, 3, 2, 3, 0, 0, 0, 0, 0, 0, 0, 5 }, // Major Pentatonic
	{ 3, 2, 2, 3, 2, 0, 0, 0, 0, 0, 0, 0, 5 }, // Minor Pentatonic
	{ 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 6 }, // Whole Tone
	{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 12}, // Chromatic
};

// ============================================================================

void setup()
{
	GPIO_Init();
	ADC_Init();
	Timer_Init();
	SPI_Init();
	I2C_Init();

	// Initialize USART buffer
	g_USARTMIDI.init();
	g_USARTMIDI.clearBufferData();

	// Start USB Subsystem:
	g_MyUSBComposite.init();

	// Enable global interrupts
	interrupts();
	delay(2000);

	// Flash, then Disable LEDs:
	flashStatusLED(2);
	disableStatusLED();

	// Initialize Display:
	Display_Init();

	// Intialize Theremin:
	g_Altura.init();
}

// ============================================================================

void loop()
{
	g_Altura.runSlowActions();
	g_Altura.runFastActions();
}

// ============================================================================
