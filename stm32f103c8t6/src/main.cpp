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

#define Q(text) #text
#define QUOTE(text) Q(text)

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
#define ArticulationPot_pin	PA7

#define TM1637_CLK			PB10		// I2C2_SCL
#define TM1637_DIO			PB11		// I2C2_SDA

#define SD_CS_pin		PB12
#define SD_Card_Ins_pin	PA8
#define SD_Card_Ins_Active	false	// Level of active state of Card Inserted pin


#define LED1_pin	PC13		// General status LED (active low)

#define USART_tx_pin	PA9
#define USART_rx_pin	PA10

SPIClass g_SPI2(2);

constexpr uint32_t TIMER_CLOCK_HZ = F_CPU;                      // Timer Clock input frequency
constexpr uint32_t TIMER_PRESCALER = 45ul;                      // Prescaler in Timer Clock cycles

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
	getADC()->regs->CR2 = ADC_CR2_CONT | ADC_CR2_ALIGN | ADC_CR2_JEXTSEL_JSWSTART | ADC_CR2_EXTSEL_SWSTART;		// AD on
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
	getADC()->regs->SR = 0;					// Clear Flags
	getADC()->regs->SQR3 = PIN_MAP[ArticulationPot_pin].adc_channel;	// Select correct channel (lower 4 bits is channel)
	adc_calibrate(getADC());					// Calibrate the ADC
	getADC()->regs->CR2 |=  ADC_CR2_SWSTART;	// Start first conversion
}

// ============================================================================

// Initialize Timer Modules
void Timer_Init()
{
	// Enable RCC clocks:
	timer_init(TIMER2);				// Timer2 is the Left Ultrasonic Echo
	timer_init(TIMER3);				// Timer3 is the Right Ultrasonic Echo

	// Setup Timer Prescalers:
	(TIMER2->regs).gen->PSC = (TIMER_PRESCALER - 1);
	(TIMER3->regs).gen->PSC = (TIMER_PRESCALER - 1);

	// Disable Output Compare Output generation setup by default configuration:
	(TIMER2->regs).gen->CCMR1 = 0;
	(TIMER2->regs).gen->CCMR2 = 0;
	(TIMER2->regs).gen->CCER = 0;
	(TIMER3->regs).gen->CCMR1 = 0;
	(TIMER3->regs).gen->CCMR2 = 0;
	(TIMER3->regs).gen->CCER = 0;

	// Disable ARPE, Dir=Up, Counter Enable:
	(TIMER2->regs).gen->CR1 = TIMER_CR1_CEN;
	(TIMER3->regs).gen->CR1 = TIMER_CR1_CEN;

	// General Timers:
	// ---------------
	// Timer2:
	(TIMER2->regs).gen->DIER = 0;
	(TIMER2->regs).gen->SR = 0;							// Clear any current IRQs
	nvic_irq_enable(NVIC_TIMER2);						// Enable IRQ callback in NVIC
//	if (SYSTEM_TIME_TIMER_BASE == TIMER2->regs.gen) {
//		(TIMER2->regs).gen->DIER |= TIMER_DIER_UIE;		// Start Timer 2 update IRQ for timing purposes
//	}

	// Timer3:
	(TIMER3->regs).gen->DIER = 0;
	(TIMER3->regs).gen->SR = 0;							// Clear any current IRQs
	nvic_irq_enable(NVIC_TIMER3);						// Enable IRQ callback in NVIC
//	if (SYSTEM_TIME_TIMER_BASE == TIMER3->regs.gen) {
//		(TIMER3->regs).gen->DIER |= TIMER_DIER_UIE;		// Start Timer 3 update IRQ for timing purposes
//	}
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

TM1637 g_TM1637(TM1637_CLK, TM1637_DIO);

void Display_Init()
{
	pinMode(TM1637_CLK, OUTPUT);
	pinMode(TM1637_DIO, OUTPUT);

	g_TM1637.init();
}

// ============================================================================


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

	// Disable LEDs:
	disableStatusLED();

	// Initialize Display:
	Display_Init();
}

void loop()
{

}
