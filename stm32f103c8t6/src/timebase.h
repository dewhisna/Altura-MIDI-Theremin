//
// System Timebase Constants
//

#ifndef TIMEBASE_H_
#define TIMEBASE_H_

#include <stdint.h>

#include <libmaple/timer.h>

// This MUST be the BASE address of the timer and be a define (not a global static).
// 	That is necessary because of the startup initialization order of global variables,
//	as we can't otherwise guarantee that this variable will initialize before any of
//	the other timer constants.  So, to make the timing work correctly when called
//	in global constructors, this must point to the raw base address of the peripheral!
#define SYSTEM_TIME_TIMER_BASE	TIMER4_BASE				// Must be TIMER2, TIMER3, TIMER4, or TIMER5

// ============================================================================

#if defined(__ARMEL__)
union ToverflowCounter
{
	struct
	{
		uint16_t m_nLow;
		uint16_t m_nHigh;
    } split;

	uint32_t m_nValue = 0;
};
#elif defined(__ARMEB__)
union ToverflowCounter
{
	struct
	{
		uint16_t m_nHigh;
		uint16_t m_nLow;
    } split;

	uint32_t m_nValue = 0;
};
#else
#error Unknown architecture
#endif

// ============================================================================

constexpr uint32_t TIMER_CLOCK_HZ = F_CPU;			// Timer Clock input frequency
constexpr uint32_t TIMER_PRESCALER = 72ul;			// Prescaler in Timer Clock cycles (Use 1MHz to measure number of microseconds on inputs)

extern volatile ToverflowCounter g_overflowCounter;

// ----------------------------------------------------------------------------

extern uint32_t SystemTime_CurrentTicks32();
inline uint16_t SystemTime_CurrentTicks16() { return SYSTEM_TIME_TIMER_BASE->CNT; }

// This function must be called by Timer Interrupt handlers!
inline void handleSystemTimeTick()
{
	++g_overflowCounter.split.m_nHigh;
	g_overflowCounter.split.m_nLow = 0;
}

inline uint32_t CPU_MillisecondsToTicks( uint32_t nMillis )
{
	uint64_t nTicks = nMillis;
	nTicks *= (TIMER_CLOCK_HZ/TIMER_PRESCALER);
	nTicks /= 1000ul;
	return static_cast<uint32_t>(nTicks);
}

inline uint32_t CPU_MicrosecondsToTicks( uint32_t uSec )
{
	uint64_t nTicks = uSec;
	nTicks *= (TIMER_CLOCK_HZ/TIMER_PRESCALER);
	nTicks /= 1000000ul;
	return static_cast<uint32_t>(nTicks);
}

// ============================================================================

#endif	// TIMEBASE_H_

