//
// System Timebase Functions
//

#include "timebase.h"

// ============================================================================

volatile ToverflowCounter g_overflowCounter;

uint32_t SystemTime_CurrentTicks32()
{
	ToverflowCounter nTemp;
	nTemp.split.m_nHigh = g_overflowCounter.split.m_nHigh;			// Get the high
	nTemp.split.m_nLow = SystemTime_CurrentTicks16();				// Capture the real low
	if (nTemp.split.m_nHigh != g_overflowCounter.split.m_nHigh) {
		// If an interrupt just happened then the high word won't
		//	match (and we don't know if we read the low word before
		//	or after the interrupt).
		// So swap out for what the interrupt just captured:
		nTemp.m_nValue = g_overflowCounter.m_nValue;
	}
	 return nTemp.m_nValue;
}

// ============================================================================
