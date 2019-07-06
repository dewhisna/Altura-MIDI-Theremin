//
// Timeout Helper Class
//

#ifndef TIMEOUT_H_
#define TIMEOUT_H_

#include "timebase.h"

#include <stdint.h>

#define TICKS_MAX	__UINT32_MAX__
#ifndef __UINT32_MAX__
#error "Compiler didn't define __UINT32_MAX__"
#endif

// ============================================================================

class CTimeout
{
public:
	CTimeout(uint32_t nTicks = 0, bool bDeltaMode = false)
		:	m_nLastClock(SystemTime_CurrentTicks32()),
			m_nTimeoutTicks(nTicks),
			m_nTicks(nTicks),
			m_bDeltaMode(bDeltaMode)
	{ }

	virtual bool hasExpired(bool bAutoReload = false)
	{
		uint32_t nCurrentClock = SystemTime_CurrentTicks32();
		if (!m_bDeltaMode) {
			uint32_t nTickDelta = (nCurrentClock - m_nLastClock);
			m_nLastClock = nCurrentClock;
			if (nTickDelta >= m_nTicks) {
				if (bAutoReload) {
					m_nTicks = m_nTimeoutTicks;
				} else {
					m_nTicks = 0;
				}
				return true;
			} else {
				m_nTicks -= nTickDelta;
			}
		} else {
			nCurrentClock -= m_nLastClock;
			if (nCurrentClock >= m_nTicks) {
				if (bAutoReload) m_nTicks += m_nTimeoutTicks;
				return true;
			}
		}
		return false;
	}

	virtual void restart(uint32_t nNewTicks = TICKS_MAX, bool bRebase = true)		// bRebase causes us to reset what zero time is
	{
		if (nNewTicks != TICKS_MAX) m_nTimeoutTicks = nNewTicks;
		if (!m_bDeltaMode) {
			m_nTicks = m_nTimeoutTicks;
			if (bRebase) {
				m_nLastClock = SystemTime_CurrentTicks32();
			}
		} else {
			if (bRebase) {
				m_nLastClock = SystemTime_CurrentTicks32();
				m_nTicks = m_nTimeoutTicks;
			} else {
				m_nTicks += m_nTimeoutTicks;
			}
		}
	}

private:
	uint32_t m_nLastClock;			// Current Clock Tick value
	uint32_t m_nTimeoutTicks;		// Number of Ticks we start with (should be considered const, except for restart() function changes)
	uint32_t m_nTicks;				// Number of Ticks remaining until expiration
	bool m_bDeltaMode;				// If True, then the m_nTicks isn't decremented and m_nLastClock doesn't update, allowing next timeout to be the sum of the previous timeouts
};

class CTimeoutMS : public CTimeout
{
public:
	CTimeoutMS(uint32_t nMilliseconds = 0, bool bDeltaMode = false)
		:	CTimeout(CPU_MillisecondsToTicks(nMilliseconds), bDeltaMode)
	{ }

	virtual void restart(uint32_t nNewMilliseconds = TICKS_MAX, bool bRebase = true)
	{
		CTimeout::restart((nNewMilliseconds != TICKS_MAX) ? CPU_MillisecondsToTicks(nNewMilliseconds) : TICKS_MAX, bRebase);
	}
};

class CTimeoutUS : public CTimeout
{
public:
	CTimeoutUS(uint32_t nMicroseconds = 0, bool bDeltaMode = false)
		:	CTimeout(CPU_MicrosecondsToTicks(nMicroseconds), bDeltaMode)
	{ }

	virtual void restart(uint32_t nNewMicroseconds = TICKS_MAX, bool bRebase = true)
	{
		CTimeout::restart((nNewMicroseconds != TICKS_MAX) ? CPU_MicrosecondsToTicks(nNewMicroseconds) : TICKS_MAX, bRebase);
	}
};

// ============================================================================

#endif	// TIMEOUT_H_

