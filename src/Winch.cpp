/*
 * Winch.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: Developer
 */

#include <Winch.h>

Winch::Winch(frc::PowerDistributionPanel* pPDP)
{
	m_pPDP = pPDP;
}

Winch::~Winch()
{
}

void Winch::Stop()
{
	m_winchMotor.Set(0.0);
	m_winchCurrent = m_pPDP->GetCurrent(PDP_CHANEL_WINCH);
}

void Winch::Raise(bool bFastSpeed)
{
	if (bFastSpeed)
		m_winchMotor.Set(RAISE_FAST_SPEED);
	else
		m_winchMotor.Set(RAISE_SLOW_SPEED);

	m_winchCurrent = m_pPDP->GetCurrent(PDP_CHANEL_WINCH);
}

void Winch::Lower()
{
	m_winchMotor.Set(LOWER_SPEED);
	m_winchCurrent = m_pPDP->GetCurrent(PDP_CHANEL_WINCH);
}

bool Winch::IsStalled()
{
	m_winchCurrent = m_pPDP->GetCurrent(PDP_CHANEL_WINCH);
	return (m_winchCurrent > STALL_CURRENT_WINCH);

}
