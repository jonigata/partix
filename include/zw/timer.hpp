/*!
  @file     timer.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: timer.hpp 15 2008-03-21 09:46:26Z Naoyuki.Hirayama $
*/
#ifndef TIMER_HPP
#define TIMER_HPP

#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

namespace zw {

class timer {
public:
	timer( int fps = 60 )
	{
		fps_ = fps;
		timeBeginPeriod(1);
		first_ = last_ = timeGetTime();

		span_ = 0;
		counter_ = 0;
		realfps_ = 0;
	}
	~timer()
	{
		timeEndPeriod(1);
	}

	template <class U>
	bool operator()( U u )
	{
		DWORD c = timeGetTime();
		DWORD d0 = last_ - first_;
		DWORD d1 = c - first_;

		if( span_ < d0 / 1000 ) {
			span_ = d0 / 1000;
			realfps_ = counter_;
			counter_ = 0;
		}

		DWORD dd0 = d0 * fps_ / 1000;
		DWORD dd1 = d1 * fps_ / 1000;
		if( dd0 < dd1 ) {
			// update
			DWORD elapsed0 = c - last_;
			float elapsed1 = elapsed0 * 0.001f;
			u( elapsed0, elapsed1 );
			last_ = c;
			counter_++;
			return true;
		}
		return false;
	}

	int realfps() { return realfps_; }
        
private:
	int fps_;
	DWORD first_;
	DWORD last_;
        
	DWORD span_;
	DWORD counter_;
	DWORD realfps_;
};

}


#endif // TIMER_HPP
