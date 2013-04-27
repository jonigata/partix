/*!
  @file     performance_counter.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: performance_counter.hpp 53 2007-04-05 04:30:51Z hirayama $
*/
#ifndef PERFORMANCE_COUNTER_HPP
#define PERFORMANCE_COUNTER_HPP

#include <cstdio>

class PerformanceCounter {
public:
    PerformanceCounter( bool print_on = true )
    {
#ifdef _WINDOWS
        print_on_ = print_on;
        QueryPerformanceCounter( &liStart );
#endif
    }
    ~PerformanceCounter() {}

    double operator()()
    {
#ifdef _WINDOWS
        LARGE_INTEGER liFinish = {0, 0};
        LARGE_INTEGER freq = {0, 0};
        QueryPerformanceCounter( &liFinish );
        QueryPerformanceFrequency( &freq );
        double time =
            (double)(liFinish.LowPart - liStart.LowPart) /
            (double)freq.LowPart;
        liStart = liFinish;
        return time;
#else
        return 0;
#endif
    }

    void print( const char* s )
    {
#ifdef _WINDOWS
        char buffer [256];
        if( print_on_ ) {
            sprintf( buffer, "%s: %f\n", s, operator()() );
            OutputDebugStringA( buffer );
        }
        operator()();
#endif
    }

private:
#ifdef _WINDOWS
    LARGE_INTEGER   liStart;
    bool            print_on_;
#endif

};


#endif // PERFORMANCE_COUNTER_HPP
