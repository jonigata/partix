// 2017/06/04 Naoyuki Hirayama

/*!
	@file	  FileLogger.hpp
	@brief	  <ŠT—v>

	<à–¾>
*/

#ifndef FILELOGGER_HPP_
#define FILELOGGER_HPP_

void DebugLog(const char*);

template <class T>
void DebugLogAny(T x) {}

template<> inline void DebugLogAny(int x) {
    char buffer[256];
    sprintf(buffer, "%d", x);
    DebugLog(buffer);
}

template<> inline void DebugLogAny(float x) {
    char buffer[256];
    sprintf(buffer, "%f", x);
    DebugLog(buffer);
}

template<class T> inline void DebugLogPointer(T* x) {
    char buffer[256];
    sprintf(buffer, "%p", x);
    DebugLog(buffer);
}

#endif // FILELOGGER_HPP_
