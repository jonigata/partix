/*!
  @file     dprintf.hpp
  @brief    <概要>

  <説明>
  $Id: dprintf.hpp 16 2008-03-28 14:58:11Z Naoyuki.Hirayama $
*/
#ifndef DPRINTF_HPP
#define DPRINTF_HPP

#include <windows.h>
#include <stdio.h>
#include <stdarg.h>

// Visual Studio のデバッグウィンドウの横幅に対応する文字数(バイト数ではない)
#define DPRINTF_MES_LENGTH 1024

// デバッグビルドのときだけ，VCの出力窓にメッセージを出す．MFCのTRACEやATLのATLTRACEとおなじ．
inline void dprintf_real( const char * fmt, ... )
{
        char buf[DPRINTF_MES_LENGTH];
        va_list ap;
        va_start(ap, fmt);
        _vsnprintf(buf, DPRINTF_MES_LENGTH, fmt, ap);
        va_end(ap);
        OutputDebugStringA(buf);
}

#ifdef _DEBUG
#  define dprintf dprintf_real
#else
#  define dprintf __noop
#endif

#endif // DPRINTF_HPP
