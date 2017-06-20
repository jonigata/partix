/*!
  @file     dprintf.hpp
  @brief    <�T�v>

  <����>
  $Id: dprintf.hpp 16 2008-03-28 14:58:11Z Naoyuki.Hirayama $
*/
#ifndef DPRINTF_HPP
#define DPRINTF_HPP

#include <windows.h>
#include <stdio.h>
#include <stdarg.h>

// Visual Studio �̃f�o�b�O�E�B���h�E�̉����ɑΉ����镶����(�o�C�g���ł͂Ȃ�)
#define DPRINTF_MES_LENGTH 1024

// �f�o�b�O�r���h�̂Ƃ������CVC�̏o�͑��Ƀ��b�Z�[�W���o���DMFC��TRACE��ATL��ATLTRACE�Ƃ��Ȃ��D
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
