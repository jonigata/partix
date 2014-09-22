/*!
  @file     stdafx.h
  @brief    <ŠT—v>

  <à–¾>
  $Id: stdafx.h 26 2008-09-19 01:26:23Z Naoyuki.Hirayama $
*/
#ifndef STDAFX_H
#define STDAFX_H

#pragma warning( disable: 4996 )
#pragma warning( disable: 4819 )

#if !defined(_WIN32_WINNT)
#define _WIN32_WINNT 0x400
#endif
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <iostream>
#include <fstream>
#include <cassert>

#define TETLIBRARY
#include "tetgen/tetgen.h"

#endif // STDAFX_H
