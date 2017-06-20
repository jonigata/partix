/*!
  @file     stdafx.h
  @brief    <ŠT—v>

  <à–¾>
  $Id: stdafx.h 21 2008-04-28 01:53:54Z Naoyuki.Hirayama $
*/
#ifndef STDAFX_H
#define STDAFX_H

#pragma warning(disable: 4996)
#pragma warning(disable: 4819)

#if !defined(_WIN32_WINNT)
#define _WIN32_WINNT 0x500
#endif
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#include <d3d9.h>
#include <d3d9types.h>
#include <d3dx9core.h>

#include <vector>
#include <deque>
#include <map>
#include <stdexcept>
#include <string>
#include <iostream>
#include <fstream>
#include <climits>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/bind.hpp>

#include "zw/basic_window.hpp"
#include "zw/window_manager.hpp"

#pragma comment(lib, "kernel32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "imm32.lib")
#pragma comment(lib, "winmm.lib")

#endif // STDAFX_H
