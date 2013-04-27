/*!
  @file     stdafx.h
  @brief    <ŠT—v>

  <à–¾>
  $Id: stdafx.h 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef STDAFX_H
#define STDAFX_H

#pragma warning(disable: 4996)
#pragma warning(disable: 4819)

#define D3D_DEBUG_INFO

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
#include <sstream>
#include <algorithm>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include "zw/dprintf.hpp"
#include "zw/basic_window.hpp"
#include "zw/window_manager.hpp"

#pragma comment(lib, "kernel32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "imm32.lib")
#pragma comment(lib, "winmm.lib")

#if 0
#ifdef _DEBUG
#pragma comment(lib, "FreeImaged.lib")
#pragma comment(lib, "LibTIFFd.lib")
#pragma comment(lib, "LibJPEGd.lib")
#pragma comment(lib, "LibPNGd.lib")
#pragma comment(lib, "LibMNGd.lib")
#pragma comment(lib, "zlibd.lib")
#else
#pragma comment(lib, "FreeImage.lib")
#pragma comment(lib, "LibTIFF.lib")
#pragma comment(lib, "LibJPEG.lib")
#pragma comment(lib, "LibPNG.lib")
#pragma comment(lib, "LibMNG.lib")
#pragma comment(lib, "zlib.lib")
#endif
#endif

#endif // STDAFX_H
