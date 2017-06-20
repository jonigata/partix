/*!
  @file     mouse_dispatcher.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: mouse_dispatcher.hpp 21 2008-04-28 01:53:54Z Naoyuki.Hirayama $
*/
#ifndef MOUSE_DISPATCHER_HPP
#define MOUSE_DISPATCHER_HPP

#include <windows.h>
#include <boost/scoped_ptr.hpp>
#include "zw/basic_window.hpp"

class IMouseAcceptor;

class MouseDispatcher {
public:
        MouseDispatcher(HWND);
        ~MouseDispatcher();

        void add_acceptor( IMouseAcceptor*, int priority );
        void remove_acceptor( IMouseAcceptor* );

        void on_mouse_message( zw::win::event::mouse& m );
        
private:
        boost::scoped_ptr<class MouseDispatcherImp> pimpl;

};

#endif // MOUSE_DISPATCHER_HPP
