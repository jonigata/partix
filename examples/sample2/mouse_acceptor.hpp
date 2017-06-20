/*!
  @file     mouse_acceptor.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: mouse_acceptor.hpp 10 2008-03-14 06:55:24Z Naoyuki.Hirayama $
*/
#ifndef MOUSE_ACCEPTOR_HPP
#define MOUSE_ACCEPTOR_HPP

#include "zw/window_base.hpp"
#include "zw/d3d.hpp"
#include <vector>

enum MouseButtonType {
        button_left,
        button_middle,
        button_right,
};

class IMouseReceiver {
public:
        virtual ~IMouseReceiver(){}

        virtual void on_down( const zw::win::offset_type& ) = 0;
        virtual void on_drag( const zw::win::offset_type& ) = 0;
        virtual void on_up( const zw::win::offset_type& ) = 0;
};

class IMouaseAcceptor;

struct MouseAcceptInfo {
        typedef std::vector<class IMouseAcceptor*> ancestors_type;

        ancestors_type  ancestors;
        IMouseReceiver* receiver;

        MouseAcceptInfo()
        {
                receiver = NULL;
        }
        MouseAcceptInfo( const ancestors_type& as, IMouseAcceptor* a, IMouseReceiver* r )
        {
                ancestors = as;
                ancestors.push_back( a );
                receiver = r;
        }
};

class IMouseAcceptor {
public:
        virtual ~IMouseAcceptor(){}

        virtual zw::win::offset_type    get_offset() = 0;

        virtual MouseAcceptInfo accept_mouse(
                const MouseAcceptInfo::ancestors_type&  ancestors,
                const zw::win::offset_type&             position,
                MouseButtonType                         button ) = 0;

        virtual void            on_hover(  const zw::win::offset_type& ) = 0;
        
};

inline
zw::win::offset_type
calcurate_local_mouse_position(
        const MouseAcceptInfo&  info,
        zw::win::offset_type    o )
{
        for( size_t i = 0 ; i < info.ancestors.size() ; i++ ) {
                o -= info.ancestors[i]->get_offset();
        }
        return o;
}

inline
MouseAcceptInfo::ancestors_type
append_ancestor( const MouseAcceptInfo::ancestors_type& ancestors, IMouseAcceptor* a )
{
        MouseAcceptInfo::ancestors_type new_ancestors( ancestors );
        new_ancestors.push_back( a );
        return new_ancestors;
}

#endif // MOUSE_ACCEPTOR_HPP
