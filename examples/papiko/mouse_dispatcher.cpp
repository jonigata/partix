// $Id: mouse_dispatcher.cpp 252 2007-06-23 08:32:33Z naoyuki $

#include "mouse_dispatcher.hpp"
#include "mouse_acceptor.hpp"
//#include "zw/dprintf.hpp"
#include <map>

/*===========================================================================*/
/*!
 * @class MouseDispatcherImp
 * @brief 
 *
 * 
 */
/*==========================================================================*/

class MouseDispatcherImp {
public:
    typedef std::multimap<int,IMouseAcceptor*> dic_type;

    struct finder {
        finder( IMouseAcceptor* m ) : m_(m){}
        bool operator()( const dic_type::value_type& p ) {
            return p.second == m_;
        }
    private:
        IMouseAcceptor* m_;
    };

public:
    MouseDispatcherImp( HWND w ) : window_( w ), down_(false)
    {
        ldown_ = false;
        l_accept_info_.receiver = NULL;

        rdown_ = false;
        r_accept_info_.receiver = NULL;

        mdown_ = false;
        m_accept_info_.receiver = NULL;
    }
    ~MouseDispatcherImp(){}

    void add_acceptor( IMouseAcceptor* acceptor, int priority )
    {
        dic_.insert( dic_type::value_type( priority, acceptor ));
    }
        
    void remove_acceptor( IMouseAcceptor* acceptor )
    {
        dic_.erase(
            std::find_if( dic_.begin(), dic_.end(), finder( acceptor ) ) );
    }
        
    void on_mouse_message( zw::win::event::mouse& m )
    {
        bool down = m.lbutton || m.rbutton || m.mbutton;

        if( !down_ && !down ) {
            for( dic_type::const_iterator i = dic_.begin();
                 i != dic_.end();
                 ++i ) {
                (*i).second->on_hover( m.position );
            }
            return;
        }

        if( !down_ && down ) {
            ::SetCapture( window_ );
            down_ = true;
        } else if( down_ && !down ) {
            ::ReleaseCapture();
            down_ = false;
        }

        dispatch(
            ldown_, m.lbutton, l_accept_info_, m.position, button_left );
        dispatch(
            rdown_, m.rbutton, r_accept_info_, m.position, button_right );
        dispatch(
            mdown_, m.mbutton, m_accept_info_, m.position, button_middle );
    }
        
    MouseAcceptInfo try_accept(
        const zw::win::offset_type& position, MouseButtonType button )
    {
        for( dic_type::const_iterator i = dic_.begin();
             i != dic_.end();
             ++i ) {
            MouseAcceptInfo::ancestors_type ancestors;
            MouseAcceptInfo a = (*i).second->accept_mouse(
                ancestors, position, button );
            if( a.receiver ) { return a; }
        }

        MouseAcceptInfo a;
        return a;
    }
        
    void dispatch(
        bool&                           old_down,
        bool                            new_down,
        MouseAcceptInfo&                accept_info,
        const zw::win::offset_type&     position,
        MouseButtonType                 button )
    {
        if( !old_down ) {
            if( new_down ) {
                // start
                MouseAcceptInfo a = try_accept( position, button );
                if( a.receiver ) { // accept
                    accept_info = a;
                    a.receiver->on_down(
                        calcurate_local_mouse_position(
                            accept_info, position ) );
                }
                old_down = true;
            } else { /* do nothing */ }
        } else {
            if( accept_info.receiver ) {
                if( new_down ) {
                    // drag
                    accept_info.receiver->on_drag(
                        calcurate_local_mouse_position(
                            accept_info, position ) );
                } else {
                    // end
                    accept_info.receiver->on_up(
                        calcurate_local_mouse_position(
                            accept_info, position ) );
                }
            } else { /* do nothing */ }
            old_down = new_down;
        }
    }
        
private:
    HWND            window_;
    bool            down_;
    dic_type        dic_;

    bool            ldown_;
    MouseAcceptInfo l_accept_info_;

    bool            rdown_;
    MouseAcceptInfo r_accept_info_;

    bool            mdown_;
    MouseAcceptInfo m_accept_info_;

};

/*============================================================================
 *
 * class MouseDispatcher 
 *
 * 
 *
 *==========================================================================*/
//<<<<<<<<<< MouseDispatcher

//****************************************************************
// constructor
MouseDispatcher::MouseDispatcher(HWND w) : pimpl( new MouseDispatcherImp( w ) )
{
}

//****************************************************************
// destructor
MouseDispatcher::~MouseDispatcher()
{
}

//****************************************************************
// add_acceptor
void MouseDispatcher::add_acceptor( IMouseAcceptor* acceptor, int priority )
{
    pimpl->add_acceptor( acceptor, priority );
}

//****************************************************************
// remove_acceptor
void MouseDispatcher::remove_acceptor( IMouseAcceptor* acceptor )
{
    pimpl->remove_acceptor( acceptor );
}

//****************************************************************
// on_mouse_message
void MouseDispatcher::on_mouse_message( zw::win::event::mouse& m )
{
    pimpl->on_mouse_message( m );
}

//>>>>>>>>>> MouseDispatcher

