/*!
  @file     basic_window.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: basic_window.hpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $
*/
#ifndef BASIC_WINDOW_HPP
#define BASIC_WINDOW_HPP

#include "window_base.hpp"
#include "window_manager.hpp"
#include "event.hpp"

namespace zw {

namespace win {

/*============================================================================
 *
 * class basic_window
 *
 * 
 *
 *==========================================================================*/

template < class E1 = void, 
           class E2 = void, 
           class E3 = void, 
           class E4 = void, 
           class E5 = void, 
           class E6 = void, 
           class E7 = void, 
           class E8 = void, 
           class E9 = void >
class basic_window {
public:
    template < class Handler >
    basic_window( Handler& handler )
        : handler_( new basic_acceptor< Handler, E1, E2, E3, E4, E5, E6, E7, E8, E9 >( handler ) )
    { 
        m_ = NULL ; 
    }
    ~basic_window(){}
        
    void    create( window_manager& m )
    {
        m_ = &m;
        h_ = m_->create_window( handler_.get() );
    }
    void    destroy()
    {
        assert( m_ );
        m_->destroy_window( h_ );
    }

    void    show()
    {
        assert( m_ );
        m_->show_window( h_ );
    }
    void    hide()
    {
        assert( m_ );
        m_->hide_window( h_ );
    }
        
    std::string title()
    {
        assert( m_ );
        return m_->get_title( h_ );
    }
    void        title(const std::string& x)
    {
        assert( m_ );
        m_->set_title( h_, x );
    }

    extent_type     window_extent()
    {
        assert( m_ );
        return m_->get_window_extent( h_ );
    }
    void            window_extent( const extent_type& e )
    {
        assert( m_ );
        m_->set_window_extent( h_, e );
    }
    offset_type     window_topleft()
    {
        assert( m_ );
        return m_->get_window_topleft( h_ );
    }
    void            window_topleft( const offset_type& o )
    {
        assert( m_ );
        m_->set_window_topleft( h_, o );
    }
    extent_type     client_extent()
    {
        assert( m_ );
        return m_->get_client_extent( h_ );
    }
    void            client_extent( const extent_type& e )
    {
        assert( m_ );
        m_->set_client_extent( h_, e );
    }
    bounds_type     window_bounds()
    {
        return bounds_type( window_topleft(), window_extent() );
    }
    bounds_type     client_bounds()
    {
        return bounds_type( client_extent() );
    }
    offset_type     to_screen( const offset_type& o )
    {
        return m_->client_to_screen( h_, o );
    }
    offset_type     from_screen( const offset_type& o )
    {
        return m_->screen_to_client( h_, o );
    }
    void    invalidate( const bounds_type& r )
    {
        assert( m_ );
        m_->invalidate( h_, r );
    }
    void    invalidate()
    {
        assert( m_ );
        m_->invalidate( h_ );
    }

    void    size_fixed( bool f )
    {
        assert( m_ );
        m_->set_size_fixed( h_, f );
    }
    bool    size_fixed()
    {
        assert( m_ );
        return m_->get_size_fixed( h_ );
    }

    void    set_capture()
    {
        assert( m_ );
        m_->set_capture( h_ );
    }
    void    release_capture()
    {
        assert( m_ );
        m_->release_capture( h_ );
    }

    void    move_cursor( const offset_type& o )
    {
        assert( m_ );
        m_->move_cursor( o );
    }

    void    set_menu( class menulist& menu )
    {
        assert( m_ );
        m_->set_menulist( h_, menu );
    }

private:
    boost::scoped_ptr< zw::detail::abstract_acceptor >      handler_;
    window_manager*                                         m_;
    window_handle_type                                      h_;
    
};

}

} // namespace zw

#endif // BASICWINDOW_HPP
