/*!
  @file     window_manager.hpp
  @brief    <概要>

  <説明>
  $Id: window_manager.hpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $
*/
#ifndef WINDOW_MANAGER_HPP
#define WINDOW_MANAGER_HPP

#if !defined(_WIN32_WINNT)
#define _WIN32_WINNT 0x500
#endif
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shellapi.h>

#include <map>
#include <boost/bind.hpp>
#include "window_base.hpp"
#include "menu.hpp"
#include "event.hpp"

#pragma comment(lib, "kernel32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "imm32.lib")
#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "comdlg32.lib")
#pragma comment(lib, "shell32.lib")

namespace zw {

namespace win {

/*============================================================================
 *
 * class window_manager
 *
 * for Win32
 *
 *==========================================================================*/

class window_manager : boost::noncopyable {
private:
    class command_table {
    public:
        command_table(){}
        ~command_table(){}

        void insert(int id,detail::menudata_ptr x){dic_[id]=x;}
        void invoke(int id)
        {
            const dic_type::iterator i = dic_.find( id );
            if( i != dic_.end() ) {
                (*i).second->invoke();
            }
        }

    private:
        typedef std::map<int,detail::menudata_ptr> dic_type;
        dic_type dic_;

    };

    struct create_gift {
        window_manager*                 wm;
        zw::detail::abstract_acceptor*  d;
        command_table*                  ct;
    };


public:                         // user public
    window_manager()
    {
        command_table_.reset( new command_table );
        command_id_seed_ = 0;
    }
    ~window_manager()
    {
        for( int i = 0 ; i < int( accelerators_.size() ) ; i++ ) {
            ::DestroyAcceleratorTable( (HACCEL)accelerators_[i].second );
        }
    }

    void run()
    {
        int     count   =0;
        bool    done    =false;
        for (;;) {
            if( !handle_messages(count,done) ) {
                return;
            }
        }
    }

    template < class T >
    void run( T& handler )
    {
        int     count   =0;
        bool    done    =false;

        for (;;) {
            if( !handle_messages(count,done) ) {
                return;
            }

            if( !done ) {
                // アイドル処理
                done = handler.on_idle(count);
                count++;
            }
        }
    }

    bool step( int& count, bool& done )
    {
        return handle_messages( count, done );
    }

    void exit()
    {
        ::PostQuitMessage( 0 );
    }

public: // for basic_window ... do not call from other classes
    window_handle_type      create_window( zw::detail::abstract_acceptor* dispatcher )
    {
        static ATOM registered_class = 0;

        // ウィンドウ作成
        WNDCLASSW wc;
        wc.style            =CS_DBLCLKS;
        wc.lpfnWndProc      =window_procedure;
        wc.cbClsExtra       =0;
        wc.cbWndExtra       =0;
        wc.hInstance        =GetModuleHandle(NULL);
        wc.hIcon            =::LoadIcon(NULL,IDI_APPLICATION);
        wc.hCursor          =::LoadCursor( NULL,IDC_ARROW );
        wc.hbrBackground    =(HBRUSH)::GetStockObject(BLACK_BRUSH);
        wc.lpszMenuName     =NULL;
        wc.lpszClassName    =L"ZwWindow";

        if( registered_class == NULL ) {
            registered_class = ::RegisterClassW(&wc);
            if( registered_class == NULL ) {
                throw window_exception("register class failed");
            }
        }

        create_gift gift;
        gift.wm = this;
        gift.d = dispatcher;
        gift.ct = command_table_.get();

        HWND w=::CreateWindowExA(0,  // WS_EX_TOPMOST,
                                 "ZwWindow","zw",
                                 WS_OVERLAPPEDWINDOW|WS_CLIPCHILDREN,
                                 CW_USEDEFAULT,CW_USEDEFAULT,
                                 CW_USEDEFAULT,CW_USEDEFAULT,
                                 NULL,NULL,wc.hInstance,&gift);
        if( !w ) {
            throw window_exception("create window failed");
        }

        //ImmAssociateContext(w,NULL);
        return w;
    }
        
    void                    destroy_window( window_handle_type h )
    {
        ::DestroyWindow( ( HWND )h );
    }

    void                    show_window( window_handle_type h )
    {
        ::ShowWindow( ( HWND )h, TRUE );
    }

    void                    hide_window( window_handle_type h )
    {
        ::ShowWindow( ( HWND )h, FALSE );
    }
        
    std::string get_title( window_handle_type h )
    {
        char buffer[1024];
        ::GetWindowTextA( HWND( h ), buffer, 1024 );
        return buffer;
    }

    void        set_title( window_handle_type h, const std::string& x )
    {
        ::SetWindowTextA( HWND(h), x.c_str() );
    }
        
    extent_type     get_window_extent( window_handle_type h )
    {
        RECT r;
        ::GetWindowRect( HWND( h ), &r );
        return extent_type( r.right-r.left, r.bottom-r.top );
    }
        
    void set_window_extent( window_handle_type h, const extent_type& e )
    {
        ::SetWindowPos( HWND( h ), NULL, 0, 0, e.w(), e.h(), SWP_NOMOVE );
    }
        
    offset_type     get_window_topleft( window_handle_type h )
    {
        RECT r;
        ::GetWindowRect( HWND( h ), &r );
        return offset_type( r.left, r.top );
    }

    void set_window_topleft( window_handle_type h, const offset_type& o )
    {
        ::SetWindowPos( HWND( h ), NULL, o.dx(), o.dy(), 0, 0, SWP_NOSIZE );
    }
        
    extent_type     get_client_extent( window_handle_type h )
    {
        RECT r;
        ::GetClientRect( HWND( h ), &r );
        return extent_type( r.right, r.bottom );
    }
        
    void set_client_extent( window_handle_type h, const extent_type& e )
    {
        bounds_type wr;
        adjust_window_bounds( HWND( h ), wr, e );
        ::SetWindowPos(
            HWND( h ), NULL, wr.x0(), wr.y0(), wr.x1(), wr.y1(), 0 );
    }

    offset_type client_to_screen( window_handle_type h, const offset_type& o )
    {
        POINT p;
        p.x = o.dx();
        p.y = o.dy();
        ClientToScreen( HWND( h ), &p );
        return offset_type( p.x, p.y );
    }

    offset_type screen_to_client( window_handle_type h, const offset_type& o )
    {
        POINT p;
        p.x = o.dx();
        p.y = o.dy();
        ScreenToClient( HWND( h ), &p );
        return offset_type( p.x, p.y );
    }

    void    invalidate( window_handle_type h, const bounds_type& r )
    {
        RECT rr;
        rr.left         =r.x0();
        rr.top          =r.y0();
        rr.right        =r.x1();
        rr.bottom       =r.y1();

        ::InvalidateRect( HWND( h ), &rr, FALSE );
    }
        
    void    invalidate( window_handle_type h )
    {
        ::InvalidateRect( HWND( h ), NULL, FALSE );
    }

    void    set_size_fixed( window_handle_type h, bool f )
    {
        LONG x=GetWindowLong(HWND(h),GWL_STYLE);
        if( f ) { 
            x &= ~( WS_SIZEBOX|WS_MAXIMIZEBOX ) ; 
        } else { 
            x |= WS_SIZEBOX|WS_MAXIMIZEBOX ; 
        }
        SetWindowLong( HWND( h ), GWL_STYLE, x );
    }
        
    bool    get_size_fixed( window_handle_type h )
    {
        LONG x = GetWindowLong( HWND( h ), GWL_STYLE );
        if( ( x & WS_SIZEBOX ) || ( x & WS_MAXIMIZEBOX ) ) { 
            return true ; 
        }
        return false;
    }

    void    set_capture( window_handle_type h )
    {
        ::SetCapture( HWND( h ) );
    }
        
    void    release_capture( window_handle_type )
    {
        ::ReleaseCapture();
    }

    void    set_menulist( window_handle_type handle, class menulist& menu)
    {
        detail::abstract_menudata* m = menu.get();

        HMENU root = ::CreateMenu();
        std::vector< detail::menudata_ptr >::const_iterator i;
        std::vector< detail::menudata_ptr >::const_iterator e;
        m->get_children( i, e );
        for( ; i != e ; i++ ){
            make_menu( handle, root, *i );
        }

        HMENU old = ::GetMenu( HWND( handle ) );
        if( old ) {
            ::DestroyMenu( old );
        }
        ::SetMenu( HWND( handle ), root );
    }

private:
    bool handle_messages( int& count, bool& done )
    {
        // ウィンドウメッセージの処理
        for(;;) {
            MSG msg;
            if( !::PeekMessage( &msg, NULL, 0, 0, PM_REMOVE ) ) {
                if( done ) {
                    ::GetMessage( &msg, NULL, 0, 0 );
                } else {
                    break;
                }
            }
            if( msg.message == WM_QUIT ) { 
                return false;
            }

            bool f = false;
            for( int i = 0 ; i < int( accelerators_.size() ) ; i++ ) {
                if( TranslateAccelerator(
                        (HWND)accelerators_[i].first,
                        (HACCEL)accelerators_[i].second, &msg ) ) {
                    f = true;
                    break;
                }
            }

            if( !f ) {
                TranslateMessage( &msg );
                DispatchMessage( &msg );
            }
            count = 0;
            done = false;
        }
        return true;
    }
        
    void adjust_window_bounds(
        window_handle_type      h,
        bounds_type&            owr,
        const extent_type&      ice )
    {
        DWORD   style        = ::GetWindowLong( HWND( h ), GWL_STYLE );
        DWORD   ex_style     = ::GetWindowLong( HWND( h ), GWL_EXSTYLE );
        BOOL    has_menu     = ::GetMenu( HWND( h ) ) != NULL;

        RECT wr;
        ::GetWindowRect( HWND( h ), &wr );
        int x = wr.left;
        int y = wr.top;

        RECT cr;
        ::GetClientRect( HWND( h ), &cr );

        wr = cr;
        ::AdjustWindowRectEx( &wr, style, has_menu, ex_style ) ; 
        
        int dx = cr.left - wr.left ; 
        int dy = cr.top - wr.top ; 
        int dw = wr.right - wr.left - cr.right; 
        int dh = wr.bottom - wr.top - cr.bottom; 

        owr.left    ( x );
        owr.top         ( y );
        owr.width   ( ice.w() + dw );
        owr.height  ( ice.h() + dh );
    }

    void make_menu(
        window_handle_type      window,
        menu_handle_type        parent,
        detail::menudata_ptr    m )
    {
        std::vector< detail::menudata_ptr >::const_iterator i;
        std::vector< detail::menudata_ptr >::const_iterator e;

        std::string s = m->get_text();
        if( m->get_children( i, e ) ) { 
            HMENU curr = ::CreateMenu() ; 

            for( ; i != e ; i++ ) { 
                make_menu( window, menu_handle_type( curr ), *i ) ; 
            }
                        
            ::AppendMenuA( 
                HMENU( parent ), 
                MF_POPUP, 
                ( intptr_t )curr, 
                s.c_str() ) ; 
            m->set_enabler( boost::bind( 
                                enable_menu, 
                                HWND( window ), 
                                HMENU( parent ), 
                                UINT( intptr_t( curr ) ), 
                                _1 ) ) ; 
            m->set_checker( boost::bind( 
                                check_menu, 
                                HWND( window ), 
                                HMENU( parent ), 
                                UINT( intptr_t( curr ) ), 
                                _1 ) ) ; 
        } else {
            if( s == "-" ) {
                ::AppendMenu(
                    HMENU(parent),
                    MF_SEPARATOR,
                    0,
                    NULL);
            } else {
                int id = command_id_seed_++;
                command_table_->insert(id,m);
                ::AppendMenuA(
                    HMENU(parent),
                    MF_STRING,
                    id,
                    s.c_str());
                m->set_enabler( boost::bind(
                                    enable_menu,
                                    HWND( window ),
                                    HMENU( parent ),
                                    id,
                                    _1 ) );
                m->set_checker( boost::bind(
                                    check_menu,
                                    HWND( window ),
                                    HMENU( parent ),
                                    id,
                                    _1 ) );

                bool ctrl;
                bool alt;
                bool shift;
                ACCEL accel;
                m->get_accelerator( ctrl, alt, shift, accel.key );
                if( accel.key != 0 ) {
                    accel.fVirt = FVIRTKEY;
                    if( ctrl  ) { accel.fVirt |= FCONTROL; }
                    if( alt   ) { accel.fVirt |= FALT; }
                    if( shift ) { accel.fVirt |= FSHIFT; }
                                
                    accel.cmd = id;
                    HACCEL ha = CreateAcceleratorTable( &accel, 1 );
                    accelerators_.push_back(
                        std::make_pair( window, (void*)ha ) );
                }
            }
        }
    }

private:
    static LRESULT CALLBACK window_procedure(
        HWND     handle,
        UINT     message,
        WPARAM   wparam,
        LPARAM   lparam )
    {
        window_manager* self=
            static_cast< window_manager* >(
                GetPropA( handle, "ZwWindowManager" ) );
        zw::detail::abstract_acceptor* dispatcher=
            static_cast<zw::detail::abstract_acceptor*>(
                GetPropA(handle,"ZwWindowDispatcher"));

        switch( message ) {
        case WM_CREATE: { 
            CREATESTRUCT* cs = (CREATESTRUCT*)lparam;
            create_gift* gift = (create_gift*)cs->lpCreateParams;

            ::SetPropA( handle, "ZwWindowManager", gift->wm );
            ::SetPropA( handle, "ZwWindowDispatcher", gift->d );
            ::SetPropA( handle, "ZwWindowCommandTable", gift->ct );
            ::SetPropA( handle, "ZwWindowNextClipboardViewer",
                        SetClipboardViewer( handle ) );

            RECT client_rect; ::GetClientRect( handle, &client_rect );

            event::create m;
            m.handle = handle;
            m.extent = extent_type( client_rect.right, client_rect.bottom );
            gift->d->accept(m);

            ::DragAcceptFiles( handle, TRUE );

            return 0;
        }
        case WM_DESTROY: {
            ::ChangeClipboardChain(
                handle ,
                (HWND)GetPropA( handle, "ZwWindowNextClipboardViewer" ) );

                event::destroy m;
                dispatcher->accept(m);
                ::PostQuitMessage(0);
                break;
        }
        case WM_CHANGECBCHAIN: {
            HWND next = (HWND)::GetPropA(
                handle, "ZwWindowNextClipboardViewer" );
            if ( (HWND)wparam == next ) {
                ::SetPropA( handle, "ZwWindowNextClipboardViewer",
                            (HWND)lparam );
            } else if ( next ) {
                ::SendMessage( next, message , wparam , lparam );
            }
            return 0;
        }
        case WM_DRAWCLIPBOARD: {
            HWND next = (HWND)::GetPropA(
                handle, "ZwWindowNextClipboardViewer" );
            if ( next ) {
                ::SendMessage( next, message , wparam , lparam );
            }

            if( dispatcher ) {
                event::clipboard m;
                dispatcher->accept( m );
            }
            return 0;
        }
        case WM_ERASEBKGND: {
            event::erase_bg m;
            m.dc = reinterpret_cast<dc_type>(wparam);
            m.result_erased = true;
            if( dispatcher->accept(m) ) {
                return m.result_erased ? 1 : 0;
            }
            return 1;
        }
        case WM_PAINT: {
            event::paint m;
            m.dc = reinterpret_cast<dc_type>(wparam);
            m.result_painted = true;

            PAINTSTRUCT ps;
            BeginPaint(handle,&ps);
            if( dispatcher->accept(m) ) {
                EndPaint(handle,&ps);
                return m.result_painted ? 0 : 1;
            }
            EndPaint(handle,&ps);
            break;
        }
        case WM_SIZING: {
            InvalidateRect(handle,NULL,FALSE);
            return TRUE;
        }
        case WM_SIZE: {
            //window->retrieve_window_positions();
            event::size m;
            switch( wparam ) {
            case SIZE_RESTORED:     m.type = event::size::sized;         break;
            case SIZE_MINIMIZED:    m.type = event::size::minimized;     break;
            case SIZE_MAXIMIZED:    m.type = event::size::maximized;     break;
            default: m.type = event::size::sized; break;
            }
            m.extent.w((int)(short) LOWORD(lparam));
            m.extent.h((int)(short) HIWORD(lparam));
            if( dispatcher->accept(m) ) {
                return 0;
            }
            break;
        }
        case WM_MOVE: {
            //window->retrieve_window_positions();
            event::move m;
            m.position.dx((int)(short) LOWORD(lparam));
            m.position.dy((int)(short) HIWORD(lparam));
            if( dispatcher->accept(m) ) {
                return 0;
            }
            break;
        }
        case WM_ACTIVATEAPP:
            //bool active=wparam ? true : false;
            //maker->activate(active);
            return 0;

        case WM_LBUTTONDBLCLK:
        case WM_LBUTTONDOWN:
        case WM_LBUTTONUP:
        case WM_RBUTTONDBLCLK:
        case WM_RBUTTONDOWN:
        case WM_RBUTTONUP:
        case WM_MBUTTONDBLCLK:
        case WM_MBUTTONDOWN:
        case WM_MBUTTONUP:
        case WM_MOUSEWHEEL:
        case WM_MOUSEMOVE: {
            event::mouse m;
            make_mouse_event(m,message,lparam,wparam);
            if( message == WM_MOUSEMOVE ) {
                m.lbutton = wparam & MK_LBUTTON ? true : false;
                m.rbutton = wparam & MK_RBUTTON ? true : false;
                m.mbutton = wparam & MK_MBUTTON ? true : false;
            }
            if( message == WM_MOUSEWHEEL ) {
                POINT p;
                p.x = (int)(short) LOWORD(lparam); 
                p.y = (int)(short) HIWORD(lparam); 
                ScreenToClient(handle,&p);
                m.position.dx(p.x);
                m.position.dy(p.y);
                m.wheel_delta = GET_WHEEL_DELTA_WPARAM(wparam);
            }

            if( dispatcher->accept(m) ) {
                return 0;
            }
            break;
        }
        case WM_CHAR: {
            event::keychar m;
            m.code = static_cast<boost::uint32_t>(wparam);
            get_modify_state(m.ctrl,m.alt,m.shift);
            if( dispatcher->accept(m) ) {
                return 0;
            }
            break;
        }
        case WM_SYSCHAR: {
            event::keychar m;
            m.code = static_cast<boost::uint32_t>(wparam);
            get_modify_state(m.ctrl,m.alt,m.shift);
            m.alt=true;
            if( dispatcher->accept(m) ) {
                return 0;
            }
            break;
        }
        case WM_KEYDOWN: {
            event::keystate m;
            m.down          =true;
            m.keycode       =static_cast<boost::uint32_t>(wparam);
            m.repeat        =(int)lparam & 0xffff;
            m.scancode      =(boost::uint32_t)((lparam >> 16) & 0xff);
            m.enhanced      =(boost::uint32_t)((lparam >> 24) & 0x01);
            m.contextcode   =0;
            m.prevdown      =(boost::uint32_t)((lparam >> 30) & 0x01);
            m.transstate    =0;
            get_modify_state(m.ctrl,m.alt,m.shift);
            if( dispatcher->accept(m) ) {
                return 0;
            }
            break;
        }
        case WM_KEYUP: {
            event::keystate m;
            m.down          =false;
            m.keycode       =static_cast<boost::uint32_t>(wparam);
            m.repeat        =1;
            m.scancode      =(boost::uint32_t)((lparam >> 16) & 0xff);
            m.enhanced      =(boost::uint32_t)((lparam >> 24) & 0x01);
            m.contextcode   =0;
            m.prevdown      =1;
            m.transstate    =0;
            get_modify_state(m.ctrl,m.alt,m.shift);
            if( dispatcher->accept(m) ) {
                return 0;
            }
            break;
        }
        case WM_SYSKEYDOWN: {
            event::keystate m;
            m.down          =true;
            m.keycode       =static_cast<boost::uint32_t>(wparam);
            m.repeat        =(boost::uint32_t)(lparam & 0xffff);
            m.scancode      =(boost::uint32_t)((lparam >> 16) & 0xff);
            m.enhanced      =(boost::uint32_t)((lparam >> 24) & 0x01);
            m.contextcode   =0;
            m.prevdown      =(lparam >> 30) & 0x01;
            m.transstate    =0;
            get_modify_state(m.ctrl,m.alt,m.shift);
            m.alt           =true;
            if( dispatcher->accept(m) ) {
                return 0;
            }
            break;
        }
        case WM_SYSKEYUP: {
            event::keystate m;
            m.down          =false;
            m.keycode       =static_cast<boost::uint32_t>(wparam);
            m.repeat        =1;
            m.scancode      =(boost::uint32_t)((lparam >> 16) & 0xff);
            m.enhanced      =(boost::uint32_t)((lparam >> 24) & 0x01);
            m.contextcode   =0;
            m.prevdown      =1;
            m.transstate    =0;
            get_modify_state(m.ctrl,m.alt,m.shift);
            m.alt           =true;
            if( dispatcher->accept(m) ) {
                return 0;
            }
            break;
        }
        case WM_IME_NOTIFY: {
            event::ime m;
            if( wparam == IMN_SETOPENSTATUS ) {
                m.command = event::ime::command_setopenstatus;
                dispatcher->accept(m);
            }
            if( wparam == IMN_CLOSESTATUSWINDOW ) {
                m.command = event::ime::command_closestatuswindow;
                dispatcher->accept(m);
            }
            break;
        }
        case WM_IME_COMPOSITION: {
            event::ime m;
            m.command = event::ime::command_text;
            bool processed = false;
            HIMC imc = ImmGetContext(handle);
            if( lparam & GCS_RESULTSTR ) {
                LONG buffer_size = ImmGetCompositionString(
                    imc, GCS_RESULTSTR, NULL, 0 );
                char* buffer = new char[buffer_size];
                ImmGetCompositionStringA(
                    imc, GCS_RESULTSTR, buffer, buffer_size);
                m.text.assign(buffer,buffer_size);
                delete [] buffer;
                if( dispatcher->accept(m) ) {
                    processed = true;
                }
            }
            ImmReleaseContext(handle,imc);
            if( processed ) {
                return 0;
            }
            break;
        }
        case WM_DROPFILES: {
            HDROP hdrop = (HDROP)wparam;
            event::filedrop m;
            size_t n = DragQueryFile( hdrop, 0xffffffff, NULL, 0 );
            for( size_t i = 0 ; i < n ; i++ ) {
                char buffer[MAX_PATH];
                DragQueryFileA( hdrop, (UINT)i, buffer, MAX_PATH );
                m.filenames.push_back( buffer );
            }
            DragFinish( hdrop );
            std::sort( m.filenames.begin(), m.filenames.end() );
            dispatcher->accept( m );
            break;
        }
        case WM_COMMAND: {
            command_table* ct=static_cast<command_table*>(
                GetPropA(handle,"ZwWindowCommandTable"));
            ct->invoke( LOWORD(wparam) );
            break;
        }
        default:
            break;
        }
    
        return ::DefWindowProc(handle,message,wparam,lparam);
    }

    static void enable_menu(
        HWND window, HMENU parent, UINT command_id, bool enable )
    {
        UINT flag;
        if( enable ) {
            flag = MF_ENABLED;
        } else {
            flag = MF_DISABLED | MF_GRAYED;
        }

        ::EnableMenuItem( parent, command_id, MF_BYCOMMAND | flag );
        ::DrawMenuBar( window );
    }

    static void check_menu(
        HWND window, HMENU parent, UINT command_id, bool check )
    {
        UINT flag;
        if( check ) {
            flag = MF_CHECKED;
        } else {
            flag = MF_UNCHECKED;
        }

        ::CheckMenuItem( parent, command_id, MF_BYCOMMAND | flag );
        ::DrawMenuBar( window );
    }

    static void get_modify_state(bool& ctrl,bool& alt,bool& shift)
    {
        ctrl    =(GetKeyState(VK_CONTROL) & 0x8000) ? true : false;
        shift   =(GetKeyState(VK_SHIFT) & 0x8000) ? true : false;
        alt     =(GetKeyState(VK_MENU) & 0x8000) ? true : false;
    }

    static void make_mouse_event(
        win::event::mouse& m, UINT message, LPARAM lparam, WPARAM wparam )
    {
        m.position.dx( (int)(short) LOWORD(lparam) ); 
        m.position.dy( (int)(short) HIWORD(lparam) ); 
        m.double_click = message==WM_LBUTTONDBLCLK || message==WM_MBUTTONDBLCLK || message==WM_RBUTTONDBLCLK;
        m.ctrl = wparam & MK_CONTROL ? true : false;
        m.shift = wparam & MK_SHIFT ? true : false;
        m.alt = GetKeyState(VK_MENU) < 0 ;
        m.lbutton = message==WM_LBUTTONDBLCLK || message==WM_LBUTTONDOWN;
        m.rbutton = message==WM_RBUTTONDBLCLK || message==WM_RBUTTONDOWN;
        m.mbutton = message==WM_MBUTTONDBLCLK || message==WM_MBUTTONDOWN;
        m.wheel_delta = 0;
    }

private:
    int                     command_id_seed_;

    boost::scoped_ptr< command_table > command_table_;
    std::vector< std::pair< window_handle_type, accel_handle_type > >
                            accelerators_;

};

} // namespace win

} // namespace zw

#endif // WINDOWMANAGER_HPP

