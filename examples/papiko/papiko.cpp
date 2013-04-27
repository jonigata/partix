// $Id: physix.cpp 254 2007-06-28 06:47:08Z naoyuki $

#include <windows.h>
#include <commdlg.h>
#include <shlobj.h>
#include <deque>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include "zw/window_manager.hpp"
#include "zw/basic_window.hpp"
#include "zw/menu.hpp"
#include "zw/d3d.hpp"
#include "zw/dprintf.hpp"
#include "zw/timer.hpp"
#include "camera.hpp"
#include "partix_common.hpp"
#include "mouse_dispatcher.hpp"
#include "papishape.hpp"
#include "finger.hpp"
#include "cloth.hpp"

#pragma pack( push,1 )
struct dot_vertex_type {
    enum { format = (D3DFVF_XYZ | D3DFVF_DIFFUSE) };

    D3DXVECTOR3     pos;
    DWORD           color;

    void operator()( FLOAT xx, FLOAT yy, FLOAT zz, DWORD cc )
    {
        pos.x = xx; pos.y = yy; pos.z = zz; color = cc;
    }

};
#pragma pack( pop )

class application : public IMouseAcceptor, public IMouseReceiver {
public:
    typedef zw::win::basic_window<
        zw::win::event::create,
        zw::win::event::size,
        zw::win::event::paint,
        zw::win::event::mouse,
        zw::win::event::keychar
        > window_type;

    typedef D3DXPartixTraits::vector_type vector_type;
public:
    void load_config( std::map< std::string, std::string >& config )
    {
        std::ifstream ifs( "data/papiko.config" );

        std::string s;
        while( std::getline( ifs, s ) ) {
            std::stringstream line( s );
            std::string opcode;
            std::string operand;
            line >> opcode;
            line >> operand;
            config[opcode] = operand;
        }                
    }


public:
    application()
    {
        show_point_ = false;

        // ウィンドウ
        window_.reset( new window_type( *this ) );
        window_->create( manager_ );
        window_->show();
        window_->client_extent( zw::win::extent_type( 800, 600 ) );
        window_->title( "papiko" );

        pushing_ = false;
        cloth_dragging_ = false;
    }
    ~application()
    {
        mouse_dispatcher_->remove_acceptor( &*camera_ );
    }

    void run()
    {
        manager_.run(*this);
    }

    bool on_idle(int counter)
    {
        if( !timer_( boost::bind( &application::update, this, _1, _2 ) ) ) {
            Sleep(1);
        }

        return false;
    }

    void accept(zw::win::event::create& m)
    {
        handle_           = HWND(m.handle);

        d3d_.reset( new zw::d3d );
        d3d_device_index_ = d3d_->add_view(handle_);
        font_.reset( new zw::d3d_font( *d3d_, L"MS PGothic", 16 ) );

        // mouse dispatcher
        mouse_dispatcher_.reset( new MouseDispatcher( handle_ ) );

        // browsing
        scale_ = 1.0f;
        camera_.reset( new Camera );
        mouse_dispatcher_->add_acceptor( &*camera_, 99 );
        mouse_dispatcher_->add_acceptor( this, 10 );
        //mouse_dispatcher_->add_acceptor( dashboard_.get(), 9 );

        // scene body
        world_.reset( new px::world_type );
        px::world_type* world = world_.get();

        // load config
        std::map< std::string, std::string > config;
        load_config( config );

        // model
        actor_.reset( new PapiShape );
        actor_->set_data_directory( "data" );
        dense_handle body = actor_->load_dense( d3d_->device(), config["body"] );
        actor_->load_coarse( world, "body",   1.0f, 1.0f );
        actor_->load_coarse( world, "bust_a", 0.2f, 0.3f );
        actor_->load_coarse( world, "bust_b", 0.2f, 0.3f );
        actor_->load_coarse( world, "hip_a" , 0.2f, 0.6f );
        actor_->load_coarse( world, "hip_b" , 0.2f, 0.6f );
        actor_->subdivide( 0 );
        actor_->tie_dense_to_coarse( body );
        actor_->attach_brdf(
            d3d_->device(), body, "defaultMat", config["body_brdf"] );
        actor_->attach_smap(
            d3d_->device(), body, "defaultMat", config["specular_map"] );
        dense_handle hair = actor_->load_dense(
            d3d_->device(),  config["hair"] );
        actor_->attach_brdf(
            d3d_->device(), hair, "defaultMat", config["hair_brdf"] );
        actor_->attach_smap(
            d3d_->device(), hair, "defaultMat", config["specular_map"] );

        finger_.reset( new Finger );
        finger_->load_coarse( world, "data/finger" );

        cloth_.reset( new Cloth );
        cloth_->load_coarse( world, "data/cloth.obj" );

        // reset
        on_reset_device( d3d_->device() );

        vector_type v0( 0, 0, 0 );

        // ground
        const vector_type bounds[2] = {
            vector_type( 0,  1.0f, 0 ), vector_type( 0, -5.0f, 0 ),
        };
        boost::shared_ptr< px::body_type > e(
            new px::boundingplane_type(  bounds[1], bounds[0] ) );
        bodies_.push_back( e );
        //world_->add_body( e.get() );

        world_->restart();
    }

    void accept(zw::win::event::size& m)
    {
        if( m.type == zw::win::event::size::sized ) {
            d3d_->reset(*this);
        }

        //dashboard_->on_size( m.extent );
    }
    void accept(zw::win::event::paint& m)
    {
        draw();
    }
    void accept(zw::win::event::keychar& m)
    {
        if( m.code == ' ' ) {
            world_->update( D3DXPartixTraits::tick() );
        }

        if( m.code == 's' ) {
            show_point_ = !show_point_;
        }
        if( m.code == 'd' ) {
            world_->dump();
        }
        if( m.code == VK_ESCAPE ) {
            actor_->reload_textures( d3d_->device() );
        }
    }
    void accept(zw::win::event::mouse& m)
    {
        mouse_dispatcher_->on_mouse_message( m );
    }

    void on_render(LPDIRECT3DDEVICE9 device)
    {
        // view matrix
        D3DXVECTOR3 view_point;
        camera_->make_view_point( view_point );
        D3DXVECTOR3 focal_point;
        camera_->make_focal_point( focal_point );
        D3DXMATRIX mat_view;
        set_view_matrix( mat_view, view_point, focal_point );
        device->SetTransform( D3DTS_VIEW,  &mat_view );

        // light
        D3DLIGHT9 d3dLight;
        memset(&d3dLight, 0, sizeof(d3dLight));
        d3dLight.Type = D3DLIGHT_DIRECTIONAL;
        d3dLight.Diffuse.r = 0.5f;
        d3dLight.Diffuse.g = 0.5f;
        d3dLight.Diffuse.b = 0.5f;
        d3dLight.Diffuse.a = 1.0f;
        d3dLight.Ambient.r = 0.1f;
        d3dLight.Ambient.g = 0.1f;
        d3dLight.Ambient.b = 0.1f;
        d3dLight.Ambient.a = 1.0f;
        d3dLight.Direction = D3DXVECTOR3(1.0f, -1.0f, 1.0f);
                
        device->SetLight(0, &d3dLight);
        device->LightEnable(0, TRUE);
                
        // render
        D3DXMATRIX mat_world;
        D3DXMatrixTranslation(
            &mat_world,
            focal_point.x,
            focal_point.y,
            focal_point.z );
        device->SetTransform( D3DTS_WORLD, &mat_world);
        D3DXMatrixScaling( &mat_world, scale_, scale_, scale_ );
        device->SetTransform( D3DTS_WORLD, &mat_world);
        actor_->render( device, view_point );
        cloth_->render( device, view_point );

        if( show_point_ && finger_.get() ) {
            std::vector< boost::shared_ptr< px::softvolume_type > >& coarses =
                actor_->get_coarses();
                        
            size_t n = coarses.size();
            for( size_t i = 0 ; i < n ; i++ ) {
                draw_wireframe( device, coarses[i].get() );
            }
            draw_wireframe( device, finger_->get_coarse() );
        }

        // frame rate
        if( font_ ) {
            wchar_t buffer[256];
            wsprintf( buffer, L"FPS: %d", timer_.realfps() );
            font_->draw( 0,
                         window_->client_extent().height() - 16,
                         128,
                         window_->client_extent().height(), 
                         buffer,
                         D3DCOLOR_XRGB(255,255,255) );
        }

        // GUI
        //dashboard_->render( device );
    }

    void on_lost_device( LPDIRECT3DDEVICE9 device )
    {
        //dashboard_->on_lost_device( device );
        font_->on_lost_device( device );
        actor_->on_lost_device( device );
    }

    void on_reset_device( LPDIRECT3DDEVICE9 device )
    {
        //dashboard_->on_reset_device( device );
        font_->on_reset_device( device );;
        actor_->on_reset_device( device );
    }

protected:
    // implementation of IMouseAcceptor
    zw::win::offset_type get_offset() { return zw::win::offset_type( 0, 0 ); }

    MouseAcceptInfo accept_mouse(
        const MouseAcceptInfo::ancestors_type&  ancestors,
        const zw::win::offset_type&             position,
        MouseButtonType                         button )
    {
        MouseAcceptInfo info;

        if( button == button_left  ) {
            D3DXVECTOR3 eye, dir;
            if( pick_body( position, eye, dir ) ) {
                info.ancestors.push_back( this );
                info.receiver = this;
            }
        }

        return info;
    }

    void            on_hover(  const zw::win::offset_type& ){}

    // implementation of IMouseReceiver
    void on_down( const zw::win::offset_type& pos )
    {
        pushing_ = true;
        drag_point_ = pos;

        D3DXVECTOR3 eye, dir;
        px::body_type* b = pick_body( drag_point_, eye, dir );
        if( b == cloth_->get_coarse() ) {
            cloth_dragging_ = true;
            cloth_origin_ = eye + dir;
            cloth_->select( eye + dir );
        }
    }
    void on_drag( const zw::win::offset_type& pos )
    {
        drag_point_ = pos;
    }
    void on_up( const zw::win::offset_type& )
    {
        pushing_ = false;
        cloth_dragging_ = false;
    }


private:
    void set_view_matrix(
        D3DXMATRIX&         mat_view,
        const D3DXVECTOR3&  view_point,
        const D3DXVECTOR3&  focal_point )
    {
        // view行列
        D3DXVECTOR3 v3(0.0f,1.0f,0.0f);
        D3DXMatrixLookAtLH(
            &mat_view,
            &view_point,
            &focal_point,
            &v3);
    }

    void update( int elapsed0, float elapsed1 )
    {
        //dashboard_->update( elapsed1 );
        world_->update( D3DXPartixTraits::tick() );
        actor_->update( D3DXPartixTraits::tick() );
        actor_->fix();
        cloth_->fix();
        draw();

        if( pushing_ ) {
            if( cloth_dragging_ ) {
                cloth_->move_to( get_drag_target( drag_point_ ) );
            } else {
                D3DXVECTOR3 eye, dir;
                px::body_type* b = pick_body( drag_point_, eye, dir );
                if( b ) {
                    finger_->attack( eye + dir );
                }
            }
        } else {
            D3DXVECTOR3 eye, focal;
            camera_->make_view_point( eye );
            camera_->make_focal_point( focal );

            D3DXVECTOR3 curr = finger_->get_current_center();


            D3DXVECTOR3 v0 = (D3DXVECTOR3&)( eye - focal );
            D3DXVec3Normalize( &v0, &v0 );
            v0 *= 6.0f;
            finger_->wait( v0 );
        }

    }

    void draw()
    {
        d3d_->check_device_lost(*this);
        d3d_->render(
            D3DCOLOR_ARGB(0xff,0x48,0x3D,0x8B), *this, d3d_device_index_ );
    }

    px::body_type* pick_body(
        const zw::win::offset_type& position,
        D3DXVECTOR3&                org,
        D3DXVECTOR3&                dir )
    {
        struct pick_filter {
        public:
            pick_filter( PapiShape* actor, Cloth* c )
                : coarses_( actor->get_coarses() ),
                  cloth_( c->get_coarse() ) {}
            bool operator()( px::body_type* q )
            {
                size_t n = coarses_.size();
                for( size_t i = 0 ; i < n ; i++ ) {
                    if( coarses_[i].get() == q ) { return true; }
                }
                if( cloth_ == q ) { return true; }
                return false;
            }
            std::vector< boost::shared_ptr< px::softvolume_type > >& coarses_;
            px::cloth_type* cloth_;
        };

        get_world_ray( org, dir, position );

        D3DXVec3Normalize( &dir, &dir );
        D3DXVECTOR3 s0 = org;
        D3DXVECTOR3 s1 = s0 + dir * 50.0f;

        float dist;
        px::body_type* r = world_->pick_filter(
            s0, s1, pick_filter( actor_.get(), cloth_.get() ), dist );
        dir *= dist * 50.0f;

        return r;
    }

    D3DXVECTOR3 get_drag_target( const zw::win::offset_type& position )
    {
        D3DXVECTOR3 org;
        D3DXVECTOR3 dir;
        get_world_ray( org, dir, position );

        D3DXVECTOR3 az = org - cloth_origin_;
        D3DXVec3Normalize( &az, &az );
                
        float t;
        partix::math< D3DXPartixTraits >::test_plane_segment(
            cloth_origin_, az, org, org + dir, t );
                
        return org + t * 0.9f * dir;
    }

    void get_world_ray(
        D3DXVECTOR3&                org,
        D3DXVECTOR3&                dir,
        const zw::win::offset_type& position )
    {
        D3DXMATRIX proj;
        d3d_->device()->GetTransform( D3DTS_PROJECTION,  &proj );

        const int viewport_w = window_->client_extent().width();
        const int viewport_h = window_->client_extent().height();

        D3DXVECTOR3 v;
        v.x =  ( position.dx() * 2.0f / viewport_w - 1 ) / proj._11;
        v.y = -( position.dy() * 2.0f / viewport_h - 1 ) / proj._22;
        v.z = 1;
        D3DXVec3Normalize( &v, &v );

        D3DXMATRIX view;
        d3d_->device()->GetTransform( D3DTS_VIEW, &view );
        D3DXMATRIX iview;
        D3DXMatrixInverse( &iview, NULL, &view );

        dir.x = v.x * iview._11 + v.y * iview._21 + v.z * iview._31;
        dir.y = v.x * iview._12 + v.y * iview._22 + v.z * iview._32;
        dir.z = v.x * iview._13 + v.y * iview._23 + v.z * iview._33;

        org.x = iview._41;
        org.y = iview._42;
        org.z = iview._43;
    }

private:
    zw::win::window_manager             manager_;
    boost::scoped_ptr< window_type >    window_;
    boost::scoped_ptr< zw::d3d >        d3d_;
    boost::scoped_ptr< zw::d3d_font >   font_;
    boost::scoped_ptr< Camera >         camera_;
    boost::scoped_ptr< px::world_type > world_;
    std::vector< boost::shared_ptr< px::cloud_type > >   clouds_;
    std::vector< boost::shared_ptr< px::block_type > >   blocks_;
    std::vector< boost::shared_ptr< px::body_type > >    bodies_;
    std::vector< boost::shared_ptr< px::body_type > >    models_;

    boost::scoped_ptr< MouseDispatcher >                mouse_dispatcher_;
    //boost::scoped_ptr<Dashboard>                      dashboard_;

    HWND                                handle_;
    size_t                              d3d_device_index_;

    float                               scale_;
    boost::scoped_ptr< PapiShape >      actor_;
    boost::scoped_ptr< Finger >         finger_;
    boost::scoped_ptr< Cloth >          cloth_;

    zw::timer                           timer_;
    bool                                show_point_;
    zw::win::offset_type                drag_point_;
    bool                                pushing_;
    bool                                cloth_dragging_;
    D3DXVECTOR3                         cloth_origin_;

};

extern "C"
int PASCAL WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow) 
{
    CoInitialize(NULL);
 
    application a;
    a.run();
    return 0;
}

