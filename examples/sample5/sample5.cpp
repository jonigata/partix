// $Id: sample1.cpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $

#include "zw/window_manager.hpp"
#include "zw/basic_window.hpp"
#include "zw/menu.hpp"
#include "zw/d3d.hpp"
#include "zw/dprintf.hpp"
#include "zw/d3dmathutils.hpp"
#include "zw/timer.hpp"
#include "partix/partix.hpp"
#include "camera.hpp"
#include "mouse_dispatcher.hpp"
#include "physical_world.hpp"

/*===========================================================================*/
/*!
 * @class application
 * @brief 
 *
 * アプリケーション本体
 */
/*==========================================================================*/

class application {
public:
    typedef zw::win::basic_window<
		zw::win::event::create,
		zw::win::event::size,
		zw::win::event::paint,
		zw::win::event::mouse,
		zw::win::event::keychar
		> window_type;

    typedef zw::win::extent_type	extent_type;
    typedef zw::win::bounds_type    bounds_type;
    typedef zw::win::offset_type    offset_type;

	void init()
	{
		scene1();
	}

public:
    application()
    {
		world_ = NULL;
		auto_update_ = false;

        // ウィンドウ
        window_.reset( new window_type( *this ) );
        window_->create( manager_ );
        window_->show();
        window_->title( "sample5" );
    }
    ~application()
    {
        mouse_dispatcher_->remove_acceptor( &*camera_ );
    }

    void run()
    {
        manager_.run(*this);
    }

	void scene1()
	{
		instances_.push_back( 
			world_->create_instance( klasses_[0], D3DXVECTOR3( 0, 0, 0 ) ) );
	}

	void scene2()
	{
		instances_.push_back( 
			world_->create_instance( klasses_[2], D3DXVECTOR3( 2.9f, 0, 0 ) ) );
		instances_.push_back( 
			world_->create_instance( klasses_[2],
									 D3DXVECTOR3( 5.855f, -0.2f, 0 ) ) );
	}

public:
    // WM_CREATEに対応
    void accept(zw::win::event::create& m)
    {
        handle_ = HWND(m.handle);

        d3d_.reset( new zw::d3d );
        d3d_device_index_ = d3d_->add_view(handle_);

        LPDIRECT3DDEVICE9 device = d3d_->device();

        // mouse dispatcher
        mouse_dispatcher_.reset( new MouseDispatcher( handle_ ) );

        // browsing
        camera_.reset( new Camera );
        mouse_dispatcher_->add_acceptor( &*camera_, 99 );

		world_ = CreatePhysicalWorld();
		
		klasses_.resize(10);
		//klasses_[0] = world_->prepare_softshell( device, "data/field.mqo" );
		klasses_[0] = world_->prepare_softshell( device, "data/floor.mqo" );
		klasses_[1] = world_->prepare_softvolume( device, "data/cube" );
		klasses_[2] = world_->prepare_softvolume( device, "data/sphere" );
		klasses_[3] = world_->prepare_softvolume( device, "data/cylinder" );
		klasses_[4] = world_->prepare_softvolume( device, "data/cone" );
		klasses_[5] = world_->prepare_softvolume( device, "data/chamfer" );
		klasses_[6] = world_->prepare_softvolume( device, "data/prism" );
		klasses_[7] = world_->prepare_softvolume( device, "data/tetra" );
		klasses_[8] = world_->prepare_softvolume( device, "data/platform" );
		klasses_[9] = world_->prepare_softvolume( device, "data/platform2" );

		init();
		
		OutputDebugStringA( "world\n" );
    }

    // WM_SIZEに対応
    void accept( zw::win::event::size& m )
    {
        if( m.type == zw::win::event::size::sized ) {
            d3d_->reset(*this);
        }
    }

    // WM_PAINTに対応
    void accept( zw::win::event::paint& m )
    {
        draw();
    }

    // WM_KEY*, WM_CHARに対応
    void accept( zw::win::event::keychar& m )
    {
        if( !auto_update_ && m.code == ' ' ) {
			D3DXVECTOR3 view_point;
			camera_->make_view_point( view_point );
			world_->update( view_point );
        }

		if( '1' <= m.code && m.code <= '9' ) {
			D3DXVECTOR3 pos( 0, 5.0f, 0 );
			if( m.code == '6' ) {
				pos.z += 0.3f;
			}
			if( m.code == '9' ) {
				pos.z = 0.01f;
			}
			if( m.code == '2' ) {
				//pos.z = 0.6f;
			}

			instances_.push_back(
				world_->create_instance( klasses_[m.code - '0'], pos ) );
			world_->set_global_accel(
				instances_.back(),
				D3DXVECTOR3( 0, -9.8f * 1.0f, 0 ) );
		}

		if( m.code == 'c' ) {
			world_->clear();
			init();
		}

        if( m.code == VK_ESCAPE ) {
            auto_update_ = !auto_update_;
        }
    }

    // WM_?MOUSE*に対応
    void accept( zw::win::event::mouse& m )
    {
        mouse_dispatcher_->on_mouse_message( m );
    }

public:
    // on_idle: timerのコールバック
    bool on_idle( int counter )
    {
        if( !timer_( boost::bind( &application::update, this, _1, _2 ) ) ) {
            Sleep(1);
        }

        return false;
    }

    // on_render: zw::d3dのコールバック
    void on_render( LPDIRECT3DDEVICE9 device )
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
        device->SetRenderState( D3DRS_LIGHTING, TRUE );

        D3DLIGHT9 light;
        memset( &light, 0, sizeof( light ) );
        light.Type = D3DLIGHT_DIRECTIONAL;
        light.Diffuse.r = 1.0f;
        light.Diffuse.g = 1.0f;
        light.Diffuse.b = 1.0f;
        light.Diffuse.a = 1.0f;
        light.Ambient.r = 0.1f;
        light.Ambient.g = 0.1f;
        light.Ambient.b = 0.1f;
        light.Ambient.a = 1.0f;
        light.Direction = D3DXVECTOR3(1.0f, -1.0f, 1.0f);
        normalize_f( (D3DXVECTOR3&)light.Direction );
        device->SetLight(0, &light);
        device->LightEnable(0, TRUE);
                
        // render
		if( world_ ) {
			world_->render( device, view_point );
		}
    }

    // on_lost_device: zw::d3dのコールバック
    void on_lost_device( LPDIRECT3DDEVICE9 device )
    {
		world_->on_lost_device( device );
    }

    // on_reset_device: zw::d3dのコールバック
    void on_reset_device( LPDIRECT3DDEVICE9 device )
    {
		world_->on_reset_device( device );
    }

private:
    void update( int elapsed0, float elapsed1 )
    {
        if( auto_update_ && world_ ) {
			D3DXVECTOR3 view_point;
			camera_->make_view_point( view_point );
            world_->update( view_point );
        }

        draw();
    }

    void draw()
    {
        d3d_->check_device_lost(*this);
        d3d_->render(
            D3DCOLOR_ARGB(0xff,0x48,0x3D,0x8B),
            *this,
            d3d_device_index_);
    }

private:
    void set_view_matrix(
        D3DXMATRIX&             mat_view,
        const D3DXVECTOR3&      view_point,
        const D3DXVECTOR3&      focal_point)
    {
        // view行列
        D3DXVECTOR3 v3( 0.0f, 1.0f, 0.0f );
        D3DXMatrixLookAtLH(
            &mat_view,
            &view_point,
            &focal_point,
            &v3 );
    }

private:
    zw::win::window_manager                 manager_;
    boost::scoped_ptr< window_type >        window_;
    boost::scoped_ptr< zw::d3d >            d3d_;
    boost::scoped_ptr< Camera >             camera_;
    boost::scoped_ptr< MouseDispatcher >    mouse_dispatcher_;

	std::vector< Physie* >					klasses_;
	std::vector< Physie* >					instances_;

    HWND            handle_;
    size_t          d3d_device_index_;

    zw::timer       timer_;
    bool            auto_update_;
    int             active_model_count_;

	PhysicalWorld*	world_;

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

