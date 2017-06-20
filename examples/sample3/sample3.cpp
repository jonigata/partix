// $Id: sample3.cpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $

#include "zw/window_manager.hpp"
#include "zw/basic_window.hpp"
#include "zw/menu.hpp"
#include "zw/d3d.hpp"
#include "zw/dprintf.hpp"
#include "zw/timer.hpp"
#include "zw/d3dmathutils.hpp"
#include "partix/partix.hpp"
#include "camera.hpp"
#include "shape.hpp"
#include "texture_cache.hpp"
#include "mouse_dispatcher.hpp"
#include "fsm_balling.hpp"
#include "ease_in_out.hpp"

/*===========================================================================*/
/*!
 * 定数・グローバル変数
 *
 *
 * 
 */
/*==========================================================================*/

const float FIELD_MASS = 1.0f;
const int MIKU_COUNT = 10;
const float MIKU_MASS = 0.01f;
const float MIKU_SCALE = 0.003f;
const int BALL_COUNT = 10;
const float BALL_MASS = 1.0f;
const float BALL_SCALE = 0.01f;
const float BALL_FLOAT = 0.20f;

/*===========================================================================*/
/*!
 * D3DXVectorTraits
 *
 *  vector traits
 */
/*==========================================================================*/

struct D3DXVectorTraits {
    typedef float           real_type;
    typedef D3DXVECTOR3     vector_type;

    static real_type epsilon(){ return real_type( 0.000001f ); }
    static real_type x( const vector_type& v ) { return v.x; }
    static real_type y( const vector_type& v ) { return v.y; }
    static real_type z( const vector_type& v ) { return v.z; }
    static void x( vector_type& v, real_type x ) { v.x = x; }
    static void y( vector_type& v, real_type y ) { v.y = y; }
    static void z( vector_type& v, real_type z ) { v.z = z; }
    static vector_type make_vector( real_type x, real_type y, real_type z )
    {
        return D3DXVECTOR3( x, y, z );
    }
    static real_type length_sq( const vector_type& v )
    {
        return D3DXVec3LengthSq( &v );
    }
    static real_type length( const vector_type& v )
    {
        return D3DXVec3Length( &v );
    }
};


/*===========================================================================*/
/*!
 * D3DXPartixTraits
 *
 *  partix用のtraitsクラス
 *  vector等はD3DXのものを使用
 */
/*==========================================================================*/

struct D3DXPartixTraits {
    typedef D3DXVectorTraits                vector_traits;
    typedef D3DXVectorTraits::real_type     real_type;
    typedef D3DXVectorTraits::vector_type   vector_type;
    typedef D3DXMATRIX                      matrix_type;
    typedef int                             index_type;

    struct body_load_type { Shape* shape; };
    struct block_load_type {};
    struct cloud_load_type {};
    struct point_load_type {};

    static float speed_drag_coefficient() { return  0.0001f; }
    static float kinetic_friction() { return 0.0f; }

    static float freeze_threshold_energy() { return 0.002f; }
    static float freeze_duration() { return 0.5f; }

    static float tick() { return 0.005f; }
    static void make_matrix(
        matrix_type& d,
        const real_type* s,
        const vector_type& t )
    {
        d._11 = s[0]; d._12 = s[3]; d._13 = s[6]; d._14 = 0;
        d._21 = s[1]; d._22 = s[4]; d._23 = s[7]; d._24 = 0;
        d._31 = s[2]; d._32 = s[5]; d._33 = s[8]; d._34 = 0;
        d._41 = t.x;  d._42 = t.y;  d._43 = t.z;  d._44 = 1;
    }
    static vector_type transform_vector(
        const matrix_type& m,
        const vector_type& v )
    {
        D3DXVECTOR4 t;
        D3DXVec3Transform( &t, &v, &m );
        return (D3DXVECTOR3&)t;
    }
};

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

    typedef D3DXPartixTraits::vector_type           vector_type;
    typedef zw::win::extent_type                    extent_type;
    typedef zw::win::bounds_type                    bounds_type;
    typedef zw::win::offset_type                    offset_type;

    typedef partix::World< D3DXPartixTraits >       world_type;
    typedef partix::Point< D3DXPartixTraits >       point_type;
    typedef partix::Cloud< D3DXPartixTraits >       cloud_type;
    typedef partix::Block< D3DXPartixTraits >       block_type;
    typedef partix::Body< D3DXPartixTraits >        body_type;
    typedef partix::SoftShell< D3DXPartixTraits >   softshell_type;
    typedef partix::SoftVolume< D3DXPartixTraits >  softvolume_type;
    typedef partix::TetrahedralMesh< D3DXPartixTraits > tetra_type;
    typedef partix::Face< D3DXPartixTraits >        face_type;

    typedef boost::shared_ptr< body_type >          body_ptr;
    typedef boost::shared_ptr< cloud_type >         cloud_ptr;
    typedef boost::shared_ptr< block_type >         block_ptr;

public:
    application()
    {
        // ウィンドウ
        window_.reset( new window_type( *this ) );
        window_->create( manager_ );
        window_->client_extent( extent_type( 512, 384 ) );
        window_->show();
        window_->title( "はちゅねボーリング" );

        auto_update_ = true;
        prev_view_point_ = D3DXVECTOR3( 0, 0, 0 );
    }
    ~application()
    {
        mouse_dispatcher_->remove_acceptor( &*camera_ );
    }

    void run()
    {
        manager_.run(*this);
    }

public:
    // WM_CREATEに対応
    void accept(zw::win::event::create& m)
    {
        handle_ = HWND(m.handle);

        d3d_.reset( new zw::d3d );
        d3d_device_index_ = d3d_->add_view(handle_);
        font_.reset( new zw::d3d_font( *d3d_, L"MS PGothic", 14 ) );
        font2_.reset( new zw::d3d_font( *d3d_, L"MS PGothic", 32 ) );

        LPDIRECT3DDEVICE9 device = d3d_->device();

        // mouse dispatcher
        mouse_dispatcher_.reset( new MouseDispatcher( handle_ ) );

        // browsing
        camera_.reset( new Camera );
        camera_->move_view_point( D3DXVECTOR3( 0, 2.00f, 10.0f ) );
        mouse_dispatcher_->add_acceptor( &*camera_, 99 );


        // texture cache
        texture_cache_.reset( new TextureCache( "data" ) );

        // bg image
        bg_ = texture_cache_->get_texture( "ponds000.jpg" );
        texture_cache_->setup( device );

        // arrow image
        arrow_ = texture_cache_->get_texture( "arrow.png" );
        texture_cache_->setup( device );

        // scene body

        // ...world
        world_.reset( new world_type );

        // ...field
        {
            std::ifstream ifs( "data/field.mqo" );
            mqo_reader::document_type doc;
            mqo_reader::read_mqo( ifs, doc );

            // ...... field view
            shape_field_.reset( new Shape( device ) );
            shape_field_->build_from_mqo(
                doc, 1.0f, D3DCOLOR_XRGB( 0, 255, 0 ), *texture_cache_  );

            // ...... field physics
            for( int i = 0 ; i < 1 ; i++ ) {
                softshell_type* v = 
                    make_shell_body( doc, "field", 16, 1.0f, FIELD_MASS );

                body_ptr e( v  );
                e->teleport( D3DXVECTOR3( 0, i * -0.1f, 0 ) );
                e->set_features( false, false, true );
                bodies_.push_back( e );
                fields_.push_back( e );
                                
                world_->add_body( v );
            }
        }

        // ...pin
        {
            std::ifstream ifs( "data/miku2_v.mqo" );
            mqo_reader::document_type doc;
            mqo_reader::read_mqo( ifs, doc );
            shape_pin_.reset( new Shape( device ) );
            shape_pin_->build_from_mqo(
                doc, MIKU_SCALE, D3DCOLOR_XRGB( 255, 255, 0 ), *texture_cache_  );
                
            D3DXVECTOR3 initial_positions[] = {
                D3DXVECTOR3(      0, 0.04f, 5.0f ),
                D3DXVECTOR3(  0.40f, 0.04f, 5.5f ),
                D3DXVECTOR3( -0.40f, 0.04f, 5.5f ),
                D3DXVECTOR3(      0, 0.04f, 6.0f ),
                D3DXVECTOR3(  0.80f, 0.04f, 6.0f ),
                D3DXVECTOR3( -0.80f, 0.04f, 6.0f ),
                D3DXVECTOR3(  0.40f, 0.04f, 6.5f ),
                D3DXVECTOR3( -0.40f, 0.04f, 6.5f ),
                D3DXVECTOR3(  1.20f, 0.04f, 6.5f ),
                D3DXVECTOR3( -1.20f, 0.04f, 6.5f ),
            };
                        

            for( int i = 0 ; i < MIKU_COUNT ; i++ ) {
                // 作成
                softvolume_type* v =
                    make_volume_body( "miku2_p", MIKU_SCALE*100, MIKU_MASS );

                // 硬さ、摩擦
                //v->set_restore_factor( 1.0f );
                //v->set_stretch_factor( 0.0f );

                tetra_type::points_type& points =
                    v->get_mesh()->get_points();
                int n = int( points.size() );
                for( int j = 0 ; j < n ; j++ ) {
                    point_type& p = points[j];
                    p.friction = 40.0f;
                }

                // 登録
                body_ptr e( v );
                e->teleport( initial_positions[i] );
                e->set_frozen( true );
                e->set_auto_freezing( true );
                e->set_global_force( D3DXVECTOR3( 0, -9.8f, 0 ) );
                bodies_.push_back( e );
                pins_.push_back( e );
                world_->add_body( v );
            }
        }

        // ...ball
        {
            std::ifstream ifs( "data/ball.mqo" );
            mqo_reader::document_type doc;
            mqo_reader::read_mqo( ifs, doc );
            shape_ball_.reset( new Shape( device ) );
            shape_ball_->build_from_mqo(
                doc, BALL_SCALE, D3DCOLOR_XRGB( 255, 255, 0 ),
                *texture_cache_  );
                
            for( int i = 0 ; i < BALL_COUNT ; i++ ) {
                softvolume_type* v =
                    make_volume_body( "ball", BALL_SCALE*100, BALL_MASS );
                v->load.shape = shape_ball_.get();
                        
                // 硬さ、摩擦
                //v->set_restore_factor( 1.0f );
                //v->set_stretch_factor( 0.0f );
                                
                tetra_type::points_type& points =
                    v->get_mesh()->get_points();
                int n = int( points.size() );
                for( int j = 0 ; j < n ; j++ ) {
                    point_type& p = points[j];
                    p.friction = 1.0f;
                }

                // 登録
                body_ptr e( v );
                v->set_global_force( D3DXVECTOR3( 0, -9.8f * 2.0f, 0 ) );
                balls_.push_back( e );
                if( i == 0 ) {
                    v->teleport( D3DXVECTOR3( 0, BALL_FLOAT, -8.0f ) );
                    world_->add_body( v );
                }
            }
            active_ball_count_ = 1;
        }

        world_->restart();

        // state machine
        fsm_.reset( new fsm_balling::StateMachine< application >( *this ) );
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
            world_->update( D3DXPartixTraits::tick() );
        }

        if( m.code == VK_ESCAPE ) {
            auto_update_ = !auto_update_;
        }

        if( m.code == 'g' ) {
            fsm_->Start();
        }
    }

    // WM_?MOUSE*に対応
    void accept(zw::win::event::mouse& m)
    {
        mouse_dispatcher_->on_mouse_message( m );
    }

public:
    // on_idle: timerのコールバック
    bool on_idle(int counter)
    {
        if( !timer_( boost::bind( &application::update,
                                  this, _1, _2 ) ) ) {
            Sleep(1);
        }

        return false;
    }

    // on_render: zw::d3dのコールバック
    void on_render( LPDIRECT3DDEVICE9 device )
    {
        // bg image
        render_bg( device );

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

        // ...pins
        render_pins( device, view_point );

        // ...balls
        render_balls( device, view_point );

        // ...field
        D3DXMATRIX mat_world;
        D3DXMatrixIdentity( &mat_world );
        device->SetTransform( D3DTS_WORLD, &mat_world );
        shape_field_->render( device );

        // frame rate
        if( font_ ) {
            extent_type ce = window_->client_extent();

            wchar_t buffer[256];
            wsprintf( buffer, L"FPS: %d", timer_.realfps() );

            font_->draw( 0, ce.height() - 16, 128, ce.height(), buffer );
        }

        // state specific 
        while( fsm_->step() );
        fsm_->Render();
        while( fsm_->step() );
    }

    // on_lost_device: zw::d3dのコールバック
    void on_lost_device( LPDIRECT3DDEVICE9 device )
    {
        shape_field_->on_lost_device( device );
        shape_pin_->on_lost_device( device );
        shape_ball_->on_lost_device( device );
        texture_cache_->on_lost_device( device );
    }

    // on_reset_device: zw::d3dのコールバック
    void on_reset_device( LPDIRECT3DDEVICE9 device )
    {
        texture_cache_->on_reset_device( device );
        shape_ball_->on_reset_device( device );
        shape_pin_->on_reset_device( device );
        shape_field_->on_reset_device( device );
    }

public:
    // StateMachineのコールバック
    template < class T >
    void unexpected_token( T t, const char* ) {}

    void Initialize()
    {
        throw_count_ = 0;
    }

    void EntryCaption()
    {
        state_timer_ = 0;
        eioi_focal.reset(
            D3DXVECTOR3( 0, 0.0f, 7.0f ),
            D3DXVECTOR3( 0, -1.0f, -4.0f ),
            2.0f );
        eioi_view.reset(
            D3DXVECTOR3( 0, 1.5f, 4.0f ),
            D3DXVECTOR3( 0, 2.0f, -10.0f ),
            2.0f );
    }
    void UpdateCaption()
    {
        state_timer_++;

        camera_->move_focal_point( eioi_focal( 1.0f / 60 ) );
        camera_->move_view_point( eioi_view( 1.0f / 60 ) );

        if( 120 < state_timer_ ) {
            fsm_->Throwing();
        }
    }
    void RenderCaption()
    {
        const wchar_t* captions[] = {
            L"第一投",
            L"第二投",
            L"第三投",
            L"第四投",
            L"第五投",
            L"第六投",
            L"第七投",
            L"第八投",
            L"第九投",
            L"第十投",
        };

        draw_center_text( captions[throw_count_] );
    }

    void EntryThrowing()
    {
        state_timer_ = 0;
        throw_offset_ = 0;
    }
    void UpdateThrowing()
    {
        state_timer_++;

        if( GetAsyncKeyState( VK_LEFT ) & 0x8000 ) {
            throw_offset_ -=0.01f;
        }
        if( GetAsyncKeyState( VK_RIGHT ) & 0x8000 ) {
            throw_offset_ +=0.01f;
        }

        softvolume_type* v = dynamic_cast< softvolume_type* >(
            balls_[0].get() );
        v->kill_inertia();
        v->teleport( D3DXVECTOR3( throw_offset_, BALL_FLOAT, -8.0f ) -
                     v->get_current_center() );
                
        if( GetAsyncKeyState( VK_UP ) & 0x8000 ) {
            throw_fullpower_ = false;
            fsm_->Rolling();
        }
                
        if( GetAsyncKeyState( 'K' ) & 0x8000 ) {
            throw_fullpower_ = true;
            fsm_->Rolling();
        }
    }
    void RenderThrowing()
    {
        struct vertex_type {
            enum { format = (D3DFVF_XYZ | D3DFVF_TEX1) };

            D3DXVECTOR3     pos;
            D3DXVECTOR2     tex0;

            void operator()(FLOAT xx,FLOAT yy,FLOAT zz,FLOAT uu, FLOAT vv)
            {
                pos.x = xx; pos.y = yy; pos.z = zz; tex0.x = uu; tex0.y = vv;
            }
        };

        vertex_type vertices[4];
        vertices[0]( -2.0f, 0, -0.25f, 0.0f, 0.0f );
        vertices[1]( -2.0f, 0,  0.25f, 1.0f, 0.0f );
        vertices[2]( -1.0f, 0, -0.25f, 0.0f, 1.0f );
        vertices[3]( -1.0f, 0,  0.25f, 1.0f, 1.0f );

        LPDIRECT3DDEVICE9 device = d3d_->device();

        D3DXMATRIX m;
        D3DXMatrixRotationY( &m,( D3DX_PI - get_throwing_angle() ) );

        D3DXMATRIX m2;
        D3DXMatrixTranslation( &m2, throw_offset_, 0.1f, -8.0f );
        m *= m2;
        device->SetTransform( D3DTS_WORLD, &m );
                
        device->SetRenderState( D3DRS_FILLMODE , D3DFILL_SOLID );
        device->SetRenderState( D3DRS_ZENABLE, D3DZB_TRUE );
        device->SetRenderState( D3DRS_ZWRITEENABLE, TRUE );
        device->SetRenderState( D3DRS_ALPHABLENDENABLE, TRUE );
        device->SetRenderState( D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA );
        device->SetRenderState( D3DRS_SRCBLEND, D3DBLEND_SRCALPHA );
        device->SetTextureStageState(
            0, D3DTSS_COLOROP, D3DTOP_SELECTARG1 );
        device->SetTextureStageState(
            0, D3DTSS_COLORARG1, D3DTA_TEXTURE );
        device->SetTextureStageState(
            0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1 );
        device->SetTextureStageState(
            0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE );
        device->SetTextureStageState(
            1, D3DTSS_COLOROP, D3DTOP_DISABLE );
        device->SetTextureStageState(
            1, D3DTSS_ALPHAOP, D3DTOP_DISABLE );
        device->SetTexture( 0, arrow_->texture );
        device->SetFVF( vertex_type::format );
        device->DrawPrimitiveUP(
            D3DPT_TRIANGLESTRIP, 2, vertices,
            sizeof( vertex_type ) );
    }

    void EntryRolling()
    {
        float angle = get_throwing_angle();

        softvolume_type* v = dynamic_cast< softvolume_type* >(
            balls_[0].get() );
        v->kill_inertia();

        if( !throw_fullpower_ ) {
            v->teleport( D3DXVECTOR3( throw_offset_, 0.5f, -8.0f ) -
                         v->get_current_center() );
            v->set_force( get_throwing_direction() * 2000.0f * BALL_MASS );
        } else {
            v->teleport( D3DXVECTOR3( 0, 2.0f, -6.0f ) -
                         v->get_current_center() );
            v->set_force(
                D3DXVECTOR3( ( rand() % 1000 - 500 ) / 2.0f, 0, 5000.0f ) *
                BALL_MASS );
            throw_repeat_ = true;
        }
    }
        
    void UpdateRolling()
    {
        float slide_power = 0;

        if( GetAsyncKeyState( VK_LEFT ) & 0x8000 ) {
            slide_power -= 5.0f;
        }
        if( GetAsyncKeyState( VK_RIGHT ) & 0x8000 ) {
            slide_power += 5.0f;
        }

        softvolume_type* v = dynamic_cast< softvolume_type* >(
            balls_[0].get() );
        v->set_force( D3DXVECTOR3( slide_power, 0, 0 ) * BALL_MASS );

        if( !throw_fullpower_ &&
            v->get_current_center().y < -2.0f ) {
            fsm_->Result();
        }

        if( GetAsyncKeyState( 'K' ) & 0x8000 ) {
            if( !throw_repeat_ ) {
                throw_repeat_ = true;
                if( active_ball_count_ < BALL_COUNT ) {
                    softvolume_type* v = dynamic_cast< softvolume_type* >(
                        balls_[active_ball_count_++].get() );

                    v->kill_inertia();
                    v->teleport( D3DXVECTOR3( 0, 2.0f, -6.0f ) -
                                 v->get_current_center() );
                    v->set_force(
                        D3DXVECTOR3(
                            ( rand() % 1000 - 500 ) / 2.0f, 0, 5000.0f )
                        * BALL_MASS );
                    v->regularize();
                    world_->add_body( v );
                }
            }
        } else {
            throw_repeat_ = false;                                
        }
    }

    void EntryResult() { state_timer_ = 0; }
    void UpdateResult()
    {
        state_timer_++;
        if( 120 < state_timer_ ) {
            throw_count_++;
            fsm_->Restart();
        }
    }

    void RenderResult()
    {
        draw_center_text( L"ガーター" );
    }

private:
    void update( int elapsed0, float elapsed1 )
    {
        //dprintf( "update: %d, %f\n", elapsed0, elapsed1 );
        //dashboard_->update( elapsed1 );

        if( auto_update_ ) {
            // モデルの数
            world_->update( D3DXPartixTraits::tick() );
            world_->update( D3DXPartixTraits::tick() );

            // TODO: 本当はinitiate後の最初の一回だけあればいい
            while( fsm_->step() );
            
            fsm_->Update();
            while( fsm_->step() );
        }

        // 重力方向
        D3DXVECTOR3 view_point;
        camera_->make_view_point( view_point );
        if( view_point != prev_view_point_ ) {
            D3DXVECTOR3 up( 0, 1.0f, 0 );
            D3DXVECTOR3 cross0 = cross( view_point, up );
            D3DXVECTOR3 cross1 = cross( cross0, view_point );
            normalize_f( cross1 );
            prev_view_point_ = view_point;
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
    softvolume_type* make_volume_body(
        const std::string& filename, float mag, float mass )
    {
        vector_type v0( 0, 0, 0 );

        tetra_type* e = new tetra_type;

        std::string basename = "data/" + filename;


        // .node(頂点座標)読み込み
        {
            std::ifstream ifs( ( basename + ".node" ).c_str() );
            int node_count, dummy;
            ifs >> node_count >> dummy >> dummy >> dummy;
            for( int i = 0 ; i < node_count ; i++ ) {
                D3DXVECTOR3 v;
                ifs >> dummy >> v.x >> v.y >> v.z;
                v.x *= mag;
                v.y *= mag;
                v.z *= mag;
                e->add_point( v, mass );
            }
        }

        // .ele(tetrahedron)読み込み
        {
            std::ifstream ifs( ( basename + ".ele" ).c_str() );
            int node_count, dummy;
            ifs >> node_count >> dummy >> dummy;
            for( int i = 0 ; i < node_count ; i++ ) {
                int i0, i1, i2, i3;
                ifs >> dummy >> i0 >> i1 >> i2 >> i3;
                e->add_tetrahedron( i0, i1, i2, i3 );
            }
        }

        // .face(外接面)読み込み
        {
            std::ifstream ifs( ( basename + ".face" ).c_str() );
            int node_count, dummy;
            ifs >> node_count >> dummy;
            for( int i = 0 ; i < node_count ; i++ ) {
                int i0, i1, i2;
                ifs >> dummy >> i0 >> i1 >> i2 >> dummy;
                e->add_face( i0, i2, i1 ); // 反転
            }
        }

        e->setup();
        softvolume_type* v = new softvolume_type;
        v->set_mesh( e );

        v->regularize();

        return v;
    }

    softshell_type* make_shell_body(
        mqo_reader::document_type& doc,
        const char*                objectname,
        int                        threshold,
        float                      scale,
        float                      mass)
    {
        // MQO version
        mqo_reader::object_type& obj = doc.objects[ objectname ];

        partix::SoftShell< D3DXPartixTraits >* e =
            new partix::SoftShell< D3DXPartixTraits >;

        // vertex
        cloud_type* c = new cloud_type;
        {
            for( mqo_reader::vertices_type::iterator i = obj.vertices.begin();
                 i != obj.vertices.end();
                 ++i ) {
                mqo_reader::vertex_type& v = *i;
                c->add_point(
                    D3DXVECTOR3( v.x, v.y, -v.z ) * scale, FIELD_MASS );
            }
        }
        clouds_.push_back( cloud_ptr( c ) );
        e->add_cloud( c );

        // index
        partix::Block< D3DXPartixTraits >* b =
            new partix::Block< D3DXPartixTraits >;
        {
            for( mqo_reader::faces_type::iterator i = obj.faces.begin();
                 i != obj.faces.end();
                 ++i ) {
                mqo_reader::face_type& f = *i;
                int* q = f.vertex_indices;
                if( f.vertex_count == 3 ) {
                    b->add_face( q[0], q[1], q[2] );
                } else if( f.vertex_count == 4 ) {
                    b->add_face( q[0], q[1], q[3] );
                    b->add_face( q[1], q[2], q[3] );
                } else {
                    assert( 0 );
                }
            }
        }

        // 分割
        divide_block( threshold, e, c, b );

        return e;
    }

    struct face_compare {
    public:
        face_compare( int a, const cloud_type::points_type& p )
            : axis( a ), points( p ) {}
        bool operator()( const partix::Face< D3DXPartixTraits >& f0,
                         const partix::Face< D3DXPartixTraits >& f1 )
        {
            const D3DXVECTOR3& v00 = points[ f0.i0 ].new_position;
            const D3DXVECTOR3& v01 = points[ f0.i1 ].new_position;
            const D3DXVECTOR3& v02 = points[ f0.i2 ].new_position;
            D3DXVECTOR3 c0 = ( v00 + v01 + v02 ) / 3.0f;

            const D3DXVECTOR3& v10 = points[ f1.i0 ].new_position;
            const D3DXVECTOR3& v11 = points[ f1.i1 ].new_position;
            const D3DXVECTOR3& v12 = points[ f1.i2 ].new_position;
            D3DXVECTOR3 c1 = ( v10 + v11 + v12 ) / 3.0f;

            switch( axis ) {
            case 0: return c0.x < c1.x; 
            case 1: return c0.y < c1.y; 
            case 2: return c0.z < c1.z; 
            default: assert(0); return false;
            }
        }

        int                             axis;
        const cloud_type::points_type&  points;
    };

    void divide_block(
        int             threshold,
        softshell_type* body,
        cloud_type*     c,
        block_type*     b )
    {
        partix::Block< D3DXPartixTraits >::faces_type& faces =
            b->get_faces();

        int n = int( faces.size() );

        if( n < threshold ) {
            if( b->get_faces().empty() ) {
                delete b;
            } else {
                b->set_cloud( c );
                b->set_body( body ); 
                b->setup();
                body->add_block( b );

                block_ptr bp( b );
                blocks_.push_back( bp );
            }
            return;
        }

        cloud_type::points_type& points = c->get_points();

        // bounding box
        D3DXVECTOR3 bbmin(  FLT_MAX,  FLT_MAX,  FLT_MAX );
        D3DXVECTOR3 bbmax( -FLT_MAX, -FLT_MAX, -FLT_MAX );
        for( int i = 0 ; i < n ; i++ ) {
            face_type& face = faces[i];
            const D3DXVECTOR3& v0 = points[ face.i0 ].new_position;
            const D3DXVECTOR3& v1 = points[ face.i1 ].new_position;
            const D3DXVECTOR3& v2 = points[ face.i2 ].new_position;
            D3DXVECTOR3 center = ( v0 + v1 + v2 ) / 3.0f;
            update_bb( bbmin, bbmax, center );
        }

        // longest axis
        int axis;
        D3DXVECTOR3 bbw = bbmax - bbmin;
        if( bbw.y <= bbw.x && bbw.z <= bbw.x ) {
            axis = 0;
        } else if( bbw.z <= bbw. y ) {
            axis = 1;
        } else {
            axis = 2;
        }
          
        // sort along axis
        std::sort(
            faces.begin(),
            faces.end(),
            face_compare( axis, c->get_points() ) );

        // divide
        block_type* b1 = new block_type;

        for( int i = n/2 ; i < n ; i++ ) {
            b1->add_face( faces[i].i0, faces[i].i1, faces[i].i2 ); 
        }
        faces.erase( faces.begin() + n/2, faces.end() );

        divide_block( threshold, body, c, b );
        divide_block( threshold, body, c, b1 );
    }

    void update_bb(
        D3DXVECTOR3& bbmin,
        D3DXVECTOR3& bbmax,
        const D3DXVECTOR3& v )
    {
        if( v.x < bbmin.x ) { bbmin.x = v.x; }
        if( v.y < bbmin.y ) { bbmin.y = v.y; }
        if( v.z < bbmin.z ) { bbmin.z = v.z; }
        if( bbmax.x < v.x ) { bbmax.x = v.x; }
        if( bbmax.y < v.y ) { bbmax.y = v.y; }
        if( bbmax.z < v.z ) { bbmax.z = v.z; }
    }

private:
    void set_view_matrix(
        D3DXMATRIX&             mat_view,
        const D3DXVECTOR3&      view_point,
        const D3DXVECTOR3&      focal_point)
    {
        // view行列
        D3DXVECTOR3 v3( 0.0f, 1.0f, 0.0f );
        D3DXMatrixLookAtLH( &mat_view, &view_point, &focal_point, &v3 );
    }

    void render_bg( LPDIRECT3DDEVICE9 device )
    {
        struct bg_vertex_type {
            enum { format = (D3DFVF_XYZRHW | D3DFVF_TEX1) };
            D3DXVECTOR4     pos;
            D3DXVECTOR2     tex0;
            void operator()( int x, int y, float u, float v )
            {
                pos.x = float(x);
                pos.y = float(y);
                pos.z = pos.w = 1;
                tex0.x = u;
                tex0.y = v;
            }
        };

        bg_vertex_type vertices[4];

        extent_type ce = window_->client_extent();
        vertices[0](      0,      0 ,0.0f, 0.0f );
        vertices[1]( ce.w(),      0 ,1.0f, 0.0f );
        vertices[2](      0, ce.h() ,0.0f, 1.0f );
        vertices[3]( ce.w(), ce.h() ,1.0f, 1.0f );

        device->SetRenderState( D3DRS_FILLMODE , D3DFILL_SOLID );
        device->SetRenderState( D3DRS_ZENABLE, D3DZB_FALSE );
        device->SetRenderState( D3DRS_ZWRITEENABLE, FALSE );
        device->SetRenderState( D3DRS_ALPHABLENDENABLE, FALSE );
        device->SetTextureStageState(
            0, D3DTSS_COLOROP, D3DTOP_SELECTARG1 );
        device->SetTextureStageState(
            0, D3DTSS_COLORARG1, D3DTA_TEXTURE );
        device->SetTextureStageState(
            0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1 );
        device->SetTextureStageState(
            0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE );
        device->SetTextureStageState(
            1, D3DTSS_COLOROP, D3DTOP_DISABLE );
        device->SetTextureStageState(
            1, D3DTSS_ALPHAOP, D3DTOP_DISABLE );
        device->SetTexture( 0, bg_->texture );
        device->SetFVF( bg_vertex_type::format );
        device->DrawPrimitiveUP(
            D3DPT_TRIANGLESTRIP, 2, vertices,
            sizeof( bg_vertex_type ) );
    }
                
    void render_pins(
        LPDIRECT3DDEVICE9       device,
        const D3DXVECTOR3&      view_point )
    {
        for( int i = 0 ; i < MIKU_COUNT ; i++ ) {
            softvolume_type* volume =
                dynamic_cast< softvolume_type* >(
                    pins_[i].get() );
            if( !volume ) { continue; }
                        
            device->SetTransform(
                D3DTS_WORLD,
                &volume->get_deformed_matrix() );

            shape_pin_->render( device );
        }
    }

    void render_balls(
        LPDIRECT3DDEVICE9       device,
        const D3DXVECTOR3&      view_point )
    {
        for( int i = 0 ; i < active_ball_count_ ; i++ ) {
            softvolume_type* volume =
                dynamic_cast< softvolume_type* >(
                    balls_[i].get() );
            if( !volume ) { continue; }
                        
            device->SetTransform(
                D3DTS_WORLD,
                &volume->get_deformed_matrix() );

            shape_ball_->render( device );
        }
    }

    void draw_center_text( const wchar_t* text )
    {
        offset_type center = window_->client_bounds().center();


        bounds_type bounds0(
            center - offset_type( 128, 16 ),
            center + offset_type( 128, 16 ) );

        bounds_type bounds1 =
            bounds0 - offset_type( 2, 2 );

        bounds_type bounds2 =
            bounds0 + offset_type( 2, 2 );

        font2_->draw_center(
            bounds1.x0(), bounds1.y0(),
            bounds1.x1(), bounds1.y1(),
            text,
            D3DCOLOR_XRGB( 0, 0, 0 ) );
        font2_->draw_center(
            bounds2.x0(), bounds2.y0(),
            bounds2.x1(), bounds2.y1(),
            text,
            D3DCOLOR_XRGB( 0, 0, 0 ) );
        font2_->draw_center(
            bounds0.x0(), bounds0.y0(),
            bounds0.x1(), bounds0.y1(),
            text );
    }

    float get_throwing_angle()
    {
        float x = cosf( state_timer_ * 0.1f );
        float angle = x * D3DX_PI / 12.0f + D3DX_PI/2;
        return angle;
    }

    D3DXVECTOR3 get_throwing_direction()
    {
        float angle = get_throwing_angle();

        return D3DXVECTOR3( cosf( angle ), 0, sin( angle ) );
    }

private:
    zw::win::window_manager                 manager_;
    boost::scoped_ptr< window_type >        window_;
    boost::scoped_ptr< zw::d3d >            d3d_;
    boost::scoped_ptr< zw::d3d_font >       font_;
    boost::scoped_ptr< zw::d3d_font >       font2_;
    boost::scoped_ptr< TextureCache >       texture_cache_;
    boost::scoped_ptr< Camera >             camera_;
    boost::scoped_ptr< Shape >              shape_field_;
    boost::scoped_ptr< Shape >              shape_pin_;
    boost::scoped_ptr< Shape >              shape_ball_;
    boost::scoped_ptr< world_type >         world_;
    std::vector< body_ptr >                 fields_;
    std::vector< body_ptr >                 balls_;
    std::vector< cloud_ptr >                clouds_;
    std::vector< block_ptr >                blocks_;
    std::vector< body_ptr >                 bodies_;
    std::vector< body_ptr >                 pins_;
    TextureHolder*                          arrow_;
    TextureHolder*                          bg_;

    boost::scoped_ptr< MouseDispatcher >    mouse_dispatcher_;

    boost::scoped_ptr< fsm_balling::StateMachine< application > > fsm_;

    HWND            handle_;
    size_t          d3d_device_index_;

    zw::timer       timer_;
    bool            auto_update_;

    D3DXVECTOR3     prev_view_point_;

    int state_timer_;
    int throw_count_;
    float throw_offset_;
    bool throw_fullpower_;
    bool throw_repeat_;
    int active_ball_count_;
    ease_in_out_interpolation< D3DXVECTOR3 > eioi_focal;
    ease_in_out_interpolation< D3DXVECTOR3 > eioi_view;
        
};

extern "C"
int PASCAL WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow) 
{
    CoInitialize(NULL);

    try {
        application a;
        a.run();
    }
    catch( std::runtime_error& e ) {
        OutputDebugStringA( e.what() );
        return 1;
    }
    return 0;
}

