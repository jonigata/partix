// $Id: sample4.cpp 34 2008-12-08 07:16:36Z Naoyuki.Hirayama $

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

/*===========================================================================*/
/*!
 * 定数・グローバル変数
 *
 *
 * 
 */
/*==========================================================================*/

const int ENTITY_COUNT = 20;
const float MIKU_MASS = 0.5f;
const float MIKU_SCALE = 0.02f;

#define DESKTOP_VIEW  D3DXVECTOR3( 0, 16.0f,     0 )
#define DESKTOP_FOCUS D3DXVECTOR3( 0,     0,     0 )
#define DESKTOP_TILT  D3DXVECTOR3( 0,     0, -0.1f )

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

    struct body_load_type {};
    struct block_load_type {};
    struct cloud_load_type {};
    struct point_load_type {};

    static float speed_drag_coefficient() { return  0.0001f; }
    static float kinetic_friction() { return 0.4f; }

    static float freeze_threshold_energy() { return 2.0f; }
    static float freeze_duration() { return 0.5f; }

    static float tick() { return 0.02f; }
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

class application : public IMouseAcceptor, public IMouseReceiver {
public:
    typedef zw::win::basic_window<
    zw::win::event::create,
    zw::win::event::size,
    zw::win::event::paint,
    zw::win::event::mouse,
    zw::win::event::keychar
    > window_type;

    typedef D3DXPartixTraits::vector_type                   vector_type;
    typedef zw::win::extent_type                            extent_type;
    typedef zw::win::bounds_type                            bounds_type;
    typedef zw::win::offset_type                            offset_type;

    typedef partix::World< D3DXPartixTraits >       world_type;
    typedef partix::Point< D3DXPartixTraits >       point_type;
    typedef partix::Cloud< D3DXPartixTraits >       cloud_type;
    typedef partix::Block< D3DXPartixTraits >       block_type;
    typedef partix::Body< D3DXPartixTraits >        body_type;
    typedef partix::SoftShell< D3DXPartixTraits >   softshell_type;
    typedef partix::SoftVolume< D3DXPartixTraits >  softvolume_type;
    typedef partix::BoundingPlane< D3DXPartixTraits > plane_type;
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

        extent_type de = desktop_extent();
        window_->client_extent( de );

        extent_type we = window_->window_extent();

        window_->window_topleft(
            offset_type(
                de.w() / 2 - we.w() / 2,
                de.h() / 2 - we.h() / 2 ) );

        window_->show();
        window_->title( "はちゅねデスクトップ" );

        prev_view_point_ = D3DXVECTOR3( 0, 0, 0 );
        selected_ = -1;
        dragging_ = false;
        old_region_ = NULL;
    }
    ~application()
    {
        mouse_dispatcher_->remove_acceptor( this );
        //mouse_dispatcher_->remove_acceptor( &*camera_ );
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
        d3d_device_index_ = d3d_->add_view( handle_, true );

        LPDIRECT3DDEVICE9 device = d3d_->device();

        // mouse dispatcher
        mouse_dispatcher_.reset( new MouseDispatcher( handle_ ) );

        // browsing
        camera_.reset( new Camera );
        camera_->move_view_point( DESKTOP_VIEW  + DESKTOP_TILT );
        //mouse_dispatcher_->add_acceptor( &*camera_, 99 );
        mouse_dispatcher_->add_acceptor( this, 10 );

        // texture cache
        texture_cache_.reset( new TextureCache( "data" ) );

        // bg image
        texture_cache_->setup( device );

        // scene body

        // ...world
        world_.reset( new world_type );

        // ...floor
        for( int i = 0 ; i < 6 ; i++ ) {
            typedef vector_type vec;

            body_ptr e( new plane_type(
                            vector_type( 0,    0, 0 ),
                            vector_type( 0, 1.0f, 0 ) ) );
            bodies_.push_back( e );
            world_->add_body( e.get() );
        }

        // ...entity
        {
            std::ifstream ifs( "data/miku2_v.mqo" );
            mqo_reader::document_type doc;
            mqo_reader::read_mqo( ifs, doc );
            shape_.reset( new Shape( device ) );
            shape_->build_from_mqo(
                doc, MIKU_SCALE, D3DCOLOR_XRGB( 255, 255, 0 ), *texture_cache_  );
                
            for( int i = 0 ; i < ENTITY_COUNT ; i++ ) {
                D3DXVECTOR3 o;
                if( i == 0 ) {
                    o.x = 1.0f;
                    o.y = 1.5f;
                    o.z = 1.0f;
                } else {
                    o.x = float( rand() % 6001 - 3000 ) / 2000;
                    o.y = float( rand() % 6001 -    0 ) / 2000;
                    o.z = float( rand() % 6001 - 3000 ) / 2000;
                }

                // 作成
                softvolume_type* v =
                    make_volume_body( shape_.get(), MIKU_SCALE*100 );

                // 硬さ、摩擦
                v->set_restore_factor( 0.3f );
                v->set_stretch_factor( 0.7f );
                                
                // 登録
                body_ptr e( v );
                e->teleport( o );
                e->set_auto_freezing( false );
                e->set_global_force( vector_type( 0, -9.8f * 2.0f, 0 ) );
                bodies_.push_back( e );
                models_.push_back( e );
            }
        }
        auto_update_ = true;
        mode_ = 0;
        active_model_count_ = 0;
        target_model_count_ = 2;

        world_->restart();

        // 透明ウィンドウ
        SetWindowLong(
            handle_,
            GWL_STYLE,
            GetWindowLong( handle_, GWL_STYLE ) ^ WS_OVERLAPPEDWINDOW );
        SetWindowLong(
            handle_,
            GWL_EXSTYLE,
            GetWindowLong( handle_, GWL_EXSTYLE ) | WS_EX_LAYERED );
        SetLayeredWindowAttributes(
            handle_,
            RGB( 255, 0, 0 ),
            0,
            LWA_COLORKEY );

        window_slide_ = false;
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
        //draw();
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

        if( m.code == 'u' ) { window_slide_ = true; }
        if( m.code == 'i' ) { window_slide_ = false; }
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
        // view matrix
        D3DXVECTOR3 focal_point = DESKTOP_FOCUS;
        D3DXVECTOR3 view_point = DESKTOP_VIEW + DESKTOP_TILT;
#if 0
        {
            softvolume_type* volume =
                dynamic_cast< softvolume_type* >(
                    models_[0].get() );
            vector_type c = volume->get_current_center();
                        
            focal_point = D3DXVECTOR3( c.x, 0, c.z );
            view_point = focal_point + DESKTOP_VIEW + DESKTOP_TILT;
        }
#endif
                
        D3DVIEWPORT9 viewport;
        device->GetViewport( &viewport );

        D3DXMATRIX mat_view;
        make_view_matrix( mat_view, view_point, focal_point );
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
        D3DXVec3Normalize( &(D3DXVECTOR3&)light.Direction, &((D3DXVECTOR3&)light.Direction) );
        device->SetLight(0, &light);
        device->LightEnable(0, TRUE);
                
        // ...entities
        render_entities( device, view_point );
    }

    // on_lost_device: zw::d3dのコールバック
    void on_lost_device( LPDIRECT3DDEVICE9 device )
    {
        shape_->on_lost_device( device );
        texture_cache_->on_lost_device( device );
    }

    // on_reset_device: zw::d3dのコールバック
    void on_reset_device( LPDIRECT3DDEVICE9 device )
    {
        texture_cache_->on_reset_device( device );
        shape_->on_reset_device( device );
    }

protected:
    // IMouseAcceptorの実装
    offset_type get_offset()
    {
        return offset_type( 0, 0 );
    }

    MouseAcceptInfo accept_mouse(
        const MouseAcceptInfo::ancestors_type&  ancestors,
        const offset_type&             position,
        MouseButtonType                         button )
    {
        MouseAcceptInfo info;
        if( button == button_left  ) {
            if( pick_entity( position ) ) {
                info.ancestors.push_back( this );
                info.receiver = this;
            }
        }
        if( button == button_right ) {
            PostQuitMessage( 0 );
        }

        return info;
    }

    void on_hover( const offset_type& )
    {
    }

    // IMouseReceiverの実装
    void on_down( const offset_type& pos )
    {
        softvolume_type* volume =
            dynamic_cast< softvolume_type* >(
                models_[0].get() );
        vector_type c = volume->get_current_center();

        drag_start_ = c;
        drag_normal_ = D3DXVECTOR3( 0, 1.0f, 0 );
        drag_dest_ = c;
        dragging_ = true;
    }
        
    void on_drag( const offset_type& client_position )
    {
        offset_type pos = window_->to_screen( client_position );
                
        D3DXMATRIX proj;
        make_desktop_proj_matrix( proj );

        extent_type de = desktop_extent();

        D3DXVECTOR3 v;
        v.x =  ( pos.dx() * 2.0f / de.w() - 1 ) / proj._11;
        v.y = -( pos.dy() * 2.0f / de.h() - 1 ) / proj._22;
        v.z = 1;

        D3DXMATRIX view;
        make_desktop_view_matrix( view );
        D3DXMATRIX m;
        D3DXMatrixInverse( &m, NULL, &view );

        D3DXVECTOR3 dir;
        dir.x  = v.x * m._11 + v.y * m._21 + v.z * m._31;
        dir.y  = v.x * m._12 + v.y * m._22 + v.z * m._32;
        dir.z  = v.x * m._13 + v.y * m._23 + v.z * m._33;

        normalize_f( dir );
        D3DXVECTOR3 s0 = DESKTOP_VIEW;
        D3DXVECTOR3 s1 = s0 + dir * 50.0f;

        float d;
        if( partix::math< D3DXPartixTraits >::test_plane_segment(
                drag_start_, drag_normal_, s0, s1, d ) ) {
            drag_dest_ = s0 + ( s1 - s0 ) * d;

            dprintf( "ds: %f, %f, %f\n", drag_start_.x, drag_start_.y, drag_start_.z );
            dprintf( "dd: %f, %f, %f\n", drag_dest_.x, drag_dest_.y, drag_dest_.z );
        }
                
    }

    void on_up( const offset_type& )
    {
        dragging_ = false;
    }


private:
    void update( int elapsed0, float elapsed1 )
    {
        //dprintf( "update: %d, %f\n", elapsed0, elapsed1 );
        //dashboard_->update( elapsed1 );

        LPDIRECT3DDEVICE9 device = d3d_->device();

        extent_type de = desktop_extent();
        bounds_type desktop_bounds( de );

        if( auto_update_ ) {
            // drag
            if( dragging_ ) {
                body_ptr selected = models_[selected_];

                D3DXVECTOR3 f =
                    drag_dest_ -
                    selected->get_current_center();
                selected->set_force( f * 2.0f );
            }

            // モデルの数
            int& amc = active_model_count_;
            if( amc < target_model_count_ ) {
                // 増やす
                for( ; amc < target_model_count_ ; amc++ ) {
                    world_->add_body( models_[amc].get() );
                }
            } else if( target_model_count_ < amc ) {
                // 減らす
                for( ; target_model_count_ < amc  ; amc-- ) {
                    world_->remove_body(
                        models_[amc-1].get() );
                }
            }

            world_->update( D3DXPartixTraits::tick() );

            // 画面外判定
            D3DXMATRIX mat_view;
            make_desktop_view_matrix( mat_view );

            D3DXMATRIX mat_proj;
            make_desktop_proj_matrix( mat_proj );

            D3DXMATRIX mat_screen;
            make_desktop_screen_matrix( mat_screen );

            D3DXMATRIX matc = mat_view * mat_proj * mat_screen;

            for( int i = 0 ; i < active_model_count_ ; i++ ) {
                // カメラ
                softvolume_type* volume =
                    dynamic_cast< softvolume_type* >(
                        models_[i].get() );
                vector_type c = volume->get_current_center();

                D3DXVECTOR3 xp;
                D3DXVec3TransformCoord( &xp, &c, &matc );

                offset_type center( int( xp.x ), int( xp.y ) );
                dprintf( "center: %d, %d\n", center.dx(), center.dy() );

                if( !desktop_bounds.contains( center ) ) {
                    volume->set_force( normalize( -c ) * 50.0f );
                }
            }
        }

        draw();
    }

    void draw()
    {
        PerformanceCounter pc( false );
        pc.print( "draw0" );
        d3d_->check_device_lost(*this);
        pc.print( "draw1" );

        D3DXMATRIX mat_view;
        make_desktop_view_matrix( mat_view );

        D3DXMATRIX mat_proj;
        make_desktop_proj_matrix( mat_proj );

        D3DXMATRIX mat_screen;
        make_desktop_screen_matrix( mat_screen );

        D3DXMATRIX matc = mat_view * mat_proj * mat_screen;

        HRGN new_region = CreateRectRgn( 0, 0, 0, 0 );

        for( int i = 0 ; i < active_model_count_ ; i++ ) {
            softvolume_type* volume =
                dynamic_cast< softvolume_type* >(
                    models_[i].get() );
            if( !volume ) { continue; }
            vector_type c = volume->get_current_center();

            // WORLD空間の1.0がSCREEN空間で何ドットにあたるかを測定
            vector_type t0( 0, c.y, 0 );
            vector_type t1( 1, c.y, 0 );
            vector_type tt0,tt1;
            D3DXVec3TransformCoord( &tt0, &t0, &matc );
            D3DXVec3TransformCoord( &tt1, &t1, &matc );
            float ul = 4.0f * ( tt1.x - tt0.x );

            D3DXVECTOR3 xp;
            D3DXVec3TransformCoord( &xp, &c, &matc );

            HRGN tmp_region = CreateRectRgn(
                int( xp.x - ul ),
                int( xp.y - ul ),
                int( xp.x + ul ),
                int( xp.y + ul ) );
            CombineRgn( new_region, new_region, tmp_region, RGN_OR );
            DeleteObject( tmp_region );
        }

        pc.print( "draw2" );
        HRGN hrgn;
        if( old_region_ ) {
            hrgn = CreateRectRgn( 0, 0, 0, 0 );
        } else {
            extent_type e = window_->client_extent();
            hrgn = CreateRectRgn( 0, 0, e.w(), e.h() );
        }
        CombineRgn( hrgn, old_region_, new_region, RGN_OR );

        pc.print( "draw3" );

        d3d_->render_with_gdi(
            //D3DCOLOR_ARGB(0xff,0x48,0x3D,0x8B),
            D3DCOLOR_ARGB(0xff,0xff,0x0,0x0),
            *this,
            d3d_device_index_,
            hrgn );

        DeleteObject( old_region_ );
        DeleteObject( hrgn );
        old_region_ = new_region;

        pc.print( "draw4" );
    }

private:
    softvolume_type* make_volume_body( Shape* shape, float mag )
    {
        vector_type v0( 0, 0, 0 );

        tetra_type* e = new tetra_type;

        // .node(頂点座標)読み込み
        {
            std::ifstream ifs( "data/miku2_p.node" );
            int node_count, dummy;
            ifs >> node_count >> dummy >> dummy >> dummy;
            for( int i = 0 ; i < node_count ; i++ ) {
                D3DXVECTOR3 v;
                ifs >> dummy >> v.x >> v.y >> v.z;
                v.x *= mag;
                v.y *= mag;
                v.z *= mag;
                e->add_point( v, MIKU_MASS );
            }
        }

        // .ele(tetrahedron)読み込み
        {
            std::ifstream ifs( "data/miku2_p.ele" );
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
            std::ifstream ifs( "data/miku2_p.face" );
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

private:
    void make_view_matrix(
        D3DXMATRIX&             mat_view,
        const D3DXVECTOR3&      view_point,
        const D3DXVECTOR3&      focal_point)
    {
        // view行列
        D3DXMatrixLookAtLH(
            &mat_view,
            &view_point,
            &focal_point,
            &D3DXVECTOR3( 0.0f, 0.0f, 1.0f ) );
    }

    void render_entities(
        LPDIRECT3DDEVICE9       device,
        const D3DXVECTOR3&      view_point )
    {
        for( int i = 0 ; i < active_model_count_ ; i++ ) {
            softvolume_type* volume =
                dynamic_cast< softvolume_type* >(
                    models_[i].get() );
            if( !volume ) { continue; }
                        
            device->SetTransform(
                D3DTS_WORLD,
                &volume->get_deformed_matrix() );

            shape_->render( device );
        }
    }

    void make_desktop_view_matrix( D3DXMATRIX& mat_view )
    {
        // ウィンドウ位置
        // view matrix
        D3DXMatrixLookAtLH(
            &mat_view,
            &DESKTOP_VIEW,
            &DESKTOP_FOCUS,
            &vector_type( 0.0f, 0.0f, 1.0f ) );
    }

    void make_desktop_proj_matrix( D3DXMATRIX& mat_projection )
    {
        extent_type de = desktop_extent();

        D3DXMatrixPerspectiveFovLH( 
            &mat_projection, 
            60.0f * D3DX_PI / 180.0f, 
            float( de.w() ) / de.h(), 
            0.1f, 
            100.0f );
    }

    void make_desktop_screen_matrix( D3DXMATRIX& mat_screen )
    {
        LPDIRECT3DDEVICE9 device = d3d_->device();

        D3DVIEWPORT9 vp;
        device->GetViewport( &vp );

        extent_type de = desktop_extent();

        float vw = float( vp.Width / 2 );
        float vh = float( vp.Height / 2 );
        float dw = float( de.w() / 2 );
        float dh = float( de.h() / 2 );
        float r = float( vp.MaxZ / ( vp.MaxZ - vp.MinZ ) );
        mat_screen = D3DXMATRIX(
            vw, 0, 0, 0,
            0, -vh, 0, 0,
            0, 0, r, 0,
            dw, dh, -r * vp.MinZ, 1 );
    }

    bool pick_entity( const offset_type& position )
    {
        // pick
        D3DXMATRIX proj;
        d3d_->device()->GetTransform(
            D3DTS_PROJECTION,  &proj );

        int viewport_w = window_->client_extent().width();
        int viewport_h = window_->client_extent().height();

        D3DXVECTOR3 v;
        v.x =  ( position.dx() * 2.0f / viewport_w - 1 ) / proj._11;
        v.y = -( position.dy() * 2.0f / viewport_h - 1 ) / proj._22;
        v.z = 1;

        D3DXMATRIX view;
        d3d_->device()->GetTransform( D3DTS_VIEW, &view );

        D3DXVECTOR3 origin;
        camera_->make_view_point( origin );
                
        D3DXVECTOR3 dir;
        dir.x  = v.x * view._11 + v.y * view._12 + v.z * view._13;
        dir.y  = v.x * view._21 + v.y * view._22 + v.z * view._23;
        dir.z  = v.x * view._31 + v.y * view._32 + v.z * view._33;

        normalize_f( dir );
        D3DXVECTOR3 s0 = origin;
        D3DXVECTOR3 s1 = s0 + dir * 50.0f;

        partix::Body< D3DXPartixTraits >* body =
            world_->pick( s0, s1 );

        selected_ = -1;
        for( size_t i = 0 ; i < models_.size() ; i++ ) {
            if( models_[i].get() == body ) {
                selected_ = int( i );
            }
        }

        if( 0 <= selected_ ) { 
            body_ptr sel = models_[selected_];

            if( sel->classid() == partix::BODY_ID_SOFTVOLUME ) {
                return true;
            }
        }
        return false;
    }

    extent_type desktop_extent()
    {
        RECT r;
        SystemParametersInfo( SPI_GETWORKAREA, 0, &r, 0 );
        return extent_type( r.right - r.left, r.bottom - r.top );
    }

private:
    zw::win::window_manager                 manager_;
    boost::scoped_ptr< window_type >        window_;
    boost::scoped_ptr< zw::d3d >            d3d_;
    boost::scoped_ptr< TextureCache >       texture_cache_;
    boost::scoped_ptr< Camera >             camera_;
    boost::scoped_ptr< Shape >              shape_;
    boost::scoped_ptr< world_type >         world_;
    std::vector< cloud_ptr >                clouds_;
    std::vector< block_ptr >                blocks_;
    std::vector< body_ptr >                 bodies_;
    std::vector< body_ptr >                 models_;

    boost::scoped_ptr< MouseDispatcher >    mouse_dispatcher_;

    HWND            handle_;
    size_t          d3d_device_index_;

    zw::timer       timer_;
    bool            auto_update_;
    int             mode_;
    int             selected_;
    int             target_model_count_;
    int             active_model_count_;

    D3DXVECTOR3     prev_view_point_;

    bool            dragging_;
    D3DXVECTOR3     drag_start_;
    D3DXVECTOR3     drag_normal_;
    D3DXVECTOR3     drag_dest_;

    bool            window_slide_;
    HRGN            old_region_;
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

