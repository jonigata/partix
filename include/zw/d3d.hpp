#if !defined(D3D_HPP)
#define D3D_HPP

#include <stdexcept>
#include <vector>
#include <d3d9.h>
#include <d3d9types.h>
#include <d3dx9core.h>
#include "window_base.hpp"

#pragma comment(lib,"d3d9.lib")
#ifdef _DEBUG
#pragma comment(lib,"d3dx9d.lib")
#else
#pragma comment(lib,"d3dx9.lib")
#endif

// module: d3d

namespace zw {

class d3d_exception : public std::runtime_error {
public:
        d3d_exception(const char* s) : std::runtime_error(s){}
};

template <class Vertex> class d3d_vertexbuffer;
template <class Vertex> class d3d_indexbuffer;

template <class T>
class com_ptr {
public:
        com_ptr(){ptr_=NULL;}
        com_ptr(T* x) : ptr_(x){}
        com_ptr(const com_ptr<T>& x) : ptr_(x.ptr_){if(ptr_)ptr_->AddRef();}
        ~com_ptr(){if(ptr_)ptr_->Release();}
        void operator=(const com_ptr<T>& x)
        {
                if(ptr_!=NULL)ptr_->Release();
                ptr_=x.ptr_;if(ptr_)ptr_->AddRef();
        }
        void operator=(T* p)
        {
                if(ptr_!=NULL)ptr_->Release();
                ptr_=p;
        }
        T* operator->(){return ptr_;}
        T* get(){return ptr_;}
private:
        T* ptr_;
};

class d3d_resource {
protected:
        class handler {
        public:
                virtual ~handler() { }
                virtual void invoke( LPDIRECT3DDEVICE9 ) = 0;
        };
        


protected:
        d3d_resource() { d3d_ = NULL; lost_handler_ = NULL; reset_handler_ = NULL; }
        d3d_resource( class d3d& );
        ~d3d_resource();

        void set_lost_handler( handler* h )
        { 
                if( lost_handler_ ) { delete lost_handler_; }
                lost_handler_ = h;
        }

        void set_reset_handler( handler* h )
        { 
                if( reset_handler_ ){ delete reset_handler_; }
                reset_handler_ = h ; 
        }

        void on_lost_device( LPDIRECT3DDEVICE9 d )
        {
                if( lost_handler_ ) { 
                        lost_handler_->invoke( d ) ; 
                }
        }

        void on_reset_device(LPDIRECT3DDEVICE9 d)
        {
                if(reset_handler_){
                        reset_handler_->invoke(d);
                }
        }

        friend class d3d;

private:
        d3d*            d3d_;
        handler*        lost_handler_;
        handler*        reset_handler_;
};

/*============================================================================
 *
 * class d3d
 *
 * 
 *
 *==========================================================================*/

class d3d {
public:
        typedef win::extent_type extent_type;

public:
        d3d()
        {
                init();
        }

        d3d( HWND w, bool lockable_backbuffer = false )
        {
                init();
                add_view_aux( w, lockable_backbuffer );
        }

        ~d3d()
        {
                for( size_t i = 0 ; i< swapchains_.size() ; i++ ) { 
                        swapchains_[ i ]->Release() ; 
                }
                if( device_ != NULL )device_->Release() ; 
                if( d3d_ != NULL ) d3d_->Release();
        }

        size_t add_view( HWND w, bool lockable_backbuffer = false )
        {
                return add_view_aux( w, lockable_backbuffer );
        }

        template <class T>
        void reset( T& t )
        {
                t.on_lost_device( device_ );
                on_lost_device();
                do_reset();
                on_reset_device();
                t.on_reset_device( device_ );
        }

        template <class T>
        void check_device_lost( T& t )
        { 
                if( device_lost_ && restore_device( t ) ) { 
                        device_lost_ = false ; 
                }
        }
        
        template < class Renderer >
        void render( Renderer& r )
        { 
                render( r, 0 ) ; 
        }
        
        template < class Renderer >
        void render( D3DCOLOR bg_color, Renderer& r )
        {
                render( bg_color, r, 0 );
        }
        
        template < class Renderer >
        void render( Renderer& r, size_t n )
        {
                assert( n< swapchains_.size() );

                IDirect3DSurface9* back_buffer; 
                swapchains_[ n ]->GetBackBuffer( 0, D3DBACKBUFFER_TYPE_MONO, &back_buffer );

                device_->SetRenderTarget( 0, back_buffer );
                set_viewport_aux( n );

                device_->BeginScene();
                r.on_render( device_ ) ; 
                device_->EndScene() ; 

                if( FAILED( swapchains_[ n ]->Present( NULL, NULL, NULL, NULL, 0 ) ) ) { 
                        device_lost_ = true ; 
                }

                back_buffer->Release();
        }

        template <class Renderer>
        void render( D3DCOLOR bg_color, Renderer& r, size_t n )
        {
                assert( n< swapchains_.size() );

                IDirect3DSurface9* back_buffer;
                swapchains_[ n ]->GetBackBuffer( 0, D3DBACKBUFFER_TYPE_MONO, &back_buffer );

                device_->SetRenderTarget( 0, back_buffer ) ; 
                set_viewport_aux( n );

                device_->Clear( 0, NULL, D3DCLEAR_TARGET|D3DCLEAR_ZBUFFER, bg_color, 1.0f, 0 );
                device_->BeginScene();
                r.on_render( device_ );
                device_->EndScene();

                if( FAILED( swapchains_[ n ]->Present( NULL, NULL, NULL, NULL, 0 ) ) ) { 
                        device_lost_ = true ; 
                }

                back_buffer->Release();
        }

        template < class Renderer >
        void render_with_gdi( D3DCOLOR bg_color, Renderer& r )
        { 
                render_with_gdi( bg_color, r, 0, NULL ) ; 
        }
        
        template < class Renderer >
        void render_with_gdi( D3DCOLOR bg_color, Renderer& r, size_t n )
        { 
                render_with_gdi( bg_color, r, n, NULL ) ; 
        }
        
        template < class Renderer >
        void render_with_gdi( D3DCOLOR bg_color, Renderer& r, size_t n, HRGN hrgn )
        {
                assert( n< swapchains_.size() );

                IDirect3DSurface9* back_buffer; 
                swapchains_[ n ]->GetBackBuffer( 0, D3DBACKBUFFER_TYPE_MONO, &back_buffer );

                device_->SetRenderTarget( 0, back_buffer );
                set_viewport_aux( n );

                device_->Clear( 0, NULL, D3DCLEAR_TARGET|D3DCLEAR_ZBUFFER, bg_color, 1.0f, 0 );
                device_->BeginScene();
                r.on_render( device_ ) ; 
                device_->EndScene() ; 

                extent_type e = display_extent( n );
                HDC window_dc, d3d_dc;
                window_dc = GetDC( windows_[n] );
                SelectClipRgn( window_dc, hrgn );
                back_buffer->GetDC( &d3d_dc );
                BitBlt( window_dc, 0, 0, e.w(), e.h(), d3d_dc, 0, 0, SRCCOPY );
                back_buffer->ReleaseDC( d3d_dc );
                SelectClipRgn( window_dc, NULL );
                ReleaseDC( windows_[n], window_dc );

                back_buffer->Release();
        }


        extent_type display_extent( size_t n = 0 )
        {
                assert( n< swapchains_.size() );

                D3DPRESENT_PARAMETERS pp;
                swapchains_[ n ]->GetPresentParameters( &pp );

                return extent_type( pp.BackBufferWidth, pp.BackBufferHeight );
        }


        LPDIRECT3DDEVICE9   device() { return device_; }

private:
        void init()
        {
                // Direct3Dオブジェクトの作成
                if( ( d3d_ = ::Direct3DCreate9( D3D_SDK_VERSION ) ) == NULL ) { 
                        throw d3d_exception( "Direct3DCreate9 failed" ) ; 
                }

                window_mode_ = true ; 
                device_lost_ = false ; 
                resource_lost_ = false ; 
                device_ = NULL;
        }

        size_t add_view_aux( HWND w, bool lockable_backbuffer )
        {
                D3DPRESENT_PARAMETERS pp;
                make_present_parameters( pp, w, lockable_backbuffer );

                if( swapchains_.empty() ) { 
                        // デバイスの作成
                        if( FAILED( d3d_->CreateDevice( 
                                          D3DADAPTER_DEFAULT, 
                                          D3DDEVTYPE_HAL, 
                                          w, 
                                          D3DCREATE_HARDWARE_VERTEXPROCESSING, 
                                          &pp, 
                                          &device_ ) ) ) { 
                                if( FAILED( d3d_->CreateDevice( 
                                                  D3DADAPTER_DEFAULT, 
                                                  D3DDEVTYPE_HAL, 
                                                  w, 
                                                  D3DCREATE_SOFTWARE_VERTEXPROCESSING, 
                                                  &pp, 
                                                  &device_ ) ) ) { 
                                        throw d3d_exception( 
                                                "CreateDevice failed" ) ; 
                                }
                        }

                        IDirect3DSwapChain9* swapchain ; 
                        device_->GetSwapChain( 0, &swapchain ) ; 
                        swapchains_.push_back( swapchain ) ; 
                        windows_.push_back( w );
                } else {
                        IDirect3DSwapChain9* swapchain;
                        if(FAILED(device_->CreateAdditionalSwapChain(
                                          &pp,
                                          &swapchain))){
                                throw d3d_exception(
                                        "CreateAdditionalSwapChain failed");
                        }
                        swapchains_.push_back(swapchain);
                        windows_.push_back( w );
                }
                
                return swapchains_.size()-1;
        }

        void make_present_parameters( D3DPRESENT_PARAMETERS& pp, HWND w, bool lockable_backbuffer )
        { 
                memset( &pp, 0, sizeof( pp ) ) ; 
                pp.hDeviceWindow = w ; 
		pp.Windowed			= TRUE ; 
		pp.SwapEffect			= D3DSWAPEFFECT_DISCARD ; 
		pp.BackBufferFormat		= D3DFMT_UNKNOWN ; 
		pp.EnableAutoDepthStencil	= TRUE ; 
		pp.AutoDepthStencilFormat       = D3DFMT_D16 ; 
                if( lockable_backbuffer ) {
                        pp.Flags = D3DPRESENTFLAG_LOCKABLE_BACKBUFFER;
                }
        }

        void do_reset()
        {
                std::vector< D3DPRESENT_PARAMETERS > v;
                for( size_t i = 0 ; i< swapchains_.size() ; i++ ) { 
                        D3DPRESENT_PARAMETERS pp ; 
                        swapchains_[ i ]->GetPresentParameters( &pp ) ; 
                        if( pp.Windowed ) { 
                                pp.BackBufferWidth = 0 ; 
                                pp.BackBufferHeight = 0 ; 
                        }
                        v.push_back( pp ) ; 
                        swapchains_[ i ]->Release() ; 
                }

                v[0].BackBufferFormat = D3DFMT_UNKNOWN;

                HRESULT r;
                if( FAILED( r = device_->Reset( &v[ 0 ] ) ) ) { 
                        throw d3d_exception( "Reset failed" ) ; 
                }

                swapchains_.clear();
                IDirect3DSwapChain9* swapchain ; 
                device_->GetSwapChain( 0, &swapchain ) ; 
                swapchains_.push_back( swapchain ) ; 
                for( size_t i = 1 ; i< v.size() ; i++ ) { 
                        device_->CreateAdditionalSwapChain( &v[ i ], &swapchain ) ; 
                        swapchains_.push_back( swapchain ) ; 
                }
        }

        void on_lost_device()
        {
                for( std::vector< d3d_resource* >::iterator i = resources_.begin() ; i != resources_.end() ; ++i ) { 
                        ( *i )->on_lost_device( device_ ) ; 
                }
        }

        void on_reset_device()
        { 
                for( std::vector< d3d_resource* >::iterator i = resources_.begin() ; i != resources_.end() ; ++i ) { 
                        ( *i )->on_reset_device( device_ ) ; 
                }
        }

        template < class T >
        bool restore_device( T& t )
        {
                HRESULT hr ; 
                if( !FAILED( hr = device_->TestCooperativeLevel() ) ){
                        // ロストしていない
                        return true;
                }
                
                if( hr == D3DERR_DEVICELOST ){
                        // 復元不可能
                        return false;
                }
                
                if( hr != D3DERR_DEVICENOTRESET ) { 
                        // あり得ない
                        throw d3d_exception( "unknown error ( TestCooperativeLevel )" ) ; 
                }

                // 復元可能
                if( !resource_lost_ ) { 
                        t.on_lost_device( device_ ); 
                        on_lost_device(); 
                        resource_lost_ = true ; 
                }

                do_reset();

                on_reset_device();
                t.on_reset_device( device_ );
                resource_lost_ = false;
                return true;
        }

        void set_viewport_aux(size_t n)
        {
                assert(n<swapchains_.size());

                D3DPRESENT_PARAMETERS pp;
                swapchains_[ n ]->GetPresentParameters(&pp);

                D3DVIEWPORT9 viewport;
                viewport.X      = 0 ; 
                viewport.Y      = 0 ; 
                viewport.Width  = pp.BackBufferWidth ; 
                viewport.Height = pp.BackBufferHeight ; 
                viewport.MinZ   = 0.0f; 
                viewport.MaxZ   = 1.0f;
                device_->SetViewport( &viewport );

                D3DXMATRIX mat_world;
                D3DXMatrixScaling( &mat_world, 1.0f, 1.0f, 1.0f );

                D3DXMATRIX mat_view;
                D3DXMatrixLookAtLH( 
                        &mat_view, 
                        &D3DXVECTOR3( 0, 0, 0 ), 
                        &D3DXVECTOR3( 0.0f, 0.0f, 0.0f ), 
                        &D3DXVECTOR3( 0.0f, 1.0f, 0.0f ) );

                D3DXMATRIX mat_proj;
                D3DXMatrixPerspectiveFovLH( 
                        &mat_proj, 
                        60.0f*D3DX_PI/180.0f, 
                        float( viewport.Width )/viewport.Height, 
                        0.1f, 
                        100.0f );

                device_->SetTransform( D3DTS_WORLD,             &mat_world );
                device_->SetTransform( D3DTS_VIEW,              &mat_view );
                device_->SetTransform( D3DTS_PROJECTION,        &mat_proj );
        }


protected:
        d3d( const d3d& ){}
        d3d& operator = ( const d3d& ) { return *this ; }

        friend class d3d_resource;

        void add_resource( d3d_resource* r )
        {
                resources_.push_back(r);
        }

        void remove_resource( d3d_resource* r )
        { 
                resources_.erase( std::find( resources_.begin(), resources_.end(), r ) ) ; 
        }

private:
        LPDIRECT3D9                     d3d_;
        bool                            window_mode_;
        bool                            device_lost_;
        bool                            resource_lost_;
        LPDIRECT3DDEVICE9               device_;
        std::vector<IDirect3DSwapChain9*> swapchains_;
        std::vector<HWND>               windows_;
        std::vector<d3d_resource*>      resources_;

};

inline
d3d_resource::d3d_resource( class d3d& x ) : d3d_( &x )
{
        lost_handler_ = NULL ; 
        reset_handler_ = NULL;
        d3d_->add_resource( this ) ; 
}

inline
d3d_resource::~d3d_resource()
{
        if( d3d_ ) {
                d3d_->remove_resource( this ) ;
        }
        if( lost_handler_ ) delete lost_handler_ ; 
        if( reset_handler_ ) delete reset_handler_;
}

/*============================================================================
 *
 * class d3d_vertexbuffer
 *
 * 
 *
 *==========================================================================*/

template <class VertexBuffer> class const_d3d_vertexbuffer_explorer;
template <class VertexBuffer> class d3d_vertexbuffer_explorer;

template < class Vertex >
class d3d_vertexbuffer : public d3d_resource {
public:
        typedef Vertex                                          vertex_type;
        typedef d3d_vertexbuffer<vertex_type>                   self_type;
        typedef const_d3d_vertexbuffer_explorer<self_type>      const_explorer_type;
        typedef d3d_vertexbuffer_explorer<self_type>            explorer_type;

public:
        d3d_vertexbuffer( d3d& x, size_t count )
                : d3d_resource( &x )
        {
                count_ = UINT(count);

                // 頂点バッファ作成
                vb_ = NULL ; 
                if( FAILED( x.device()->CreateVertexBuffer( 
                                    count_*sizeof( Vertex ), 
                                    0, 
                                    Vertex::format(), 
                                    D3DPOOL_DEFAULT, 
                                    &vb_, 
                                    NULL ) ) ){
                        throw d3d_exception( "CreateVertexBuffer failed" );
                }
                vb_->GetDesc( &desc_ );
        }
        ~d3d_vertexbuffer()
        {
                remove_from(*d3d_);
                vb_->Release();
        }

        IDirect3DVertexBuffer9* retrieve() const { return vb_ ; }
        UINT size() { return count_ ; }

protected:
        void lost_device() { vb_->Release() ; vb_ = NULL ; }
        void reset_device() { }

private:
        IDirect3DVertexBuffer9* vb_;
        D3DVERTEXBUFFER_DESC    desc_;
        UINT                    count_;

private:
        friend class const_d3d_vertexbuffer_explorer<self_type>;
        friend class d3d_vertexbuffer_explorer<self_type>;
        const D3DVERTEXBUFFER_DESC*   desc()const     {return &desc_;}
};

template < class VertexBuffer >
class const_d3d_vertexbuffer_lock {
public:
        typedef VertexBuffer                                    vertexbuffer_type;
        typedef typename vertexbuffer_type::vertex_type         vertex_type;
        typedef typename const vertexbuffer_type::vertex_type*  const_iterator;

public:
        const_d3d_vertexbuffer_lock( const vertexbuffer_type& vb )
                : vb_(vb)
        { 
                buffer_ = NULL ; 
                if( FAILED( vb_.retrieve()->Lock( 0, 0, ( void** )&buffer_, 0 ) ) ) { 
                        throw d3d_exception( "Lock vertexbuffer failed" ) ; 
                }
        }
        ~const_d3d_vertexbuffer_lock()
        {
                if( buffer_ != NULL ) { 
                        vb_.retrieve()->Unlock() ; 
                }
        }

        size_t          size()  const { return vb_.desc()->Size/sizeof( vertex_type ) ; }
        const_iterator  begin() const { return buffer_ ; }
        const_iterator  end()   const { return buffer_+size() ; }

        const vertex_type& operator[ ]( int index ) const { return *( buffer_+index ) ; }
        const vertex_type& operator[ ]( size_t index ) const { return *( buffer_+index ) ; }

protected:
        const vertexbuffer_type&    vb_;
        vertex_type*                buffer_;

};

template < class VertexBuffer >
class d3d_vertexbuffer_lock : public const_d3d_vertexbuffer_lock<VertexBuffer> {
public:
        typedef typename vertexbuffer_type::vertex_type* iterator;

public:
        d3d_vertexbuffer_lock( vertexbuffer_type& vb )
                : const _d3d_vertexbuffer_lock< vertexbuffer_Type >( vb )
        {
        }
        ~d3d_vertexbuffer_lock() { }

        iterator        begin() { return buffer_ ; }
        iterator        end()   { return buffer_+size() ; }

        vertex_type& operator[] ( int index ) { return *( buffer_+index ) ; }
        vertex_type& operator[] ( size_t index ) { return *( buffer_+index ) ; }

private:

};


/*============================================================================
 *
 * class d3d_indexbuffer
 *
 * 
 *
 *==========================================================================*/

template < class IndexBuffer > class const_d3d_indexbuffer_lock;
template < class IndexBuffer > class d3d_indexbuffer_lock;

template <class Index>
class d3d_indexbuffer : public d3d_resource {
public:
        typedef Index                           index_type;
        typedef d3d_indexbuffer<index_type>     self_type;

public:
        d3d_indexbuffer( d3d& x, size_t count ) : d3d_resource( &x )
        {
                assert( sizeof( Index ) == 2 || sizeof( Index ) == 4 );

                count_ = UINT( count );

                ib_ = NULL;
                if( FAILED( x.device()->CreateIndexBuffer( 
                                    count_*sizeof( Index ), 
                                    0, 
                                    sizeof( Index ) == 2 ? D3DFMT_INDEX16 : D3DFMT_INDEX32, 
                                    D3DPOOL_DEFAULT, 
                                    &ib_, 
                                    NULL ) ) ) { 
                        throw d3d_exception( "CreateIndexBuffer failed" ) ; 
                }
                ib_->GetDesc( &desc_ );
        }
        ~d3d_indexbuffer()
        {
                if( ib_ != NULL ) { 
                        ib_->Release() ; 
                }
        }

        IDirect3DIndexBuffer9* retrieve() const { return ib_ ; }
        UINT size() { return count_ ; }
        
protected:
        void on_lost_device( LPDIRECT3DDEVICE9 ) { ib_->Release() ; ib_ = NULL ; }
        void on_reset_device( LPDIRECT3DDEVICE9 ) { }

private:
        IDirect3DIndexBuffer9*  ib_;
        D3DINDEXBUFFER_DESC     desc_;
        UINT                    count_;

private:
        friend class const_d3d_indexbuffer_lock<self_type>;
        friend class d3d_indexbuffer_lock<self_type>;
        D3DINDEXBUFFER_DESC*   desc()const     {return &desc_;}
};

template <class IndexBuffer>
class const_d3d_indexbuffer_lock {
public:
        typedef IndexBuffer                                     indexbuffer_type;
        typedef typename indexbuffer_type::index_type           index_type;
        typedef typename const indexbuffer_type::index_type*    const_iterator;

public:
        const_d3d_indexbuffer_lock( const indexbuffer_type& ib )
                : ib_(ib)
        {
                if( FAILED( ib_.retrieve()->Lock( 0, 0, ( void** )&buffer_, 0 ) ) ){
                        throw d3d_exception("lock index_buffer failed");
                }
        }
        ~const_d3d_indexbuffer_lock()
        {
                ib_.retrieve()->Unlock();
        }

        size_t          size()  const { return ib_.desc()->Size/sizeof( Index ) ; }
        const_iterator  begin() const { return buffer_ ; }
        const_iterator  end()   const { return buffer_+size() ; }

        const index_type& operator[]( int index ) const { return *( buffer_+index ) ; }
        const index_type& operator[]( size_t index ) const { return *( buffer_+index ) ; }

protected:
        const indexbuffer_type& ib_;
        index_type*             buffer_;

};

template <class IndexBuffer>
class d3d_indexbuffer_lock : public const_d3d_indexbuffer_lock<IndexBuffer> {
public:
        typedef typename indexbuffer_type::index_type* iterator;

public:
        d3d_indexbuffer_lock( indexbuffer_type& ib )
                : const_d3d_indexbuffer_lock(ib)
        {
        }
        ~d3d_indexbuffer_lock(){}

        iterator        begin() { return buffer_ ; }
        iterator        end()   { return buffer_+size() ; }

        index_type& operator[]( int index ) { return *( buffer_+index ) ; }
        index_type& operator[]( size_t index ) { return *( buffer_+index ) ; }
private:

};


/*============================================================================
 *
 * class d3d_texture
 *
 * 
 *
 *==========================================================================*/

class d3d_texture : public d3d_resource {
public:
        typedef win::extent_type extent_type;
        typedef win::bounds_type bounds_type;

        friend class creator;
        friend class destroyer;

        class creator : public handler {
        public:
                creator( d3d_texture* x ) : x_( x ) {}
                void invoke( LPDIRECT3DDEVICE9 d )
                {
                        d->CreateTexture( 
                                UINT( x_->info_.Width ), UINT( x_->info_.Height ), x_->info_.MipLevels, 0, 
                                D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &x_->texture_, NULL );
                }
        private:
                d3d_texture* x_;
        };

        class destroyer : public handler { 
        public:
                destroyer( d3d_texture* x ) : x_( x ) { }
                void invoke( LPDIRECT3DDEVICE9 d )
                { 
                        x_->texture_->Release() ; 
                        x_->texture_ = NULL;
                }
        private:
                d3d_texture* x_ ; 
        };

        class reloader : public handler {
        public:
                reloader( d3d_texture* x ) : x_( x ) { }
                void invoke( LPDIRECT3DDEVICE9 d )
                {
                        assert( !x_->texture_ ) ; 
                        if( D3DXCreateTextureFromFileExA( 
                                   d, x_->filename_.c_str(), 0, 0, 
                                   x_->info_.MipLevels, 0, 
                                   D3DFMT_UNKNOWN, D3DPOOL_DEFAULT, 
                                   D3DX_DEFAULT, D3DX_DEFAULT, 0, 
                                   &x_->info_, NULL, &x_->texture_ ) != D3D_OK ) { 
                                throw d3d_exception( 
                                        "D3DXCreateTextureFromFile failed" ) ; 
                        }
                }
        private:
                d3d_texture* x_;
        };

        template < class F >
        class custom_initializer : public handler {
        public:
                custom_initializer( d3d_texture* x, F f ) : x_( x ), f_( f ) { }
                void invoke( LPDIRECT3DDEVICE9 d )
                {
                        f_( d, &x_ );
                }
        private:
                d3d_texture*    x_;
                F               f_;
        };
        
public:
        d3d_texture( d3d& x, size_t w, size_t h, int miplevels = 1 )
                : d3d_resource( x )
        {
                assert( x.device() );
                info_.Width     = UINT( w );
                info_.Height    = UINT( h );
                info_.MipLevels = UINT( miplevels );
                x.device()->CreateTexture( 
                        UINT( w ), UINT( h ), miplevels, 0, 
                        D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &texture_, NULL );
                set_lost_handler( new destroyer( this ) );
                set_reset_handler( new creator( this ) );
        }

        template < class F >
        d3d_texture( d3d& x, F intializer, size_t w, size_t h, int miplevels = 1 )
                : d3d_resource( x )
        {
                assert( x.device() );
                info_.Width     = UINT( w );
                info_.Height    = UINT( h );
                info_.MipLevels = UINT( miplevels );
                x.device()->CreateTexture( 
                        UINT( w ), UINT( h ), miplevels, 0, 
                        D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT, &texture_, NULL );
                set_lost_handler( new destroyer( this ) );
                set_reset_handler( new custom_initializer( this, initializer ) );
        }



        d3d_texture( d3d& x, const char* filename, int miplevels = 1 )
                : d3d_resource( x ), filename_( filename )
        {
                texture_ = NULL;
                if( D3DXCreateTextureFromFileExA( 
                           x.device(), filename, 0, 0, miplevels, 0, 
                           D3DFMT_UNKNOWN, D3DPOOL_DEFAULT, 
                           D3DX_DEFAULT, D3DX_DEFAULT, 0, 
                           &info_, NULL, &texture_ ) != D3D_OK ) { 
                        throw d3d_exception( 
                                "D3DXCreateTextureFromFile failed" ) ; 
                }
                set_lost_handler( new destroyer( this ) );
                set_reset_handler( new reloader( this ) );
        }
        ~d3d_texture()
        { 
                if( texture_ != NULL ) { texture_->Release() ; }
        }

        int             width() const	{return info_.Width;}
        int		height()const	{return info_.Height;}
        extent_type     extent()const	{return extent_type(width(),height());}
        bounds_type     bounds()const	{return bounds_type(extent());}
    
        LPDIRECT3DTEXTURE9  retrieve() { return texture_ ; }

protected:
        d3d_texture() {}
        d3d_texture( const d3d_texture& ) { }
        d3d_texture& operator = ( const d3d_texture& ) { return *this ; }

private:
        LPDIRECT3DTEXTURE9      texture_;
        std::string             filename_;
        D3DXIMAGE_INFO          info_;

};

/*===========================================================================*
/*
 * class d3d_font
 *
 * 
 */
/*==========================================================================*/

class d3d_font : public d3d_resource {
protected:
        class lost_handler : public handler { 
        public:
                lost_handler( d3d_font* x ) : x_( x ) { }
                void invoke( LPDIRECT3DDEVICE9 d )
                { 
                        x_->font_->OnLostDevice() ; 
                }
        private:
                d3d_font* x_ ; 
        };

        class reset_handler : public handler {
        public:
                reset_handler( d3d_font* x ) : x_( x ) { }
                void invoke( LPDIRECT3DDEVICE9 d )
                {
                        x_->font_->OnResetDevice();
                }
        private:
                d3d_font* x_;
        };


public:
        d3d_font( d3d& x, const wchar_t* fontname, int height )
                : d3d_resource( x )
        {
                // フォント
                logfont_.Height          = height;
                logfont_.Width           = 0;
                logfont_.Weight          = FW_NORMAL;
                logfont_.MipLevels       = 0;
                logfont_.Italic          = FALSE;
                logfont_.CharSet         = 0;
                logfont_.OutputPrecision = 0;
                logfont_.Quality         = DEFAULT_QUALITY;
                logfont_.PitchAndFamily  = DEFAULT_PITCH || FF_DONTCARE;
                lstrcpyW( logfont_.FaceName, fontname );
                D3DXCreateFontIndirectW( x.device(), &logfont_, &font_ );

                set_lost_handler( new lost_handler( this ) );
                set_reset_handler( new reset_handler( this ) );
        }
        ~d3d_font() { font_->Release(); }

        void draw( int x0, int y0, int x1, int y1, const wchar_t* s, DWORD color = D3DCOLOR_XRGB(255,255,255) )
        {
                RECT r;
                r.left          = x0;
                r.top           = y0;
                r.right         = x1;
                r.bottom        = y1;
                font_->DrawTextW( NULL, s, -1, &r, DT_LEFT, color );
        }

        void draw_center( int x0, int y0, int x1, int y1, const wchar_t* s, DWORD color = D3DCOLOR_XRGB(255,255,255) )
        {
                RECT r;
                r.left          = x0;
                r.top           = y0;
                r.right         = x1;
                r.bottom        = y1;
                font_->DrawTextW( NULL, s, -1, &r, DT_CENTER, color );
        }

        void on_lost_device( LPDIRECT3DDEVICE9 ) { font_->OnLostDevice(); }
        void on_reset_device( LPDIRECT3DDEVICE9 device ) { font_->OnResetDevice(); }

        ID3DXFont* font() { return font_; }

private:
        D3DXFONT_DESCW logfont_;
        ID3DXFont* font_;

};

} // namespace zw

#endif
