// $Id: shape.cpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $

#include "shape.hpp"
#include "zw/dprintf.hpp"
#include "zw/d3dmathutils.hpp"

/*===========================================================================*/
/*!
 * @class ShapeImp
 * @brief 
 *
 * 
 */
/*==========================================================================*/

class ShapeImp  {
private:
#pragma pack(push,1)
    struct model_vertex_type {
        enum { format = (D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_DIFFUSE ) };
        D3DXVECTOR3     position;
        D3DXVECTOR3     normal;
        DWORD           diffuse;
    };
#pragma pack(pop)  

    struct SubModel {
        std::vector< model_vertex_type >    vertex_source;
        std::vector< WORD >                 index_source;
        IDirect3DVertexBuffer9*             vb;
        IDirect3DIndexBuffer9*              ib;
        model_vertex_type*                  locked_vb;
        WORD*                               locked_ib;
    };                
    typedef std::vector< SubModel > submodels_type;

public:
    ShapeImp( LPDIRECT3DDEVICE9 device ) : device_(device) { }
    ~ShapeImp() { clear(); }

    void clear()
    {
        clear_submodels();
    }

    bool empty()
    {
        return submodels_.empty();
    }

    void update( float elapsed )
    {
    }

    void render( LPDIRECT3DDEVICE9 device )
    {
        device->SetRenderState( D3DRS_ZENABLE, D3DZB_TRUE );
        device->SetRenderState( D3DRS_ZWRITEENABLE, TRUE );
        device->SetRenderState( D3DRS_ALPHABLENDENABLE, FALSE );

        for( submodels_type::iterator i = submodels_.begin() ;
             i != submodels_.end() ;
             ++i ) {
            if( !(*i).vb ) { continue; }
            render_submodel( device, *i );
        }
    }

    void render_submodel( LPDIRECT3DDEVICE9 device, SubModel& submodel )
    {
        device->SetTextureStageState(
            0, D3DTSS_COLOROP, D3DTOP_DISABLE );
        device->SetTextureStageState(
            0, D3DTSS_ALPHAOP, D3DTOP_DISABLE );

        device->SetIndices( submodel.ib );
        device->SetStreamSource(
            0, submodel.vb, 0, sizeof(model_vertex_type) );
        device->SetFVF( model_vertex_type::format );
        device->DrawIndexedPrimitive(
            D3DPT_TRIANGLELIST, 0, 0, submodel.index_source.size(), 0,
            submodel.index_source.size() / 3 );
    }


    void on_reset_device( LPDIRECT3DDEVICE9 device )
    {
        build_submodels();
    }

    void on_lost_device( LPDIRECT3DDEVICE9 device )
    {
        clear_vertex_buffers();
    }

    //
    // MQO
    //  このバージョンではテクスチャ、マテリアルなどを扱わない
    //
    void build_from_mqo(
        mqo_reader::document_type& doc, float scale, DWORD color )
    {
        clear();

        // マテリアルはとりあえずひとつ
        {
            SubModel m;
            m.vb                            = NULL;
            m.ib                            = NULL;
            submodels_.push_back( m );
        }
        SubModel& m = submodels_[0];

        for( mqo_reader::objdic_type::const_iterator i =
                 doc.objects.begin() ;
             i != doc.objects.end() ;
             ++i ) {
            const mqo_reader::object_type& obj = (*i).second;

            // とりあえず頂点全部放り込む
            int base_index = int( m.vertex_source.size() );
            for( mqo_reader::vertices_type::const_iterator j =
                     obj.vertices.begin() ;
                 j != obj.vertices.end() ;
                 ++j ) {
                const mqo_reader::vertex_type& src = *j;

                model_vertex_type dst;
                dst.position = D3DXVECTOR3(
                    src.x*scale,
                    src.y*scale,
                    src.z*-scale );
                dst.normal = D3DXVECTOR3( 0, 0, 0 );
                dst.diffuse = color;
                m.vertex_source.push_back( dst );
            }

            // 面も全部放り込む
            // 面倒なので四角ポリゴンはないものと仮定
            for( mqo_reader::faces_type::const_iterator j =
                     obj.faces.begin() ;
                 j != obj.faces.end() ;
                 ++j ) {
                const mqo_reader::face_type& face = *j;
                int i0 = base_index + face.vertex_indices[0];
                int i1 = base_index + face.vertex_indices[1];
                int i2 = base_index + face.vertex_indices[2];
                m.index_source.push_back( i0 );
                m.index_source.push_back( i1 );
                m.index_source.push_back( i2 );

                model_vertex_type& v0 = m.vertex_source[ i0 ];
                model_vertex_type& v1 = m.vertex_source[ i1 ];
                model_vertex_type& v2 = m.vertex_source[ i2 ];
                D3DXVECTOR3 normal = cross(
                    v1.position - v0.position,
                    v2.position - v0.position );
                v0.normal += normal;
                v1.normal += normal;
                v2.normal += normal;
            }
        }

        for( std::vector< model_vertex_type >::iterator i =
                 m.vertex_source.begin() ;
             i != m.vertex_source.end() ;
             ++i ) {
            D3DXVec3Normalize( &(*i).normal, &(*i).normal );
        }

        build_submodels();
    }        

private:
    void clear_submodels()
    {
        clear_vertex_buffers();
        submodels_.clear();
    }

    void clear_vertex_buffers()
    {
        for( int i = 0 ; i < int( submodels_.size() ) ; i++ ) {
            SubModel& m = submodels_[i];
            if( m.vb ) { m.vb->Release(); m.vb = NULL; }
            if( m.ib ) { m.ib->Release(); m.ib = NULL; }
        }
    }

    void build_submodels()
    {
        clear_vertex_buffers();

        int n = int( submodels_.size() );

        // vertex buffer 作成
        for( int i = 0 ; i < n ; i++ ) {
            build_submodel( submodels_[i] );
        }
    }

    void build_submodel( SubModel& m )
    {
        const std::vector< model_vertex_type >& vsrc = m.vertex_source;
        const std::vector< WORD >&              isrc = m.index_source;

        if( vsrc.empty() || isrc.empty() ) {
            return;
        }

        // triangle vb
        {
            IDirect3DVertexBuffer9* vb = NULL;
            if( FAILED( device_->CreateVertexBuffer(
                            UINT( vsrc.size() ) *
                            sizeof( model_vertex_type ),
                            0,
                            0,
                            D3DPOOL_MANAGED,
                            &vb,
                            NULL ) ) ) {
                onError( "create vertex buffer failed\n" );
                return;
            }
                        
            model_vertex_type* p;
            if( FAILED( vb->Lock( 0, 0, (void**)&p, 0 ) ) ) {
                onError( "Lock vertexbuffer failed\n" );
                vb->Release();
                return;
            }

            int mm = int(vsrc.size());
            for( int j = 0 ; j < mm ; j++ ){
                *p++ = vsrc[j];
            }

            vb->Unlock();
            m.vb = vb;
        }

        // triangle ib
        {
            IDirect3DIndexBuffer9* ib = NULL;
            if( FAILED( device_->CreateIndexBuffer(
                            UINT( isrc.size() ) *
                            sizeof( WORD ),
                            0,
                            D3DFMT_INDEX16,
                            D3DPOOL_MANAGED,
                            &ib,
                            NULL ) ) ) {
                onError( "create index buffer failed\n" );
                m.vb->Release();
                return;
            }
                                
            WORD* p;
            if( FAILED( ib->Lock( 0, 0, (void**)&p, 0 ) ) ) {
                onError( "Lock vertexbuffer failed\n" );
                m.vb->Release();
                ib->Release();
                return;
            }

            int mm = int(isrc.size());
            for( int j = 0 ; j < mm ; j++ ){
                *p++ = isrc[j];
            }

            ib->Unlock();
            m.ib = ib;
        }                        
    }

    void onError( const char* s )
    {
        throw std::runtime_error( s );
    }

private:
    LPDIRECT3DDEVICE9                       device_;
    std::vector<SubModel>                   submodels_;

};


/*============================================================================
 *
 * class Shape 
 *
 * 
 *
 *==========================================================================*/
//<<<<<<<<<< Shape

//****************************************************************
// constructor
Shape::Shape( LPDIRECT3DDEVICE9 device ) : pimpl( new ShapeImp( device ) )
{
}

//****************************************************************
// destructor
Shape::~Shape()
{
}

//****************************************************************
// clear
void Shape::clear()
{
    pimpl->clear();
}

//****************************************************************
// empty
bool Shape::empty()
{
    return pimpl->empty();
}

//****************************************************************
// update
void Shape::update( float elapsed )
{
    pimpl->update( elapsed );
}

//****************************************************************
// render
void Shape::render( LPDIRECT3DDEVICE9 device )
{
    pimpl->render( device );
}

//****************************************************************
// build_from_mqo
void Shape::build_from_mqo(
    mqo_reader::document_type& doc, float scale, DWORD color )
{
    pimpl->build_from_mqo( doc, scale, color );
}

//****************************************************************
// on_lost_device
void Shape::on_lost_device( LPDIRECT3DDEVICE9 device )
{
    pimpl->on_lost_device( device );
}

//****************************************************************
// on_reset_device
void Shape::on_reset_device( LPDIRECT3DDEVICE9 device )
{
    pimpl->on_reset_device( device );
}

//>>>>>>>>>> Shape

