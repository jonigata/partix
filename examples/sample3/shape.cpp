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
        enum {
            format = (D3DFVF_XYZ|D3DFVF_NORMAL|D3DFVF_DIFFUSE|D3DFVF_TEX1 )
        };
        D3DXVECTOR3     position;
        D3DXVECTOR3     normal;
        DWORD           diffuse;
        D3DXVECTOR2     uv;
    };
#pragma pack(pop)  

    struct SubModel {
        std::vector< model_vertex_type >    vertex_source;
        std::vector< WORD >                 index_source;
        IDirect3DVertexBuffer9*             vb;
        IDirect3DIndexBuffer9*              ib;
        model_vertex_type*                  locked_vb;
        WORD*                               locked_ib;
        TextureHolder*                      texture;
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
        device->SetRenderState( D3DRS_LIGHTING, TRUE );
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
        if( submodel.texture ) {
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
            device->SetTexture( 0, submodel.texture->texture );
        } else {
            device->SetTextureStageState(
                0, D3DTSS_COLOROP, D3DTOP_DISABLE );
            device->SetTextureStageState(
                0, D3DTSS_ALPHAOP, D3DTOP_DISABLE );
            device->SetTexture( 0, NULL );
        }

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
    //  面倒なので四角ポリゴンはないものと仮定
    //
    void build_from_mqo(
        mqo_reader::document_type& doc, float scale, DWORD color,
        TextureCache& tc )
    {
        clear();

        // マテリアル
        for( mqo_reader::materials_type::const_iterator i =
                 doc.materials.begin() ;
             i != doc.materials.end() ;
             ++i ) {
            SubModel m;
            m.vb            = NULL;
            m.ib            = NULL;
            m.texture       = NULL;
            if( (*i).texture != "" ) {
                m.texture = tc.get_texture( (*i).texture );
            }

            submodels_.push_back( m );
        }
        {
            // default material
            SubModel m;
            m.vb                            = NULL;
            m.ib                            = NULL;
            submodels_.push_back( m );
        }

        // 頂点, 面
        for( mqo_reader::objdic_type::const_iterator i =
                 doc.objects.begin() ;
             i != doc.objects.end() ;
             ++i ) {
            const mqo_reader::object_type& obj = (*i).second;

            // dictionary:
            //  ( source vertex index, uv ) => destination vertex index
            struct VertexKey {
                int             index;
                D3DXVECTOR2     uv;

                VertexKey(){}
                VertexKey( int aindex, const D3DXVECTOR2& auv )
                    : index( aindex ), uv( auv ) { }

                bool operator<( const VertexKey& a ) const
                {
                    if( index < a.index ) { return true; }
                    if( a.index < index ) { return false; }
                    if( uv.x < a.uv.x ) { return true; }
                    if( a.uv.x < uv.x ) { return false; }
                    return uv.y < a.uv.y;
                }
            };

            std::vector< std::map< VertexKey, int > > used_vertices;
            used_vertices.resize( submodels_.size() );

            // マテリアルごとに使用頂点を分類
            for( mqo_reader::faces_type::const_iterator j =
                     obj.faces.begin() ;
                 j != obj.faces.end() ;
                 ++j ) {
                const mqo_reader::face_type& face = *j;
                int material_index = face.material_index;
                if( material_index == -1 ) {
                    material_index =
                        int( submodels_.size() - 1 );
                }
                int i0 = face.vertex_indices[0];
                int i1 = face.vertex_indices[1];
                int i2 = face.vertex_indices[2];
                D3DXVECTOR2 uv0( face.uv[0].u, face.uv[0].v );
                D3DXVECTOR2 uv1( face.uv[1].u, face.uv[1].v );
                D3DXVECTOR2 uv2( face.uv[2].u, face.uv[2].v );
                                
                std::map< VertexKey, int >& c =
                    used_vertices[material_index];
                c[ VertexKey( i0, uv0 ) ] = -1 ; 
                c[ VertexKey( i1, uv1 ) ] = -1 ; 
                c[ VertexKey( i2, uv2 ) ] = -1 ; 
            }
                        
            // マテリアルごとに使われている頂点を追加
            size_t n = submodels_.size();
            for( size_t i = 0 ; i < n ; i++ ) {
                SubModel& m = submodels_[i];
                std::map< VertexKey, int >& c = used_vertices[i];

                int no = int( m.vertex_source.size() );
                for( std::map< VertexKey, int >::iterator j = c.begin();
                     j != c.end() ;
                     ++j ) {
                    const mqo_reader::vertex_type& src =
                        obj.vertices[(*j).first.index];

                    model_vertex_type dst;
                    dst.position = D3DXVECTOR3(
                        src.x * scale,
                        src.y * scale,
                        src.z * -scale );
                    dst.normal = D3DXVECTOR3( 0, 0, 0 );
                    dst.diffuse = color;
                    dst.uv = (*j).first.uv;
                    m.vertex_source.push_back( dst );

                    (*j).second = no++;
                }
            }

            // マテリアルごとに面を追加
            for( mqo_reader::faces_type::const_iterator j =
                     obj.faces.begin() ;
                 j != obj.faces.end() ;
                 ++j ) {
                const mqo_reader::face_type& face = *j;
                int material_index = face.material_index;
                if( material_index == -1 ) {
                    material_index =
                        int( submodels_.size() - 1 );
                }
                                
                int i0 = face.vertex_indices[0];
                int i1 = face.vertex_indices[1];
                int i2 = face.vertex_indices[2];
                D3DXVECTOR2 uv0( face.uv[0].u, face.uv[0].v );
                D3DXVECTOR2 uv1( face.uv[1].u, face.uv[1].v );
                D3DXVECTOR2 uv2( face.uv[2].u, face.uv[2].v );

                std::map< VertexKey, int >& c =
                    used_vertices[material_index];
                int k0 = c[VertexKey( i0, uv0 )];
                int k1 = c[VertexKey( i1, uv1 )];
                int k2 = c[VertexKey( i2, uv2 )];

                SubModel& m =
                    submodels_[material_index];
                m.index_source.push_back( k0 );
                m.index_source.push_back( k1 );
                m.index_source.push_back( k2 );

                model_vertex_type& v0 = m.vertex_source[ k0 ];
                model_vertex_type& v1 = m.vertex_source[ k1 ];
                model_vertex_type& v2 = m.vertex_source[ k2 ];
                D3DXVECTOR3 normal = cross(
                    v1.position - v0.position,
                    v2.position - v0.position );
                v0.normal += normal;
                v1.normal += normal;
                v2.normal += normal;
            }
        }

        // 法線後処理
        size_t n = submodels_.size();
        for( size_t j = 0 ; j < n ; j++ ) {
            SubModel& m = submodels_[j];
            for( std::vector< model_vertex_type >::iterator i =
                     m.vertex_source.begin() ;
                 i != m.vertex_source.end() ;
                 ++i ) {
                D3DXVec3Normalize( &(*i).normal, &(*i).normal );
            }
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
    mqo_reader::document_type& doc, float scale, DWORD color,
    TextureCache& tc )
{
    pimpl->build_from_mqo( doc, scale, color, tc );
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

