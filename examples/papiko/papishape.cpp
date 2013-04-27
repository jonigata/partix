// Copyright (C) 2006 Naoyuki Hirayama.
// All Rights Reserved.

// $Id$

#include "papishape.hpp"
#include "zw/d3d.hpp"
#include "zw/d3dmathutils.hpp"
#include "wavefront_obj.hpp"
#include "partix_common.hpp"
#include <vector>
#include <set>
#include <algorithm>

const int TABLE_WIDTH = 512;
const int TABLE_HEIGHT = 16;
const int DIFFUSE_STAGE = 0;
const int NORMALMAP_STAGE = 3;
const int BRDF_STAGE = 1;
const int SMAP_STAGE = 2;
const int LAST_STAGE = 4;

const float UV_EPSILON = 0.0001f; // テクスチャが2048x2048なので

D3DVERTEXELEMENT9 sample_declaration[] = {
    {0,0,D3DDECLTYPE_FLOAT3,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_POSITION,0},
    {0,12,D3DDECLTYPE_FLOAT3,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_NORMAL,0},
    {0,24,D3DDECLTYPE_FLOAT4,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_TANGENT,0},
    {0,40,D3DDECLTYPE_FLOAT2,D3DDECLMETHOD_DEFAULT,D3DDECLUSAGE_TEXCOORD,0},
    D3DDECL_END()
};

#pragma pack(push,1)
struct model_vertex_type {
    D3DXVECTOR3 position;
    D3DXVECTOR3 normal;
    D3DXVECTOR4 tangent;
    D3DXVECTOR2 tex0;
};

struct line_vertex_type {
    enum { format = (D3DFVF_XYZ | D3DFVF_DIFFUSE) };

    D3DXVECTOR3     position;
    DWORD           color;
};
#pragma pack(pop)

struct VertexProxy {
    int     vertex_index;
    float   u;
    float   v;
    int     actual_index;

    bool operator<( const VertexProxy& x ) const
    {
        if( vertex_index < x.vertex_index ) { return true; }
        if( x.vertex_index < vertex_index ) { return false; }
        if( u < x.u - UV_EPSILON ) { return true; }
        if( x.u < u - UV_EPSILON ) { return false; }
        if( v < x.v - UV_EPSILON ) { return true; }
        if( x.v < v - UV_EPSILON ) { return false; }
        return false;
    }
};

struct PlainVertex {
    D3DXVECTOR3 position;
    D3DXVECTOR3 normal;
    D3DXVECTOR4 tangent;
};

struct TexturedVertex {
    int         plain_index; // 分割前頂点index
    D3DXVECTOR2 uv;
};

struct Face {
    int indices[3];
};

struct TmpFace {
    int indices[3];
    size_t source_index;

    bool operator<( const TmpFace& x ) const
    {
        return std::lexicographical_compare(
            indices, indices+3, x.indices, x.indices+3 );
    }
};

struct Edge {
    int indices[2];
    bool operator<( const Edge& x ) const
    {
        return std::lexicographical_compare(
            indices, indices+2, x.indices, x.indices+2 );
    }
};

struct TmpEdge {
    int indices[2];

    struct AdjacentFace {
        int face_point;
        bool clockwise; // face_pointを3つ目の点と考えたときに時計回りかどうか
    };
    AdjacentFace    adjacent_faces[2];
    int             adjacent_face_count;

    bool operator<( const TmpEdge& x ) const
    {
        return std::lexicographical_compare(
            indices, indices+2, x.indices, x.indices+2 );
    }
};

struct Tying {
    int triangle_index;
    float uq;
    float vq;
    float wq;
    float u;
    float v;
    float w;
};

struct Material {
    std::string     decal_name;
    std::string     brdf_name;
    std::string     smap_name;
};

struct Group {
    std::vector< Face >     faces;
    std::string             matname;
    IDirect3DTexture9*      decal;
    IDirect3DTexture9*      brdf;
    IDirect3DTexture9*      smap;
    IDirect3DIndexBuffer9*  ib;
    DWORD                   ib_size;
    bool                    transparent;
    bool                    visible;
};                

struct Part {
    IDirect3DVertexBuffer9*             vb;
    IDirect3DVertexBuffer9*             vb_normal;
    IDirect3DVertexBuffer9*             vb_tangent;
    int                                 source_vertex_count;
    std::vector< PlainVertex >          plain_vertices;     // UV分割前頂点
    std::vector< TexturedVertex >       textured_vertices;  // UV分割済み頂点
    std::vector< Group >                groups;
    std::map< std::string, Material >   matdic;
    bool                                tied;
    std::vector< Tying >                tyings;

};

struct SubdivVertex {
    px::tetrahedralmesh_type*   mesh;           // 非NULLの場合は直接頂点、
                                                // NULLの場合は参照頂点
    int                         indices[3];     // 直接頂点の場合は[0]のみ有効
    D3DXVECTOR3                 position;
    D3DXVECTOR3                 normal;
    D3DXVECTOR3                 neighbor_sum;
    int                         neighbor_count;
};

class PapiShapeImp {
public:
    PapiShapeImp()
    {
        toggle_display_normal_ = false;
        vertex_declaration_ = NULL;
        effect_ = NULL;
    }
    ~PapiShapeImp () {}

    void set_data_directory( const std::string& data_directory )
    {
        data_directory_ = data_directory;
    }

    dense_handle load_dense(
        LPDIRECT3DDEVICE9 device, const std::string& objfilename )
    {
        D3DXVECTOR3 zero( 0, 0, 0 );

        Part* p = new_part();

        std::ifstream ifs( ( data_directory_ + "\\" + objfilename ).c_str() );
        wavefront_obj_reader wobj( ifs, data_directory_ );

        // マテリアル
        for( std::map< std::string, std::string>::const_iterator i =
                 wobj.mat2tex.begin() ;
             i != wobj.mat2tex.end() ;
             ++i ) {
            Material m;
            m.decal_name = (*i).second;
            p->matdic[(*i).first] = m;
        }

        // 前処理

        // 元頂点をコピー
        size_t n = wobj.v.size();
        for( size_t i = 0; i < n ; i++ ) {
            PlainVertex v;
            v.position = wobj.v[i];
            v.normal = zero;
            p->plain_vertices.push_back( v );
        }

        // 法線の計算
        n = wobj.groups.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            wavefront_obj_reader::group& g = wobj.groups[i];

            size_t m = g.faces.size();
            for( size_t j = 0 ; j < m ; j++ ) {
                wavefront_obj_reader::face& f = g.faces[j];
                                
                int i0 = f.corners[0].vertex_index;
                int i1 = f.corners[1].vertex_index;
                int i2 = f.corners[2].vertex_index;

                const D3DXVECTOR3& v0 = wobj.v[i0];
                const D3DXVECTOR3& v1 = wobj.v[i1];
                const D3DXVECTOR3& v2 = wobj.v[i2];
                                
                D3DXVECTOR3 normal = cross( v2 - v0, v1 - v0 );
                p->plain_vertices[i0].normal += normal;
                p->plain_vertices[i1].normal += normal;
                p->plain_vertices[i2].normal += normal;

                if( f.corners.size() == 4 ) {
                    int i3 = f.corners[3].vertex_index;
                    const D3DXVECTOR3& v3 = wobj.v[i3];
                    normal = cross( v3 - v0, v2 - v0 );
                    p->plain_vertices[i0].normal += normal;
                    p->plain_vertices[i2].normal += normal;
                    p->plain_vertices[i3].normal += normal;
                }
            }
        }
        n = p->plain_vertices.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            normalize_f( p->plain_vertices[i].normal );
        }
                
        // タンジェントの計算
        D3DXVECTOR3* tan1 = new D3DXVECTOR3[wobj.v.size()];
        D3DXVECTOR3* tan2 = new D3DXVECTOR3[wobj.v.size()];
        memset( tan1, 0, sizeof( D3DXVECTOR3 ) * wobj.v.size() );
        memset( tan2, 0, sizeof( D3DXVECTOR3 ) * wobj.v.size() );
                
        n = wobj.groups.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            wavefront_obj_reader::group& g = wobj.groups[i];

            size_t m = g.faces.size();
            for( size_t j = 0 ; j < m ; j++ ) {
                wavefront_obj_reader::face& f = g.faces[j];
                                
                int i0 = f.corners[0].vertex_index;
                int i1 = f.corners[1].vertex_index;
                int i2 = f.corners[2].vertex_index;
                int q0 = f.corners[0].uv_index;
                int q1 = f.corners[1].uv_index;
                int q2 = f.corners[2].uv_index;
                make_tangent( wobj, i0, i2, i1, q0, q2, q1, tan1, tan2 );
                if( f.corners.size() == 4 ) {
                    int i3 = f.corners[3].vertex_index;
                    int q3 = f.corners[3].uv_index;
                    make_tangent( wobj, i0, i3, i2, q0, q3, q2, tan1, tan2 );
                }
            }
        }

        n = p->plain_vertices.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            const D3DXVECTOR3& n = p->plain_vertices[i].normal;
            const D3DXVECTOR3& t = tan1[i];

            // Gram-Schmidt orthogonalize
            (D3DXVECTOR3&)p->plain_vertices[i].tangent = t - n * dot( n, t );
        
            // Calculate handedness
            D3DXVECTOR3 b = cross( n, t );
            p->plain_vertices[i].tangent.w =
                dot( b, tan2[i] ) < 0 ? -1.0f : 1.0f;
        }
                
        delete [] tan1;
        delete [] tan2;
                
        std::set< VertexProxy > proxies;

        // UVがfaceごとなのでvertexごとに変換
        n = wobj.groups.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            wavefront_obj_reader::group& g = wobj.groups[i];

            Group group;
            group.matname = g.material;
            group.decal = NULL;
            group.brdf = NULL;
            group.smap = NULL;
            group.ib = NULL;
            group.ib_size = 0;
            group.transparent = false;
            group.visible = true;

            size_t m = g.faces.size();
            for( size_t j = 0 ; j < m ; j++ ) {
                wavefront_obj_reader::face& f = g.faces[j];
                assert( f.corners.size() <= 4 ); // 5角形以上はサポートしない
                                
                Face tf;
                tf.indices[0] = make_vertex( wobj, proxies, f.corners[0], p );
                tf.indices[1] = make_vertex( wobj, proxies, f.corners[2], p );
                tf.indices[2] = make_vertex( wobj, proxies, f.corners[1], p );
                group.faces.push_back( tf );

                if( f.corners.size() == 4 ) {
                    Face tf;
                    tf.indices[0] =
                        make_vertex( wobj, proxies, f.corners[0], p );
                    tf.indices[1] =
                        make_vertex( wobj, proxies, f.corners[3], p );
                    tf.indices[2] =
                        make_vertex( wobj, proxies, f.corners[2], p );
                    group.faces.push_back( tf );
                }
            }
                        
            p->groups.push_back( group );
        }

        parts_.push_back( boost::shared_ptr< Part >( p ) );

        return (dense_handle)p;
    }

    void load_coarse(
        px::world_type*     world,
        const std::string&  fn,
        float               mass,
        float               stiffness )
    {
        px::softvolume_type* v;
        v = load_body( ( data_directory_ + "\\" + fn ).c_str(), mass );
        v->set_name( "body" );
        v->set_features( true, false, true );
        v->set_restore_factor( stiffness );
        v->set_stretch_factor( 1.0f - stiffness );

        meshes_.push_back(
            boost::shared_ptr< px::tetrahedralmesh_type >( v->get_mesh() ) );
        coarses_.push_back(
            boost::shared_ptr< px::softvolume_type >( v ) );

        world->add_body( v );
    }

    void subdivide( int level )
    {
        subdivide_aux( level );
        calculate_subdiv_vertices();
    }

    void tie_dense_to_coarse( dense_handle yy )
    {
        Part* y = (Part*)yy; // object mesh
        y->tied = true;

        size_t n = y->plain_vertices.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            // nearest triangle in x
            const D3DXVECTOR3& p = y->plain_vertices[i].position;

            Tying tying;
            float dist = ( std::numeric_limits< float >::max )();
                        
            D3DXVECTOR3 nq;
            D3DXVECTOR3 Ui;
            D3DXVECTOR3 Vi;
            D3DXVECTOR3 Wi;
            D3DXVECTOR3 nearest_q;

            size_t m = subdiv_faces_.size() ;
            for( size_t j = 0 ; j < m ; j++ ) {
                Face& f = subdiv_faces_[j];
                const SubdivVertex& v0 = subdiv_vertices_[f.indices[0]];
                const SubdivVertex& v1 = subdiv_vertices_[f.indices[1]];
                const SubdivVertex& v2 = subdiv_vertices_[f.indices[2]];

                D3DXVECTOR3 q;
                D3DXVECTOR3 uvw;
                point_to_triangle_distance(
                    q, uvw, v0.position, v1.position, v2.position, p );

                float len = D3DXVec3Length( &( q - p ) );
                if( len < dist ) {
                    dist = len;
                    tying.triangle_index = j;
                    tying.uq = uvw.x;
                    tying.vq = uvw.y;
                    tying.wq = uvw.z;
                    nq =
                        tying.uq * v0.normal +
                        tying.vq * v1.normal +
                        tying.wq * v2.normal;
                    Ui = v1.position - v0.position;
                    Vi = v2.position - v0.position;
                    Wi = nq;
                    nearest_q = q;
                }
            }
                        
            D3DXMATRIX mm;
            D3DXMatrixIdentity( &mm );
            mm._11 = Ui.x; mm._12 = Ui.y; mm._13 = Ui.z;
            mm._21 = Vi.x; mm._22 = Vi.y; mm._23 = Vi.z;
            mm._31 = Wi.x; mm._32 = Wi.y; mm._33 = Wi.z;
            D3DXMATRIX imm;
            D3DXMatrixInverse( &imm, NULL, &mm );

            D3DXVECTOR3 uvw;
            D3DXVec3TransformCoord( &uvw, &( p - nearest_q ), &imm );
            tying.u = uvw.x;
            tying.v = uvw.y;
            tying.w = uvw.z;
            y->tyings.push_back( tying );
        }
    }

    void attach_brdf(
        LPDIRECT3DDEVICE9   device,
        dense_handle        d,
        const char*         material,
        const std::string&  brdf_filename )
    {
        Part* p = (Part*)d;
        p->matdic[material].brdf_name = brdf_filename;
    }

    void attach_smap(
        LPDIRECT3DDEVICE9   device,
        dense_handle        d,
        const char*         material,
        const std::string&  smap_filename )
    {
        Part* p = (Part*)d;
        p->matdic[material].smap_name = smap_filename;
    }

    void update( float tick )
    {
    }

    void render( LPDIRECT3DDEVICE9 device, const D3DXVECTOR3& eye )
    {
        update_parts_vertices();

        //device->SetRenderState( D3DRS_LIGHTING, FALSE );
        device->SetRenderState( D3DRS_ZENABLE, D3DZB_TRUE );
        device->SetRenderState( D3DRS_ZWRITEENABLE, TRUE );
        device->SetRenderState( D3DRS_ALPHABLENDENABLE, TRUE );

#if 1
        if( effect_ ) {
            device->SetVertexDeclaration( vertex_declaration_ );

            // matrix
            D3DXMATRIX mat_world, mat_view, mat_proj;
            device->GetTransform( D3DTS_WORLD        , &mat_world );
            device->GetTransform( D3DTS_VIEW         , &mat_view );
            device->GetTransform( D3DTS_PROJECTION   , &mat_proj );

            D3DXMATRIX m = mat_world * mat_view * mat_proj;
            effect_->SetMatrix( par_local_to_projection_, &m );
            effect_->SetMatrix( par_local_to_world_, &mat_world );

            // light
            (D3DXVECTOR3&)light_position_ = eye; // +D3DXVECTOR4( 0, 5, 0, 0 );
            effect_->SetVector( par_light_position_, &light_position_ );

            // eye
            D3DXVECTOR4 eye2; (D3DXVECTOR3&)eye2 =  eye; eye2.w = 1;
            effect_->SetVector( par_eye_, &eye2 );

            // shader
            effect_->SetTechnique( technique_ );
        }
#endif

        // alphaのないもの
        device->SetRenderState( D3DRS_ALPHABLENDENABLE, FALSE );

        size_t m = parts_.size();
        for( size_t j = 0 ; j < m ; j++ ) {
            Part* p = parts_[j].get();

            size_t n = p->groups.size();
            for( size_t i = 0 ; i < n ; i++ ) {
                Group& group = p->groups[i];
                if( !group.visible ) { continue; }
                if( group.transparent ) { continue; }
                                
                render_group( device, p, group );
            }
        }

        // alphaのあるもの
        device->SetRenderState( D3DRS_ALPHABLENDENABLE, TRUE );

        m = parts_.size();
        for( size_t j = 0 ; j < m ; j++ ) {
            Part* p = parts_[j].get();

            size_t n = p->groups.size();
            for( size_t i = 0 ; i < n ; i++ ) {
                Group& group = p->groups[i];
                if( !group.visible ) { continue; }
                if( !group.transparent ) { continue; }
                                
                render_group( device, p, group );
            }
        }

        if( toggle_display_normal_ ) {
            device->SetRenderState(D3DRS_LIGHTING,FALSE);
            device->SetRenderState(D3DRS_ZENABLE,D3DZB_TRUE);
            device->SetRenderState(D3DRS_ZWRITEENABLE,FALSE);
            device->SetRenderState(D3DRS_ALPHABLENDENABLE,TRUE);
            device->SetRenderState(D3DRS_SRCBLEND,D3DBLEND_SRCALPHA);
            device->SetRenderState(D3DRS_DESTBLEND,D3DBLEND_INVSRCALPHA);
            device->SetTextureStageState(0,D3DTSS_COLOROP,D3DTOP_SELECTARG1);
            device->SetTextureStageState(0,D3DTSS_COLORARG1,D3DTA_DIFFUSE);
            device->SetTextureStageState(0,D3DTSS_ALPHAOP,D3DTOP_SELECTARG1);
            device->SetTextureStageState(0,D3DTSS_ALPHAARG1,D3DTA_DIFFUSE);
            device->SetTextureStageState(0,D3DTSS_COLOROP,D3DTOP_DISABLE);
            device->SetTextureStageState(0,D3DTSS_ALPHAOP,D3DTOP_DISABLE);
            device->SetFVF( line_vertex_type::format );

            m = parts_.size();
            for( size_t j = 0 ; j < m ; j++ ) {
                Part* p = parts_[j].get();
                device->SetStreamSource(
                    0, p->vb_tangent, 0, sizeof(line_vertex_type) );
                device->DrawPrimitive(
                    D3DPT_LINELIST, 0, UINT( p->plain_vertices.size() ) );
                device->SetStreamSource(
                    0, p->vb_normal, 0, sizeof(line_vertex_type) );
                device->DrawPrimitive(
                    D3DPT_LINELIST, 0, UINT( p->plain_vertices.size() ) );
            }
        }
    }
        
    void render_group( LPDIRECT3DDEVICE9 device, Part* p, Group& group )
    {
        //device->SetRenderState( D3DRS_FILLMODE , D3DFILL_WIREFRAME );

#if 1
        if( effect_ ) {
            effect_->Begin( NULL, 0 );
            effect_->BeginPass( 0 );
        }
#endif

        if( group.decal ) {
            device->SetTexture( DIFFUSE_STAGE, group.decal );
            device->SetTextureStageState(
                DIFFUSE_STAGE, D3DTSS_COLOROP,    D3DTOP_SELECTARG2 );
            device->SetTextureStageState(
                DIFFUSE_STAGE, D3DTSS_COLORARG1,  D3DTA_CURRENT );
            device->SetTextureStageState(
                DIFFUSE_STAGE, D3DTSS_COLORARG2,  D3DTA_TEXTURE );
            device->SetSamplerState(
                DIFFUSE_STAGE, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );
            device->SetSamplerState(
                DIFFUSE_STAGE, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
            device->SetSamplerState(
                DIFFUSE_STAGE, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP);
            device->SetSamplerState(
                DIFFUSE_STAGE, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP);
        } else {
            device->SetTextureStageState(
                DIFFUSE_STAGE, D3DTSS_COLOROP, D3DTOP_DISABLE );
            device->SetTextureStageState(
                DIFFUSE_STAGE, D3DTSS_ALPHAOP, D3DTOP_DISABLE );
        }

        if( group.brdf ) {
            device->SetTexture( BRDF_STAGE, group.brdf );
            device->SetTextureStageState(
                BRDF_STAGE, D3DTSS_COLOROP,    D3DTOP_SELECTARG1 );
            device->SetTextureStageState(
                BRDF_STAGE, D3DTSS_COLORARG1,  D3DTA_TEXTURE );
            device->SetTextureStageState(
                BRDF_STAGE, D3DTSS_ALPHAOP,    D3DTOP_SELECTARG1 );
            device->SetTextureStageState(
                BRDF_STAGE, D3DTSS_ALPHAARG1,  D3DTA_TEXTURE );
            device->SetSamplerState(
                BRDF_STAGE, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );
            device->SetSamplerState(
                BRDF_STAGE, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
            device->SetSamplerState(
                BRDF_STAGE, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
            device->SetSamplerState(
                BRDF_STAGE, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
        }
        if( group.smap ) {
            device->SetTexture(
                SMAP_STAGE, group.smap );
            device->SetTextureStageState(
                SMAP_STAGE, D3DTSS_COLOROP,    D3DTOP_SELECTARG1 );
            device->SetTextureStageState(
                SMAP_STAGE, D3DTSS_COLORARG1,  D3DTA_TEXTURE );
            device->SetTextureStageState(
                SMAP_STAGE, D3DTSS_ALPHAOP,    D3DTOP_SELECTARG1 );
            device->SetTextureStageState(
                SMAP_STAGE, D3DTSS_ALPHAARG1,  D3DTA_TEXTURE );
            device->SetSamplerState(
                SMAP_STAGE, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR );
            device->SetSamplerState(
                SMAP_STAGE, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );
            device->SetSamplerState(
                SMAP_STAGE, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
            device->SetSamplerState(
                SMAP_STAGE, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
        }
        device->SetTextureStageState(
            LAST_STAGE, D3DTSS_COLOROP, D3DTOP_DISABLE );
        device->SetTextureStageState(
            LAST_STAGE, D3DTSS_ALPHAOP, D3DTOP_DISABLE );

        device->SetIndices( group.ib );
        device->SetStreamSource( 0, p->vb, 0, sizeof(model_vertex_type) );
        device->DrawIndexedPrimitive(
            D3DPT_TRIANGLELIST, 0, 0,
            p->textured_vertices.size(), 0, group.ib_size / 3 );

#if 1
        if( effect_ ) {
            effect_->EndPass();
            effect_->End();
        }
#endif
    }

    void fix()
    {
        struct body_fix_filter {
            bool operator()( const D3DXVECTOR3& v ) const { return true; }
        };
        struct bust_fix_filter {
            bool operator()( const D3DXVECTOR3& v ) const
            {
                return -0.5f <= v.z || abs( v.x ) <= 0.1f;
            }
        };
        struct hip_fix_filter {
            bool operator()( const D3DXVECTOR3& v ) const
            {
                return v.z <= 0.25f;
            }
        };


        // body
        fix_aux( meshes_[0].get(), 0.9f, body_fix_filter() );

        // bust
        fix_aux( meshes_[1].get(), 0.2f, bust_fix_filter() );
        fix_aux( meshes_[2].get(), 0.2f, bust_fix_filter() );

        // hip
        fix_aux( meshes_[3].get(), 0.2f, hip_fix_filter() );
        fix_aux( meshes_[4].get(), 0.2f, hip_fix_filter() );
    }

    template < class T >
    void fix_aux( px::tetrahedralmesh_type* mesh, float restore, T filter )
    {
        px::tetrahedralmesh_type::points_type& points = mesh->get_points();

        size_t n = points.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            if( filter( points[i].source_position ) ) {
                D3DXVECTOR3 d =
                    points[i].source_position -
                    points[i].new_position;
                points[i].new_position += d * restore;
            }
        }
    }

    std::vector< boost::shared_ptr< px::softvolume_type > >& get_coarses()
    {
        return coarses_;
    }

    void on_reset_device( LPDIRECT3DDEVICE9 device )
    {
        build_textures( device );
        build_parts( device );
        if( !setup_shader( device ) ) { exit( 0 ); }
    }

    void on_lost_device( LPDIRECT3DDEVICE9 device )
    {
        destroy_shader();
        clear_textures();
        clear_parts();
    }

    void reload_textures( LPDIRECT3DDEVICE9 device )
    {
        clear_textures();
        build_textures( device );
    }
        
private:
    ////////////////////////////////////////////////////////////////
    // subdivision
    void subdivide_aux( int level )
    {
        // level 0
        subdiv_vertices_.clear();
        subdiv_faces_.clear();
        subdiv_edges_.clear();
        subdiv_vertices_upper_.clear();
        subdiv_edges_upper_.clear();

        std::vector< std::vector< int > > vertex_mapping;

        // ... vertices
        size_t n = meshes_.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            px::tetrahedralmesh_type* mesh = meshes_[i].get();
            vertex_mapping.push_back( std::vector< int >() );
            std::vector< int >& v = vertex_mapping.back();
                        
            const px::tetrahedralmesh_type::points_type& points =
                mesh->get_points();
            size_t m = points.size();
            for( size_t j = 0 ; j < m ; j++ ) {
                SubdivVertex sv;
                sv.mesh = mesh;
                sv.indices[0] = j;
                subdiv_vertices_.push_back( sv );
                v.push_back( subdiv_vertices_.size() - 1 );
            }
        }
        subdiv_vertices_upper_.push_back( subdiv_vertices_.size() );

        // ... faces
        n = meshes_.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            px::tetrahedralmesh_type* mesh = meshes_[i].get();

            const px::tetrahedralmesh_type::faces_type& faces =
                mesh->get_faces();
            size_t m = faces.size();
            for( size_t j = 0 ; j < m ; j++ ) {
                const px::face_type& sf = faces[j];

                Face df;
                df.indices[0] = vertex_mapping[i][sf.i0];
                df.indices[1] = vertex_mapping[i][sf.i1];
                df.indices[2] = vertex_mapping[i][sf.i2];

                subdiv_faces_.push_back( df );
            }
        }


        // edges
        // level 0では影響力のあるエッジはない
        subdiv_edges_upper_.push_back( 0 ); 

        // level 1～
        for( int i = 0 ; i < level ; i++ ) {
            subdivide_step( i );
        }

        subdivision_ = true;
    }

    void subdivide_step( int level )
    {
        // sqrt(3) subdivision

        const std::vector< SubdivVertex >& src_vertices = subdiv_vertices_;
        const std::vector< Face >&         src_faces = subdiv_faces_;

        std::vector< SubdivVertex >     subdiv_vertices;
        std::vector< Face >             subdiv_faces;

        // 元の頂点をすべて入れる
        subdiv_vertices.assign( src_vertices.begin(), src_vertices.end() );
        size_t original_vertex_count = subdiv_vertices.size();

        // 面の重心を入れる
        size_t n = src_faces.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            const Face& f = src_faces[i];

            SubdivVertex v;
            v.mesh = NULL;
            for( int j = 0 ; j < 3 ; j++ ) {
                v.indices[j] = f.indices[j];
            }
            subdiv_vertices.push_back( v );
        }

        // エッジを抽出
        std::set< TmpEdge > tmpedges;

        n = src_faces.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            const Face& f = src_faces[i];

            make_tmpedge(
                tmpedges, original_vertex_count + i,
                f.indices[0], f.indices[1] );
            make_tmpedge(
                tmpedges, original_vertex_count + i,
                f.indices[1], f.indices[2] );
            make_tmpedge(
                tmpedges, original_vertex_count + i,
                f.indices[2], f.indices[0] );
        }

        // 新しい辺/面を生成

        std::set< Edge > edges;
        for( std::set< TmpEdge >::iterator i = tmpedges.begin() ;
             i != tmpedges.end() ;
             ++i ) {
            TmpEdge& te = (*i);

            if( te.adjacent_face_count == 1 ) {
                Face f;
                f.indices[0] = te.indices[0];
                f.indices[1] = te.indices[1];
                f.indices[2] = te.adjacent_faces[0].face_point;
                if( !te.adjacent_faces[0].clockwise ) {
                    std::swap( f.indices[0], f.indices[1] );
                }
                subdiv_faces.push_back( f );
                make_edge( edges, f.indices[0], f.indices[1] );
                make_edge( edges, f.indices[1], f.indices[2] );
                make_edge( edges, f.indices[2], f.indices[0] );
            } else {
                assert( te.adjacent_face_count == 2 );
                Face f;
                f.indices[0] =
                    te.adjacent_faces[0].clockwise ?
                    te.indices[1] : te.indices[0];
                f.indices[1] = te.adjacent_faces[0].face_point;
                f.indices[2] = te.adjacent_faces[1].face_point;
                subdiv_faces.push_back( f );
                make_edge( edges, f.indices[0], f.indices[1] );
                make_edge( edges, f.indices[1], f.indices[2] );
                make_edge( edges, f.indices[2], f.indices[0] );

                f.indices[0] =
                    te.adjacent_faces[1].clockwise ?
                    te.indices[1] : te.indices[0];
                f.indices[1] = te.adjacent_faces[1].face_point;
                f.indices[2] = te.adjacent_faces[0].face_point;
                subdiv_faces.push_back( f );
                make_edge( edges, f.indices[0], f.indices[1] );
                make_edge( edges, f.indices[1], f.indices[2] );
                make_edge( edges, f.indices[2], f.indices[0] );
            }
        }

        subdiv_vertices_.swap( subdiv_vertices );
        subdiv_vertices_upper_.push_back( subdiv_vertices_.size() );
        subdiv_faces_.swap( subdiv_faces );
        subdiv_edges_.insert( subdiv_edges_.end(), edges.begin(),edges.end() );
        subdiv_edges_upper_.push_back( subdiv_edges_.size() );
    }

    void make_tmpedge(
        std::set< TmpEdge >& edges, int face_point, int i0, int i1 )
    {
        bool clockwise = true;
        if( i1 < i0 ) {
            std::swap( i0, i1 );
            clockwise = false;
        }
        TmpEdge e;
        e.indices[0] = i0;
        e.indices[1] = i1;

        std::set< TmpEdge >::iterator i = edges.find( e );
        if( i == edges.end() ) {
            e.adjacent_faces[0].face_point = face_point;
            e.adjacent_faces[0].clockwise = clockwise;
            e.adjacent_face_count = 1;
            edges.insert( e );
        } else {
            TmpEdge& e2 = (*i);
            if( e2.adjacent_face_count == 1 ) {
                e2.adjacent_faces[1].face_point = face_point;
                e2.adjacent_faces[1].clockwise = clockwise;
                e2.adjacent_face_count = 2;
            } else {
                assert( 0 );
            }
        }
    }

    void make_edge(  std::set< Edge >& edges, int i0, int i1 )
    {
        Edge e;
        e.indices[0] = i0;
        e.indices[1] = i1;
        if( e.indices[1] < e.indices[0] ) {
            std::swap( e.indices[0], e.indices[1] );
        }
        edges.insert( e );
    }




private:
    Part* new_part()
    {
        Part* p = new Part;
        p->vb = NULL;
        p->vb_normal = NULL;
        p->vb_tangent = NULL;
        p->source_vertex_count = 0;
        p->tied = false;
        return p;
    }

    void clear_textures()
    {
        for( std::map< std::string, IDirect3DTexture9* >::iterator i =
                 textures_.begin() ;
             i != textures_.end() ;
             ++i ) {
            if( (*i).second ) { (*i).second->Release(); }
        }
        textures_.clear();

        size_t n = parts_.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            Part* p = parts_[i].get();

            size_t m = p->groups.size();
            for( size_t j = 0 ; j < m ; j++ ) {
                Group& g = p->groups[j];
                if( g.decal ) { g.decal = NULL; }
                if( g.brdf ) { g.brdf = NULL; }
                if( g.smap ) { g.smap = NULL; }
            }
        }
    }

    void clear_parts()
    {
        size_t n = parts_.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            clear_part( parts_[i].get() );
        }
    }

    void clear_part( Part* p )
    {
        // clear buffers
        if( p->vb ) { p->vb->Release(); p->vb = NULL; }
        if( p->vb_normal ) { p->vb_normal->Release(); p->vb_normal = NULL; }
        if( p->vb_tangent ) { p->vb_tangent->Release(); p->vb_tangent = NULL; }
        for( int i = 0 ; i < int( p->groups.size() ) ; i++ ) {
            Group& g = p->groups[i];
            if( g.ib ) { g.ib->Release(); g.ib = NULL; g.ib_size = 0; }
        }
    }

    void build_parts( LPDIRECT3DDEVICE9 device )
    {
        clear_parts();

        size_t n = parts_.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            build_part( device, parts_[i].get() );
        }
    }

    void build_part( LPDIRECT3DDEVICE9 device, Part* p )
    {
        if( p->textured_vertices.empty() ) { return; }

        // vertex buffer 作成

        // ...triangle vb
        {
            IDirect3DVertexBuffer9* vb = NULL;
            UINT length =
                UINT( p->textured_vertices.size() ) *
                sizeof( model_vertex_type );
            if( FAILED( device->CreateVertexBuffer( 
                            length, 0, 0, D3DPOOL_MANAGED, &vb, NULL ) ) ) {
                on_error( "create vertex buffer failed\n" );
                return;
            }
                        
            model_vertex_type* q;
            if( FAILED( vb->Lock( 0, 0, (void**)&q, 0 ) ) ) {
                on_error( "Lock vertexbuffer failed\n" );
                vb->Release();
                return;
            }

            write_part_to_vertex_buffer( p, q );

            vb->Unlock();
            p->vb = vb;
        }

        // ...normal vb
        {
            IDirect3DVertexBuffer9* vb = NULL;
            UINT length =
                UINT( p->plain_vertices.size() ) *
                sizeof( line_vertex_type ) * 2;
            if( FAILED( device->CreateVertexBuffer(
                            length, 0, 0, D3DPOOL_MANAGED, &vb, NULL ) ) ) {
                on_error( "create vertex buffer failed\n" );
                p->vb->Release();
                return;
            }
                        
            line_vertex_type* q;
            if( FAILED( vb->Lock( 0, 0, (void**)&q, 0 ) ) ) {
                on_error( "Lock vertexbuffer failed\n" );
                p->vb->Release();
                vb->Release();
                return;
            }

            size_t mm = p->plain_vertices.size();
            for( size_t j = 0 ; j < mm ; j++ ){
                q[0].position = p->plain_vertices[j].position;
                q[0].color = D3DCOLOR_XRGB( 0, 0, 255 );
                q[1].position =
                    p->plain_vertices[j].position +
                    p->plain_vertices[j].normal * 0.1f;
                q[1].color = D3DCOLOR_XRGB( 0, 0, 255 );
                q += 2;
            }

            vb->Unlock();
            p->vb_normal = vb;
        }

        // ...tangent vb
        {
            IDirect3DVertexBuffer9* vb = NULL;
            UINT length =
                UINT( p->plain_vertices.size() ) *
                sizeof( line_vertex_type ) * 2;
            if( FAILED( device->CreateVertexBuffer(
                            length, 0, 0, D3DPOOL_MANAGED, &vb, NULL ) ) ) {
                on_error( "create vertex buffer failed\n" );
                p->vb->Release();
                p->vb_normal->Release();
                return;
            }
                        
            line_vertex_type* q;
            if( FAILED( vb->Lock( 0, 0, (void**)&q, 0 ) ) ) {
                on_error( "Lock vertex buffer failed\n" );
                p->vb->Release();
                p->vb_normal->Release();
                vb->Release();
                return;
            }

            size_t mm = p->plain_vertices.size();
            for( size_t j = 0 ; j < mm ; j++ ){
                q[0].position = p->plain_vertices[j].position;
                q[0].color = D3DCOLOR_XRGB( 0, 255, 0 );
                q[1].position =
                    p->plain_vertices[j].position +
                    (D3DXVECTOR3&)p->plain_vertices[j].tangent * 0.1f;
                q[1].color = D3DCOLOR_XRGB( 0, 255, 0 );
                q += 2;
            }

            vb->Unlock();
            p->vb_tangent = vb;
        }


        size_t n = p->groups.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            Group& g = p->groups[i];

            if( g.faces.empty() ) { continue; }

            // triangle ib
            {
                //assert( g.faces.size() * 3 < 65000 );
                IDirect3DIndexBuffer9* ib = NULL;
                if( FAILED( device->CreateIndexBuffer(
                                UINT( g.faces.size() * 3 ) * sizeof( DWORD ),
                                0,
                                D3DFMT_INDEX32,
                                D3DPOOL_MANAGED,
                                &ib,
                                NULL ) ) ) {
                    on_error( "create index buffer failed\n" );
                    return;
                }
                                
                DWORD* q;
                if( FAILED( ib->Lock( 0, 0, (void**)&q, 0 ) ) ) {
                    on_error( "Lock index buffer failed\n" );
                    ib->Release();
                    return;
                }

                size_t mm = g.faces.size();
                for( size_t j = 0 ; j < mm ; j++ ){
                    *q++ = g.faces[j].indices[0];
                    *q++ = g.faces[j].indices[1];
                    *q++ = g.faces[j].indices[2];
                }

                ib->Unlock();
                g.ib = ib;
                g.ib_size = g.faces.size() * 3;
            }                        
        }
    }

    void build_textures( LPDIRECT3DDEVICE9 device )
    {
        clear_textures();

        size_t n = parts_.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            build_texture( device, parts_[i].get() );
        }
    }

    void build_texture( LPDIRECT3DDEVICE9 device, Part* p )
    {
        size_t n = p->groups.size() ;
        for( size_t i = 0 ; i < n ; i++ ) {
            Group& g = p->groups[i];

            const Material& m = p->matdic[ g.matname ];
            if( m.decal_name != "" ) {
                g.decal = ensure_texture( device, m.decal_name );
            }
            if( m.brdf_name != "" ) {
                g.brdf = ensure_texture( device, m.brdf_name );
            }
            if( m.smap_name != "" ) {
                g.smap = ensure_texture( device, m.smap_name );
            }
        }
    }

    IDirect3DTexture9* ensure_texture( LPDIRECT3DDEVICE9 device, const std::string& name )
    {
        std::map< std::string, IDirect3DTexture9* >::const_iterator i = 
            textures_.find( name );

        if( i == textures_.end() ) {
#if 1
            IDirect3DTexture9* texture;
            if( FAILED( D3DXCreateTextureFromFileA(
                            device, ( data_directory_ + "\\" + name ).c_str(),
                            &texture ) ) ) {
                on_error( "can't load texture: " + name);
                exit(0);
            }
#else
            IDirect3DTexture9* texture = load_image( device, data_directory_ + "/" + name );
#endif
            textures_[name] = texture;
            return texture;
        } else {
            return (*i).second;
        }
    }

    int make_vertex( const wavefront_obj_reader&            wobj,
                     std::set< VertexProxy >&               proxies, 
                     const wavefront_obj_reader::corner&    c,
                     Part*                                  p )
    {
        VertexProxy proxy;
        proxy.vertex_index = c.vertex_index;
        proxy.u = wobj.vt[c.uv_index].x;
        proxy.v = wobj.vt[c.uv_index].y;

        std::set< VertexProxy >::iterator i = proxies.find( proxy );
        if( i == proxies.end() ) {
                        
            TexturedVertex tv;
            tv.plain_index = c.vertex_index;
            if( 0 <= c.uv_index ) { tv.uv = wobj.vt[ c.uv_index ]; }

            p->textured_vertices.push_back( tv );
                        
            proxy.actual_index = int( p->textured_vertices.size() - 1 );
            proxies.insert( proxy );

            return proxy.actual_index;
        } else {
            return (*i).actual_index;
        }
    }

    void make_tangent(
        wavefront_obj_reader&   wobj,
        int                     i0,
        int                     i1,
        int                     i2,
        int                     q0,
        int                     q1,
        int                     q2,
        D3DXVECTOR3*            tan1,
        D3DXVECTOR3*            tan2 )
    {
        const D3DXVECTOR3& v0 = wobj.v[i0];
        const D3DXVECTOR3& v1 = wobj.v[i1];
        const D3DXVECTOR3& v2 = wobj.v[i2];

        const D3DXVECTOR2& w0 = wobj.vt[q0];
        const D3DXVECTOR2& w1 = wobj.vt[q1];
        const D3DXVECTOR2& w2 = wobj.vt[q2];

        float x0 = v1.x - v0.x;
        float x1 = v2.x - v0.x;
        float y0 = v1.y - v0.y;
        float y1 = v2.y - v0.y;
        float z0 = v1.z - v0.z;
        float z1 = v2.z - v0.z;

        float s0 = w1.x - w0.x;
        float s1 = w2.x - w0.x;
        float t0 = w1.y - w0.y;
        float t1 = w2.y - w0.y;
                                
        float r = 1.0F / (s0 * t1 - s1 * t0);
        D3DXVECTOR3 sdir( (t1 * x0 - t0 * x1) * r,
                          (t1 * y0 - t0 * y1) * r,
                          (t1 * z0 - t0 * z1) * r );
        D3DXVECTOR3 tdir( (s0 * x1 - s1 * x0) * r,
                          (s0 * y1 - s1 * y0) * r,
                          (s0 * z1 - s1 * z0) * r );
 
        tan1[i0] += sdir;
        tan1[i1] += sdir;
        tan1[i2] += sdir;

        tan2[i0] += tdir;
        tan2[i1] += tdir;
        tan2[i2] += tdir;
    }

    void on_error( const std::string& s )
    {
        OutputDebugStringA( s.c_str() );
    }

    bool setup_shader( LPDIRECT3DDEVICE9 device )
    {
        ID3DXEffect*    effect;
        ID3DXBuffer*    error;
        HRESULT hr;

        // shaders
        char filename[MAX_PATH];
        strcpy( filename, data_directory_.c_str() );
        strcat( filename, "\\" );
        strcat( filename, "generic_brdf.fx" );
        //strcat( filename, "lambert.fx" );
        //strcat( filename, "CookTorrance.fx" );

        hr = D3DXCreateEffectFromFileA(
            device,
            filename,
            NULL,
            NULL,
            D3DXSHADER_DEBUG,
            NULL,
            &effect,
            &error);
        if( FAILED( hr ) ) {
            if( error ) {
                OutputDebugStringA( (LPCSTR)error->GetBufferPointer() );
            } else {
                OutputDebugStringA( "fatal error: D3DXCreateEffectFromFile\n" );
            }
            return false;
        }

        destroy_shader();
        effect_ = effect;
                
        technique_ =
            effect_->GetTechniqueByName( "TShader" );
        par_local_to_projection_ =
            effect_->GetParameterByName( NULL, "local_to_projection" );
        par_local_to_world_ =
            effect_->GetParameterByName( NULL, "local_to_world" );
        par_eye_ =
            effect_->GetParameterByName( NULL, "eye_position" );
        par_light_position_ =
            effect_->GetParameterByName( NULL, "light0_position" );

        device->CreateVertexDeclaration(
            sample_declaration,&vertex_declaration_);

        return true;
    }

    void destroy_shader()
    {
        if( vertex_declaration_ ) {
            vertex_declaration_->Release();
            vertex_declaration_ = NULL;
        }
        if( effect_ ) {
            effect_->Release();
            effect_ = NULL;
        }
    }

    int search_face( std::set< TmpFace >& tmpfaces, int i0, int i1, int i2 ) 
    {
        TmpFace t;
        t.indices[0] = i0;
        t.indices[1] = i1;
        t.indices[2] = i2;

        std::set< TmpFace >::iterator i = tmpfaces.find( t );
        if( i == tmpfaces.end() ) {
            return -1;
        } else {
            return (*i).source_index;
        }
    }

    void calculate_subdiv_vertices()
    {
        D3DXVECTOR3 zero( 0, 0, 0 );

        // positionの初期化
        size_t n = subdiv_vertices_upper_[0];
        for( size_t i = 0 ; i < n ; i++ ) {
            SubdivVertex& v = subdiv_vertices_[i];
            v.position = v.mesh->get_points()[v.indices[0]].new_position;
            v.normal = v.mesh->get_points()[v.indices[0]].normal;
        }

        n = subdiv_vertices_upper_.size();
        size_t vertex_first = subdiv_vertices_upper_[0];
        size_t edge_first = subdiv_edges_upper_[0];
        for( size_t i = 1 ; i < n ; i++ ) {
            // positionの計算
            size_t m = subdiv_vertices_upper_[i];
            for( size_t j = vertex_first ; j < m ; j++ ) {
                SubdivVertex& v = subdiv_vertices_[j];
                v.position = zero;
                v.normal = zero;

                assert( v.mesh == NULL );
                for( int j = 0 ; j < 3 ; j++ ) {
                    v.position += subdiv_vertices_[v.indices[j]].position;
                    v.normal += subdiv_vertices_[v.indices[j]].normal;
                }
                v.position *= 1.0f / 3.0f;
                D3DXVec3Normalize( &v.normal, &v.normal );
            }

            // neighbor情報を収集
            m = subdiv_vertices_upper_[i];
            for( size_t j = 0 ; j < m ; j++ ) {
                SubdivVertex& v = subdiv_vertices_[j];
                v.neighbor_sum = zero;
                v.neighbor_count = 0;
            }

            m = subdiv_edges_upper_[i];
            for( size_t j = edge_first ; j < m ; j++ ) {
                const Edge& e = subdiv_edges_[j];
                int i0 = e.indices[0];
                int i1 = e.indices[1];

                subdiv_vertices_[i0].neighbor_sum +=
                    subdiv_vertices_[i1].position;
                subdiv_vertices_[i0].neighbor_count++;
                subdiv_vertices_[i1].neighbor_sum +=
                    subdiv_vertices_[i0].position;
                subdiv_vertices_[i1].neighbor_count++;
            }

            // alphaを計算してsmoothing
            m = subdiv_vertices_upper_[i];
            for( size_t j = 0 ; j < m ; j++ ) {
                SubdivVertex& v = subdiv_vertices_[j];

                float alpha =
                    ( 4.0f -
                      2.0f * cosf( D3DX_PI * 2.0f / v.neighbor_count ) ) /
                    9.0f;
                v.position =
                    ( 1.0f - alpha ) * v.position +
                    alpha * v.neighbor_sum / v.neighbor_count;
            }
                        
            vertex_first = subdiv_vertices_upper_[i];
            edge_first = subdiv_edges_upper_[i];
        }
    }
        
    void update_parts_vertices()
    {
        calculate_subdiv_vertices();

        size_t n = parts_.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            update_part_vertices( parts_[i].get() );
        }
    }

    void update_part_vertices( Part* y )
    {
        D3DXVECTOR3 zero( 0, 0, 0 );

        // 頂点書き換え
        if( !y->tied ) { return; }

        size_t n = y->tyings.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            Tying& yt = y->tyings[i];
            Face& xf = subdiv_faces_[yt.triangle_index];

            const SubdivVertex& v0 = subdiv_vertices_[xf.indices[0]];
            const SubdivVertex& v1 = subdiv_vertices_[xf.indices[1]];
            const SubdivVertex& v2 = subdiv_vertices_[xf.indices[2]];

            D3DXVECTOR3 q =
                yt.uq * v0.position +
                yt.vq * v1.position +
                yt.wq * v2.position;
            D3DXVECTOR3 n =
                yt.uq * v0.normal +
                yt.vq * v1.normal +
                yt.wq * v2.normal;
            D3DXVECTOR3 U = v1.position - v0.position;
            D3DXVECTOR3 V = v2.position - v0.position;
            D3DXVECTOR3 W = n;
            D3DXVECTOR3 p = q + yt.u * U + yt.v * V + yt.w * W;
            y->plain_vertices[i].position = p;
            y->plain_vertices[i].normal = zero;
        }

        n = y->groups.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            const Group& g = y->groups[i];
            size_t m = g.faces.size();
            for( size_t j = 0 ; j < m ; j++ ) {
                const Face& f = g.faces[j];
                const TexturedVertex& tv0 = y->textured_vertices[f.indices[0]];
                const TexturedVertex& tv1 = y->textured_vertices[f.indices[1]];
                const TexturedVertex& tv2 = y->textured_vertices[f.indices[2]];
                PlainVertex& v0 = y->plain_vertices[tv0.plain_index];
                PlainVertex& v1 = y->plain_vertices[tv1.plain_index];
                PlainVertex& v2 = y->plain_vertices[tv2.plain_index];
                D3DXVECTOR3 n = cross(
                    v1.position - v0.position,
                    v2.position - v0.position );
                v0.normal += n;
                v1.normal += n;
                v2.normal += n;
            }
        }

        n = y->plain_vertices.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            normalize_f( y->plain_vertices[i].normal );
        }

        model_vertex_type* q;
        if( FAILED( y->vb->Lock( 0, 0, (void**)&q, 0 ) ) ) {
            on_error( "Lock vertexbuffer failed\n" );
            y->vb->Release();
            return;
        }
                
        write_part_to_vertex_buffer( y, q );
                
        y->vb->Unlock();
    }

    void write_part_to_vertex_buffer( Part* p, model_vertex_type* q )
    {
        size_t mm = p->textured_vertices.size();
        for( size_t j = 0 ; j < mm ; j++ ){
            const TexturedVertex& tv = p->textured_vertices[j];
            const PlainVertex& pv = p->plain_vertices[tv.plain_index];

            model_vertex_type v;
            v.position      = pv.position;
            v.normal        = pv.normal;
            v.tangent       = pv.tangent;
            //v.tex0.x = v.tex1.x = v.tex2.x = v.tex3.x = 0;
            //v.tex0.y = v.tex1.y = v.tex2.y = v.tex3.y = 0;
            v.tex0          = tv.uv;
            *q++ = v;
        }
    }

private:
    bool                            valid_;
    bool                            toggle_display_normal_;
    std::string                     data_directory_;

    // appearance
    std::vector< boost::shared_ptr< Part > > parts_;

    // texture library
    std::map< std::string, IDirect3DTexture9* > textures_;

    // physics
    std::vector< boost::shared_ptr< px::tetrahedralmesh_type > > meshes_;
    std::vector< boost::shared_ptr< px::softvolume_type > > coarses_;

    // shader
    IDirect3DVertexDeclaration9*    vertex_declaration_;
    ID3DXEffect*                    effect_;
    D3DXHANDLE                      technique_;
    D3DXHANDLE                      par_local_to_projection_;
    D3DXHANDLE                      par_local_to_world_;
    D3DXHANDLE                      par_eye_;
    D3DXHANDLE                      par_light_position_;
    D3DXVECTOR4                     light_position_;

    // subdivision
    bool                            subdivision_;
    std::vector< SubdivVertex >     subdiv_vertices_;
    std::vector< Face >             subdiv_faces_;
    std::vector< Edge >             subdiv_edges_;
    std::vector< size_t >           subdiv_vertices_upper_;
    std::vector< size_t >           subdiv_edges_upper_;
};

/*============================================================================
 *
 * class PapiShape 
 *
 * 
 *
 *==========================================================================*/
//<<<<<<<<<< PapiShape

//****************************************************************
// constructor
PapiShape::PapiShape()
{
    pimpl.reset( new PapiShapeImp );
}

//****************************************************************
// destructor
PapiShape::~PapiShape()
{
}

//****************************************************************
// load_dense
dense_handle PapiShape::load_dense(
    LPDIRECT3DDEVICE9 device, const std::string& objfilename )
{
    return pimpl->load_dense( device, objfilename );
}

//****************************************************************
// load_coarse
void PapiShape::load_coarse(
    px::world_type*     world,
    const std::string&  filename,
    float               mass,
    float               stiffness )
{
    pimpl->load_coarse( world, filename, mass, stiffness );
}

//****************************************************************
// update
void PapiShape::update( float tick )
{
    pimpl->update( tick );
}

//****************************************************************
// render
void PapiShape::render( LPDIRECT3DDEVICE9 device, const D3DXVECTOR3& eye )
{
    pimpl->render( device, eye );
}

//****************************************************************
// fix
void PapiShape::fix()
{
    pimpl->fix();
}

//****************************************************************
// on_lost_device
void PapiShape::on_lost_device( LPDIRECT3DDEVICE9 device )
{
    pimpl->on_lost_device( device );
}

//****************************************************************
// on_reset_device
void PapiShape::on_reset_device( LPDIRECT3DDEVICE9 device )
{
    pimpl->on_reset_device( device );
}

//****************************************************************
// subdivide
void PapiShape::subdivide( int level )
{
    pimpl->subdivide( level );
}

//****************************************************************
// tie_dense_to_coarse
void PapiShape::tie_dense_to_coarse( dense_handle d )
{
    pimpl->tie_dense_to_coarse( d );
}

//****************************************************************
// get_coarses
std::vector< boost::shared_ptr< px::softvolume_type > >&
PapiShape::get_coarses()
{
    return pimpl->get_coarses();
}

//****************************************************************
// attach_brdf
void PapiShape::attach_brdf(
    LPDIRECT3DDEVICE9   device,
    dense_handle        d,
    const char*         material,
    const std::string&  brdf_filename )
{
    pimpl->attach_brdf( device, d, material, brdf_filename );
}

//****************************************************************
// attach_smap
void PapiShape::attach_smap(
    LPDIRECT3DDEVICE9   device,
    dense_handle        d,
    const char*         material,
    const std::string&  smap_filename )
{
    pimpl->attach_smap( device, d, material, smap_filename );
}

//****************************************************************
// reload_textures
void PapiShape::reload_textures( LPDIRECT3DDEVICE9 device )
{
    pimpl->reload_textures( device );
}

//****************************************************************
// reload_models
void PapiShape::reload_models( LPDIRECT3DDEVICE9 device )
{
}

//****************************************************************
// set_data_directory
void PapiShape::set_data_directory( const std::string& dir )
{
    pimpl->set_data_directory( dir );
}

//>>>>>>>>>> PapiShape

