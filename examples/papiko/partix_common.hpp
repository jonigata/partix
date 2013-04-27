#ifndef PHYSICS_COMMON_HPP
#define PHYSICS_COMMON_HPP

#include "partix/cpu_ray_triangle_tester.hpp"
#include "partix/partix.hpp"

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
    struct point_load_type {
        float           selection;
        D3DXVECTOR3     select_offset;
    };

    static float speed_drag_coefficient() { return  0.0001f; }
    static float kinetic_friction() { return 40.0f; }

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

typedef partix::package< D3DXPartixTraits > px;

////////////////////////////////////////////////////////////////
// load_body
inline
partix::SoftVolume< D3DXPartixTraits >*
load_body( const char* filename, float mass = 0.1f )
{
    std::string basename( filename );

    D3DXVECTOR3 v0( 0, 0, 0 );
        
    px::tetrahedralmesh_type* e = new px::tetrahedralmesh_type;

    // .node(頂点座標)読み込み
    {
        std::ifstream ifs( ( basename + ".node" ).c_str() );
        int node_count, dummy;
        ifs >> node_count >> dummy >> dummy >> dummy;
        for( int i = 0 ; i < node_count ; i++ ) {
            D3DXVECTOR3 v;
            ifs >> dummy >> v.x >> v.y >> v.z;
            e->add_point( v, mass );
        }
    }

    // .ele(tetrahedron)読み込み
    {
        std::ifstream ifs( ( basename + ".ele" ).c_str());
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
            e->add_face( i0, i2, i1 ); // 裏返ってるので反転
        }
    }
        
    e->setup();
    px::softvolume_type* v = new px::softvolume_type;
    v->set_mesh( e );

#if 0
    char buffer[256];
    sprintf( buffer, "edge ave: %f", v->get_mesh()->get_average_edge_length() );
    OutputDebugStringA( buffer );
#endif
    v->regularize();

    return v;
}

inline
void draw_wireframe( LPDIRECT3DDEVICE9 device, px::softvolume_type* volume )
{
#pragma pack( push,1 )
    struct dot_vertex_type {
        enum { format = (D3DFVF_XYZ | D3DFVF_DIFFUSE) };
        D3DXVECTOR3     pos;
        DWORD           color;
    };
#pragma pack( pop )


    D3DXMATRIX m;
    D3DXMatrixIdentity(&m);
    device->SetTransform( D3DTS_WORLD, &m );

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
    device->SetFVF( dot_vertex_type::format );

    dot_vertex_type vertices[32768];
    int vertex_count = 0;
    px::tetrahedralmesh_type::points_type& points =
        volume->get_mesh()->get_points();
    px::tetrahedralmesh_type::edges_type& edges =
        volume->get_mesh()->get_edges();

    int n = int( edges.size() );
    for( int j = 0 ; j < n ; j++ ) {
        dot_vertex_type v;

        partix::TetrahedralMesh< D3DXPartixTraits >::edge_type& e = edges[j];
        if( points[e.indices.i0].collided && points[e.indices.i1].collided ) {
            v.color = D3DCOLOR_XRGB( 255, 0, 0 );
            //v.color = D3DCOLOR_XRGB( 0, 0, 255 );
        } else if( points[e.indices.i0].collided ||
                   points[e.indices.i1].collided ) {
            v.color = D3DCOLOR_XRGB( 255, 0, 255 );
        } else {
            v.color = D3DCOLOR_XRGB( 0, 0, 255 );
            if( volume->get_frozen() ) {
                v.color = D3DCOLOR_XRGB( 0, 255, 255 );
                if( volume->get_defrosting() ) {
                    v.color = D3DCOLOR_XRGB( 255, 255, 0 );
                }
            }
        }
        v.pos = points[e.indices.i0].new_position;
        vertices[vertex_count++] = v;
        v.pos = points[e.indices.i1].new_position;
        vertices[vertex_count++] = v;

    }
                                
    device->DrawPrimitiveUP(
        D3DPT_LINELIST, vertex_count/2, vertices, sizeof(dot_vertex_type));
}

#endif // PHYSICS_COMMON_HPP
