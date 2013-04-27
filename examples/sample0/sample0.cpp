// $Id: sample0.cpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $

// 環境非依存のphysics sample
// レンダリングはSVGを生成する。

#pragma warning(disable: 4996)
#pragma warning(disable: 4819)

#include <cstdlib>
#include <string>
#include <fstream>
#include "geometry.hpp"
#include "svgout.hpp"
#include "partix/partix.hpp"

const float SIMULATION_TICK = 0.01f;
const float PIG_MASS = 0.1f;              // パーティクルごとの質量

////////////////////////////////////////////////////////////////
// サンプル vector_traits
struct VectorTraits {
    typedef float   real_type;
    typedef Vector  vector_type;

    static real_type epsilon(){ return real_type( 0.000001f ); }
    static real_type x( const vector_type& v ) { return v.x; }
    static real_type y( const vector_type& v ) { return v.y; }
    static real_type z( const vector_type& v ) { return v.z; }
    static void x( vector_type& v, real_type x ) { v.x = x; }
    static void y( vector_type& v, real_type y ) { v.y = y; }
    static void z( vector_type& v, real_type z ) { v.z = z; }

    static vector_type make_vector( real_type x, real_type y, real_type z )
    {
        vector_type v;
        v.x = x;
        v.y = y;
        v.z = z;
        return v;
    }
    static real_type length_sq( const vector_type& v )
    {
        return ::length_sq( v );
    }
    static real_type length( const vector_type& v )
    {
        return ::length( v );
    }
};

////////////////////////////////////////////////////////////////
// サンプル traits
struct SampleTraits {
    typedef VectorTraits    vector_traits;
    typedef float           real_type;
    typedef Vector          vector_type;
    typedef Matrix          matrix_type;
    typedef int             index_type;

    struct body_load_type {};
    struct block_load_type {};
    struct cloud_load_type {};
    struct point_load_type {};

    static float speed_drag_coefficient() { return 0.0001f; }
    static float kinetic_friction() { return 40.0f; }

    static float freeze_threshold_energy() { return 2.0f; }
    static float freeze_duration() { return 0.5f; }

    static void make_matrix(
        matrix_type&            d,
        const real_type*        s,
        const vector_type&      t )
    {
        d.m00 = s[0]; d.m01 = s[3]; d.m02 = s[6]; d.m03 = 0;
        d.m10 = s[1]; d.m11 = s[4]; d.m12 = s[7]; d.m13 = 0;
        d.m20 = s[2]; d.m21 = s[5]; d.m22 = s[8]; d.m23 = 0;
        d.m30 = t.x;  d.m31 = t.y;  d.m32 = t.z;  d.m33 = 1;
    }
};

////////////////////////////////////////////////////////////////
// forward declaration
template < class Traits >
void
make_softvolume(
    const char*                             filename,   // ファイル名
    float                                   mass,       // パーティクルの質量
    partix::SoftVolume< Traits >*&          volume,     // volume (OUT)
    partix::TetrahedralMesh< Traits >*&     mesh        // tetra mesh (OUT)
    );

template < class Traits >
void
svgout_mesh(
    svgout&                                 svgout,
    partix::TetrahedralMesh< Traits >*&     mesh,
    const char*                             color
    );

////////////////////////////////////////////////////////////////
// メインルーチン
int main( int argc, char* argv[] )
{
    // ワールドの作成
    std::cout << "make world" << std::endl;
    partix::World< SampleTraits >* world =
        new partix::World< SampleTraits >;

    // モデルの読み込み(2体)
    std::cout << "load models" << std::endl;
    partix::SoftVolume< SampleTraits >* volume0;        
    partix::SoftVolume< SampleTraits >* volume1;
    partix::TetrahedralMesh< SampleTraits >* mesh0;
    partix::TetrahedralMesh< SampleTraits >* mesh1;

    make_softvolume( "data/pig", PIG_MASS, volume0, mesh0 );
    make_softvolume( "data/pig", PIG_MASS, volume1, mesh1 );
    world->add_body( volume0 );
    world->add_body( volume1 );

    volume0->teleport( VectorTraits::make_vector(  0, 3.0f, 0 ) );
    volume1->teleport( VectorTraits::make_vector(  0, 1.0f, 0 ) );
        
    // 床の作成
    std::cout << "make floor" << std::endl;
    partix::BoundingPlane< SampleTraits >* floor = 
        new partix::BoundingPlane< SampleTraits >(
            VectorTraits::make_vector( 0, -1.0f, 0 ),     // 中心座標
            VectorTraits::make_vector( 0,  1.0f, 0 )      // 法線
            );
    world->add_body( floor );

    // 重力の設定
    std::cout << "set gravity" << std::endl;
    world->set_global_force( VectorTraits::make_vector( 0, -3.0f, 0 ) );

    // シミュレーション
    std::cout << "start simulation" << std::endl;
    world->restart();
    for( int i = 0 ; i <= 250 ; i++ ) {
        world->update( SIMULATION_TICK );

        // SVGで出力
        if( i % 10 == 0 ) {
            char filename[256];
            sprintf( filename, "result/world%03d.svg", i / 10 );
            svgout svg( filename, 512, 512 );
                        
            svgout_mesh( svg, mesh0, "navy" );
            svgout_mesh( svg, mesh1, "blueviolet" );
            std::cout << "output: " << filename << std::endl;
        }
    }

    // 後始末
    delete floor;
    delete mesh0;
    delete mesh1;
    delete volume0;
    delete volume1;
    delete world;

    return 0;
}

////////////////////////////////////////////////////////////////
// TetGen形式( .node, .ele, .face )の読み込み
template < class Traits >
void
make_softvolume(
    const char*                             basename,   // ファイル名
                                                        // (拡張子を除いた部分)
    float                                   mass,       // パーティクルの質量
    partix::SoftVolume< Traits >*&          volume,     // volume (OUT)
    partix::TetrahedralMesh< Traits >*&     mesh        // tetra mesh (OUT)
    )
{
    typedef partix::Point< Traits >         point_type;
    typedef typename Traits::vector_type    vector_type;

    std::string filename( basename );

    mesh = new partix::TetrahedralMesh< Traits >;

    // .node(頂点座標)読み込み
    {
        std::ifstream ifs( ( filename + ".node" ).c_str() );
        int node_count, dummy;
        ifs >> node_count >> dummy >> dummy >> dummy;
        for( int i = 0 ; i < node_count ; i++ ) {
            vector_type p;
            ifs >> dummy >> p.x >> p.y >> p.z;
            mesh->add_point( p, mass );
        }
    }

    // .ele(tetrahedron)読み込み
    {
        std::ifstream ifs( ( filename + ".ele" ).c_str() );
        int node_count, dummy;
        ifs >> node_count >> dummy >> dummy;
        for( int i = 0 ; i < node_count ; i++ ) {
            int i0, i1, i2, i3;
            ifs >> dummy >> i0 >> i1 >> i2 >> i3;
            mesh->add_tetrahedron( i0, i1, i2, i3 );
        }
    }

    // .face(外接面)読み込み
    {
        std::ifstream ifs( ( filename + ".face" ).c_str() );
        int node_count, dummy;
        ifs >> node_count >> dummy;
        for( int i = 0 ; i < node_count ; i++ ) {
            int i0, i1, i2;
            ifs >> dummy >> i0 >> i1 >> i2 >> dummy;
            mesh->add_face( i0, i2, i1 ); // 裏返ってるので反転
        }
    }

    mesh->setup();
    volume = new partix::SoftVolume< Traits >;
    volume->set_mesh( mesh );
    volume->regularize();
}

////////////////////////////////////////////////////////////////
// SVG出力
template < class Traits >
void
svgout_mesh(
    svgout&                                 svgout,
    partix::TetrahedralMesh< Traits >*&     mesh,
    const char*                             color
    )
{
    typedef typename Traits::vector_traits vector_traits;

    // 変換行列
    Vector camera_position = vector_traits::make_vector( 1.5f, 1.5f, 1.5f );
    Vector camera_target   = vector_traits::make_vector( 0, 0, 0 );
    Vector camera_up       = vector_traits::make_vector( 0, 1, 0 );

    Matrix matrix = look_at_rh(
        camera_position,
        camera_target,
        camera_up );

    typedef typename partix::TetrahedralMesh<Traits>::points_type points_type;
    typedef typename partix::TetrahedralMesh<Traits>::edges_type  edges_type;
        
    const points_type& points = mesh->get_points();
    const edges_type& edges = mesh->get_edges();

    for( typename edges_type::const_iterator i =
             edges.begin() ;
         i != edges.end() ;
         ++i ) {
        Vector v0 = points[(*i).indices.i0].new_position;
        Vector v1 = points[(*i).indices.i1].new_position;

        Vector pv0 = v0 * matrix;
        Vector pv1 = v1 * matrix;

        float v0x =  pv0.x * 64 + 256;
        float v0y = -pv0.y * 64 + 256;
        float v1x =  pv1.x * 64 + 256;
        float v1y = -pv1.y * 64 + 256;

        svgout.line( v0x, v0y, v1x, v1y, color );
    }
}

