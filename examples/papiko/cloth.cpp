// Copyright (C) 2006 Naoyuki Hirayama.
// All Rights Reserved.

// $Id$

#include "cloth.hpp"
#include "wavefront_obj.hpp"
#include "partix/partix_cloth.hpp"

struct tmp_spring {
    int     i0;
    int     i1;

    bool operator<( const tmp_spring& x ) const
    {
        if( i0 < x.i0 ) { return true; }
        if( i0 > x.i0 ) { return false; }
        if( i1 < x.i1 ) { return true; }
        return false;
    }
};

float square( float x ) { return x * x; }


/*============================================================================
 *
 * class Cloth 
 *
 * 
 *
 *==========================================================================*/
//<<<<<<<<<< Cloth

//****************************************************************
// constructor
Cloth::Cloth()
{
}

//****************************************************************
// destructor
Cloth::~Cloth()
{
}

//****************************************************************
// load_coarse
void Cloth::load_coarse( px::world_type* world, const char* filename )
{
    // mesh
    std::ifstream ifs ( filename );
    wavefront_obj_reader wobj( ifs );
        
    // physical object
    cloud_.reset( new px::cloud_type ); 
    cloth_.reset( new px::cloth_type );
    cloth_->set_cloud( cloud_.get() );
    cloth_->set_global_force( D3DXVECTOR3( 0, -5.0f, 0 ) );
    cloth_->set_drag_factor( 0.3f );
    cloth_->set_thickness( 0.1f );

    // vertex
    size_t n = wobj.v.size();
    for( size_t i = 0 ; i < n ; i++ ) {
        cloud_->add_point(
            wobj.v[i], 0.02f * ( 0.3f + abs( wobj.v[i].x ) / 3.0f ) );
    }

    // spring
    std::vector< std::vector< size_t > > neighbors( n );
    std::set< tmp_spring > s;

    // ...regular
    n = wobj.groups.size();
    for( size_t i = 0 ; i < n ; i++ ) {
        wavefront_obj_reader::group& g = wobj.groups[i];
        size_t m = g.faces.size();
        for( size_t j = 0 ; j < m ; j++ ) {
            tmp_spring ts;
            wavefront_obj_reader::face& f = g.faces[j];
            size_t l = f.corners.size();
            for( size_t k = 0 ; k < l ; k++ ) {
                tmp_spring ts;
                ts.i0 = f.corners[k].vertex_index;
                ts.i1 = f.corners[(k+1)%l].vertex_index;
                                
                if( ts.i1 < ts.i0 ) { std::swap( ts.i0, ts.i1 ); }
                s.insert( ts );
                neighbors[ts.i0].push_back( ts.i1 );
                neighbors[ts.i1].push_back( ts.i0 );
            }
        }
    }

    // ...shear/bend
#if 1
    n = wobj.v.size();
    for( size_t i = 0 ; i < n ; i++ ) {
        std::vector< size_t >& v = neighbors[i];
        std::sort( v.begin(), v.end() );
        v.erase( std::unique( v.begin(), v.end() ), v.end() );
        size_t m = v.size();
        for( size_t j = 0 ; j < m ; j++ ) {
            for( size_t k = j+1 ; k < m ; k++ ) {
                tmp_spring ts;
                ts.i0 = v[j];
                ts.i1 = v[k];

                s.insert( ts );
            }
        }
    }
#endif

    for( std::set< tmp_spring >::const_iterator i = s.begin() ;
         i != s.end() ;
         ++i ) {
        cloth_->add_spring( (*i).i0, (*i).i1 );
    }

    // faces
    n = wobj.groups.size();
    for( size_t i = 0 ; i < n ; i++ ) {
        wavefront_obj_reader::group& g = wobj.groups[i];

        size_t m = g.faces.size();
        for( size_t j = 0 ; j < m ; j++ ) {
            wavefront_obj_reader::face& f = g.faces[j];
                                
            int i0 = f.corners[0].vertex_index;
            int i1 = f.corners[1].vertex_index;
            int i2 = f.corners[2].vertex_index;
            cloth_->add_face( i0, i1, i2 );
            if( f.corners.size() == 4 ) {
                int i3 = f.corners[3].vertex_index;
                cloth_->add_face( i0, i2, i3 );
            }
        }
    }


    // cloth
    cloth_->regularize();
    //cloth_->teleport( D3DXVECTOR3( -0.0f, 2.4f, 0 ) ) ;
    world->add_body( cloth_.get() );
}

//****************************************************************
// render
void Cloth::render( LPDIRECT3DDEVICE9 device, const D3DXVECTOR3& eye )
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
    px::cloud_type::points_type& points = cloud_->get_points();

#if 1
    px::cloth_type::faces_type& faces = cloth_->get_faces();

    size_t n = faces.size();
    for( int j = 0 ; j < n ; j++ ) {

        px::cloth_type::face_type f = faces[j];

        dot_vertex_type v;
        v.color = D3DCOLOR_ARGB( 128, 0, 0, 255 );
        v.pos = points[f.i0].new_position; vertices[vertex_count++] = v;
        v.pos = points[f.i1].new_position; vertices[vertex_count++] = v;
        v.pos = points[f.i2].new_position; vertices[vertex_count++] = v;
        v.pos = points[f.i0].new_position; vertices[vertex_count++] = v;
        v.pos = points[f.i2].new_position; vertices[vertex_count++] = v;
        v.pos = points[f.i1].new_position; vertices[vertex_count++] = v;
    }
                                
    device->DrawPrimitiveUP(
        D3DPT_TRIANGLELIST, vertex_count/3, vertices, sizeof(dot_vertex_type));
#else
    px::cloth_type::springs_type& springs = cloth_->get_springs();

    int n = int( springs.size() );
    for( int j = 0 ; j < n ; j++ ) {

        px::cloth_type::spring_type s = springs[j];

        dot_vertex_type v;
        v.color = D3DCOLOR_XRGB( 0, 0, 255 );
        v.pos = points[s.indices.i0].new_position;
        vertices[vertex_count++] = v;
        v.pos = points[s.indices.i1].new_position;
        vertices[vertex_count++] = v;
    }
                                
    device->DrawPrimitiveUP(
        D3DPT_LINELIST, vertex_count/2, vertices, sizeof(dot_vertex_type));
#endif
}

//****************************************************************
// fix
void Cloth::fix()
{
    px::cloth_type::points_type& points = cloud_->get_points();

    size_t n = points.size();
    for( size_t i = 0 ; i < n ; i++ ) {
        if( 0.3f <= points[i].source_position.y ) {
            D3DXVECTOR3 d = points[i].source_position - points[i].new_position;
            points[i].new_position += d;
        }
    }
}

//****************************************************************
// select
void Cloth::select( const D3DXVECTOR3& pos )
{
    const float range = 0.7f;

    px::cloth_type::points_type& points = cloud_->get_points();

    size_t n = points.size();
    for( size_t i = 0 ; i < n ; i++ ) {
        px::point_type& p = points[i];
                
        D3DXVECTOR3 offset = p.new_position - pos;
        float d = D3DXVec3Length( &offset );
        if( d < range ) {
            p.load.selection = ( range - d ) / range;
            p.load.select_offset = offset;
        } else {
            p.load.selection = 0;
        }
    }

    select_origin_ = pos;
}

//****************************************************************
// move_to
void Cloth::move_to( const D3DXVECTOR3& t )
{
    float max_distance = 1.5f;

    px::cloth_type::points_type& points = cloud_->get_points();

    size_t n = points.size();
    for( size_t i = 0 ; i < n ; i++ ) {
        px::point_type& p = points[i];

        D3DXVECTOR3 it = t + p.load.select_offset;
        D3DXVECTOR3 v = it - p.new_position;
        float l = D3DXVec3Length( &v );
        if( max_distance < l ) {
            v *= max_distance / l;
        }

        p.forces += v * p.load.selection * 1.5f;
    }
}

//>>>>>>>>>> Cloth

