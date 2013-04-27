// Copyright (C) 2006 Naoyuki Hirayama.
// All Rights Reserved.

// $Id$

#include "finger.hpp"

/*============================================================================
 *
 * class Finger 
 *
 * 
 *
 *==========================================================================*/
//<<<<<<<<<< Finger

//****************************************************************
// constructor
Finger::Finger()
{
    waiting_ = true;
}

//****************************************************************
// destructor
Finger::~Finger()
{
}

//****************************************************************
// load_coarse
void Finger::load_coarse( px::world_type* world, const char* filename )
{
    partix::SoftVolume< D3DXPartixTraits >* v = load_body( filename );

    mesh_.reset( v->get_mesh() );
    coarse_.reset( v );
        
    //coarse_->set_restore_factor( 0.2f );
    //coarse_->set_stretch_factor( 0.8f );
    coarse_->set_name( "finger" );
    coarse_->set_drag_factor( 0.4f );
    coarse_->teleport( D3DXVECTOR3( 0, 2.0f, -10.0f ) );

    world->add_body( coarse_.get() );
}

//****************************************************************
// render
void Finger::render( LPDIRECT3DDEVICE9 device, const D3DXVECTOR4& eye )
{
}

//****************************************************************
// attack
void Finger::attack( const D3DXVECTOR3& pos )
{
    float thre = 2.0f;
        
    D3DXVECTOR3 v = pos - coarse_->get_current_center();
    float l = D3DXVec3Length( &v );
    if( waiting_ && 1.0f < l ) {
        D3DXVec3Normalize( &v, &v );
        v *= -0.5f;
        D3DXVECTOR3 t = pos + v;
        coarse_->kill_inertia();
        coarse_->teleport( t - coarse_->get_current_center() );
    } else {
        px::tetrahedralmesh_type::points_type& points = mesh_->get_points();

        size_t n = points.size();
        for( size_t i = 0 ; i < n ; i++ ) {
            if( 0.2f < points[i].source_position.y ) {
                D3DXVECTOR3 d = pos - points[i].new_position;
                if( thre < D3DXVec3Length( &d ) ) {
                    d /= thre;
                    D3DXVec3Normalize( &d, &d );
                    d *= thre;
                }
                points[i].forces += d * 50.0f;
            }
        }
    }

    coarse_->set_positive( true );

    waiting_ = false;
}

//****************************************************************
// wait
void Finger::wait( const D3DXVECTOR3& pos )
{
#if 0
    float thre = 2.0f;
        
    px::tetrahedralmesh_type::points_type& points = mesh_->get_points();

    size_t n = points.size();
    for( size_t i = 0 ; i < n ; i++ ) {
        if( points[i].source_position.y < - 0.2f ) {
            D3DXVECTOR3 d = pos - points[i].new_position;
            if( thre < D3DXVec3Length( &d ) ) {
                d /= thre;
                D3DXVec3Normalize( &d, &d );
                d *= thre;
            }
            points[i].forces += d * 50.0f;
        }
    }
#else
    D3DXVECTOR3 v = pos - coarse_->get_current_center();
    coarse_->kill_inertia();
    coarse_->teleport( v );

#endif

    waiting_ = true;
    coarse_->set_positive( false );
}

//>>>>>>>>>> Finger

