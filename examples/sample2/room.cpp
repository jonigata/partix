// $Id: room.cpp 32 2008-10-25 12:19:56Z Naoyuki.Hirayama $

#include "room.hpp"

/*============================================================================
 *
 * class Room 
 *
 * 
 *
 *==========================================================================*/
//<<<<<<<<<< Room

//****************************************************************
// constructor
Room::Room()
{
    D3DCOLOR c = D3DCOLOR_ARGB( 0x40, 0x20, 0x30, 0xee );
    face_vertices_[0]( -5.0f, -5.0f, -5.0f, c );
    face_vertices_[1](  5.0f, -5.0f, -5.0f, c );
    face_vertices_[2](  5.0f,  5.0f, -5.0f, c );
    face_vertices_[3]( -5.0f,  5.0f, -5.0f, c );
    face_vertices_[4]( -5.0f,  5.0f,  5.0f, c );
    face_vertices_[5](  5.0f,  5.0f,  5.0f, c );
    face_vertices_[6](  5.0f, -5.0f,  5.0f, c );
    face_vertices_[7]( -5.0f, -5.0f,  5.0f, c );

    const static int indices[] = {
        0, 1, 2,  0, 2, 3,
        2, 4, 3,  2, 5, 4,
        4, 5, 6,  4, 6, 7,
        6, 0, 7,  6, 1, 0,
        0, 3, 4,  0, 4, 7,
        1, 5, 2,  1, 6, 5,
    };
    for( int i = 0 ; i < 36 ; i++ ) { face_indices_[i] = indices[i]; }
        
    c = D3DCOLOR_ARGB( 0xaa, 0xee, 0xee, 0xaa );
    edge_vertices_[0]( -5.0f, -5.0f, -5.0f, c );
    edge_vertices_[1](  5.0f, -5.0f, -5.0f, c );
    edge_vertices_[2](  5.0f,  5.0f, -5.0f, c );
    edge_vertices_[3]( -5.0f,  5.0f, -5.0f, c );
    edge_vertices_[4]( -5.0f,  5.0f,  5.0f, c );
    edge_vertices_[5](  5.0f,  5.0f,  5.0f, c );
    edge_vertices_[6](  5.0f, -5.0f,  5.0f, c );
    edge_vertices_[7]( -5.0f, -5.0f,  5.0f, c );

    edge_indices_[0] = 0; edge_indices_[1] = 1;
    edge_indices_[2] = 1; edge_indices_[3] = 2;
    edge_indices_[4] = 2; edge_indices_[5] = 3;
    edge_indices_[6] = 3; edge_indices_[7] = 0;

    edge_indices_[8] = 4; edge_indices_[9] = 5;
    edge_indices_[10] = 5; edge_indices_[11] = 6;
    edge_indices_[12] = 6; edge_indices_[13] = 7;
    edge_indices_[14] = 7; edge_indices_[15] = 4;

    edge_indices_[16] = 0; edge_indices_[17] = 7;
    edge_indices_[18] = 1; edge_indices_[19] = 6;
    edge_indices_[20] = 2; edge_indices_[21] = 5;
    edge_indices_[22] = 3; edge_indices_[23] = 4;

        
}

//****************************************************************
// destructor
Room::~Room()
{
}

//****************************************************************
// render
void Room::render( LPDIRECT3DDEVICE9 device )
{
    D3DXMATRIX m;
    D3DXMatrixIdentity( &m );
    device->SetTransform( D3DTS_WORLD, &m );

    device->SetRenderState( D3DRS_FILLMODE, D3DFILL_SOLID ) ; 
    device->SetRenderState( D3DRS_LIGHTING, FALSE ) ; 
    device->SetRenderState( D3DRS_ZENABLE, D3DZB_TRUE ) ; 
    device->SetRenderState( D3DRS_ZWRITEENABLE, TRUE ) ; 
    device->SetRenderState( D3DRS_ALPHABLENDENABLE, TRUE ) ; 
    device->SetRenderState( D3DRS_SRCBLEND, D3DBLEND_SRCALPHA ) ; 
    device->SetRenderState( D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA ) ; 
    device->SetTextureStageState( 0, D3DTSS_COLOROP, D3DTOP_DISABLE ) ; 
    device->SetTextureStageState( 0, D3DTSS_ALPHAOP, D3DTOP_DISABLE ) ; 
    device->SetFVF( vertex_type::format ) ; 

    //device->SetRenderState( D3DRS_CULLMODE, D3DCULL_NONE ) ; 
    device->DrawIndexedPrimitiveUP( 
        D3DPT_TRIANGLELIST, 
        0, 
        8, 
        12, 
        face_indices_, 
        D3DFMT_INDEX16, 
        face_vertices_, 
        sizeof( vertex_type ) );
    device->DrawIndexedPrimitiveUP( 
        D3DPT_LINELIST, 
        0, 
        8, 
        12, 
        edge_indices_, 
        D3DFMT_INDEX16, 
        edge_vertices_, 
        sizeof( vertex_type ) );
}

//>>>>>>>>>> Room

