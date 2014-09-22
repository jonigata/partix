// 2008/11/12 Naoyuki Hirayama

/*!
	@file	  draw_physie.hpp
	@brief	  <ŠT—v>

	<à–¾>
*/

#ifndef DRAW_PHYSIE_HPP_
#define DRAW_PHYSIE_HPP_

template < class Physie >
void draw_physie( LPDIRECT3DDEVICE9 device, Physie* volume )
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
    Physie::points_type& points = volume->get_mesh()->get_points();
    Physie::edges_type& edges = volume->get_mesh()->get_edges();

    int n = int( edges.size() );
    for( int j = 0 ; j < n ; j++ ) {
        dot_vertex_type v;
		
        Physie::edge_type& e = edges[j];
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

#if 0
		// collision normal
		v.color = D3DCOLOR_XRGB( 0, 255, 255 );
        v.pos = points[e.indices.i0].new_position;
        vertices[vertex_count++] = v;
        v.pos = v.pos + e.collision_normal;
        vertices[vertex_count++] = v;

        v.pos = points[e.indices.i1].new_position;
        vertices[vertex_count++] = v;
        v.pos = v.pos + e.collision_normal;
        vertices[vertex_count++] = v;
#endif
    }

	n = int( points.size() );
	for( int j = 0 ; j < n ; j++ ) {
		Physie::point_type& p = points[j];

        dot_vertex_type v;
#if 0
		// velocity
		v.color = D3DCOLOR_XRGB( 0, 255, 0 );
		v.pos = p.new_position;
		vertices[vertex_count++] = v;
		v.pos = v.pos - p.tmp_velocity * 50.0f;
		vertices[vertex_count++] = v;
#endif

#if 1
		// view_vector1
		v.color = D3DCOLOR_XRGB( 255, 255, 0 );
		v.pos = p.new_position;
		vertices[vertex_count++] = v;
		v.pos = v.pos - p.passive_contact_pushout * 1.0f;
		vertices[vertex_count++] = v;
#endif

#if 1
		// penetration vector
		v.color = D3DCOLOR_XRGB( 0, 255, 0 );
		v.pos = p.new_position;
		vertices[vertex_count++] = v;
#if 1
		v.pos = v.pos - p.active_contact_pushout * 1.0f;
#else
		v.pos = v.pos -
			( p.active_contact_pushout +
			  p.passive_contact_pushout ) * 1.0f;
#endif
		vertices[vertex_count++] = v;
#endif		

#if 1
		// friction vector
		v.color = D3DCOLOR_XRGB( 0, 255, 255 );
		v.pos = p.new_position;
		vertices[vertex_count++] = v;
		v.pos = v.pos + p.friction_vector * 100.0f;
		vertices[vertex_count++] = v;
#endif
	}
                                
    device->DrawPrimitiveUP(
        D3DPT_LINELIST, vertex_count/2, vertices, sizeof(dot_vertex_type));
}

#endif // DRAW_PHYSIE_HPP_
