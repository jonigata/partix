#ifndef PARTIX_UTILS_HPP
#define PARTIX_UTILS_HPP

#include <iostream> 

namespace partix_utils_detail {

template < class PartixTraits >
struct face_compare {
public:
	face_compare(
		int a,
		const typename partix::Cloud< PartixTraits >::points_type& p )
		: axis( a ), points( p ) {}
	bool operator()( const partix::Face< PartixTraits >& f0,
					 const partix::Face< PartixTraits >& f1 ) const
	{
		typedef typename PartixTraits::vector_type vector_type;

		const vector_type& v00 = points[ f0.i0 ].new_position;
		const vector_type& v01 = points[ f0.i1 ].new_position;
		const vector_type& v02 = points[ f0.i2 ].new_position;
		vector_type c0 = ( v00 + v01 + v02 ) / 3.0f;

		const vector_type& v10 = points[ f1.i0 ].new_position;
		const vector_type& v11 = points[ f1.i1 ].new_position;
		const vector_type& v12 = points[ f1.i2 ].new_position;
		vector_type c1 = ( v10 + v11 + v12 ) / 3.0f;

		switch( axis ) {
		case 0: return c0.x < c1.x; 
		case 1: return c0.y < c1.y; 
		case 2: return c0.z < c1.z; 
		default: assert(0); return false;
		}
	}

	int														   axis;
	const typename partix::Cloud< PartixTraits >::points_type& points;
};

template < class PartixTraits >
inline void update_minmax( typename PartixTraits::real_type& mn,
						   typename PartixTraits::real_type& mx,
						   typename PartixTraits::real_type v )
{
	if( v < mn ) { mn = v; }
	if( mx < v ) { mx = v; }
}

template < class PartixTraits >
inline void update_bb( typename PartixTraits::vector_type& bbmin,
					   typename PartixTraits::vector_type& bbmax,
					   const typename PartixTraits::vector_type& p )
{
	update_minmax<PartixTraits>( bbmin.x, bbmax.x, p.x );
	update_minmax<PartixTraits>( bbmin.y, bbmax.y, p.y );
	update_minmax<PartixTraits>( bbmin.z, bbmax.z, p.z );
}

template < class PartixTraits, class BlockFactory >
void divide_block(
	BlockFactory					   f_b,
	int								   threshold,
	partix::SoftShell< PartixTraits >* body,
	partix::Cloud< PartixTraits >*	   c,
	partix::Block< PartixTraits >*	   b )
{
	typedef typename PartixTraits::vector_type vector_type;

	typename partix::Block< PartixTraits >::faces_type& faces =
		b->get_faces();

	int n = int( faces.size() );

	if( n < threshold ) {
		assert( !b->get_faces().empty() );

		b->set_cloud( c );
		b->set_body( body ); 
		b->setup();
		body->add_block( b );
		return;
	}

	typename partix::Cloud< PartixTraits >::points_type& points =
		c->get_points();

	// bounding box
	vector_type bbmin, bbmax;

	const float fmax = (std::numeric_limits< PartixTraits::real_type >::max)();
	bbmin = vector_type(  fmax,	 fmax,	fmax );
	bbmax = vector_type( -fmax, -fmax, -fmax );

	for( int i = 0 ; i < n ; i++ ) {
		partix::Face< PartixTraits >& face = faces[i];
		const vector_type& v0 = points[ face.i0 ].new_position;
		const vector_type& v1 = points[ face.i1 ].new_position;
		const vector_type& v2 = points[ face.i2 ].new_position;
		vector_type center = ( v0 + v1 + v2 ) / 3.0f;
		update_bb<PartixTraits>( bbmin, bbmax, center );

	}

	// longest axis
	int axis;
	vector_type bbw = bbmax - bbmin;
	if( bbw.y <= bbw.x && bbw.z <= bbw.x ) {
		axis = 0;
	} else if( bbw.z <= bbw. y ) {
		axis = 1;
	} else {
		axis = 2;
	}
		  
	// sort along axis
	std::sort( faces.begin(), faces.end(),
			   face_compare< PartixTraits  >( axis, points ) );

	// divide
	if( 2 <= n ) {
		partix::Block< PartixTraits >* b1 = f_b();

		int m = n/2;
		partix::Block< PartixTraits >::faces_type new_faces;
		new_faces.reserve( m );
		for( int i = 0 ; i < m ; i++ ) {
			new_faces.push_back( faces[i] );
		}
		for( int i = m ; i < n ; i++ ) {
			b1->add_face( faces[i].i0, faces[i].i1, faces[i].i2 ); 
		}
		faces.swap( new_faces );

		divide_block( f_b, threshold, body, c, b );
		divide_block( f_b, threshold, body, c, b1 );
	}
}

} // namespace partix_utils_detail

template < class PartixTraits,
		   class SoftVolumeFactory,
		   class TetrahedralMeshFactory >
partix::SoftVolume< PartixTraits >*
make_volume_body(
	const char*				basename,
	SoftVolumeFactory		f_sv,
	TetrahedralMeshFactory	f_tm,
	float					scale,
	float					mass )
{
	typedef typename PartixTraits::vector_type vector_type;

	vector_type v0( 0, 0, 0 );

	partix::TetrahedralMesh< PartixTraits >* e = f_tm();

#if 0
	char filename[256]; 
	sprintf( filename, "physics/%s.tcf", basename );
#endif

	char line[1024];
	const char* p = NULL;

	{
		std::ifstream ifs( filename );

		flStreamText<flStreamFile> ifs;
		if( !ifs.fOpen( filename ) ) {
			flASSERT(0);
		}

		ifs.fGetLine( line, 1024 );

		// node
		int node_count;
		p = ifs.GetInt( &node_count, line );
		for( int i = 0 ; i < node_count ; i++ ) {
			ifs.fGetLine( line, 1024 );

			vector_type v;
			int dummy;
			p = ifs.GetInt( &dummy, line );
			p = ifs.GetDV( &v.x, p );
			p = ifs.GetDV( &v.y, p );
			p = ifs.GetDV( &v.z, p );
			v.x *= scale;
			v.y *= scale;
			v.z *= -scale;

			e->add_point( v, mass );
		}

		// ele			
		ifs.fGetLine( line, 1024 );

		int ele_count;
		p = ifs.GetInt( &ele_count, line );
		for( int i = 0 ; i < ele_count ; i++ ) {
			ifs.fGetLine( line, 1024 );

			int dummy, i0, i1, i2, i3;
			p = ifs.GetInt( &dummy, line );
			p = ifs.GetInt( &i0, p );
			p = ifs.GetInt( &i1, p );
			p = ifs.GetInt( &i2, p );
			p = ifs.GetInt( &i3, p );

			e->add_tetrahedron( i0, i1, i2, i3 );
		}

		// face
		ifs.fGetLine( line, 1024 );

		int face_count;
		p = ifs.GetInt( &face_count, line );
		for( int i = 0 ; i < face_count ; i++ ) {
			ifs.fGetLine( line, 1024 );

			int dummy, i0, i1, i2;
			p = ifs.GetInt( &dummy, line );
			p = ifs.GetInt( &i0, p );
			p = ifs.GetInt( &i1, p );
			p = ifs.GetInt( &i2, p );

			e->add_face( i0, i1, i2 ); // ”½“]
		}
	}

	e->setup();
	partix::SoftVolume< PartixTraits >* v = f_sv();
	v->set_mesh( e );

	v->regularize();

	return v;
}

#endif // PARTIX_UTILS_HPP
