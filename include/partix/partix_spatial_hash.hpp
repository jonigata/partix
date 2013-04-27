/*!
  @file     partix_spatial_hash.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: partix_spatial_hash.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_SPATIAL_HASH_HPP
#define PARTIX_SPATIAL_HASH_HPP

#include "fixed_pool.hpp"
#include "voxel_traverser.hpp"
#include "partix_forward.hpp"
#include "partix_geometry.hpp"
#include "partix_math.hpp"

namespace partix {

template < class Traits >
class SpatialHashBase {
public:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::vector_type    vector_type;

public:
	int coord( real_type c )
	{
		return int( floor( c * rgridsize_ ) );
	}
        
	size_t hash_value( int x, int y, int z )
	{
		// large prime numbers
		const int p1 = 73856093;
		const int p2 = 19349663;
		const int p3 = 83492791;

		return size_t( ( ( x * p1 ) ^ ( y * p2 ) ^ ( z * p3 ) )  ) % tablesize_;
	}

	size_t hash( const vector_type& q )
	{
		return hash_value( coord( q.x ), coord( q.y ), coord( q.z ) );
	}

	real_type gridsize() { return gridsize_; }

protected:
	SpatialHashBase( real_type gridsize, size_t tablesize )
		: gridsize_( gridsize ),
		  rgridsize_( real_type( 1.0 ) / gridsize ),
		  tablesize_( tablesize ) {}
        
protected:
	real_type       gridsize_;
	real_type       rgridsize_;
	size_t          tablesize_;

};

// node‚Ì‚Ý‚ÌŒ`Ž®
template < class Traits, class Node >
class DirectSpatialHash : public SpatialHashBase< Traits > {
public:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::vector_type    vector_type;

	struct InternalNode {
		InternalNode*   next;
		Node            node;
	};

	typedef InternalNode* InternalNodePtr;

	typedef InternalNodePtr table_entry_type;

public:
	DirectSpatialHash(
		real_type       gridsize,
		int             tablesize )
		: SpatialHashBase< Traits >( gridsize, tablesize ),
		  pool_( page_provider_ )
	{
		table_ = new InternalNodePtr[ tablesize ];
		memset( table_, 0, sizeof( InternalNodePtr ) * tablesize );
	}
	~DirectSpatialHash()
	{
		delete [] table_;
	}

	void clear( real_type gridsize )
	{
		memset( table_, 0, sizeof( InternalNodePtr ) * this->tablesize_ );
		pool_.clear();
		this->gridsize_ = gridsize;
		this->rgridsize_ = real_type( 1.0 ) / this->gridsize_;
	}

	void insert( size_t hv, const Node& t )
	{
		InternalNode* r = (InternalNode*)pool_.allocate();
		r->next = table_[hv];
		r->node = t;
		table_[hv] = r;
	}

	InternalNodePtr entry( size_t hv )
	{
		return table_[hv];
	}
	InternalNodePtr next( InternalNodePtr p )
	{
		return p->next;
	}
	Node* unwrap( InternalNodePtr p )
	{
		return &p->node;
	}

	template < class Tiee, class Callback >
	void apply( Tiee& tiee, const Callback& c )
	{
		for( size_t i = 0 ; i < this->tablesize_ ; i++ ) {
			if( !entry( i ) ) { continue; }

			for( typename Tiee::table_entry_type p = tiee.entry( i ) ;
				 p ;
				 p = tiee.next( p ) ) {
				for( table_entry_type q = entry(i) ; q ; q = next( q ) ) {
					c( unwrap( p ), unwrap( q ) );
				}
			}

		}
	}

protected:
	default_page_provider										page_provider_;
	fixed_pool< sizeof( InternalNode ), default_page_provider >     pool_;
	InternalNodePtr*                                                table_;

};

// node + refererŒ`Ž®
template < class Traits, class Node >
class IndirectSpatialHash : public SpatialHashBase< Traits > {
public:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::vector_type    vector_type;

	typedef Node InternalNode;
	struct InternalReferer {
		InternalReferer*        next;
		InternalNode*           node;
	};

	typedef InternalReferer* InternalRefererPtr;

	typedef InternalRefererPtr table_entry_type;

public:
	IndirectSpatialHash(
		real_type       gridsize,
		int             tablesize )
		: SpatialHashBase< Traits >( gridsize, tablesize ),
		  pool0_( page_provider_ ),
		  pool1_( page_provider_ )
	{
		table_ = new InternalRefererPtr[ tablesize ];
		memset( table_, 0, sizeof( InternalRefererPtr ) * tablesize );
	}
	~IndirectSpatialHash()
	{
		delete [] table_;
	}

	void clear( real_type gridsize )
	{
		memset( table_, 0, sizeof( InternalRefererPtr ) * this->tablesize_ );
		pool0_.clear();
		pool1_.clear();
		this->gridsize_ = gridsize;
		this->rgridsize_ = real_type( 1.0 ) / this->gridsize_;
	}

	Node* alloc_node()
	{
		return (Node*)pool0_.allocate();
	}

	void is_entry_empty( size_t hv )
	{
		return table_[hv] == NULL;
	}

	void insert( size_t hv, Node* t )
	{
		InternalReferer* r = (InternalReferer*)pool1_.allocate();
		r->next = table_[hv];
		r->node = t;
		table_[hv] = r;
	}

	InternalRefererPtr entry( size_t hv )
	{
		return table_[hv];
	}
	InternalRefererPtr next( InternalRefererPtr p )
	{
		return p->next;
	}
	Node* unwrap( InternalRefererPtr p )
	{
		return p->node;
	}

	template < class Tiee, class Callback >
	void apply( Tiee& tiee, const Callback& c )
	{
		for( size_t i = 0 ; i < this->tablesize_ ; i++ ) {
			//if( !entry( i ) ) { continue; }
                        
			for( typename Tiee::table_entry_type p = tiee.entry( i ) ;
				 p ;
				 p = tiee.next( p ) ) {
				for( table_entry_type q = entry(i) ; q ; q = next( q ) ) {
					if( c( tiee.unwrap( p ), unwrap( q ) ) ) { break; }
				}
			}
		}
#if 0
		char buffer[256];
		sprintf( buffer, "apply %f, %f\n",
				 float(m) / this->tablesize_,
				 float(n) / this->tablesize_ );
		OutputDebugStringA( buffer );
#endif		
	}

protected:
	default_page_provider										page_provider_;
	fixed_pool< sizeof( InternalNode ), default_page_provider >     pool0_;
	fixed_pool< sizeof( InternalReferer ), default_page_provider >  pool1_;
	InternalRefererPtr*                                             table_;

};

/*===========================================================================*
 *
 * general
 *
 *
 * 
 *
 *==========================================================================*/

////////////////////////////////////////////////////////////////
// segment - triangle
template < class Traits >
class SegmentTriangleSpatialHash {
private:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::vector_type    vector_type;

	struct SegmentHashNode {
		vector_type     s0;
		vector_type     s1;
		vector_type     bbmin;
		vector_type     bbmax;
	};
	struct TriangleHashNode {
		vector_type     v0;
		vector_type     v1;
		vector_type     v2;
		vector_type     bbmin;
		vector_type     bbmax;
	};

	struct InternalCallback {
	public:
		InternalCallback( real_type& dist ) : dist_( dist ) {}
		bool operator()(
			const SegmentHashNode* p, const TriangleHashNode* q ) const
		{
			vector_type uvt;
			if( !collide( p, q, uvt ) ) { return false; }
			if( uvt.z < dist_ ) { dist_ = uvt.z; }
			return false;
		}

	private:
		bool collide(
			const SegmentHashNode* pp,
			const TriangleHashNode* qq,
			vector_type& uvt ) const
		{
			if( !math< Traits >::test_aabb_aabb(
					pp->bbmin, pp->bbmax, qq->bbmin, qq->bbmax ) ) {
				return false;
			}

			const vector_type& s0 = pp->s0;
			const vector_type& s1 = pp->s1;

			const vector_type& v0 = qq->v0;
			const vector_type& v1 = qq->v1;
			const vector_type& v2 = qq->v2;

			return math< Traits >::test_segment_triangle(
				s0, s1, v0, v1, v2, uvt );
		}
        
		real_type& dist_;
	};

public:
	SegmentTriangleSpatialHash( real_type gridsize, int tablesize )
		: active_table_( gridsize, tablesize ),
		  passive_table_( gridsize, tablesize )
	{
	}
	~SegmentTriangleSpatialHash()
	{
	}

	void clear( real_type gridsize )
	{
		active_table_.clear( gridsize );
		passive_table_.clear( gridsize );
	}

	void add_segment( const vector_type s0, const vector_type& s1 )
	{
		SegmentHashNode* t = active_table_.alloc_node();
		t->s0 = s0;
		t->s1 = s1;
		math< Traits >::get_segment_bb( s0, s1, t->bbmin, t->bbmax );

		voxel_traverser< real_type, vector_type > vt(
			s0, s1, active_table_.gridsize() );
		int x, y, z;
		while( vt( x, y, z ) ) {
			size_t hv = active_table_.hash_value( x, y, z );
			active_table_.insert( hv, t );
		}
	}

	void add_triangle(
		const vector_type& v0,
		const vector_type& v1,
		const vector_type& v2 )
	{
		real_type gridsize = passive_table_.gridsize();

		vector_type bbmin, bbmax;
		math< Traits >::get_triangle_bb( v0, v1, v2, bbmin, bbmax );

		int x0 = passive_table_.coord( bbmin.x );
		int x1 = passive_table_.coord( bbmax.x );
		int y0 = passive_table_.coord( bbmin.y );
		int y1 = passive_table_.coord( bbmax.y );
		int z0 = passive_table_.coord( bbmin.z );
		int z1 = passive_table_.coord( bbmax.z );

		vector_type n = math< Traits >::normalize(
			math< Traits >::cross( v1 - v0, v2 - v0 ) );
		real_type   d = math< Traits >::dot( n, v0 );

		real_type e = gridsize * real_type( 0.5 );
		real_type r = square( e ) * real_type( 3.0 ); // e*e + e*e + e*e

		TriangleHashNode* t = NULL;
                
		for( int z = z0 ; z <= z1 ; z++ ) {
			for( int y = y0 ; y <= y1 ; y++ ) {
				for( int x = x0 ; x <= x1; x++ ) {
					size_t hv = passive_table_.hash_value( x, y, z );
					if( !active_table_.entry( hv ) ) {
						// optimize: active node‚ª‚È‚¢‚Æ‚±‚ë‚Å‚ÍÈ—ª
						continue;
					}

					// ƒZƒ‹‚Ì’†‰›
					vector_type c;
					c.x = ( x + real_type( 0.5 ) ) * gridsize;
					c.y = ( y + real_type( 0.5 ) ) * gridsize;
					c.z = ( z + real_type( 0.5 ) ) * gridsize;

					// plane - sphere(ƒZƒ‹•‚Ì‘ÎŠpü‚ð’¼Œa‚Æ‚·‚é)‚ÅƒJƒŠƒ“ƒO
					real_type dist = math< Traits >::dot( c, n ) - d;
					if( r < square( dist ) ) { continue; }

					if( !t ) {
						t = passive_table_.alloc_node();
						t->v0    = v0;
						t->v1    = v1;
						t->v2    = v2;
						t->bbmin = bbmin;
						t->bbmax = bbmax;
					}
					passive_table_.insert( hv, t );
				}
			}
		}
	}

	bool apply( real_type& dist )
	{
		dist = math< Traits >::real_max();
		passive_table_.apply( active_table_, InternalCallback( dist ) );
		return dist < math< Traits >::real_max();
	}

private:
	IndirectSpatialHash< Traits, SegmentHashNode >  active_table_;
	IndirectSpatialHash< Traits ,TriangleHashNode > passive_table_;

};

////////////////////////////////////////////////////////////////
// sphere - triangle
template < class Traits >
class SphereTriangleSpatialHash {
private:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::vector_type    vector_type;

	struct SphereHashNode {
		vector_type     center;
		real_type		radius;
	};
	struct TriangleHashNode {
		vector_type     v0;
		vector_type     v1;
		vector_type     v2;
		vector_type     bbmin;
		vector_type     bbmax;
	};

	struct InternalCallback {
	public:
		InternalCallback( real_type& dist ) : dist_( dist ) {}
		bool operator()(
			const SphereHashNode* p, const TriangleHashNode* q ) const
		{
			vector_type uvt;
			if( !collide( p, q, uvt ) ) { return false; }
			if( uvt.z < dist_ ) { dist_ = uvt.z; }
			return false;
		}

	private:
		bool collide(
			const SphereHashNode* pp,
			const TriangleHashNode* qq,
			vector_type& uvt ) const
		{
			const vector_type& s0 = pp->center;
			real_type s1 = pp->radius;

			const vector_type& v0 = qq->v0;
			const vector_type& v1 = qq->v1;
			const vector_type& v2 = qq->v2;

			return math< Traits >::test_sphere_triangle(
				s0, s1, v0, v1, v2, uvt );
		}
        
		real_type& dist_;
	};

public:
	SphereTriangleSpatialHash( real_type gridsize, int tablesize )
		: active_table_( gridsize, tablesize ),
		  passive_table_( gridsize, tablesize )
	{
	}
	~SphereTriangleSpatialHash()
	{
	}

	void clear( real_type gridsize )
	{
		active_table_.clear( gridsize );
		passive_table_.clear( gridsize );
	}

	void add_sphere( const vector_type center, real_type radius )
	{
		SphereHashNode* t = active_table_.alloc_node();
		t->center = center;
		t->radius = radius;

		vector_type r( radius, radius, radius );
		vector_type bbmin = center-r;
		vector_type bbmax = center+r;
		int x0 = passive_table_.coord( bbmin.x );
		int x1 = passive_table_.coord( bbmax.x );
		int y0 = passive_table_.coord( bbmin.y );
		int y1 = passive_table_.coord( bbmax.y );
		int z0 = passive_table_.coord( bbmin.z );
		int z1 = passive_table_.coord( bbmax.z );

		for( int z = z0 ; z <= z1 ; z++ ) {
			for( int y = y0 ; y <= y1 ; y++ ) {
				for( int x = x0 ; x <= x1; x++ ) {
					// TODO: ŽáŠ±Œ¸‚ç‚¹‚é
					size_t hv = active_table_.hash_value( x, y, z );
					active_table_.insert( hv, t );
				}
			}
		}
	}

	void add_triangle(
		const vector_type& v0,
		const vector_type& v1,
		const vector_type& v2 )
	{
		real_type gridsize = passive_table_.gridsize();

		vector_type bbmin, bbmax;
		math< Traits >::get_triangle_bb( v0, v1, v2, bbmin, bbmax );

		int x0 = passive_table_.coord( bbmin.x );
		int x1 = passive_table_.coord( bbmax.x );
		int y0 = passive_table_.coord( bbmin.y );
		int y1 = passive_table_.coord( bbmax.y );
		int z0 = passive_table_.coord( bbmin.z );
		int z1 = passive_table_.coord( bbmax.z );

		vector_type n = math< Traits >::normalize(
			math< Traits >::cross( v1 - v0, v2 - v0 ) );
		real_type   d = math< Traits >::dot( n, v0 );

		real_type e = gridsize * real_type( 0.5 );
		real_type r = square( e ) * real_type( 3.0 ); // e*e + e*e + e*e

		TriangleHashNode* t = NULL;
                
		for( int z = z0 ; z <= z1 ; z++ ) {
			for( int y = y0 ; y <= y1 ; y++ ) {
				for( int x = x0 ; x <= x1; x++ ) {
					size_t hv = passive_table_.hash_value( x, y, z );
					if( !active_table_.entry( hv ) ) {
						// optimize: active node‚ª‚È‚¢‚Æ‚±‚ë‚Å‚ÍÈ—ª
						continue;
					}

					// ƒZƒ‹‚Ì’†‰›
					vector_type c;
					c.x = ( x + real_type( 0.5 ) ) * gridsize;
					c.y = ( y + real_type( 0.5 ) ) * gridsize;
					c.z = ( z + real_type( 0.5 ) ) * gridsize;

					// plane - sphere(ƒZƒ‹•‚Ì‘ÎŠpü‚ð’¼Œa‚Æ‚·‚é)‚ÅƒJƒŠƒ“ƒO
					real_type dist = math< Traits >::dot( c, n ) - d;
					if( r < square( dist ) ) { continue; }

					if( !t ) {
						t = passive_table_.alloc_node();
						t->v0    = v0;
						t->v1    = v1;
						t->v2    = v2;
						t->bbmin = bbmin;
						t->bbmax = bbmax;
					}
					passive_table_.insert( hv, t );
				}
			}
		}
	}

	bool apply( real_type& dist )
	{
		dist = math< Traits >::real_max();
		passive_table_.apply( active_table_, InternalCallback( dist ) );
		return dist < math< Traits >::real_max();
	}

private:
	IndirectSpatialHash< Traits, SphereHashNode >  active_table_;
	IndirectSpatialHash< Traits ,TriangleHashNode > passive_table_;

};

////////////////////////////////////////////////////////////////
// ray - triangle
template < class Traits, class RayLoad, class TriangleLoad >
class RayTriangleSpatialHash {
private:
	typedef typename Traits::vector_traits  vector_traits;
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::vector_type    vector_type;

	struct RayHashNode {
		vector_type     s0;
		vector_type     s1;
		vector_type     bbmin;
		vector_type     bbmax;
		real_type       distance;
		RayLoad         load;
	};
	struct TriangleHashNode {
		vector_type     v0;
		vector_type     v1;
		vector_type     v2;
		vector_type     bbmin;
		vector_type     bbmax;
		TriangleLoad    load;
	};

	template < class CallBack >
	struct InternalCallback {
	public:
		InternalCallback( const CallBack& c ) : c_( c ) {}
		bool operator()(
			const RayHashNode* p, const TriangleHashNode* q ) const
		{
			vector_type uvt;
			if( !collide( p, q, uvt ) ) { return false; }
			c_( p->load, q->load, uvt );
			return false;
		}

	private:
		bool collide(
			const RayHashNode* pp,
			const TriangleHashNode* qq,
			vector_type& uvt ) const
		{
			if( !math< Traits >::test_aabb_aabb(
					pp->bbmin, pp->bbmax, qq->bbmin, qq->bbmax ) ) {
				return false;
			}

			const vector_type& s0 = pp->s0;
			const vector_type& s1 = pp->s1;

			const vector_type& v0 = qq->v0;
			const vector_type& v1 = qq->v1;
			const vector_type& v2 = qq->v2;

			return math< Traits >::test_segment_triangle(
				s0, s1, v0, v1, v2, uvt );
		}

		const CallBack& c_;
	};

public:
	RayTriangleSpatialHash( real_type gridsize, int tablesize )
		: active_table_( gridsize, tablesize ),
		  passive_table_( gridsize, tablesize )
	{
	}
	~RayTriangleSpatialHash()
	{
	}

	void clear( real_type gridsize )
	{
		active_table_.clear( gridsize );
		passive_table_.clear( gridsize );
	}

	void add_ray( const vector_type s0, const vector_type& s1,
				  const RayLoad& load )
	{
		RayHashNode* t = active_table_.alloc_node();
		t->s0 = s0;
		t->s1 = s1;
		t->distance = math< Traits >::real_max();
		t->load = load;
		math< Traits >::get_segment_bb( s0, s1, t->bbmin, t->bbmax );

		voxel_traverser< real_type, vector_type > vt(
			s0, s1, active_table_.gridsize() );
		int x, y, z;
		while( vt( x, y, z ) ) {
			size_t hv = active_table_.hash_value( x, y, z );
			active_table_.insert( hv, t );
		}
	}

	void add_triangle(
		const vector_type& v0,
		const vector_type& v1,
		const vector_type& v2,
		const vector_type& n,
		const TriangleLoad& load )
	{
		real_type gridsize = passive_table_.gridsize();

		vector_type bbmin, bbmax;
		math< Traits >::get_triangle_bb( v0, v1, v2, bbmin, bbmax );

		int x0 = passive_table_.coord( bbmin.x );
		int x1 = passive_table_.coord( bbmax.x );
		int y0 = passive_table_.coord( bbmin.y );
		int y1 = passive_table_.coord( bbmax.y );
		int z0 = passive_table_.coord( bbmin.z );
		int z1 = passive_table_.coord( bbmax.z );

#if 0
		vector_type n = math< Traits >::normalize(
			math< Traits >::cross( v1 - v0, v2 - v0 ) );
#endif
		real_type   d = math< Traits >::dot( n, v0 );

		real_type e = gridsize * real_type( 0.5 );
		real_type r = square( e ) * real_type( 3.0 ); // e*e + e*e + e*e

		TriangleHashNode* t = NULL;
                
		for( int z = z0 ; z <= z1 ; z++ ) {
			for( int y = y0 ; y <= y1 ; y++ ) {
				for( int x = x0 ; x <= x1; x++ ) {
					size_t hv = passive_table_.hash_value( x, y, z );
					if( !active_table_.entry( hv ) ) {
						// optimize: active node‚ª‚È‚¢‚Æ‚±‚ë‚Å‚ÍÈ—ª
						continue;
					}

					// ƒZƒ‹‚Ì’†‰›
					vector_type c = vector_traits::make_vector(
						( x + real_type( 0.5 ) ) * gridsize,
						( y + real_type( 0.5 ) ) * gridsize,
						( z + real_type( 0.5 ) ) * gridsize );

					// plane - sphere(ƒZƒ‹•‚Ì‘ÎŠpü‚ð’¼Œa‚Æ‚·‚é)‚ÅƒJƒŠƒ“ƒO
					real_type dist = math< Traits >::dot( c, n ) - d;
					if( r < square( dist ) ) { continue; }

					if( !t ) {
						t = passive_table_.alloc_node();
						t->v0    = v0;
						t->v1    = v1;
						t->v2    = v2;
						t->bbmin = bbmin;
						t->bbmax = bbmax;
						t->load  = load;
					}
					passive_table_.insert( hv, t );
				}
			}
		}
	}

	template < class CallBack > 
	void apply( const CallBack& c )
	{
		passive_table_.apply(
			active_table_, InternalCallback< CallBack >( c ) );
	}

private:
	IndirectSpatialHash< Traits, RayHashNode >  active_table_;
	IndirectSpatialHash< Traits ,TriangleHashNode > passive_table_;

};

////////////////////////////////////////////////////////////////
// tetrahedron
template < class Traits >
class TetrahedronSpatialHash {
private:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::index_type     index_type;
	typedef typename Traits::vector_type    vector_type;
	typedef TetrahedralMesh< Traits >       mesh_type;
	typedef typename mesh_type::tetrahedron_type            tetrahedron_type;
	typedef typename mesh_type::tetrahedra_type             tetrahedra_type;
	typedef typename mesh_type::cloud_type::points_type     points_type;

	struct TetrahedronHashNode {
		mesh_type*      mesh;
		index_type      index;
		vector_type     bbmin;
		vector_type     bbmax;
		real_type       invA[9];
		vector_type     v0;
	};

public:
	TetrahedronSpatialHash(
		real_type       gridsize,
		int             tablesize )
		: table_( gridsize, tablesize )
	{
	}
	~TetrahedronSpatialHash()
	{
	}

	void clear( real_type gridsize )
	{
		table_.clear( gridsize );
	}

	void add_mesh( mesh_type* p )
	{
		points_type& pv = p->get_points();

		tetrahedra_type& tetrahedra = p->get_tetrahedra();

		int ii = 0;
		for( typename tetrahedra_type::const_iterator i =
				 tetrahedra.begin() ;
			 i != tetrahedra.end() ;
			 ++i, ++ii ) {
			const tetrahedron_type& tet = *i;
			const vector_type& v0 = pv[tet.i0].new_position;
			const vector_type& v1 = pv[tet.i1].new_position;
			const vector_type& v2 = pv[tet.i2].new_position;
			const vector_type& v3 = pv[tet.i3].new_position;

			vector_type bbmin;
			vector_type bbmax;
			math< Traits >::get_tetrahedron_bb( v0, v1, v2, v3, bbmin, bbmax );

			TetrahedronHashNode* t = NULL;
                        
			int x0 = table_.coord( bbmin.x );
			int x1 = table_.coord( bbmax.x );
			int y0 = table_.coord( bbmin.y );
			int y1 = table_.coord( bbmax.y );
			int z0 = table_.coord( bbmin.z );
			int z1 = table_.coord( bbmax.z );

			for( int z = z0 ; z <= z1 ; z++ ) {
				for( int y = y0 ; y <= y1 ; y++ ) {
					for( int x = x0 ; x <= x1; x++ ) {
						size_t hv = table_.hash_value( x, y, z );
						if( !t ) {
							t = table_.alloc_node();
							t->mesh = p;
							t->index = ii;
							t->bbmin = bbmin;
							t->bbmax = bbmax;
							t->v0 = v0;

							real_type A[9];
							A[0] = v1.x - v0.x;
							A[1] = v2.x - v0.x;
							A[2] = v3.x - v0.x;
							A[3] = v1.y - v0.y;
							A[4] = v2.y - v0.y;
							A[5] = v3.y - v0.y;
							A[6] = v1.z - v0.z;
							A[7] = v2.z - v0.z;
							A[8] = v3.z - v0.z;

							math< Traits >::inverse_matrix( t->invA, A );
						}
                                                
						table_.insert( hv, t );
					}
				}
			}
		}
	}

	bool hit(
		const vector_type& q,
		mesh_type*& mesh,
		int& tetra_index,
		vector_type& bcc )
	{
		size_t hv = table_.hash( q );

		for( typename IndirectSpatialHash<
				 Traits, TetrahedronHashNode >::table_entry_type p =
				 table_.entry( hv ) ;
			 p ;
			 p = table_.next( p ) ) {

			TetrahedronHashNode* pp = table_.unwrap( p );

			if( !math< Traits >::test_aabb_point( pp->bbmin, pp->bbmax, q ) ) {
				continue;
			}

			vector_type vs = q - pp->v0;
			vector_type vd;
			math< Traits >::transform_vector( vd, pp->invA, vs );
			if( 0 <= vd.x &&
				0 <= vd.y &&
				0 <= vd.z &&
				( vd.x + vd.y + vd.z ) <= 1 ) {
				mesh = pp->mesh;
				tetra_index = pp->index;
				bcc = vd;
				return true;
			}
		}
		return false;
	}

private:
	IndirectSpatialHash< Traits, TetrahedronHashNode > table_;

};

/*===========================================================================*
 *
 * for volume - volume
 * 
 *
 * 
 *
 *==========================================================================*/

////////////////////////////////////////////////////////////////
// point - tetrahedron
template < class Traits >
class PointTetrahedronSpatialHash {
private:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::index_type     index_type;
	typedef typename Traits::vector_type    vector_type;
	typedef TetrahedralMesh< Traits >       mesh_type;
	typedef typename mesh_type::tetrahedron_type            tetrahedron_type;
	typedef typename mesh_type::tetrahedra_type             tetrahedra_type;
	typedef typename mesh_type::cloud_type::points_type     points_type;

	struct PointHashNode {
		mesh_type*      mesh;
		index_type      index;
		vector_type     position;
	};

	struct TetrahedronHashNode {
		mesh_type*      mesh;
		index_type      index;
		vector_type     bbmin;
		vector_type     bbmax;
		real_type       invA[9];
		vector_type     v0;
	};

	template < class T >
	struct InternalCallback {
	public:
		InternalCallback( const T& real_callback ) : c_( real_callback ) {}
		bool operator()(
			const PointHashNode* p, const TetrahedronHashNode* q ) const
		{
			if( p->mesh == q->mesh ) { return false; }
			if( !collide( p, q ) ) { return false; }

			c_( p->mesh, p->index, q->mesh, q->index );
			return true;
		}

	private:
		bool collide(
			const PointHashNode* p,
			const TetrahedronHashNode* q ) const
		{
			if( !math< Traits >::test_aabb_point(
					q->bbmin, q->bbmax, p->position ) ) {
				return false;
			}

			vector_type vs = p->position - q->v0;
			vector_type vd;
			math< Traits >::transform_vector( vd, q->invA, vs );
			return
				0 <= vd.x &&
				0 <= vd.y &&
				0 <= vd.z &&
				( vd.x + vd.y + vd.z ) <= 1;
		}
        
		const T& c_;
	};

public:
	PointTetrahedronSpatialHash( real_type gridsize, int tablesize )
		: active_table_( gridsize, tablesize ),
		  passive_table_( gridsize, tablesize )
	{
	}
	~PointTetrahedronSpatialHash()
	{
	}
        
	void clear( real_type gridsize )
	{
		active_table_.clear( gridsize );
		passive_table_.clear( gridsize );
	}

	void add_active_mesh( mesh_type* p )
	{
		points_type& points = p->get_points();

		int ii = 0;
		for( typename points_type::const_iterator i = points.begin() ;
			 i != points.end() ;
			 ++i, ++ii ) {
			const vector_type& q = (*i).new_position;

			size_t hv = active_table_.hash( q );

			PointHashNode n;
			n.mesh = p;
			n.index = ii;
			n.position = q;

			active_table_.insert( hv, n );
		}
	}

	void add_passive_mesh( mesh_type* mesh )
	{
		points_type&            points     = mesh->get_points();
		tetrahedra_type&        tetrahedra = mesh->get_tetrahedra();

		int ii = 0;
		for( typename tetrahedra_type::const_iterator i = tetrahedra.begin() ;
			 i != tetrahedra.end() ;
			 ++i, ++ii ) {
			const tetrahedron_type& tet = *i;
			const vector_type& v0 = points[tet.i0].new_position;
			const vector_type& v1 = points[tet.i1].new_position;
			const vector_type& v2 = points[tet.i2].new_position;
			const vector_type& v3 = points[tet.i3].new_position;

			vector_type bbmin;
			vector_type bbmax;
			math< Traits >::get_tetrahedron_bb( v0, v1, v2, v3, bbmin, bbmax );

			TetrahedronHashNode* t = NULL;
                        
			int x0 = passive_table_.coord( bbmin.x );
			int x1 = passive_table_.coord( bbmax.x );
			int y0 = passive_table_.coord( bbmin.y );
			int y1 = passive_table_.coord( bbmax.y );
			int z0 = passive_table_.coord( bbmin.z );
			int z1 = passive_table_.coord( bbmax.z );

			for( int z = z0 ; z <= z1 ; z++ ) {
				for( int y = y0 ; y <= y1 ; y++ ) {
					for( int x = x0 ; x <= x1; x++ ) {
						size_t hv = passive_table_.hash_value( x, y, z );

						if( !active_table_.entry( hv ) ) {
							// optimize: active node‚ª‚È‚¢‚Æ‚±‚ë‚Å‚ÍÈ—ª
							continue;
						}
                                                
						if( !t ) {
							t = passive_table_.alloc_node();
							t->mesh = mesh;
							t->index = ii;
							t->bbmin = bbmin;
							t->bbmax = bbmax;
							t->v0 = v0;

							real_type A[9];
							A[0] = v1.x - v0.x;
							A[1] = v2.x - v0.x;
							A[2] = v3.x - v0.x;
							A[3] = v1.y - v0.y;
							A[4] = v2.y - v0.y;
							A[5] = v3.y - v0.y;
							A[6] = v1.z - v0.z;
							A[7] = v2.z - v0.z;
							A[8] = v3.z - v0.z;

							math< Traits >::inverse_matrix( t->invA, A );
						}

						passive_table_.insert( hv, t );
					}
				}
			}
		}
	}

	template < class Callback >
	void apply( const Callback& c )
	{
		passive_table_.apply(
			active_table_, InternalCallback< Callback >( c ) );
	}

private:
	DirectSpatialHash< Traits, PointHashNode >              active_table_;
	IndirectSpatialHash< Traits ,TetrahedronHashNode >      passive_table_;

};

////////////////////////////////////////////////////////////////
// edge - face
template < class Traits >
class EdgeFaceSpatialHash {
private:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::index_type     index_type;
	typedef typename Traits::vector_type    vector_type;
	typedef Point< Traits >                 point_type;
	typedef Face< Traits >                  face_type;
	typedef Edge< Traits >                  edge_type;
	typedef TetrahedralMesh< Traits >       mesh_type;
	typedef typename mesh_type::tetrahedron_type		tetrahedron_type;
	typedef typename mesh_type::tetrahedra_type         tetrahedra_type;
	typedef typename mesh_type::cloud_type::points_type points_type;

	struct EdgeHashNode {
		mesh_type*      mesh;
		index_type      index;
		vector_type     bbmin;
		vector_type     bbmax;
	};
	struct FaceHashNode {
		mesh_type*      mesh;
		index_type      index;
		vector_type     bbmin;
		vector_type     bbmax;
	};

	template < class T >
	struct InternalCallback {
	public:
		InternalCallback( const T& real_callback ) : c_( real_callback ) {}
		bool operator()( const EdgeHashNode* p, const FaceHashNode* q ) const
		{
			if( p->mesh == q->mesh ) { return false; }

			vector_type uvt;
			if( !collide( p, q, uvt ) ) { return false; }

			c_( p->mesh, p->index, q->mesh, q->index, uvt );
			return true;
		}

	private:
		bool collide(
			const EdgeHashNode* pp,
			const FaceHashNode* qq,
			vector_type& uvt ) const
		{
			mesh_type* p = pp->mesh;
			mesh_type* q = qq->mesh;

			if( !math< Traits >::test_aabb_aabb(
					pp->bbmin, pp->bbmax, qq->bbmin, qq->bbmax ) ) {
				return false;
			}

			const points_type& pv = p->get_points();
			const points_type& qv = q->get_points();
                
			const edge_type& e = p->get_edges()[pp->index].indices;
			index_type ei0 = e.i0;
			index_type ei1 = e.i1;
			if( pv[ei0].collided ) { std::swap( ei0, ei1 ); }
			const vector_type& r0 = pv[ei0].new_position;
			const vector_type& r1 = pv[ei1].new_position;

			const face_type& t = q->get_faces()[qq->index];
			const vector_type& v0 = qv[t.i0].new_position;
			const vector_type& v1 = qv[t.i1].new_position;
			const vector_type& v2 = qv[t.i2].new_position;

			return math< Traits >::test_segment_triangle(
				r0, r1, v0, v1,v2, uvt );
		}
        
        
		const T& c_;
	};

public:
	EdgeFaceSpatialHash(
		real_type      gridsize,
		int             tablesize )
		: active_table_( gridsize, tablesize ),
		  passive_table_( gridsize, tablesize )
	{
	}
	~EdgeFaceSpatialHash()
	{
	}

	void clear( real_type gridsize )
	{
		active_table_.clear( gridsize ); 
		passive_table_.clear( gridsize ); 
	}

	void add_edge( mesh_type* p, index_type i )
	{
		const edge_type& e = p->get_edges()[i].indices;
		const point_type& v0 = p->get_points()[e.i0];
		const point_type& v1 = p->get_points()[e.i1];

		EdgeHashNode* t = active_table_.alloc_node();
		t->mesh = p;
		t->index = i;
		math< Traits >::get_segment_bb(
			v0.new_position, v1.new_position, t->bbmin, t->bbmax );

		voxel_traverser< real_type, vector_type > vt(
			v0.new_position, v1.new_position, active_table_.gridsize() );

		int x, y, z;
		while( vt( x, y, z ) ) {
			active_table_.insert( passive_table_.hash_value( x, y, z ), t );
		}
	}

	void add_face( mesh_type* p, index_type i )
	{
		real_type gridsize = passive_table_.gridsize();

		face_type& f = p->get_faces()[i];
		points_type& points = p->get_points();
		const vector_type& v0 = points[f.i0].new_position;
		const vector_type& v1 = points[f.i1].new_position;
		const vector_type& v2 = points[f.i2].new_position;

		vector_type bbmin, bbmax;
		math< Traits >::get_triangle_bb( v0, v1, v2, bbmin, bbmax );

		int x0 = passive_table_.coord( bbmin.x );
		int x1 = passive_table_.coord( bbmax.x );
		int y0 = passive_table_.coord( bbmin.y );
		int y1 = passive_table_.coord( bbmax.y );
		int z0 = passive_table_.coord( bbmin.z );
		int z1 = passive_table_.coord( bbmax.z );

		vector_type n = math< Traits >::normalize(
			math< Traits >::cross( v1 - v0, v2 - v0 ) );
		real_type   d = math< Traits >::dot( n, v0 );

		real_type e = gridsize * real_type( 0.5 );
		real_type r = square( e ) * real_type( 3.0 ); // e*e + e*e + e*e

		FaceHashNode* t = NULL;
                
		for( int z = z0 ; z <= z1 ; z++ ) {
			for( int y = y0 ; y <= y1 ; y++ ) {
				for( int x = x0 ; x <= x1; x++ ) {
					size_t hv = passive_table_.hash_value( x, y, z );
					if( !active_table_.entry( hv ) ) {
						// optimize: active node‚ª‚È‚¢‚Æ‚±‚ë‚Å‚ÍÈ—ª
						continue;
					}

					// ƒZƒ‹‚Ì’†‰›
					vector_type c;
					c.x = ( x + real_type( 0.5 ) ) * gridsize;
					c.y = ( y + real_type( 0.5 ) ) * gridsize;
					c.z = ( z + real_type( 0.5 ) ) * gridsize;

					// plane - sphere(ƒZƒ‹•‚Ì‘ÎŠpü‚ð’¼Œa‚Æ‚·‚é)‚ÅƒJƒŠƒ“ƒO
					real_type dist = math< Traits >::dot( c, n ) - d;
					if( r < square( dist ) ) { continue; }

					if( !t ) {
						t = passive_table_.alloc_node();
						t->mesh = p;
						t->index = i;
						t->bbmin = bbmin;
						t->bbmax = bbmax;
					}

					passive_table_.insert( hv, t );
				}
			}
		}
	}

	template < class Callback >
	void apply( const Callback& c )
	{
		passive_table_.apply(
			active_table_, InternalCallback< Callback >( c ) );
	}

private:
	IndirectSpatialHash< Traits, EdgeHashNode >     active_table_;
	IndirectSpatialHash< Traits ,FaceHashNode >     passive_table_;
        

};

////////////////////////////////////////////////////////////////
// penetration - face
template < class Traits >
class PenetrationFaceSpatialHash {
private:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::index_type     index_type;
	typedef typename Traits::vector_type    vector_type;
	typedef Point< Traits >                 point_type;
	typedef Face< Traits >                  face_type;
	typedef Edge< Traits >                  edge_type;
	typedef TetrahedralMesh< Traits >       mesh_type;
	typedef typename mesh_type::tetrahedron_type            tetrahedron_type;
	typedef typename mesh_type::tetrahedra_type             tetrahedra_type;
	typedef typename mesh_type::cloud_type::points_type     points_type;

	struct PenetrationHashNode {
		mesh_type*      mesh;
		index_type      index;
	};
	struct FaceHashNode {
		mesh_type*      mesh;
		index_type      index;
	};

	template < class T >
	struct InternalCallback {
	public:
		InternalCallback( const T& real_callback ) : c_( real_callback ) {}
		bool operator()(
			const PenetrationHashNode* p, const FaceHashNode* q ) const
		{
			if( p->mesh == q->mesh ) { return false; }

			vector_type uvt;
			if( !collide( p, q, uvt ) ) { return false; }

			c_( p->mesh, p->index, q->mesh, q->index, uvt );
			return true;
		}

	private:
		bool collide(
			const PenetrationHashNode* pp,
			const FaceHashNode* qq,
			vector_type& uvt ) const
		{
			mesh_type* p = pp->mesh;
			mesh_type* q = qq->mesh;
			const points_type& pv = p->get_points();
			const points_type& qv = q->get_points();
                
			const point_type& rv = pv[pp->index];
			const vector_type& r0 = rv.new_position;
			vector_type r1 = r0 + rv.penetration_vector;

			const face_type& t = q->get_faces()[qq->index];
			const vector_type& v0 = qv[t.i0].new_position;
			const vector_type& v2 = qv[t.i1].new_position; // — •\”½“]
			const vector_type& v1 = qv[t.i2].new_position; // — •\”½“]

			return math< Traits >::test_ray_triangle( r0, r1, v0, v1,v2, uvt );
		}
        
        
		const T& c_;
	};

public:
	PenetrationFaceSpatialHash( real_type gridsize, int tablesize )
		: active_table_( gridsize, tablesize ),
		  passive_table_( gridsize, tablesize )
	{
	}
	~PenetrationFaceSpatialHash()
	{
	}

	void clear( real_type gridsize )
	{
		active_table_.clear( gridsize );
		passive_table_.clear( gridsize );
	}

	void add_penetration( mesh_type* p, index_type i )
	{
		PenetrationHashNode* t = active_table_.alloc_node();
		t->mesh = p;
		t->index = i;

		const point_type& v = p->get_points()[i];

		voxel_traverser< real_type, vector_type > vt(
			v.new_position,
			v.new_position + v.penetration_vector,
			active_table_.gridsize() );

		int x, y, z;
		while( vt( x, y, z ) ) {
			size_t hv = active_table_.hash_value( x, y, z );
			active_table_.insert( hv, t );
		}
	}

	void add_face( mesh_type* p, index_type i )
	{
		const face_type& f = p->get_faces()[i];
		const points_type& points = p->get_points();
		const vector_type& v0 = points[f.i0].new_position;
		const vector_type& v1 = points[f.i1].new_position;
		const vector_type& v2 = points[f.i2].new_position;

		vector_type bbmin, bbmax;
		math< Traits >::get_triangle_bb( v0, v1, v2, bbmin, bbmax );

		int x0 = passive_table_.coord( bbmin.x );
		int x1 = passive_table_.coord( bbmax.x );
		int y0 = passive_table_.coord( bbmin.y );
		int y1 = passive_table_.coord( bbmax.y );
		int z0 = passive_table_.coord( bbmin.z );
		int z1 = passive_table_.coord( bbmax.z );

		FaceHashNode* t = NULL;

		for( int z = z0 ; z <= z1 ; z++ ) {
			for( int y = y0 ; y <= y1 ; y++ ) {
				for( int x = x0 ; x <= x1; x++ ) {
					size_t hv = passive_table_.hash_value( x, y, z );
					if( !active_table_.entry( hv ) ) {
						// optimize: active node‚ª‚È‚¢‚Æ‚±‚ë‚Å‚ÍÈ—ª
						continue; 
					}

					if( !t ) {
						t = passive_table_.alloc_node();
						t->mesh = p;
						t->index = i;
					}
					passive_table_.insert( hv, t );
				}
			}
		}
	}

	template < class Callback >
	void apply( const  Callback& c )
	{
		passive_table_.apply(
			active_table_, InternalCallback< Callback >( c ) );
	}

private:
	IndirectSpatialHash< Traits, PenetrationHashNode >      active_table_;
	IndirectSpatialHash< Traits ,FaceHashNode >             passive_table_;

};

/*===========================================================================*
 *
 * for volume - cloth
 * 
 *
 * 
 *
 *==========================================================================*/

////////////////////////////////////////////////////////////////
// point - tetrahedron
template < class Traits >
class ClothPointTetrahedronSpatialHash {
private:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::index_type     index_type;
	typedef typename Traits::vector_type    vector_type;
	typedef Point< Traits >                 point_type;
	typedef Face< Traits >                  face_type;
	typedef Edge< Traits >                  edge_type;
	typedef TetrahedralMesh< Traits >       mesh_type;
	typedef Cloth< Traits >                 cloth_type;
	typedef typename mesh_type::tetrahedron_type            tetrahedron_type;
	typedef typename mesh_type::tetrahedra_type             tetrahedra_type;
	typedef typename mesh_type::cloud_type::points_type     points_type;

	struct PointHashNode {
		cloth_type*     cloth;
		index_type      index;
		vector_type     position;
		int             offset; // -1 == — , ( 0 == –{‘Ì ), 1 == •\.
	};
	struct TetrahedronHashNode {
		mesh_type*      mesh;
		index_type      index;
		vector_type     bbmin;
		vector_type     bbmax;
		real_type       invA[9];
		vector_type     v0;
	};

	template < class T >
	struct InternalCallback {
	public:
		InternalCallback( const T& real_callback ) : c_( real_callback ) {}
		bool operator()(
			const PointHashNode* p, const TetrahedronHashNode* q ) const
		{
			vector_type bcc;
			if( !collide( p, q, bcc ) ) { return false; }

			c_( p->cloth, p->index, p->offset, q->mesh, q->index, bcc );
			return true;
		}

	private:
		bool collide(
			const PointHashNode* p,
			const TetrahedronHashNode* q,
			vector_type& bcc ) const
		{
			if( !math< Traits >::test_aabb_point(
					q->bbmin, q->bbmax, p->position ) ) {
				return false;
			}

			vector_type v = p->position - q->v0;
			math< Traits >::transform_vector( bcc, q->invA, v );
			return
				0 <= bcc.x &&
				0 <= bcc.y &&
				0 <= bcc.z &&
				( bcc.x + bcc.y + bcc.z ) <= 1;
		}
        
		const T& c_;
	};

public:
	ClothPointTetrahedronSpatialHash( real_type gridsize, int tablesize )
		: active_table_( gridsize, tablesize ),
		  passive_table_( gridsize, tablesize )
	{
	}
	~ClothPointTetrahedronSpatialHash()
	{
	}

	void clear( real_type gridsize )
	{
		active_table_.clear( gridsize );
		passive_table_.clear( gridsize );
	}

	void add_cloth( cloth_type* cloth )
	{
		real_type thickness = cloth->get_thickness();
		points_type& points = cloth->get_cloud()->get_points();

		int ii = 0;
		for( typename points_type::const_iterator i = points.begin() ;
			 i != points.end() ;
			 ++i, ++ii ) {
			const vector_type& q = (*i).new_position;
                        
			PointHashNode n;
			n.cloth = cloth;
			n.index = ii;
                        
			n.position = q;
			n.offset = 0;
			active_table_.insert( active_table_.hash( n.position ), n );

			n.position = q + (*i).normal * thickness;
			n.offset = 1;
			active_table_.insert( active_table_.hash( n.position ), n );

			n.position = q - (*i).normal * thickness;
			n.offset = -1;
			active_table_.insert( active_table_.hash( n.position ), n );
		}
	}

	void add_mesh( mesh_type* p )
	{
		points_type& points = p->get_points();

		tetrahedra_type& tetrahedra = p->get_tetrahedra();

		int ii = 0;
		for( typename tetrahedra_type::const_iterator i = tetrahedra.begin() ;
			 i != tetrahedra.end() ;
			 ++i, ++ii ) {
			const tetrahedron_type& tet = *i;
			const vector_type& v0 = points[tet.i0].new_position;
			const vector_type& v1 = points[tet.i1].new_position;
			const vector_type& v2 = points[tet.i2].new_position;
			const vector_type& v3 = points[tet.i3].new_position;

			vector_type bbmin;
			vector_type bbmax;
			math< Traits >::get_tetrahedron_bb( v0, v1, v2, v3, bbmin, bbmax );

			TetrahedronHashNode* t = NULL;
                        
			int x0 = passive_table_.coord( bbmin.x );
			int x1 = passive_table_.coord( bbmax.x );
			int y0 = passive_table_.coord( bbmin.y );
			int y1 = passive_table_.coord( bbmax.y );
			int z0 = passive_table_.coord( bbmin.z );
			int z1 = passive_table_.coord( bbmax.z );

			for( int z = z0 ; z <= z1 ; z++ ) {
				for( int y = y0 ; y <= y1 ; y++ ) {
					for( int x = x0 ; x <= x1; x++ ) {
						size_t hv = passive_table_.hash_value( x, y, z );
						if( !active_table_.entry( hv ) ) {
							// optimize: active node‚ª‚È‚¢‚Æ‚±‚ë‚Å‚ÍÈ—ª
							continue;
						}
                                                
						if( !t ) {
							t = passive_table_.alloc_node();
							t->mesh = p;
							t->index = ii;
							t->bbmin = bbmin;
							t->bbmax = bbmax;
							t->v0 = v0;

							real_type A[9];
							A[0] = v1.x - v0.x;
							A[1] = v2.x - v0.x;
							A[2] = v3.x - v0.x;
							A[3] = v1.y - v0.y;
							A[4] = v2.y - v0.y;
							A[5] = v3.y - v0.y;
							A[6] = v1.z - v0.z;
							A[7] = v2.z - v0.z;
							A[8] = v3.z - v0.z;

							math< Traits >::inverse_matrix( t->invA, A );
						}
                                                
						passive_table_.insert( hv, t );
					}
				}
			}
		}
	}

	template < class Callback >
	void apply( const Callback& c )
	{
		passive_table_.apply(
			active_table_, InternalCallback< Callback >( c ) );
	}

private:
	DirectSpatialHash< Traits, PointHashNode >              active_table_;
	IndirectSpatialHash< Traits, TetrahedronHashNode >      passive_table_;

};

////////////////////////////////////////////////////////////////
// spike - face
template < class Traits >
class ClothSpikeFaceSpatialHash {
private:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::index_type     index_type;
	typedef typename Traits::vector_type    vector_type;
	typedef Point< Traits >                 point_type;
	typedef Face< Traits >                  face_type;
	typedef Edge< Traits >                  edge_type;
	typedef TetrahedralMesh< Traits >       mesh_type;
	typedef Cloth< Traits >                 cloth_type;
	typedef typename mesh_type::tetrahedron_type            tetrahedron_type;
	typedef typename mesh_type::tetrahedra_type             tetrahedra_type;
	typedef typename mesh_type::cloud_type::points_type     points_type;

	struct SpikeHashNode {
		cloth_type*     cloth;
		index_type      index;
		vector_type     v0;
		vector_type     v1;
		vector_type     bbmin;
		vector_type     bbmax;
	};
	struct FaceHashNode {
		mesh_type*      mesh;
		index_type      index;
		vector_type     bbmin;
		vector_type     bbmax;
	};

	template < class T >
	struct InternalCallback {
	public:
		InternalCallback( const T& real_callback ) : c_( real_callback ) {}
		bool operator()( const SpikeHashNode* p, const FaceHashNode* q ) const
		{
			vector_type uvt;
			if( !collide( p, q, uvt ) ) { return false; }

			c_( p->cloth, p->index, q->mesh, q->index, uvt );
			return true;
		}

	private:
		bool collide(
			const SpikeHashNode* pp,
			const FaceHashNode* qq,
			vector_type& uvt ) const
		{
			cloth_type* p = pp->cloth;
			mesh_type* q = qq->mesh;

			if( !math< Traits >::test_aabb_aabb(
					pp->bbmin, pp->bbmax, qq->bbmin, qq->bbmax ) ) {
				return false;
			}

			const points_type& qv = q->get_points();
                
			const vector_type& r0 = pp->v0;
			const vector_type& r1 = pp->v1;

			const face_type& t = q->get_faces()[qq->index];
			const vector_type& v0 = qv[t.i0].new_position;
			const vector_type& v1 = qv[t.i1].new_position;
			const vector_type& v2 = qv[t.i2].new_position;

			return math< Traits >::test_segment_triangle(
				r0, r1, v0, v1,v2, uvt );
		}
        
        
		const T& c_;
	};

public:
	ClothSpikeFaceSpatialHash(
		real_type      gridsize,
		int             tablesize )
		: active_table_( gridsize, tablesize ),
		  passive_table_( gridsize, tablesize )
	{
	}
	~ClothSpikeFaceSpatialHash()
	{
	}

	void clear( real_type gridsize )
	{
		active_table_.clear( gridsize ); 
		passive_table_.clear( gridsize ); 
	}

	void add_spike( cloth_type* p, index_type i )
	{
		real_type thickness = p->get_thickness();

		const points_type& points = p->get_cloud()->get_points();
		const point_type& q = points[i];

		SpikeHashNode* t = active_table_.alloc_node();
		t->cloth = p;
		t->index = i;
		t->v0 = q.new_position + q.normal * thickness;
		t->v1 = q.new_position - q.normal * thickness;
		math< Traits >::get_segment_bb( t->v0, t->v1, t->bbmin, t->bbmax );

		voxel_traverser< real_type, vector_type > vt(
			t->v0, t->v1, active_table_.gridsize() );

		int x, y, z;
		while( vt( x, y, z ) ) {
			active_table_.insert( passive_table_.hash_value( x, y, z ), t );
		}
	}

	void add_face( mesh_type* p, index_type i )
	{
		real_type gridsize = passive_table_.gridsize();

		face_type& f = p->get_faces()[i];
		points_type& points = p->get_points();
		const vector_type& v0 = points[f.i0].new_position;
		const vector_type& v1 = points[f.i1].new_position;
		const vector_type& v2 = points[f.i2].new_position;

		vector_type bbmin, bbmax;
		math< Traits >::get_triangle_bb( v0, v1, v2, bbmin, bbmax );

		int x0 = passive_table_.coord( bbmin.x );
		int x1 = passive_table_.coord( bbmax.x );
		int y0 = passive_table_.coord( bbmin.y );
		int y1 = passive_table_.coord( bbmax.y );
		int z0 = passive_table_.coord( bbmin.z );
		int z1 = passive_table_.coord( bbmax.z );

		vector_type n = math< Traits >::normalize(
			math< Traits >::cross( v1 - v0, v2 - v0 ) );
		real_type   d = math< Traits >::dot( n, v0 );

		real_type e = gridsize * real_type( 0.5 );
		real_type r = square( e ) * real_type( 3.0 ); // e*e + e*e + e*e

		FaceHashNode* t = NULL;
                
		for( int z = z0 ; z <= z1 ; z++ ) {
			for( int y = y0 ; y <= y1 ; y++ ) {
				for( int x = x0 ; x <= x1; x++ ) {
					size_t hv = passive_table_.hash_value( x, y, z );
					if( !active_table_.entry( hv ) ) {
						// optimize: active node‚ª‚È‚¢‚Æ‚±‚ë‚Å‚ÍÈ—ª
						continue;
					}

					// ƒZƒ‹‚Ì’†‰›
					vector_type c;
					c.x = ( x + real_type( 0.5 ) ) * gridsize;
					c.y = ( y + real_type( 0.5 ) ) * gridsize;
					c.z = ( z + real_type( 0.5 ) ) * gridsize;

					// plane - sphere(ƒZƒ‹•‚Ì‘ÎŠpü‚ð’¼Œa‚Æ‚·‚é)‚ÅƒJƒŠƒ“ƒO
					real_type dist = math< Traits >::dot( c, n ) - d;
					if( r < square( dist ) ) { continue; }

					if( !t ) {
						t = passive_table_.alloc_node();
						t->mesh = p;
						t->index = i;
						t->bbmin = bbmin;
						t->bbmax = bbmax;
					}

					passive_table_.insert( hv, t );
				}
			}
		}
	}

	template < class Callback >
	void apply( const Callback& c )
	{
		passive_table_.apply(
			active_table_, InternalCallback< Callback >( c ) );
	}

private:
	IndirectSpatialHash< Traits, SpikeHashNode >     active_table_;
	IndirectSpatialHash< Traits ,FaceHashNode >     passive_table_;

};

////////////////////////////////////////////////////////////////
// edge - face
template < class Traits >
class ClothEdgeFaceSpatialHash {
private:
	typedef typename Traits::real_type      real_type;
	typedef typename Traits::index_type     index_type;
	typedef typename Traits::vector_type    vector_type;
	typedef Point< Traits >                 point_type;
	typedef Face< Traits >                  face_type;
	typedef Edge< Traits >                  edge_type;
	typedef TetrahedralMesh< Traits >       mesh_type;
	typedef Cloth< Traits >                 cloth_type;
	typedef typename mesh_type::tetrahedron_type            tetrahedron_type;
	typedef typename mesh_type::tetrahedra_type             tetrahedra_type;
	typedef typename mesh_type::cloud_type::points_type     points_type;

	struct EdgeHashNode {
		cloth_type*     cloth;
		index_type      index;
		vector_type     v0;
		vector_type     v1;
		vector_type     bbmin;
		vector_type     bbmax;
	};
	struct FaceHashNode {
		mesh_type*      mesh;
		index_type      index;
		vector_type     bbmin;
		vector_type     bbmax;
	};

	template < class T >
	struct InternalCallback {
	public:
		InternalCallback( const T& real_callback ) : c_( real_callback ) {}
		bool operator()( const EdgeHashNode* p, const FaceHashNode* q ) const
		{
			vector_type uvt;
			if( !collide( p, q, uvt ) ) { return false; }

			c_( p->cloth, p->index, q->mesh, q->index, uvt );
			return true;
		}

	private:
		bool collide(
			const EdgeHashNode* pp,
			const FaceHashNode* qq,
			vector_type& uvt ) const
		{
			cloth_type* p = pp->cloth;
			mesh_type* q = qq->mesh;

			if( !math< Traits >::test_aabb_aabb(
					pp->bbmin, pp->bbmax, qq->bbmin, qq->bbmax ) ) {
				return false;
			}

			const points_type& qv = q->get_points();
                
			const vector_type& r0 = pp->v0;
			const vector_type& r1 = pp->v1;

			const face_type& t = q->get_faces()[qq->index];
			const vector_type& v0 = qv[t.i0].new_position;
			const vector_type& v1 = qv[t.i1].new_position;
			const vector_type& v2 = qv[t.i2].new_position;

			return math< Traits >::test_segment_triangle(
				r0, r1, v0, v1,v2, uvt );
		}
        
        
		const T& c_;
	};

public:
	ClothEdgeFaceSpatialHash(
		real_type      gridsize,
		int             tablesize )
		: active_table_( gridsize, tablesize ),
		  passive_table_( gridsize, tablesize )
	{
	}
	~ClothEdgeFaceSpatialHash()
	{
	}

	void clear( real_type gridsize )
	{
		active_table_.clear( gridsize ); 
		passive_table_.clear( gridsize ); 
	}

	void add_edge(
		cloth_type* p, const points_type& points, int i, int i0, int i1 )
	{
		if( points[i0].collided ) { std::swap( i0, i1 ); }

		EdgeHashNode* t = active_table_.alloc_node();
		t->cloth = p;
		t->index = i;
		t->v0 = points[i0].new_position;
		t->v1 = points[i1].new_position;
		math< Traits >::get_segment_bb( t->v0, t->v1, t->bbmin, t->bbmax );

		voxel_traverser< real_type, vector_type > vt(
			t->v0, t->v1, active_table_.gridsize() );

		int x, y, z;
		while( vt( x, y, z ) ) {
			active_table_.insert( passive_table_.hash_value( x, y, z ), t );
		}
	}

	void add_face( mesh_type* p, index_type i )
	{
		real_type gridsize = passive_table_.gridsize();

		face_type& f = p->get_faces()[i];
		points_type& points = p->get_points();
		const vector_type& v0 = points[f.i0].new_position;
		const vector_type& v1 = points[f.i1].new_position;
		const vector_type& v2 = points[f.i2].new_position;

		vector_type bbmin, bbmax;
		math< Traits >::get_triangle_bb( v0, v1, v2, bbmin, bbmax );

		int x0 = passive_table_.coord( bbmin.x );
		int x1 = passive_table_.coord( bbmax.x );
		int y0 = passive_table_.coord( bbmin.y );
		int y1 = passive_table_.coord( bbmax.y );
		int z0 = passive_table_.coord( bbmin.z );
		int z1 = passive_table_.coord( bbmax.z );

		vector_type n = math< Traits >::normalize(
			math< Traits >::cross( v1 - v0, v2 - v0 ) );
		real_type   d = math< Traits >::dot( n, v0 );

		real_type e = gridsize * real_type( 0.5 );
		real_type r = square( e ) * real_type( 3.0 ); // e*e + e*e + e*e

		FaceHashNode* t = NULL;
                
		for( int z = z0 ; z <= z1 ; z++ ) {
			for( int y = y0 ; y <= y1 ; y++ ) {
				for( int x = x0 ; x <= x1; x++ ) {
					size_t hv = passive_table_.hash_value( x, y, z );
					if( !active_table_.entry( hv ) ) {
						// optimize: active node‚ª‚È‚¢‚Æ‚±‚ë‚Å‚ÍÈ—ª
						continue;
					}

					// ƒZƒ‹‚Ì’†‰›
					vector_type c;
					c.x = ( x + real_type( 0.5 ) ) * gridsize;
					c.y = ( y + real_type( 0.5 ) ) * gridsize;
					c.z = ( z + real_type( 0.5 ) ) * gridsize;

					// plane - sphere(ƒZƒ‹•‚Ì‘ÎŠpü‚ð’¼Œa‚Æ‚·‚é)‚ÅƒJƒŠƒ“ƒO
					real_type dist = math< Traits >::dot( c, n ) - d;
					if( r < square( dist ) ) { continue; }

					if( !t ) {
						t = passive_table_.alloc_node();
						t->mesh = p;
						t->index = i;
						t->bbmin = bbmin;
						t->bbmax = bbmax;
					}

					passive_table_.insert( hv, t );
				}
			}
		}
	}

	template < class Callback >
	void apply( const Callback& c )
	{
		passive_table_.apply(
			active_table_, InternalCallback< Callback >( c ) );
	}

private:
	IndirectSpatialHash< Traits, EdgeHashNode >     active_table_;
	IndirectSpatialHash< Traits ,FaceHashNode >     passive_table_;
        

};

} // namespace partix

#endif // PARTIX_SPATIAL_HASH_HPP
