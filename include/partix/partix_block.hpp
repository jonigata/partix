/*!
  @file		partix_block.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: partix_block.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_BLOCK_HPP
#define PARTIX_BLOCK_HPP

#include "partix_cloud.hpp"
#include "partix_collidable.hpp"

namespace partix {

// Block
template < class Traits >
class Block : public Collidable< Traits > {
public:
	typedef typename Traits::vector_traits			vector_traits;
	typedef typename Traits::real_type				real_type;
	typedef typename Traits::vector_type			vector_type;
	typedef typename Traits::index_type				index_type;
	typedef Point< Traits >							point_type;
	typedef std::vector< point_type >				points_type;
	typedef Cloud< Traits >							cloud_type;
	typedef std::vector< index_type >				indices_type;
	typedef Face< Traits >							face_type;
	typedef std::vector< face_type >				faces_type;
	typedef Body< Traits >							body_type;
	typedef Block< Traits >							block_type;
	typedef Collidable< Traits >					collidable_type;
	typedef std::vector< collidable_type* >			collidables_type;
	typedef typename Traits::block_load_type		load_type;

public:
	Block() { grid_size_ = 0; }
	~Block() {}

	// implements Collidable
	body_type*				get_body() { return body_; }
	cloud_type*				get_cloud() { return cloud_; }
	indices_type&			get_indices() { return indices_; }
	faces_type&				get_faces() { return faces_; }
	vector_type				get_center() { return center_; }
	vector_type				get_bbmin() { return bbmin_; }
	vector_type				get_bbmax() { return bbmax_; }
	void					clear_neighbors() { neighbors_.clear(); }
	void					add_neighbor( collidable_type* p )
	{
		neighbors_.push_back( p );
	}
	collidables_type&		get_neighbors() { return neighbors_; }
	bool					pick( const vector_type& s0,
								  const vector_type& s1,
								  real_type& dist )
	{
		return pick_internal( s0, s1, dist );
	}
		
	void					mark() { marked_ = true; }
	void					unmark() { marked_ = false; }
	bool					marked() { return marked_; }

	// original
	void set_body( body_type* b ) { body_ = b; }
	void set_cloud( cloud_type* c ) { cloud_ = c; }
	void set_center( const vector_type& v ) { center_ = v; }
	void set_bbmin( const vector_type& v ) { bbmin_ = v; }
	void set_bbmax( const vector_type& v ) { bbmax_ = v; }

	void add_face( index_type i0, index_type i1, index_type i2 )
	{
		face_type f; f.i0 = i0; f.i1 = i1; f.i2 = i2;
		faces_.push_back( f );
	}
	void setup()
	{
		indices_.clear();
		std::set< index_type > s;

		for( typename faces_type::const_iterator i =
				 faces_.begin() ;
			 i != faces_.end() ;
			 ++i ) {
			const face_type& f = *i;
			s.insert( f.i0 );
			s.insert( f.i1 );
			s.insert( f.i2 );
		}

		for( typename std::set< index_type>::const_iterator i =
				 s.begin() ;
			 i != s.end();
			 ++i ) {
			indices_.push_back( *i );
		}
	}

	load_type		load;

private:
	Block( const Block& ){}
	void operator=( const Block& ){}

private:
	bool pick_internal( const vector_type& s0,
						const vector_type& s1,
						real_type& dist )
	{
		if( math< Traits >::test_aabb_segment(
				bbmin_, bbmax_, s0, s1 ) ) {
			points_type& points = cloud_->get_points();

			SegmentTriangleSpatialHash< Traits > hash(
				get_recommended_grid_size(), 4999 );
			hash.add_segment( s0, s1 );

			for( typename faces_type::iterator i =
					 faces_.begin() ;
				 i != faces_.end() ;
				 ++i ) {
				const face_type& f = *i;
				hash.add_triangle(
					points[f.i0].new_position,
					points[f.i1].new_position,
					points[f.i2].new_position );
			}
			return hash.apply( dist );
		}
		return false;
	}

	real_type get_recommended_grid_size()
	{
		if( 0 < grid_size_ ) { return grid_size_; }

		points_type& points = cloud_->get_points();

		grid_size_ = 0;
		for( typename faces_type::const_iterator i =
				 faces_.begin() ;
			 i != faces_.end() ;
			 ++i ) {
			const face_type& f = *i;
			const point_type& v0 = points[f.i0];
			const point_type& v1 = points[f.i1];
			const point_type& v2 = points[f.i2];

			real_type l0 = vector_traits::length(
				v1.source_position - v0.source_position );
			real_type l1 = vector_traits::length(
				v2.source_position - v1.source_position );
			real_type l2 = vector_traits::length(
				v0.source_position - v2.source_position );

			grid_size_ += l0 + l1 + l2;
		}
		grid_size_ /= faces_.size() * 3.0f;

		return grid_size_;
	}


private:
	body_type*				body_;
	cloud_type*				cloud_;
	faces_type				faces_;
	indices_type			indices_;
	vector_type				center_;
	vector_type				bbmin_;
	vector_type				bbmax_;
	collidables_type		neighbors_;
	bool					marked_;
	real_type				grid_size_;

};

} // namespace partix

#endif // PARTIX_BLOCK_HPP
