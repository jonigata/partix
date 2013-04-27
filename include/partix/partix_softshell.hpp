/*!
  @file		partix_soft.hpp
  @brief	<概要>

  <説明>
  $Id: partix_softshell.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_SOFTSHELL_HPP
#define PARTIX_SOFTSHELL_HPP

#include "partix_forward.hpp"
#include "partix_id.hpp"
#include "partix_shell.hpp"
#include "partix_math.hpp"

namespace partix {

// soft shell
template < class Traits >
class SoftShellSnapShot : public BodySnapShot< Traits > {
public:
	typedef typename Traits::vector_type	vector_type;
	typedef typename Traits::matrix_type	matrix_type;

	BodySnapShot< Traits >* clone()
	{
		return new SoftShellSnapShot< Traits >( *this );
	}

	std::vector< Cloud< Traits > >	clouds;
	vector_type						initial_center;
	vector_type						global_force;
};

template < class Traits >
class SoftShell : public Shell< Traits > {
public:
	typedef typename Traits::vector_traits	vector_traits;
	typedef typename Traits::real_type		real_type;
	typedef typename Traits::vector_type	vector_type;
	typedef typename Traits::matrix_type	matrix_type;
	typedef Cloud< Traits >					cloud_type;
	typedef Block< Traits >					block_type;
	typedef std::vector< cloud_type* >		clouds_type;
	typedef std::vector< block_type* >		blocks_type;
	typedef typename cloud_type::point_type	 point_type;
	typedef typename cloud_type::points_type  points_type;
	typedef typename block_type::indices_type indices_type;
	typedef Collidable< Traits >			collidable_type;
	typedef std::vector< collidable_type* > collidables_type;
		
public:
	SoftShell()
	{
		touch_level_	= 2;
		current_center_ = initial_center_ =
			math< Traits >::vector_zero();
		restore_factor_ = 1.0f;
		stretch_factor_ = 0.0f;
		math< Traits >::make_identity( R_ );
		math< Traits >::make_identity( G_ );
	}
	~SoftShell() {}

	// implements Body
	int classid() { return BODY_ID_SOFTSHELL; }

	SoftShellSnapShot< Traits >* make_snapshot()
	{
		return make_snapshot_internal();
	}
	void apply_snapshot( const BodySnapShot< Traits >* ss )
	{
		apply_snapshot_internal( ss );
	}

	void regularize() { regularize_internal(); }
	void list_collision_units( collidables_type& s )
	{
		list_collision_units_internal( s );
	}

	void move( const vector_type& v ) { move_internal( v ); }
	void teleport( const vector_type& v ) { teleport_internal( v ); }

	void begin_frame() { begin_frame_internal(); }
	void update_velocity( real_type dt, real_type idt )
	{
		update_velocity_internal( dt, idt );
	}
	void apply_forces( real_type dt, real_type idt )
	{
		apply_forces_internal( dt, idt );
	}
	void match_shape()
	{
		match_shape_internal();
	}
	void restore_shape( real_type dt, real_type idt, int kmax )
	{
		restore_shape_internal( dt, idt, kmax );
	}
	void end_frame()
	{
		end_frame_internal();
	}

	void	  update_display_matrix() { update_display_matrix_internal(); }
	void	  update_boundingbox() { update_boundingbox_internal(); }
	vector_type get_initial_center() { return initial_center_; }
	vector_type get_current_center() { return current_center_; }

	// original
	void set_orientation( const matrix_type& m )
	{
		set_orientation_internal( m );
	}
	void rotate(
		real_type w, real_type x, real_type y, real_type z // quaternion
		)
	{
		rotate_internal( w, x, y, z, initial_center_ );
	} 
	void rotate(
		real_type w, real_type x, real_type y, real_type z, // quaternion
		const vector_type& pivot ) 
	{
		rotate_internal( w, x, y, z, pivot );
	}
	matrix_type get_deformed_matrix() { return deformed_matrix_; }
	matrix_type get_orientation_matrix() { return orientation_matrix_; }

	void set_restore_factor( real_type x ) { restore_factor_ = x; }
	void set_stretch_factor( real_type x ) { stretch_factor_ = x; }

private:
	SoftShell( const SoftShell& ){}
	void operator=( const SoftShell& ){}

private:
	SoftShellSnapShot< Traits >* make_snapshot_internal()
	{
		// 所有権はSnapShotに移動する
		SoftShellSnapShot< Traits >* ds =
			new SoftShellSnapShot< Traits >;

		for( typename clouds_type::iterator j =
				 this->clouds_.begin() ;
			 j != this->clouds_.end() ;
			 ++j ) {
			ds->clouds.push_back( **j );
		}
		ds->initial_center = initial_center_;
		ds->global_force   = this->get_global_force();

		return ds;
	}

	void apply_snapshot_internal( const BodySnapShot< Traits >* ss )
	{
		const SoftShellSnapShot< Traits >* ds =
			dynamic_cast< const SoftShellSnapShot< Traits >* >( ss );
		assert( ds );

		int jj = 0;
		for( typename clouds_type::iterator j =
				 this->clouds_.begin() ;
			 j != this->clouds_.end() ;
			 ++j ) {
			**j = ds->clouds[jj++];
		}
		initial_center_ = ds->initial_center;
		set_global_force( ds->global_force );
	}

	void list_collision_units_internal( collidables_type& s )
	{
		for( typename blocks_type::const_iterator i =
				 this->blocks_.begin() ;
			 i != this->blocks_.end() ;
			 ++i ) {
			block_type* b = *i;
			s.push_back( b );
		}
	}

	void begin_frame_internal()
	{
		if( touch_level_ == 2 ) {
			regularize_internal();
		}
	}

	void update_velocity_internal( real_type dt, real_type idt )
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return; }
			if( this->get_frozen() && !this->get_defrosting() ) { return; }
		}
		for( typename clouds_type::const_iterator i = this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* c = *i;
			c->update_velocity( dt, idt );
		}
	}

	void apply_forces_internal( real_type dt, real_type idt )
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return ; }
			if( this->get_frozen() && !this->get_defrosting() ) { return; }
		}

		const real_type drag_coefficient =
			real_type( 1.0 ) -
			pow(  real_type( 1.0 ) - this->get_drag_factor(), dt );
				
		for( typename clouds_type::const_iterator i = this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* c = *i;
			c->apply_forces(
				dt,
				idt,
				this->get_force(),
				this->get_global_force(),
				drag_coefficient,
				this->get_internal_dump_factor() );
		}
		set_force( math< Traits>::vector_zero() );
	}

	void update_frozen( real_type dt, real_type idt )
	{
		if( this->get_auto_freezing() ) {
			vector_type energy = math< Traits >::vector_zero();
			real_type total_mass = 0;

			for( typename clouds_type::const_iterator i =
					 this->clouds_.begin() ;
				 i != this->clouds_.end() ;
				 ++i ) {
				cloud_type* c = *i;

				for( typename points_type::iterator j =
						 c->get_points().begin() ;
					 j != c->get_points().end() ;
					 ++j ) {
					point_type& p = *j;
					energy += p.velocity * p.mass;
					total_mass += p.mass;
				}
			}

#if 0
			char buffer[256];
			sprintf(
				buffer, "energy: %f\n",
				vector_traits::length( energy ) / total_mass );
			OutputDebugStringA( buffer );
#endif
			if( vector_traits::length( energy ) / total_mass <=
				Traits::freeze_threshold_energy() ) {
				freezing_duration_ += dt;
				if( Traits::freeze_duration() <=
					freezing_duration_ ) {
					this->set_frozen( true );
					freezing_duration_ = 0;
				}
			} else {
				this->set_frozen( false );
			}
		}
		this->set_defrosting( false );
	}

	void match_shape_internal()
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return ; }
			if( this->get_frozen() ) { return; }
		}

		// 重心を計算
		calculate_center();

		// 「あるべき点」にひっぱられる
		real_type Apq[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		for( typename clouds_type::const_iterator i =
				 this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* c = *i;
								
			for( typename points_type::iterator j =
					 c->get_points().begin() ;
				 j != c->get_points().end() ;
				 ++j ) {
				point_type& point = *j;

				vector_type p =
					point.new_position - current_center_;
				const vector_type& q = point.ideal_offset;
								
				real_type px = vector_traits::x( p );
				real_type py = vector_traits::y( p );
				real_type pz = vector_traits::z( p );
				real_type qx = vector_traits::x( q );
				real_type qy = vector_traits::y( q );
				real_type qz = vector_traits::z( q );
								
				real_type m = point.mass;
						
				Apq[0] += m * px * qx;
				Apq[1] += m * px * qy;
				Apq[2] += m * px * qz;
				Apq[3] += m * py * qx;
				Apq[4] += m * py * qy;
				Apq[5] += m * py * qz;
				Apq[6] += m * pz * qx;
				Apq[7] += m * pz * qy;
				Apq[8] += m * pz * qz;
			}
		}

		real_type ApqT[9];
		math< Traits >::transpose_matrix( ApqT, Apq );

		real_type ApqT_Apq[9];
		math< Traits >::multiply_matrix( ApqT_Apq, ApqT, Apq );
				
		real_type sqrt_ApqT_Apq[9];
		math< Traits >::sqrt_matrix( sqrt_ApqT_Apq, ApqT_Apq );

		real_type inverse_sqrt_ApqT_Apq[9];
		math< Traits >::inverse_matrix(
			inverse_sqrt_ApqT_Apq, sqrt_ApqT_Apq );

		real_type R[9];
		math< Traits >::multiply_matrix(
			R, Apq, inverse_sqrt_ApqT_Apq );
		real_type detR = math< Traits >::determinant_matrix( R );
		if( 0.1f < std::abs( 1.0f - detR ) ) {
			// Rが壊れすぎ
#if 0
			char buffer[256];
			sprintf( buffer, "det0: %f\n", detR );
			OutputDebugStringA( buffer );
#endif
		} else {
			real_type A[9];
			math< Traits >::multiply_matrix( A, Apq, Aqq_ );
			real_type detA =
				math< Traits >::determinant_matrix( A );
			if( 0.5f < std::abs( 1.0f - detA ) ) {	
				// Aが壊れすぎ
#if 0
				char buffer[256];
				sprintf( buffer, "det1: %f\n", detA );
				OutputDebugStringA( buffer );
#endif
			} else {
				real_type cbrt =
					pow( std::abs( detA ), real_type(1.0/3.0) );
								
				real_type Adash[9];
				math< Traits >::multiply_matrix(
					Adash, A, real_type( 1.0 ) / cbrt );

				// detAがおかしいほど硬くなる
				real_type stretch_factor =
					stretch_factor_ - ( 1.0f - detA );
				if( stretch_factor < 0 ) {
					stretch_factor = 0;
				}

				// detAが小さいほどエネルギー喪失
				if( detA < 1.0f ) {
					real_type dump = 1.0f - detA;
					dump = 1.0f	 - dump * dump * 10.0f;
					if( dump < real_type( 0.7 ) ) {
						dump = real_type( 0.7 );
					}
					set_internal_dump_factor( dump ) ;

#if 0
					char buffer [256];
					sprintf( buffer, "dump: %.8f\n", dump );
					OutputDebugStringA( buffer );
#endif
				} else {
					set_internal_dump_factor(
						real_type( 1.0 ) ) ;
				}
										
				real_type G0[9];
				real_type G1[9];
				math< Traits >::multiply_matrix(
					G0, Adash, stretch_factor );
				math< Traits >::multiply_matrix(
					G1, R, 1.0f - stretch_factor );

				real_type G[9];
				math< Traits >::add_matrix( G, G0, G1 );
				real_type detG = 
					math< Traits >::determinant_matrix(G);
				if( 0.5f < detG ) { 
					// マトリックスが正当なので承認
					math< Traits >::copy_matrix( R_, R );
					math< Traits >::copy_matrix( G_, G );
				}
			}
		}

	}
		
	void restore_shape_internal( real_type dt, real_type idt, int kmax )
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return ; }
			if( this->get_frozen() ) { return; }
		}
				
		real_type rest =
			real_type( 1.0 ) -
			pow( real_type( 1.0 ) - restore_factor_,
				 real_type( 1.0 ) / ( kmax + real_type ( 1.0 ) ) );
				
		// 復元の適用
		for( typename clouds_type::const_iterator i =
				 this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* c = *i;

			for( typename points_type::iterator j =
					 c->get_points().begin() ;
				 j != c->get_points().end() ;
				 ++j ) {
				point_type& point = *j;

				const vector_type& q = point.ideal_offset;
								
				vector_type g;
				math< Traits >::transform_vector( g, G_, q );
								
				g += current_center_;

				vector_type velocity_dt =
					( g - point.new_position ) * rest;
				point.new_position += velocity_dt;
			}
		}

	}

	void update_boundingbox_internal( )
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return; }
			if( this->get_frozen() ) { return; }
		}

		bbmin_ = math< Traits >::vector_max();
		bbmax_ = math< Traits >::vector_min();
				
		for( typename blocks_type::const_iterator i =
				 this->blocks_.begin() ;
			 i != this->blocks_.end() ;
			 ++i ) {
			block_type* b = *i;
			cloud_type* c = b->get_cloud();

			vector_type block_bbmin = math< Traits >::vector_max();
			vector_type block_bbmax = math< Traits >::vector_min();
						
			const indices_type& indices = b->get_indices();
						
			for( typename indices_type::const_iterator j =
					 indices.begin() ;
				 j != indices.end() ;
				 ++j ) {
				typename cloud_type::point_type& p =
					c->get_points()[*j];
								
				// バウンディングボックスの更新
				math< Traits >::update_bb(
					bbmin_,
					bbmax_,
					p.new_position );
				math< Traits >::update_bb(
					block_bbmin,
					block_bbmax,
					p.new_position );
			}

			b->set_bbmin( block_bbmin );
			b->set_bbmax( block_bbmax );
		}
	}

	void update_display_matrix_internal()
	{
		// 表示用マトリックス
		vector_type ic = -initial_center_; // 重心からみた原点
		vector_type tic;
		math< Traits >::transform_vector( tic, R_, ic );
		Traits::make_matrix(
			orientation_matrix_, R_, tic + current_center_ );
		math< Traits >::transform_vector( tic, G_, ic );
		Traits::make_matrix( transform_, G_, tic + current_center_ );
		Traits::make_matrix(
			deformed_matrix_, G_, tic + current_center_ );
	}

	void end_frame_internal()
	{
		touch_level_ = 0;
	}

	void move_internal( const vector_type& v )
	{
		for( typename clouds_type::const_iterator i = this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* cloud = *i;
			cloud->move( v );
		}
		set_touch_level( 1 );
	}

	void teleport_internal( const vector_type& v )
	{
		for( typename clouds_type::const_iterator i = this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* cloud = *i;
			cloud->teleport( v );
		}
		set_touch_level( 1 );
	}

	void set_orientation_internal( const matrix_type& m )
	{
		for( typename clouds_type::const_iterator i = this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* c = *i;
			c->set_orientation( m );
		}
		set_touch_level( 1 );
	}

	void rotate_internal(
		real_type w, real_type x, real_type y, real_type z,
		const vector_type& pivot )
	{
		quaternion< real_type > q( w, x, y, z );
		quaternion< real_type > rq( w, -x, -y, -z );
		vector_type center = transform( pivot - initial_center_ );

		for( typename clouds_type::const_iterator i = this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* c = *i;
			c->rotate( q, rq, center );
		}
		set_touch_level( 1 );
	}

private:
	void set_touch_level( int n )
	{
		if( touch_level_ < n ) { touch_level_ = n; }
	}

	void regularize_internal()
	{
		vector_type v0 = math< Traits >::vector_zero();

		calculate_initial_center();

		// init
		for( typename clouds_type::const_iterator i =
				 this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type*		c = *i;

			for( typename points_type::iterator j =
					 c->get_points().begin() ;
				 j != c->get_points().end() ;
				 ++j ) {
				point_type& p = *j;
								
				p.ideal_offset =
					p.source_position - initial_center_;
			}
		}

		calculate_Aqq();
		calculate_center();
	}

	void calculate_Aqq()
	{
		// calculate Aqq
		real_type Aqq[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, };
		for( typename clouds_type::const_iterator i =
				 this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* c = *i;

			for( typename points_type::iterator j =
					 c->get_points().begin() ;
				 j != c->get_points().end() ;
				 ++j ) {
				point_type& point = *j;

				const vector_type& q = point.ideal_offset;
								
				real_type qx = vector_traits::x( q );
				real_type qy = vector_traits::y( q );
				real_type qz = vector_traits::z( q );
								
				real_type m = point.mass;
						
				Aqq[0] += m * qx * qx;
				Aqq[1] += m * qx * qy;
				Aqq[2] += m * qx * qz;
				Aqq[3] += m * qy * qx;
				Aqq[4] += m * qy * qy;
				Aqq[5] += m * qy * qz;
				Aqq[6] += m * qz * qx;
				Aqq[7] += m * qz * qy;
				Aqq[8] += m * qz * qz;
			}
		}

		math< Traits >::inverse_matrix( Aqq_, Aqq );
	}

	void calculate_initial_center()
	{
		vector_type total_center = math< Traits >::vector_zero();
		real_type total_mass = 0;

		for( typename clouds_type::const_iterator i =
				 this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* c = *i;

			const points_type& points = c->get_points();
			for( typename points_type::const_iterator j =
					 points.begin() ;
				 j != points.end() ;
				 ++j ) {
				const point_type& p = *j;
								
				total_center += p.source_position * p.mass;
				total_mass += p.mass;
			}
		}
		total_center *= real_type(1) / total_mass;
		initial_center_ = total_center;
	}

	void calculate_center()
	{
		vector_type total_center = math< Traits >::vector_zero();
		real_type total_mass = 0;

		for( typename clouds_type::const_iterator i =
				 this->clouds_.begin() ;
			 i != this->clouds_.end() ;
			 ++i ) {
			cloud_type* c = *i;

			const points_type& points = c->get_points();
			for( typename points_type::const_iterator j =
					 points.begin() ;
				 j != points.end() ;
				 ++j ) {
				const point_type& p = *j;
								
				total_center += p.new_position * p.mass;
				total_mass += p.mass;
			}
		}
		total_center *= real_type(1) / total_mass;
		current_center_ = total_center;

		for( typename blocks_type::const_iterator i =
				 this->blocks_.begin() ;
			 i != this->blocks_.end() ;
			 ++i ) {
			block_type*		b = *i;
			cloud_type*		c = b->get_cloud();

			vector_type		block_center = math< Traits >::vector_zero();
			real_type		block_mass = 0;

			const points_type& points = c->get_points();
			const indices_type& indices = b->get_indices();
			for( typename indices_type::const_iterator j =
					 indices.begin();
				 j != indices.end() ;
				 ++j ) {
				const point_type& p = points[*j];
								
				block_center += p.new_position * p.mass;
				block_mass += p.mass;
			}
			block_center *= real_type(1) / block_mass;
			b->set_center( block_center );
		}
	}


private:
	// touch_level_
	//	 1 ... 位置
	//	 2 ... 位置 + 質量
	int				touch_level_;
		
	vector_type		initial_center_;
	vector_type		current_center_;
	matrix_type		transform_;
	matrix_type		deformed_matrix_;
	matrix_type		orientation_matrix_;
	vector_type		bbmin_;
	vector_type		bbmax_;
	real_type		Aqq_[9];
	real_type		R_[9];
	real_type		G_[9];
	real_type		restore_factor_;
	real_type		stretch_factor_;
	real_type		freezing_duration_;

	template < class T > friend class World;
};

} // namespace partix
 
#endif // PARTIX_SOFTSHELL_HPP
