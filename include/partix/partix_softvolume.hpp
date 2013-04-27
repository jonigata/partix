/*!
  @file		partix_softvolume.hpp
  @brief	<�T�v>

  <����>
  $Id: partix_softvolume.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_SOFTVOLUME_HPP
#define PARTIX_SOFTVOLUME_HPP

#include "partix_forward.hpp"
#include "partix_volume.hpp"
#include "partix_collidable.hpp"
#include "partix_spatial_hash.hpp"
#include <string>

namespace partix {

// soft volume
template < class Traits >
class SoftVolumeSnapShot : public BodySnapShot< Traits > {
public:
	typedef typename Traits::vector_type	vector_type;
	typedef typename Traits::matrix_type	matrix_type;

	BodySnapShot< Traits >* clone()
	{
		return new SoftVolumeSnapShot< Traits >( *this );
	}

	Cloud< Traits > cloud;
	vector_type		initial_center;
	vector_type		global_force;
};

template < class Traits >
class SoftVolume : public Volume< Traits > {
private:
	typedef typename Traits::vector_traits	vector_traits;
	typedef typename Traits::real_type		real_type;
	typedef typename Traits::vector_type	vector_type;
	typedef typename Traits::matrix_type	matrix_type;
	typedef TetrahedralMesh< Traits >		mesh_type;
	typedef Body< Traits >					body_type;
	typedef Cloud< Traits >					cloud_type;
	typedef Collidable< Traits >			collidable_type;
	typedef std::vector< collidable_type* > collidables_type;

	typedef typename mesh_type::point_type	point_type;
	typedef typename mesh_type::points_type points_type;
	typedef typename mesh_type::edge_type	edge_type;
	typedef typename mesh_type::edges_type	edges_type;
	typedef typename mesh_type::face_type	face_type;
	typedef typename mesh_type::faces_type	faces_type;
	typedef typename mesh_type::indices_type indices_type;
	typedef typename mesh_type::tetrahedron_type tetrahedron_type;
	typedef typename mesh_type::tetrahedra_type tetrahedra_type;

public:
	SoftVolume()
	{
		touch_level_	= 2;
		current_center_ = initial_center_ =
			math< Traits >::vector_zero();
		restore_factor_ = 1.0f;
		stretch_factor_ = 0.0f;
		freezing_duration_ = 0;
		crush_duration_ = 0;
		debug_flag_ = false;
		math< Traits >::make_identity( criterion_ );
		math< Traits >::make_identity( R_ );
		math< Traits >::make_identity( G_ );
	}
	~SoftVolume(){}

	// implements Body
	int classid() { return BODY_ID_SOFTVOLUME; }

	SoftVolumeSnapShot< Traits >* make_snapshot()
	{
		return make_snapshot_internal();
	}
	void apply_snapshot( const BodySnapShot< Traits >* ss )
	{
		apply_snapshot_internal( ss );
	}

	void regularize()
	{
		regularize_internal();
		match_shape_internal();
		update_display_matrix_internal();
	}
	void list_collision_units( collidables_type& s )
	{
		list_collision_units_internal( s );
	}
	void move( const vector_type& v )
	{
		move_internal( v );
	}
	void teleport( const vector_type& v )
	{
		teleport_internal( v );
	}
		
	// original
	void set_name( const char* p ) { name_ = p; }
		
	bool crushed() { return crushed_; }
		
	vector_type get_initial_center() { return initial_center_; }
		
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
	void rotate_teleportal(
		real_type w, real_type x, real_type y, real_type z // quaternion
		)
	{
		rotate_teleportal_internal( w, x, y, z, initial_center_ );
	}
	void rotate_teleportal(
		real_type w, real_type x, real_type y, real_type z, // quaternion
		const vector_type& pivot )
	{
		rotate_teleportal_internal( w, x, y, z, pivot );
	}
	void kill_inertia() { kill_inertia_internal(); }
	void reset_rotation() { reset_rotation_internal(); }

	const matrix_type& get_deformed_matrix() { return deformed_matrix_; }
	const matrix_type& get_orientation_matrix() { return orientation_matrix_; }

	void update_mass() { update_mass_internal(); }

	void set_restore_factor( real_type x ) { restore_factor_ = x; }
	void set_stretch_factor( real_type x ) { stretch_factor_ = x; }

	vector_type get_current_origin()
	{
		return transform( -initial_center_ );
	}
	vector_type get_current_center()
	{
		return current_center_;
	}

private:
	SoftVolume( const SoftVolume& ){}
	void operator=( const SoftVolume& ){}

private:
	void begin_frame()
	{
		begin_frame_internal();
	}
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
	void update_display_matrix()
	{
		update_display_matrix_internal();
	}
	void end_frame()
	{
		end_frame_internal();
	}

	void update_boundingbox()
	{
		update_boundingbox_internal();
	}

	// implements Collidable
	body_type* get_body() { return this; }
	cloud_type* get_cloud() { return this->get_mesh()->get_cloud(); }
	indices_type& get_indices() { return this->get_mesh()->get_indices(); }
	faces_type& get_faces() { return this->get_mesh()->get_faces(); }
	vector_type get_center() { return current_center_; }
	vector_type get_bbmin() { return bbmin_; }
	vector_type get_bbmax() { return bbmax_; }
	real_type get_recommended_grid_size()
	{
		return this->get_mesh()->get_average_edge_length();
	}

	void clear_neighbors() { clear_neighbors_internal(); }
	void add_neighbor( collidable_type* p ) { neighbors_.push_back( p ); }
	collidables_type& get_neighbors() { return neighbors_; }

	bool pick(
		const vector_type& s0,
		const vector_type& s1,
		real_type& dist )
	{
		return pick_internal( s0, s1, dist );
	}

	void mark() { marked_ = true; }
	void unmark() { marked_ = false; }
	bool marked() { return marked_; }

	void set_collision_mark( bool f ) { this->collision_mark_ = f; }
	void get_collision_mark() { return this->collision_mark_; }

	void set_criterial_matrix( const matrix_type&  m )
	{
		set_criterial_matrix_internal( m );
	}

	void mark_border_edges( EdgeFaceSpatialHash< Traits >& hash )
	{
		mark_border_edges_internal( hash );
	}

	void set_debug_flag( bool f ) { debug_flag_ = f; }
	void dump()
	{
#ifdef _WINDOWS
		char buffer[256];
		sprintf( buffer, "model: %d\n", get_id() );
		OutputDebugStringA( buffer );

		points_type& points = get_mesh()->get_points();

		for( typename points_type::const_iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			const point_type& v = *i;
			char buffer[256];
			sprintf( buffer,
					 "point %f, %f, %f; %f, %f, %f; %f, %f, %f\n",
					 v.forces.x,
					 v.forces.y,
					 v.forces.z,
					 v.old_position.x,
					 v.old_position.y,
					 v.old_position.z,
					 v.new_position.x,
					 v.new_position.y,
					 v.new_position.z );
			OutputDebugStringA( buffer );
		}

		sprintf( buffer, "entity force: %f, %f, %f\n",
				 get_force().x,
				 get_force().y,
				 get_force().z );
		OutputDebugStringA( buffer );
#endif
	}

private:
	SoftVolumeSnapShot< Traits >* make_snapshot_internal()
	{
		// ���L����SnapShot�Ɉړ�����

		// TODO: ������
		SoftVolumeSnapShot< Traits >* ds =
			new SoftVolumeSnapShot< Traits >;
		return ds;
	}

	void apply_snapshot_internal( const BodySnapShot< Traits >* ss )
	{
		// TODO: ������
		const SoftShellSnapShot< Traits >* ds =
			dynamic_cast< const SoftShellSnapShot< Traits >* >(
				ss );
		assert( ds );
	}

	void list_collision_units_internal( collidables_type& s )
	{
		s.push_back( this );
	}

	void begin_frame_internal()
	{
		if( touch_level_ == 2 ) {
			regularize_internal();
		}
		crushed_ = false;
	}

	void update_velocity_internal( real_type dt, real_type idt )
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return; }
			if( this->get_frozen() && !this->get_defrosting() ) { return; }
		}
		this->get_mesh()->get_cloud()->update_velocity( dt, idt );
	}

	void apply_forces_internal( real_type dt, real_type idt )
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return; }
			if( this->get_frozen() && !this->get_defrosting() ) { return; }
		}

		vector_type v0 = math< Traits >::vector_zero();
		const real_type drag_coefficient =
			real_type( 1.0 ) -
			pow(  real_type( 1.0 ) - this->get_drag_factor(), dt );
				
		// ����dumping (adhoc, �Î~�ɗ��p)
		real_type dump_factor =
			pow(  real_type( 1.0 ) - this->get_internal_dump_factor(), dt );
#if 0
		{
			char buffer[256];
			sprintf( buffer, "dump_factor: %f\n", dump_factor );
			OutputDebugStringA( buffer );
		}
		real_type dump_factor = real_type( 1.0 );
#endif
		this->get_mesh()->get_cloud()->apply_forces(
			dt,
			idt,
			this->get_force(),
			this->get_global_force(),
			drag_coefficient,
			dump_factor );
		this->set_force( v0 );
	}

	void match_shape_internal()
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return; }
			if( this->get_frozen() && !this->get_defrosting() ) { return; }
		}

		// �d�S���v�Z
		calculate_center();
#if 0
		if( isnan( current_center_.x ) ) { DebugBreak(); }
		if( isnan( current_center_.y ) ) { DebugBreak(); }
		if( isnan( current_center_.z ) ) { DebugBreak(); }
#endif

		// �u����ׂ��_�v�ɂЂ��ς���
		real_type Apq[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		real_type e[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		points_type& points = this->get_mesh()->get_points();
		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& point = *i;
			if( !point.surface ) { continue; }

			point.check();

			vector_type p = point.new_position - current_center_;
			const vector_type& q = point.ideal_offset;
								
			real_type px = vector_traits::x( p );
			real_type py = vector_traits::y( p );
			real_type pz = vector_traits::z( p );
			real_type qx = vector_traits::x( q );
			real_type qy = vector_traits::y( q );
			real_type qz = vector_traits::z( q );
								
			real_type m = point.mass;

			math< Traits >::mount( Apq[0], e[0], m * px * qx );
			math< Traits >::mount( Apq[1], e[1], m * px * qy );
			math< Traits >::mount( Apq[2], e[2], m * px * qz );
			math< Traits >::mount( Apq[3], e[3], m * py * qx );
			math< Traits >::mount( Apq[4], e[4], m * py * qy );
			math< Traits >::mount( Apq[5], e[5], m * py * qz );
			math< Traits >::mount( Apq[6], e[6], m * pz * qx );
			math< Traits >::mount( Apq[7], e[7], m * pz * qy );
			math< Traits >::mount( Apq[8], e[8], m * pz * qz );
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
		if( 0.7f < std::abs( 1.0f - detR ) ) {
			// R����ꂷ��
#if 0
			char buffer[256];
			sprintf( buffer, "det0: %f\n", detR );
			OutputDebugStringA( buffer );
#endif
			crushed_ = true;
		} else {
			real_type A[9];
			math< Traits >::multiply_matrix( A, Apq, Aqq_ );
			real_type detA =
				math< Traits >::determinant_matrix( A );

			if( 0.5f < std::abs( 1.0f - detA ) ) {	
				// A����ꂷ��
#if 0
				char buffer[256];
				sprintf( buffer, "det1: %f\n", detA );
				OutputDebugStringA( buffer );
#endif
				crushed_ = true;
			} else {
#if 0
				if( 0.4f < std::abs( 1.0f - detA ) ) {	
					char buffer[256];
					sprintf( buffer, "det2: %f\n", detA );
					OutputDebugStringA( buffer );
				}
#endif

				real_type cbrt = pow(
					std::abs( detA ), real_type( 1.0/3.0 ) );

				real_type Adash[9];
				math< Traits >::multiply_matrix(
					Adash, A, real_type( 1.0 ) / cbrt );
								
				// det�����������قǍd���Ȃ�
				real_type stretch_factor =
					stretch_factor_ - std::abs( 1.0f - detA );
				if( stretch_factor < 0 ) { stretch_factor = 0; }

				// det���������قǃG�l���M�[�r��
				real_type dump =
					square( real_type( 1.0 ) - detA ) *
					real_type( 10.0 );
#if 0
				if( isnan( dump ) ) { DebugBreak(); }
#endif
				if( real_type( 0.9 ) < dump ) {
					dump = real_type( 0.9 );
				}
				set_internal_dump_factor( dump ) ;
#if 0
				{
					real_type energy = 0;
					for( typename points_type::iterator i =
							 points.begin() ;
						 i != points.end() ;
						 ++i ) {
						point_type& point = *i;
						energy += point.energy;
					}
										
					char buffer [256];
					sprintf( buffer, "dump: %.8f, %f\n",
							 dump, energy );
					OutputDebugStringA( buffer );
				}
#endif
										
				real_type G0[9];
				real_type G1[9];
				math< Traits >::multiply_matrix(
					G0, Adash, stretch_factor );
				math< Traits >::multiply_matrix(
					G1, R, 1.0f - stretch_factor );

				real_type G[9];
				math< Traits >::add_matrix( G, G0, G1 );

				real_type detG =
					math< Traits >::determinant_matrix(
						G );
				if( 0.5f < detG ) { 
					// �}�g���b�N�X�������Ȃ̂ŏ��F
					math< Traits >::copy_matrix( R_, R );
					math< Traits >::copy_matrix( G_, G );
				}
			}
		}
	}

	void restore_shape_internal( real_type dt, real_type idt, int kmax )
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return; }
			if( this->get_frozen() && !this->get_defrosting() ) { return; }
		}

		points_type& points = this->get_mesh()->get_points();

		if( crushed_ ) {
			// TODO:
			if( crush_duration_ == 0 ) {
				for( typename points_type::iterator i =
						 points.begin() ;
					 i != points.end() ;
					 ++i ) {
					point_type& p = *i;
					p.new_position = p.old_position;
					p.old_position += p.velocity * dt;
					p.velocity *= -1;
				}
			}
			crush_duration_ += dt;
			if( 0.03f <= crush_duration_ ) {
				kill_inertia_internal();
			}
		} else {
			crush_duration_ = 0;
		}
				
		real_type rest =
			real_type( 1.0 ) -
			pow( real_type( 1.0 ) - restore_factor_,
				 real_type( 1.0 ) / ( kmax + real_type ( 1.0 ) ) );
				
		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& p = *i;

			const vector_type& q = p.ideal_offset;
								
			vector_type g;
			math< Traits >::transform_vector( g, G_, q );

			//real_type dump = vector_traits::length_sq( g );
			// �ł��҂��Ԃ̗}��

			g += current_center_;
								
			vector_type velocity_dt =
				( g - p.new_position ) * rest;
			p.new_position += velocity_dt;
			p.check();
		}
	}

	void update_display_matrix_internal()
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return; }
			if( this->get_frozen() && !this->get_defrosting() ) { return; }
		}

		// �\���p�}�g���b�N�X
		vector_type ic = -initial_center_; // �d�S����݂����_
		vector_type tic;
		math< Traits >::transform_vector( tic, R_, ic );

#if 0
		if( isnan( current_center_.x ) ) { DebugBreak(); }
		if( isnan( current_center_.y ) ) { DebugBreak(); }
		if( isnan( current_center_.z ) ) { DebugBreak(); }
#endif

		Traits::make_matrix(
			orientation_matrix_, R_, tic + current_center_ );

		real_type G[9];
		math< Traits >::multiply_matrix( G, G_, criterion_ );
		math< Traits >::transform_vector( tic, G, ic );
		Traits::make_matrix(
			deformed_matrix_, G, tic + current_center_ );
	}

	void update_boundingbox_internal()
	{
		if( touch_level_ == 0 ) {
			if( !this->get_alive() ) { return; }
			if( this->get_frozen() && !this->get_defrosting() ) { return; }
		}

		bbmin_ = math< Traits >::vector_max();
		bbmax_ = math< Traits >::vector_min();
				
		this->get_mesh()->get_cloud()->update_bb( bbmin_, bbmax_ );
	}

	void clear_neighbors_internal()
	{
		neighbors_.clear();
		points_type& points = this->get_mesh()->get_points();
		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& v = *i;
			v.collided = false;
		}
		edges_type& edges = this->get_mesh()->get_edges();
		for( typename edges_type::iterator i = edges.begin() ;
			 i != edges.end() ;
			 ++i ) {
			edge_type& e = *i;
			e.border = false;
			e.t = math< Traits >::real_max();
		}
		make_normals();
	}

	void update_frozen( real_type dt, real_type idt )
	{
		if( this->get_auto_freezing() ) {
			real_type energy = 0;
			real_type e = 0;
			points_type& points = this->get_mesh()->get_points();
			for( typename points_type::iterator i =
					 points.begin() ;
				 i != points.end() ; ++i ) {
				point_type& p = *i;
				math< Traits >::mount( energy, e, p.energy );
			}

			if( energy <= Traits::freeze_threshold_energy() ) {
				freezing_duration_ += dt;
				if( Traits::freeze_duration() <=
					freezing_duration_ ) {
					this->set_frozen( true );
					freezing_duration_ = 0;
#if 0
					for( typename points_type::iterator i =
							 points.begin() ;
						 i != points.end() ;
						 ++i ) {
						point_type& p = *i;
						p.old_position =
							p.new_position;
					}
#endif

#if 0
					OutputDebugStringA( "frozen\n" );
#endif
				}
			} else {
#if 0
				{
					char buffer[256];
					sprintf( buffer, "energy: %f\n",
							 energy );
					OutputDebugStringA( buffer );
				}
#endif
				this->set_frozen( false );
			}
		}
		this->set_defrosting( false );
	}

	void end_frame_internal()
	{
		touch_level_ = 0;
	}

	void move_internal( const vector_type& v )
	{
		this->get_mesh()->get_cloud()->move( v );
		set_touch_level( 1 );
	}

	void teleport_internal( const vector_type& v )
	{
		this->get_mesh()->get_cloud()->teleport( v );
		set_touch_level( 1 );
	}

	void rotate_internal(
		real_type w, real_type x, real_type y, real_type z,
		const vector_type& pivot )
	{
		quaternion< real_type > q( w, x, y, z );
		quaternion< real_type > rq( w, -x, -y, -z );
		vector_type center = transform( pivot - initial_center_ );

		this->get_mesh()->get_cloud()->rotate( q, rq, center );
		set_touch_level( 1 );
	}

	void rotate_teleportal_internal(
		real_type w, real_type x, real_type y, real_type z,
		const vector_type& pivot )
	{
		quaternion< real_type > q( w, x, y, z );
		quaternion< real_type > rq( w, -x, -y, -z );
		vector_type center = transform( pivot - initial_center_ );

		this->get_mesh()->get_cloud()->rotate_teleportal( q, rq, center );
		set_touch_level( 1 );
	}

	void kill_inertia_internal()
	{
		this->get_mesh()->get_cloud()->kill_inertia();
		set_touch_level( 1 );
	}

	void reset_rotation_internal()
	{
		calculate_center();
		points_type& points = this->get_mesh()->get_points();
		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& p = *i;
			p.old_position = p.new_position =
				current_center_ - initial_center_ +
				p.source_position;
		}
		set_touch_level( 1 );
	}

	void update_mass_internal()
	{
		set_touch_level( 2 );
	}

	void set_criterial_matrix_internal( const matrix_type& m )
	{
		Traits::dump_matrix( criterion_, m );
		set_touch_level( 2 );
	}

	void mark_border_edges_internal( EdgeFaceSpatialHash< Traits >& hash )
	{
		points_type& points = this->get_mesh()->get_points();

		edges_type& edges = this->get_mesh()->get_edges();
		int ii = 0;
		for( typename edges_type::iterator i = edges.begin() ;
			 i != edges.end() ;
			 ++i, ++ii ) {
			edge_type& e = *i;
			if( points[e.indices.i0].collided !=
				points[e.indices.i1].collided ) {
				e.border = true;
				hash.add_edge( this->get_mesh(), ii );
			}
		}
	}

	bool pick_internal(
		const vector_type& s0,
		const vector_type& s1,
		real_type& dist )
	{
		if( math< Traits >::test_aabb_segment(
				bbmin_, bbmax_, s0, s1 ) ) {
			const points_type& points = this->get_mesh()->get_points();
			const faces_type& faces = this->get_mesh()->get_faces();

			SegmentTriangleSpatialHash< Traits > hash(
				this->get_mesh()->get_average_edge_length(), 4999 );
			hash.add_segment( s0, s1 );
			for( typename faces_type::const_iterator i =
					 faces.begin() ;
				 i != faces.end() ;
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

private:
#if 1
	void debug_check()
	{
	}
#else
#pragma optimize( "", off )
	void debug_check()
	{
		if( !debug_flag_ ) { return; }

		points_type& points = get_mesh()->get_points();
		int n = int( points.size() );

		edges_type& edges = get_mesh()->get_edges();
		for( typename edges_type::const_iterator i = edges.begin() ;
			 i != edges.end() ;
			 ++i ) {
			const edge_type& e = *i;
			int ei0 = e.indices.i0;
			int ei1 = e.indices.i1;

			if( ei0 < 0 || n <= ei0 ) {
				DebugBreak();
			}
			if( ei1 < 0 || n <= ei1 ) {
				DebugBreak();
			}
		}
	}
#pragma optimize( "", on )
#endif


	void set_touch_level( int n )
	{
		if( touch_level_ < n ) { touch_level_ = n; }
	}

	void regularize_internal()
	{
		vector_type v0 = math< Traits >::vector_zero();

		points_type& points = this->get_mesh()->get_points();
		faces_type& faces = this->get_mesh()->get_faces();

		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& p = *i;
			p.surface = false;
		}

		for( typename faces_type::iterator i = faces.begin() ;
			 i != faces.end() ;
			 ++i ) {
			face_type& f = *i;
			points[f.i0].surface = true;
			points[f.i1].surface = true;
			points[f.i2].surface = true;
		}

		calculate_initial_center();
		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& p = *i;
			p.invmass = 1 / p.mass;
			math< Traits >::transform_vector(
				p.ideal_offset,
				criterion_,
				p.source_position - initial_center_ );
		}

		calculate_Aqq();
		calculate_center();
		make_normals();
	}

	void calculate_Aqq()
	{
		points_type& points = this->get_mesh()->get_points();

		real_type Aqq[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, };
		real_type e[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, };
		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& p = *i;
			if( !p.surface ) { continue; }

			const vector_type& q = p.ideal_offset;
								
			real_type qx = vector_traits::x( q );
			real_type qy = vector_traits::y( q );
			real_type qz = vector_traits::z( q );
						
			real_type m = p.mass;

			math< Traits >::mount( Aqq[0], e[0], m * qx * qx );
			math< Traits >::mount( Aqq[1], e[1], m * qx * qy );
			math< Traits >::mount( Aqq[2], e[2], m * qx * qz );
			math< Traits >::mount( Aqq[3], e[3], m * qy * qx );
			math< Traits >::mount( Aqq[4], e[4], m * qy * qy );
			math< Traits >::mount( Aqq[5], e[5], m * qy * qz );
			math< Traits >::mount( Aqq[6], e[6], m * qz * qx );
			math< Traits >::mount( Aqq[7], e[7], m * qz * qy );
			math< Traits >::mount( Aqq[8], e[8], m * qz * qz );
		}

		math< Traits >::inverse_matrix( Aqq_, Aqq );
	}

	void calculate_initial_center()
	{
		vector_type total_center = math< Traits >::vector_zero();
		vector_type e = math< Traits >::vector_zero();
		real_type total_mass = 0;
		real_type mass_e = 0;

		points_type& points = this->get_mesh()->get_points();
		for( typename points_type::const_iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			const point_type& p = *i;
			if( !p.surface ) { continue; }
						
			math< Traits >::mount(
				total_center, e, p.source_position * p.mass );
			math< Traits >::mount(
				total_mass, mass_e, p.mass );
		}

		total_center *= real_type(1.0) / total_mass;
		initial_center_ = total_center;
	}

	void calculate_center()
	{
		vector_type total_center = math< Traits >::vector_zero();
		vector_type e = math< Traits >::vector_zero();
		real_type total_mass = 0;
		real_type mass_e = 0;

		points_type& points = this->get_mesh()->get_points();
		for( typename points_type::const_iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			const point_type& p = *i;
			if( !p.surface ) { continue; }

			math< Traits >::mount(
				total_center, e, p.new_position * p.mass );
			math< Traits >::mount(
				total_mass, mass_e, p.mass );
		}

		total_center *= real_type(1.0) / total_mass;
		current_center_ = total_center;
	}

	void set_orientation_internal( const matrix_type& m )
	{
		points_type& points = this->get_mesh()->get_points();
		for( typename points_type::const_iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& p = *i;

			p.new_position = Traits::transform_vector(
				m, p.initial_offset ) + current_center_;
		}
	}

	void make_normals()
	{
		vector_type v0 = math< Traits >::vector_zero();
				
		points_type& points = this->get_mesh()->get_points();
		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			(*i).normal = v0;
		}

		faces_type& faces = this->get_mesh()->get_faces();
		for( typename faces_type::iterator i = faces.begin() ;
			 i != faces.end() ;
			 ++i ) {
			face_type& f = *i;
			vector_type e1 =
				points[f.i1].new_position -
				points[f.i0].new_position;
			vector_type e2 =
				points[f.i2].new_position -
				points[f.i0].new_position;
			vector_type n = math< Traits >::cross( e1, e2 );
			//math< Traits >::normalize_f( n );
			points[f.i0].normal += n;
			points[f.i1].normal += n;
			points[f.i2].normal += n;
		}

		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			math< Traits >::normalize_f( (*i).normal );
		}
	}

	void calculate_penetration_direction()
	{
		vector_type v0 = math< Traits >::vector_zero();

		points_type& points = this->get_mesh()->get_points();
		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& v = *i;
			v.penetration_depth_numerator = 0;
			v.penetration_direction_numerator = v0;
			v.penetration_denominator = 0;
		}

		edges_type& edges = this->get_mesh()->get_edges();
		for( typename edges_type::iterator i = edges.begin() ;
			 i != edges.end() ;
			 ++i ) {
			edge_type& e = *i;
			if( !e.border ) { continue; }
			int ei0 = e.indices.i0;
			int ei1 = e.indices.i1;

#ifdef _WINDOWS						   
#if 1
			if( ei0 < 0 || int( points.size() ) <= ei0 ) {
				DebugBreak();
			}
			if( ei1 < 0 || int( points.size() ) <= ei1 ) {
				DebugBreak();
			}
#else
			// WORKAROUND bug
			if( ei0 < 0 || int( points.size() ) <= ei0 ) {
				continue;
			}
			if( ei1 < 0 || int( points.size() ) <= ei1 ) {
				continue;
			}
#endif
#endif
			if( points[ei1].collided ) { std::swap( ei0, ei1 ); }
						
			point_type& v0 = points[ei0];
			point_type& v1 = points[ei1];						 

#ifdef _WINDOWS
			if( !v0.collided || v1.collided ) {
				DebugBreak();
			}
#endif

			if( e.t == math< Traits >::real_max() ) {
				// edge-face intersection ���s
				// v0.penetration_denominator = 0;
				// �����X�Ȃ��Ă�
			} else {
				// edge-face intersection ����
				vector_type ov =
					( v1.new_position -
					  v0.new_position ) * e.t;
				real_type ovlensq =
					vector_traits::length_sq( ov );
				if( ovlensq < math< Traits >::epsilon() ) {
					continue;
				}

				real_type omega = real_type( 1.0 ) / ovlensq;

				v0.penetration_depth_numerator +=
					omega * math< Traits >::dot(
						ov, e.collision_normal );
				v0.penetration_direction_numerator +=
					e.collision_normal * omega;
				v0.penetration_denominator += omega;
			}
		}
				
		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& v = *i;
			if( !v.collided ) { continue; }

			vector_type& v0 = v.penetration_direction_numerator;
			real_type v0len = vector_traits::length( v0 );
			if( v0len * v.penetration_denominator <
				math< Traits >::epsilon() ) { continue; }

			v.penetration_vector =
				v0 * ( v.penetration_depth_numerator /
					   ( v0len * v.penetration_denominator ) );

#if 0
			if( isnan( v.penetration_vector.x ) ||
				isnan( v.penetration_vector.y ) ||
				isnan( v.penetration_vector.z ) ) {
				DebugBreak();
			}
#endif
						
			// normalize�ς�
		}
	}

	void propagate_penetration()
	{
		vector_type v0 = math< Traits >::vector_zero();

		points_type& points = this->get_mesh()->get_points();
		edges_type& edges = this->get_mesh()->get_edges();

		// ������
		for( typename points_type::iterator i = points.begin() ;
			 i != points.end() ;
			 ++i ) {
			point_type& v = *i;
			v.process_flag = false;
			v.propagating = false;
		}

		// border edge�ɏ����σ}�[�N
		// border point�ɏ����σ}�[�N
		for( typename edges_type::iterator i = edges.begin() ;
			 i != edges.end() ;
			 ++i ) {
			edge_type& e = *i;
			if( !e.border ) { continue; }
			int ei0 = e.indices.i0;
			int ei1 = e.indices.i1;

#ifdef _WINDOWS
#if 1
			if( ei0 < 0 || int( points.size() ) <= ei0 ) {
				DebugBreak();
			}
			if( ei1 < 0 || int( points.size() ) <= ei1 ) {
				DebugBreak();
			}
#else
			// WORKAROUND bug
			if( ei0 < 0 || int( points.size() ) <= ei0 ) {
				continue;
			}
			if( ei1 < 0 || int( points.size() ) <= ei1 ) {
				continue;
			}
#endif
#endif
			if( points[ei1].collided ) { std::swap( ei0, ei1 ); }

			point_type& v0 = points[ei0];
			point_type& v1 = points[ei1];

#ifdef _WINDOWS
			if( !v0.collided || v1.collided ) {
				DebugBreak();
			}
#endif

			v0.process_flag = true;
		}

		// �C�e���[�V����
		// TODO: �o�b�N�X���b�v�L���[�ō������\.
		int processed_count;
		do {
			for( typename edges_type::iterator i = edges.begin() ;
				 i != edges.end() ;
				 ++i ) {
				edge_type& e = *i;
				int ei0 = e.indices.i0;
				int ei1 = e.indices.i1;

#ifdef _WINDOWS
#if 1
				if( ei0 < 0 || int( points.size() ) <= ei0 ) {
					DebugBreak();
				}
				if( ei1 < 0 || int( points.size() ) <= ei1 ) {
					DebugBreak();
				}
#else
				// WORKAROUND bug
				if( ei0 < 0 || int( points.size() ) <= ei0 ) {
					continue;
				}
				if( ei1 < 0 || int( points.size() ) <= ei1 ) {
					continue;
				}
#endif
#endif
				if( points[ei0].process_flag ==
					points[ei1].process_flag ) {
					continue;
				}
				if( !points[ei0].process_flag &&
					points[ei1].process_flag ) {
					std::swap( ei0, ei1 );
				}

				point_type& v0 = points[ei0];
				point_type& v1 = points[ei1];

#ifdef _WINDOWS
				if( !v0.collided ) {
					DebugBreak();
				}
#endif
								
				if( !v1.collided ) { continue; }

				if( v0.penetration_denominator <
					math< Traits >::epsilon() ) {
					continue;
				}

				vector_type dv =
					v1.new_position -
					v0.new_position;
				real_type dvlensq =
					vector_traits::length_sq( dv );
				if( dvlensq < math< Traits >::epsilon() ) {
					continue;
				}
				real_type mu = real_type( 1.0 ) / dvlensq;

				v1.penetration_denominator += mu;
				v1.penetration_depth_numerator += 
					mu *
					( math< Traits >::dot(
						dv, v0.penetration_vector ) +
					  ( v0.penetration_depth_numerator /
						v0.penetration_denominator ) );
				v1.penetration_direction_numerator +=
					v0.penetration_vector * mu;
				v1.propagating = true;
			}

			processed_count = 0;
			for( typename points_type::iterator i = points.begin();
				 i != points.end() ;
				 ++i ) {
				point_type& v = *i;
				if( !v.propagating ) { continue; }

				v.process_flag = true;
				v.propagating = false;
				v.penetration_vector =
					v.penetration_direction_numerator *
					( real_type(1) / v.penetration_denominator ); 
								
#if 0
				if( isnan( v.penetration_vector.x ) ||
					isnan( v.penetration_vector.y ) ||
					isnan( v.penetration_vector.z ) ) {
					DebugBreak();
				}
#endif
								
				processed_count++;
			}
						
		} while( 0 < processed_count );
	}

	vector_type transform( const vector_type& v )
	{
		real_type G[9];
		math< Traits >::multiply_matrix( G, G_, criterion_ );

		vector_type trv;
		math< Traits >::transform_vector( trv, G, v );

		return current_center_ + trv;
	}

private:
	std::string		name_;

	// touch_level_
	//	 1 ... �ʒu
	//	 2 ... �ʒu + ����
	int				touch_level_;
		
	vector_type		initial_center_;
	vector_type		current_center_;
	matrix_type		deformed_matrix_;
	matrix_type		orientation_matrix_;
	vector_type		bbmin_;
	vector_type		bbmax_;
	real_type		Aqq_[9];
	real_type		R_[9];
	real_type		G_[9];
	real_type		criterion_[9];
	real_type		restore_factor_;
	real_type		stretch_factor_;
	real_type		freezing_duration_;
	bool			crushed_;
	real_type		crush_duration_;

	std::vector< Collidable< Traits >* >	neighbors_;
	bool									marked_;

	bool			debug_flag_;

	template < class T > friend class World;

};


} // namespace partix

#endif // PARTIX_SOFTVOLUME_HPP
