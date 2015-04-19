/*!
  @file		partix_rigid.hpp
  @brief	<概要>

  <説明>
  $Id: partix_rigid.hpp 252 2007-06-23 08:32:33Z naoyuki $
  WARNING: このモジュールメンテナンスしてない
*/
#ifndef PARTIX_RIGID_HPP
#define PARTIX_RIGID_HPP

#include "partix_body.hpp"
#include "partix_math.hpp"

namespace partix {

template < class Traits >
class Rigid : public Mesh< Traits > {
public:
	typedef typename Traits::index_type		index_type;
	typedef typename Traits::float_type		float_type;
	typedef typename Traits::vector_type	vector_type;

	Rigid(){ clear(); }
	~Rigid(){ clear(); }

	int classid() { return 2; }

	void clear()
	{
		initialized_ = false;
		Mesh< Traits >::clear();
	}

	void restart()
	{
		start();
		initialized_ = true;
	}

	void apply_inertia( float_type dt, float_type idt )
	{
		// position
		position_ += linear_velocity_ * dt;

		// orientation
		vector_type v = angular_velocity_ * ( dt * 0.5f );

		quaternion_ *= quaternion< float_type >( v.x, v.y, v.z, float_type( 1.0 ) );
	}

	void apply_forces( float_type dt, float_type idt )
	{
	}

	void update_boundingbox()
	{
	}

	void prepare_render()
	{
		quaternion_.normalize();

		float_type m[9];
		quaternion_.make_matrix( m );
		Traits::make_matrix( transform_, m, position_ );
	}

	void add_linear_momenutm( const vector_type& v ) { linear_momentum_ += v; }
	void add_angular_momentum( const vector_type& v ) { angular_momentum_ += v; }
	void add_linear_velocity( const vector_type& v ) { linear_velocity_ += v; }
	void add_angular_velocity( const vector_type& v ) { angular_velocity_ += v; }

	// implements ImpulseReceiver
	float_type get_mass_inv() { return mass_inv_; }
	const float_type* get_inertia_tensor_inv()
	{
		static const float_type tensor[] = {
			float_type( 1.0 ), 0, 0,
			0, float_type( 1.0 ), 0,
			0, 0, float_type( 1.0 ),
		};
		return tensor;
	}
	vector_type get_position() { return position_; }
	vector_type get_velocity( const vector_type& relative_position )
	{
		return linear_velocity_ + math< Traits >::cross( angular_velocity_, relative_position );
	}

	void apply_active_impulse( const vector_type& impulse, const vector_type& relative_position )
	{
		linear_velocity_ += impulse * mass_inv_;

		vector_type torque = math< Traits >::cross( relative_position, impulse );
		vector_type v; math< Traits >::transform_vector( v, get_inertia_tensor_inv(), torque );
		angular_velocity_ += v;
	}
	void apply_passive_impulse( const vector_type& impulse, const vector_type& relative_position )
	{
		apply_active_impulse( -impulse, relative_position );
	}

	vector_type get_center() { return position_; }

private:
	Rigid( const Rigid& ){}
	void operator=( const Rigid& ){}

private:
	void start()
	{
		vector_type v0 = math< Traits >::vector_zero();

		vector_type center_of_mass;
		compute_inertia_tensor( mass_, center_of_mass, inertia_tensor_ );
		//shift( center_of_mass );
		math< Traits >::inverse_matrix( inertia_tensor_inv_, inertia_tensor_ );

		mass_inv_				= 1.0f / mass_;
		position_				= v0;
		linear_momentum_		= v0;
		angular_momentum_		= v0;
		linear_velocity_		= v0;
		angular_velocity_		= v0;
		math< Traits >::make_identity( orientation_ );
		quaternion_				= quaternion< float_type >( 1, 0, 0, 0 );
	}

	void compute_inertia_tensor(
		float_type&		mass,
		vector_type&	center_of_mass,
		float_type*		inertia_tensor )
	{
		const float_type fOneDiv6 = float_type( 1.0 / 6.0 );
		const float_type fOneDiv24 = float_type( 1.0 / 24.0 );
		const float_type fOneDiv60 = float_type( 1.0 / 60.0 ) ; 
		const float_type fOneDiv120 = float_type( 1.0 / 120.0 );

		// order:  1, x, y, z, x^2, y^2, z^2, xy, yz, zx
		float_type afIntegral[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

		for( blocks_type::const_iterator i = blocks_.begin() ; i != blocks_.end() ; ++i ) {
			block_type*		b = *i;
			cloud_type*		c = b->get_cloud();
			vector_type		cloud_center( 0, 0, 0 );
			float_type		cloud_mass = 0;

			int n = int( b->indices_.size() / 3 );
			for( int j = 0 ; j != n ; j++ ) {
				index_type i0 = b->indices_[ j * 3 + 0 ];
				index_type i1 = b->indices_[ j * 3 + 1 ];
				index_type i2 = b->indices_[ j * 3 + 2 ];

				// get vertices of triangle i
				const vector_type& kV0 = c->particles_[i0].position;
				const vector_type& kV1 = c->particles_[i1].position;
				const vector_type& kV2 = c->particles_[i2].position;

				// get cross product of edges
				vector_type kV1mV0 = kV1 - kV0;
				vector_type kV2mV0 = kV2 - kV0;
				vector_type kN = math< Traits >::cross( kV1mV0, kV2mV0 );

				// compute integral terms
				float_type fTmp0, fTmp1, fTmp2;
				float_type fF1x, fF2x, fF3x, fG0x, fG1x, fG2x;
				fTmp0 = kV0.x + kV1.x;
				fF1x = fTmp0 + kV2.x;
				fTmp1 = kV0.x*kV0.x;
				fTmp2 = fTmp1 + kV1.x*fTmp0;
				fF2x = fTmp2 + kV2.x*fF1x;
				fF3x = kV0.x*fTmp1 + kV1.x*fTmp2 + kV2.x*fF2x;
				fG0x = fF2x + kV0.x*(fF1x + kV0.x);
				fG1x = fF2x + kV1.x*(fF1x + kV1.x);
				fG2x = fF2x + kV2.x*(fF1x + kV2.x);

				float_type fF1y, fF2y, fF3y, fG0y, fG1y, fG2y;
				fTmp0 = kV0.y + kV1.y;
				fF1y = fTmp0 + kV2.y;
				fTmp1 = kV0.y*kV0.y;
				fTmp2 = fTmp1 + kV1.y*fTmp0;
				fF2y = fTmp2 + kV2.y*fF1y;
				fF3y = kV0.y*fTmp1 + kV1.y*fTmp2 + kV2.y*fF2y;
				fG0y = fF2y + kV0.y*(fF1y + kV0.y);
				fG1y = fF2y + kV1.y*(fF1y + kV1.y);
				fG2y = fF2y + kV2.y*(fF1y + kV2.y);

				float_type fF1z, fF2z, fF3z, fG0z, fG1z, fG2z;
				fTmp0 = kV0.z + kV1.z;
				fF1z = fTmp0 + kV2.z;
				fTmp1 = kV0.z*kV0.z;
				fTmp2 = fTmp1 + kV1.z*fTmp0;
				fF2z = fTmp2 + kV2.z*fF1z;
				fF3z = kV0.z*fTmp1 + kV1.z*fTmp2 + kV2.z*fF2z;
				fG0z = fF2z + kV0.z*(fF1z + kV0.z);
				fG1z = fF2z + kV1.z*(fF1z + kV1.z);
				fG2z = fF2z + kV2.z*(fF1z + kV2.z);

				// update integrals
				afIntegral[0] += kN.x*fF1x;
				afIntegral[1] += kN.x*fF2x;
				afIntegral[2] += kN.y*fF2y;
				afIntegral[3] += kN.z*fF2z;
				afIntegral[4] += kN.x*fF3x;
				afIntegral[5] += kN.y*fF3y;
				afIntegral[6] += kN.z*fF3z;
				afIntegral[7] += kN.x*(kV0.y*fG0x + kV1.y*fG1x + kV2.y*fG2x);
				afIntegral[8] += kN.y*(kV0.z*fG0y + kV1.z*fG1y + kV2.z*fG2y);
				afIntegral[9] += kN.z*(kV0.x*fG0z + kV1.x*fG1z + kV2.x*fG2z);
			}
		}

		afIntegral[0] *= fOneDiv6;
		afIntegral[1] *= fOneDiv24;
		afIntegral[2] *= fOneDiv24;
		afIntegral[3] *= fOneDiv24;
		afIntegral[4] *= fOneDiv60;
		afIntegral[5] *= fOneDiv60;
		afIntegral[6] *= fOneDiv60;
		afIntegral[7] *= fOneDiv120;
		afIntegral[8] *= fOneDiv120;
		afIntegral[9] *= fOneDiv120;

		// mass
		mass = afIntegral[0];

		// center of mass
		center_of_mass = vector_traits::make_vector(
			afIntegral[1], afIntegral[2], afIntegral[3] ) / mass;

		// inertia relative to world origin
		math< Traits >::make_identity( inertia_tensor );
		inertia_tensor[0] = afIntegral[5] + afIntegral[6];
		inertia_tensor[1] = -afIntegral[7];
		inertia_tensor[2] = -afIntegral[9];
		inertia_tensor[3] = inertia_tensor[2];
		inertia_tensor[4] = afIntegral[4] + afIntegral[6];
		inertia_tensor[5] = -afIntegral[8];
		inertia_tensor[6] = inertia_tensor[3];
		inertia_tensor[7] = inertia_tensor[6];
		inertia_tensor[8] = afIntegral[4] + afIntegral[5];

		// inertia relative to center of mass
		inertia_tensor[0] -= mass*(center_of_mass.y*center_of_mass.y +
								   center_of_mass.z*center_of_mass.z);
		inertia_tensor[1] += mass*center_of_mass.x*center_of_mass.y;
		inertia_tensor[2] += mass*center_of_mass.z*center_of_mass.x;
		inertia_tensor[3] = inertia_tensor[2];
		inertia_tensor[4] -= mass*(center_of_mass.z*center_of_mass.z +
								   center_of_mass.x*center_of_mass.x);
		inertia_tensor[5] += mass*center_of_mass.y*center_of_mass.z;
		inertia_tensor[6] = inertia_tensor[3];
		inertia_tensor[7] = inertia_tensor[6];
		inertia_tensor[8] -= mass*(center_of_mass.x*center_of_mass.x +
								   center_of_mass.y*center_of_mass.y);

#if 0
		// 正方形
		D3DXMatrixIdentity( &inertia_tensor_ );
		inertia_tensor._11 /= 6.0f;
		inertia_tensor._22 /= 6.0f;
		inertia_tensor._33 /= 6.0f;
#endif
	}

//private:
public:
	bool			initialized_;

	vector_type		initial_center_;
	vector_type		current_center_;
	matrix_type		transform_;
	vector_type		bbmin_;
	vector_type		bbmax_;

	float_type		mass_;
	float_type		mass_inv_;
	vector_type		position_;
	float_type		inertia_tensor_[9];
	float_type		inertia_tensor_inv_[9];
	float_type		orientation_[9];
	quaternion< float_type > quaternion_;
	vector_type		linear_momentum_;
	vector_type		angular_momentum_;
	vector_type		linear_velocity_;
	vector_type		angular_velocity_;

	template < class T > friend class World;
};

} // namespace partix

#endif // PARTIX_RIGID_HPP
