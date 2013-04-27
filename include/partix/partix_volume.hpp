/*!
  @file		partix_volume.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: partix_volume.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_VOLUME_HPP
#define PARTIX_VOLUME_HPP

#include "partix_body.hpp"
#include "partix_tetrahedral_mesh.hpp"

namespace partix {

template < class Traits >
class Volume : public Body< Traits >, public Collidable< Traits > {
public:
	typedef TetrahedralMesh< Traits > mesh_type;

	Volume() { mesh_ = NULL; }

	void			clear() { mesh_ = NULL; }
	void			set_mesh( mesh_type* p )
	{
		mesh_ = p;
		mesh_->set_volume( this );
	}
	mesh_type*		get_mesh() { return mesh_; }

protected:
	mesh_type*		mesh_;

};

} // namespace partix

#endif // PARTIX_VOLUME_HPP
