/*!
  @file		partix.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: partix.hpp 53 2007-04-05 04:30:51Z hirayama $
*/
#ifndef PARTIX_HPP
#define PARTIX_HPP

#include <vector>
#include <set>
#include "partix_world.hpp"

namespace partix {

template < class Traits > class Cloth;

template < class Traits >
struct package {
	typedef Face< Traits >					face_type;
	typedef Edge< Traits >					edge_type;
	typedef Tetrahedron< Traits >			tetrahedron_type;
	typedef Point< Traits >					point_type;
	typedef Cloud< Traits >					cloud_type;
	typedef Block< Traits >					block_type;
	typedef TetrahedralMesh< Traits >		tetrahedralmesh_type;
	typedef Body< Traits >					body_type;
	typedef SoftShell< Traits >				softshell_type;
	typedef SoftVolume< Traits >			softvolume_type;
	typedef Cloth< Traits >					cloth_type;
	typedef BoundingPlane< Traits >			boundingplane_type;
	typedef World< Traits >					world_type;
};

} // namespace partix

#endif // PARTIX_HPP
