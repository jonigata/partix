/*!
  @file		partix_geometry.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: partix_geometry.hpp 53 2007-04-05 04:30:51Z hirayama $
*/
#ifndef PARTIX_GEOMETRY_HPP
#define PARTIX_GEOMETRY_HPP

namespace partix {

template < class Traits >
struct Edge {
	typedef typename Traits::index_type index_type;

	index_type i0;
	index_type i1;
};

template < class Traits >
struct Face {
	typedef typename Traits::index_type index_type;

	index_type i0;
	index_type i1;
	index_type i2;
};

template < class Traits >
struct Tetrahedron {
	typedef typename Traits::index_type index_type;

	index_type i0;
	index_type i1;
	index_type i2;
	index_type i3;
};

};

#endif // PARTIX_GEOMETRY_HPP
