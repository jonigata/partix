/*!
  @file		partix_constraint.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: partix_constraint.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_CONSTRAINT_HPP
#define PARTIX_CONSTRAINT_HPP

#include "partix_point.hpp"
#include "partix_body.hpp"

namespace partix {

template < class Traits >
class Constraint {
public:
	typedef typename Traits::real_type		real_type;
	typedef typename Traits::vector_type	vector_type;

	Body< Traits >*			A_body;
	Body< Traits >*			B_body;
	Point< Traits >*		point;
	vector_type				plane_normal; 
	vector_type				plane_position;
};

} // namespace partix

#endif // PARTIX_CONSTRAINT_HPP
