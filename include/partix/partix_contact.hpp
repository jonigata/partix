/*!
  @file		partix_contact.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: partix_contact.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_CONTACT_HPP
#define PARTIX_CONTACT_HPP

#include "partix_forward.hpp"

namespace partix {

template < class Traits >
class Contact {
public:
	typedef typename Traits::real_type		real_type;
	typedef typename Traits::vector_type	vector_type;

	Body< Traits >*			A_body;
	Body< Traits >*			B_body;
	Point< Traits >*		A_point;
	Point< Traits >*		B_point0;
	Point< Traits >*		B_point1;
	Point< Traits >*		B_point2;
	real_type				u; // barycentric coordinates for p0
	real_type				v; // barycentric coordinates for p1
	real_type				w; // barycentric coordinates for p2
	real_type				t; // intersect. pos = ap->penetration_vector * t;
	real_type				alpha; // pushout factor

	void check()
	{
#if 0
		if( isnan( u ) || isnan( v ) || isnan( w ) || isnan( alpha ) ){
			char buffer[256];
			sprintf( buffer, "contact: %f, %f, %f\n", u, v, w, alpha );
			OutputDebugStringA( buffer );
			DebugBreak();
		}
#endif
	}

		
};

} // namespace partix

#endif // PARTIX_CONTACT_HPP
