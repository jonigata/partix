/*!
  @file		RayTriangleTester.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: cpu_ray_triangle_tester.hpp 27 2008-10-24 10:00:08Z Naoyuki.Hirayama $
*/
#ifndef RAYTRIANGLETESTER_HPP
#define RAYTRIANGLETESTER_HPP

template < class VectorTraits, int MAX_RAY_COUNT, int MAX_TRIANGLE_COUNT >
class CpuRayTriangleTester {
public:
	typedef typename VectorTraits::real_type		real_type;
	typedef typename VectorTraits::vector_type		vector_type;

	struct Ray {
		vector_type		p0;
		vector_type		p1;
		int				id;
	};

	struct Tri {
		vector_type		v0;
		vector_type		v1;
		vector_type		v2;
		int				id;
	};

	struct Result {
		vector_type		uvt;
		int				triangle_index;
	};

	typedef Ray ray_type;
	typedef Tri triangle_type;
	typedef Result result_type;

public:
	CpuRayTriangleTester() { }
	~CpuRayTriangleTester() { }

	void begin() {}
	void end() {}

	int max_ray_count() { return MAX_RAY_COUNT; }
	int max_triangle_count() { return MAX_TRIANGLE_COUNT; }

	void begin_ray_upstream() { rays_ptr_ = rays_; }
	void end_ray_upstream() {}

	void begin_triangle_upstream() { triangles_ptr_ = triangles_; }
	void end_triangle_upstream() {}

	void add_ray(
		const vector_type& p0,
		const vector_type& p1,
		int id )
	{
		Ray*& rp = rays_ptr_;
		rp->p0 = p0;
		rp->p1 = p1;
		rp->id = id;
		rp++;
	}
	void add_triangle(
		const vector_type& v0,
		const vector_type& v1,
		const vector_type& v2,
		int id )
	{
		Tri*& tp = triangles_ptr_;
		tp->v0 = v0;
		tp->v1 = v1;
		tp->v2 = v2;
		tp->id = id;
		tp++;
	}

	template < class OutputArray >
	void do_test( OutputArray& output )
	{
		apply( rays_, rays_ptr_, triangles_, triangles_ptr_, output );
	}

private:
	// CPU_VERSION
	template < class RayIterator,
			   class TriangleIterator,
			   class OutputArray >
	void apply( 
		RayIterator				rb,
		RayIterator				re,
		TriangleIterator		tb,
		TriangleIterator		te,
		OutputArray&			output )
	{
		for( int row = 0 ; tb != te ; row++ ) {
			Tri t = *tb++;

			RayIterator ri = rb;

			for( int col = 0 ; ri != re ; col++ ) {
				Ray r = *ri++;

				Result result;
				result.uvt.z = (std::numeric_limits< real_type >::max)();
				result.triangle_index = -1;

				if( r.id == t.id ) {
					if( row == 0 ) { output[col] = result; }
					continue;
				}
								
				line_triangle_intersection(
					result.uvt,
					r.p0, r.p1, 
					t.v0, t.v1, t.v2 );
										
				if( row == 0 || result.uvt.z < output[col].uvt.z ) {
					result.triangle_index = row;
					output[col] = result;
				}
			}
		}
	}

	bool line_triangle_intersection(
		vector_type& uvt,
		const vector_type& r0,
		const vector_type& r1,
		const vector_type& v0,
		const vector_type& v1,
		const vector_type& v2 )
	{
		const real_type epsilon = VectorTraits::epsilon(); // real_type( 0.000001f );

		vector_type dir = r1 - r0;

		/* find vectors for two edges sharing vert0 */
		vector_type e1 = v1 - v0;
		vector_type e2 = v2 - v0;
				
		/* begin calculating determinant -
		   also used to calculate U parameter */
		vector_type pvec = cross( dir, e2 );

		/* if determinant is near zero,
		   ray lies in plane of triangle */
		float det = dot( e1, pvec );

		if ( det < epsilon )
			return false;

		/* calculate distance from vert0 to ray origin */
		vector_type tvec = r0 - v0;

		/* calculate U parameter and test bounds */
		real_type u = dot( tvec, pvec );
		if ( u < 0 || u > det )
			return false;

		/* prepare to test V parameter */
		vector_type qvec = cross( tvec, e1 );

		/* calculate V parameter and test bounds */
		real_type v = dot( dir, qvec );
		if ( v < 0 || u + v > det)
			return false;
				
		/* calculate t, scale parameters, ray intersects triangle */
		float t = dot( e2, qvec );
		if( t < 0 || det < t ) 
			return false;

		real_type idet = real_type(1) / det;
		VectorTraits::x( uvt, u * idet );
		VectorTraits::y( uvt, v * idet );
		VectorTraits::z( uvt, t * idet );

		return true;
	}

	vector_type cross( const vector_type& u, const vector_type& v )
	{
		real_type ux = VectorTraits::x( u );
		real_type uy = VectorTraits::y( u );
		real_type uz = VectorTraits::z( u );

		real_type vx = VectorTraits::x( v );
		real_type vy = VectorTraits::y( v );
		real_type vz = VectorTraits::z( v );

		return VectorTraits::make_vector(
			uy * vz - uz * vy,
			-ux * vz + uz * vx,
			ux * vy - uy * vx );
	}

	real_type dot( const vector_type& u, const vector_type& v )
	{
		real_type ux = VectorTraits::x( u );
		real_type uy = VectorTraits::y( u );
		real_type uz = VectorTraits::z( u );

		real_type vx = VectorTraits::x( v );
		real_type vy = VectorTraits::y( v );
		real_type vz = VectorTraits::z( v );

		return ux * vx + uy * vy + uz * vz;
	}

private:
	Tri		triangles_[MAX_TRIANGLE_COUNT];
	Ray		rays_[MAX_RAY_COUNT];
	Tri*	triangles_ptr_;
	Ray*	rays_ptr_;

};

#endif // RAYTRIANGLETESTER_HPP
