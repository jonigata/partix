/*!
  @file		aabbtree.hpp
  @brief	<概要>

  <説明>
  $Id: aabb_tree.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef AABBTREE_HPP
#define AABBTREE_HPP

#include "fixed_pool.hpp"

namespace partix {

template < class Traits, class T >
class aabb_tree {
public:
	typedef typename Traits::real_type		real_type;
	typedef typename Traits::vector_type	vector_type;

	struct aabb_node {
		vector_type		min;
		vector_type		max;
		aabb_node*		car;
		aabb_node*		cdr;
		int				axis;
		T				user;
	};

public:
	aabb_tree() : pool_( page_provider_, "aabbt" ) { root_ = NULL; }
	~aabb_tree() { /* destroy_node( root_ ); */ }

	void clear()
	{
		root_ = NULL;
		pool_.clear();
	}

	void insert(
		const vector_type& min,
		const vector_type& max,
		const T& x )
	{
		//aabb_node* n = new aabb_node;
		aabb_node* n = create_node();
		n->min = min;
		n->max = max;
		n->car = NULL;
		n->cdr = NULL;
		n->user = x;
		if( !root_ )  { root_ = n; return; }
		insert_aux( &root_, n );
	}

	template < class CallBack >
	void detect(
		const vector_type& min,
		const vector_type& max,
		const CallBack& c )
	{
		detect_aux( root_, min, max, c );
	}

private:
	aabb_node* create_node()
	{
		return (aabb_node*)pool_.allocate();
	}

	void destroy_node( aabb_node* p )
	{
		if( !p ) { return; }
		destroy_node( p->car );
		destroy_node( p->cdr );
		pool_.deallocate( p );
	}
		
	template < class CallBack >
	void detect_aux(
		aabb_node* p,
		const vector_type& min,
		const vector_type& max,
		const CallBack& c )
	{
		if( !test_aabb_aabb( p->min, p->max, min, max ) ) { return; }

		if( !p->car /* && !p->cdr */ ) {
			c( p->user );
		} else {
			detect_aux( p->car, min, max, c );
			detect_aux( p->cdr, min, max, c );
		}
	}

	void insert_aux( aabb_node** pp, aabb_node* n )
	{
	  tail_call:
		aabb_node* p = *pp;
		if( !p->car /* && !p->cdr */ ) {
			//aabb_node* q = new aabb_node;
			aabb_node* q = create_node();
			unify_aabb( q->min, q->max, p, n );
			q->axis = get_longest_axis( q->min, q->max );
			split( q->axis, q->car, q->cdr, p, n );
			*pp = q;
			return;
		}

		unify_aabb( p->min, p->max, p, n );
		if( test_aabb_aabb( p->car, n ) ) {
			// 手動で末尾呼び出しの最適化、意味は↓
			// insert_aux( &p->car, n );
			pp = &p->car;
			goto tail_call;
		} else if( test_aabb_aabb( p->cdr, n ) ) {
			// 手動で末尾呼び出しの最適化、意味は↓
			//insert_aux( &p->cdr, n );
			pp = &p->cdr;
			goto tail_call;
		} else {
			aabb_node* car;
			aabb_node* cdr;
			split( p->axis, car, cdr, p, n );
			if( car == n ) {
				assert( cdr == p );
				// 手動で末尾呼び出しの最適化、意味は↓
				//insert_aux( &p->car, n );
				pp = &p->car;
				goto tail_call;
			}
			if( cdr == n ) {
				assert( car == p );
				// 手動で末尾呼び出しの最適化、意味は↓
				//insert_aux( &p->cdr, n );
				pp = &p->cdr;
				goto tail_call;
			}
		}
	}

	int get_longest_axis( const vector_type& min, const vector_type& max )
	{
		vector_type d = max - min;
		if( d.x > d.y && d.x > d.z ) { return 0; }
		if( d.y > d.z ) { return 1; }
		return 2;
	}

	void split(
		int axis,
		aabb_node*& car,
		aabb_node*& cdr,
		aabb_node* p,
		aabb_node* q )
	{
		switch( axis ) {
		case 0:
			if( ( p->max.x + p->min.x ) < ( q->max.x + q->min.x ) ) {
				car = p; cdr = q;
			} else {
				car = q; cdr = p;
			}
			break;
		case 1:
			if( ( p->max.y + p->min.y ) < ( q->max.y + q->min.y ) ) {
				car = p; cdr = q;
			} else {
				car = q; cdr = p;
			}
			break;
		case 2:
			if( ( p->max.z + p->min.z ) < ( q->max.z + q->min.z ) ) {
				car = p; cdr = q;
			} else {
				car = q; cdr = p;
			}
			break;
		default: assert(0);
		}
	}

	void unify_aabb(
		vector_type& min,
		vector_type& max,
		aabb_node* p,
		aabb_node* q )
	{
		const vector_type& pmin = p->min;
		const vector_type& pmax = p->max;
		const vector_type& qmin = q->min;
		const vector_type& qmax = q->max;
				
		min.x = pmin.x < qmin.x ? pmin.x : qmin.x;
		min.y = pmin.y < qmin.y ? pmin.y : qmin.y;
		min.z = pmin.z < qmin.z ? pmin.z : qmin.z;
				
		max.x = pmax.x < qmax.x ? qmax.x : pmax.x;
		max.y = pmax.y < qmax.y ? qmax.y : pmax.y;
		max.z = pmax.z < qmax.z ? qmax.z : pmax.z;
	}

	bool test_aabb_aabb( aabb_node* a, aabb_node* b )
	{
		return test_aabb_aabb( a->min, a->max, b->min, b->max );
	}

	bool test_aabb_aabb( const vector_type& amin, const vector_type& amax,
						 const vector_type& bmin, const vector_type& bmax ) 
	{
		return !( amax.x < bmin.x || amin.x > bmax.x ||
				  amax.y < bmin.y || amin.y > bmax.y ||
				  amax.z < bmin.z || amin.z > bmax.z );
	}

private:
	default_page_provider page_provider_;
	fixed_pool< sizeof( aabb_node ), default_page_provider > pool_;

	aabb_node* root_;

};

} // namespace partix

#endif // AABBTREE_HPP
