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

        typedef typename Traits::vector_traits vt;

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
            const T& x) {
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
		if( vt::x(d) > vt::y(d) && vt::x(d) > vt::z(d) ) { return 0; }
		if( vt::y(d) > vt::z(d) ) { return 1; }
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
                    if( ( vt::x(p->max) + vt::x(p->min) ) < ( vt::x(q->max) + vt::x(q->min) ) ) {
                        car = p; cdr = q;
                    } else {
                        car = q; cdr = p;
                    }
                    break;
		case 1:
                    if( ( vt::y(p->max) + vt::y(p->min) ) < ( vt::y(q->max) + vt::y(q->min) ) ) {
                        car = p; cdr = q;
                    } else {
                        car = q; cdr = p;
                    }
                    break;
		case 2:
                    if( ( vt::z(p->max) + vt::z(p->min) ) < ( vt::z(q->max) + vt::z(q->min) ) ) {
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
				
		vt::x(min, vt::x(pmin) < vt::x(qmin) ? vt::x(pmin) : vt::x(qmin));
		vt::y(min, vt::y(pmin) < vt::y(qmin) ? vt::y(pmin) : vt::y(qmin));
		vt::z(min, vt::z(pmin) < vt::z(qmin) ? vt::z(pmin) : vt::z(qmin));
				
		vt::x(max, vt::x(pmax) < vt::x(qmax) ? vt::x(qmax) : vt::x(pmax));
		vt::y(max, vt::y(pmax) < vt::y(qmax) ? vt::y(qmax) : vt::y(pmax));
		vt::z(max, vt::z(pmax) < vt::z(qmax) ? vt::z(qmax) : vt::z(pmax));
	}

	bool test_aabb_aabb( aabb_node* a, aabb_node* b )
	{
		return test_aabb_aabb( a->min, a->max, b->min, b->max );
	}

	bool test_aabb_aabb( const vector_type& amin, const vector_type& amax,
						 const vector_type& bmin, const vector_type& bmax ) 
	{
		return !(
                    vt::x(amax) < vt::x(bmin) || vt::x(amin) > vt::x(bmax) ||
                    vt::y(amax) < vt::y(bmin) || vt::y(amin) > vt::y(bmax) ||
                    vt::z(amax) < vt::z(bmin) || vt::z(amin) > vt::z(bmax));
	}

private:
	default_page_provider page_provider_;
	fixed_pool< sizeof( aabb_node ), default_page_provider > pool_;

	aabb_node* root_;

};

} // namespace partix

#endif // AABBTREE_HPP
