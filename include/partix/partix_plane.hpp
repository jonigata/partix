/*!
  @file     partix_plane.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: partix_plane.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_PLANE_HPP
#define PARTIX_PLANE_HPP

namespace partix {

template < class Traits >
class BoundingPlane : public Body< Traits >, public Collidable< Traits > {
public:
    typedef typename Traits::vector_traits  vector_traits;
    typedef typename Traits::real_type      real_type;
    typedef typename Traits::vector_type    vector_type;
    typedef typename Traits::matrix_type    matrix_type;
    typedef typename Traits::index_type     index_type;
    typedef Body< Traits >                  body_type;
    typedef Cloud< Traits >                 cloud_type;
    typedef Collidable< Traits >            collidable_type;
    typedef std::vector< collidable_type* > collidables_type;
    typedef std::vector< index_type >       indices_type;
    typedef std::vector< Face< Traits > >   faces_type;

    BoundingPlane( const vector_type& p, const vector_type& n )
        : position_( p ), normal_( n )
    {
        this->set_features( false, false, true );
    }
    ~BoundingPlane() {}

    // implements Body
    int classid() { return BODY_ID_PLANE; }
        
    void            regularize() {}
    matrix_type     get_world_matrix() { return matrix_type(); }
    void            list_collision_units( collidables_type& s )
    {
        s.push_back( this );
    }

    void            move( const vector_type& ) { }
    void            teleport( const vector_type& ) { }
    void add_force( const vector_type& v ) { }

    void            begin_frame() {}
	void			compute_motion( real_type, real_type, real_type ) {}
    void            update_frozen( real_type, real_type ) {}
    void            match_shape() {}
    void            restore_shape( real_type, real_type, int ) {}
    void            update_display_matrix() {}
    void            update_boundingbox() {}
    vector_type     get_initial_center() { return math< Traits >::vector_zero(); }
    vector_type     get_current_center() { return position_; }
    void            end_frame() {}

    // implements Collidable
    body_type*      get_body() { return this; }
    cloud_type*     get_cloud() { return NULL; }
    indices_type&   get_indices() { return *((indices_type*)NULL); }
    faces_type&     get_faces() { return *((faces_type*)NULL); }
    vector_type     get_center() { return position_; }
    vector_type     get_bbmin() { return math< Traits >::vector_min(); }
    vector_type     get_bbmax() { return math< Traits >::vector_max(); }
    real_type       get_recommended_grid_size() { return real_type( 0.5 ); }

    void clear_neighbors() { neighbors_.clear(); }
    void add_neighbor( collidable_type* p ) { neighbors_.push_back( p ); }
    collidables_type& get_neighbors() { return neighbors_; }
    bool pick(
        const vector_type& s0,
        const vector_type& s1,
        real_type& dist )
    {
        if( 0 < math< Traits >::dot( s1 - s0, normal_ ) ) {
            return false;
        }
        return math< Traits >::test_plane_segment(
            position_, normal_, s0, s1, dist );
    }
    void            mark() {}
    void            unmark() {}
    bool            marked() { return false; }

    // original
    const vector_type& get_position() { return position_; }
    const vector_type& get_normal() { return normal_; }

private:
    BoundingPlane( const BoundingPlane& ){}
    void operator=( const BoundingPlane& ){}

private:
    vector_type     position_;
    vector_type     normal_;
    std::vector< Collidable< Traits >* >    neighbors_;

    template < class T > friend class World;

};

} // namespace partix

#endif // PARTIX_PLANE_HPP
