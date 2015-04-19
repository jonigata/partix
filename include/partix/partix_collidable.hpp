/*!
  @file		partix_collidable.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: partix_collidable.hpp 56 2007-04-05 10:04:29Z hirayama $
*/
#ifndef PARTIX_COLLIDABLE_HPP
#define PARTIX_COLLIDABLE_HPP

#include "partix_forward.hpp"
#include "partix_geometry.hpp"

namespace partix{

template < class Traits >
class Collidable {
public:
    typedef std::vector<Collidable<Traits>*> collidables_type;
    typedef typename Traits::index_type    index_type;
    typedef std::vector<index_type>    indices_type;
    typedef std::vector<Face<Traits>>   faces_type;

public:
    virtual ~Collidable() {}

    virtual Body<Traits>*     get_body() = 0;
    virtual Cloud<Traits>*    get_cloud() = 0;
    virtual indices_type&     get_indices() = 0;
    virtual faces_type&      get_faces() = 0;
    virtual typename Traits::vector_type get_center() = 0;
    virtual typename Traits::vector_type get_bbmin() = 0;
    virtual typename Traits::vector_type get_bbmax() = 0;
    virtual typename Traits::real_type  get_recommended_grid_size() = 0;
    virtual void       clear_neighbors() = 0;
    virtual void add_neighbor(Collidable<Traits>*) = 0;
    virtual collidables_type&    get_neighbors() = 0;
    virtual bool       pick(
        const typename Traits::vector_type& s0,
        const typename Traits::vector_type& s1,
        typename Traits::real_type& dist) = 0;

    virtual void       mark() = 0;
    virtual void       unmark() = 0;
    virtual bool       marked() = 0;

};

} // namespace partix 

#endif // PARTIX_COLLIDABLE_HPP
