/*!
  @file		partix_body.hpp
  @brief	<ŠT—v>

  <à–¾>
  $Id: partix_body.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_BODY_HPP
#define PARTIX_BODY_HPP

#include "aabb_tree.hpp"
#include "partix_forward.hpp"

namespace partix {

template <class Traits>
class Body {
public:
    typedef typename Traits::vector_traits vector_traits;
    typedef typename Traits::real_type  real_type;
    typedef typename Traits::vector_type vector_type;
    typedef typename Traits::matrix_type matrix_type;
    typedef typename Traits::body_load_type body_load_type;
    typedef std::vector<Body<Traits>*> bodies_type;

public:
    Body() {
        id_    = -1;
        auto_freezing_ = false;
        frozen_   = false;
        defrosting_  = false;
        global_force_ = force_ = math<Traits>::vector_zero();
        dump_   = 0.0f;
        drag_   = Traits::speed_drag_coefficient();
        alive_   = true;
        positive_  = true;
        influential_ = true;
    }
    virtual ~Body() {}

    virtual int classid() = 0;

    virtual void regularize() = 0;
    virtual void list_collision_units(std::vector<Collidable<Traits>*>&) = 0;

    virtual void move(const vector_type& v) = 0;
    virtual void teleport(const vector_type& v) = 0;
    virtual void add_force(const vector_type& v) = 0;

    virtual void begin_frame() = 0;
    virtual void compute_motion(real_type pdt, real_type dt, real_type idt) = 0;
    virtual void match_shape() = 0;
    virtual void restore_shape(real_type dt, real_type idt, int kmax) = 0;
    virtual void update_display_matrix() = 0;
    virtual void update_frozen(real_type dt, real_type idt) = 0;
    virtual void end_frame() = 0;

    virtual void update_boundingbox() = 0;
    virtual vector_type get_initial_center() = 0; 
    virtual vector_type get_current_center() = 0;

    virtual Volume<Traits>* as_volume() { return nullptr; }

    virtual void dump() {}
    virtual void debug_check() {}

    void set_id(int id) { id_ = id; }
    int  get_id() { return id_; }

    void set_features(bool alive, bool positive, bool influential) {
        set_alive(alive);
        set_positive(positive);
        set_influential(influential);
    }

    void set_alive(bool f) { alive_ = f; }
    bool get_alive() { return alive_; }

    void set_positive(bool f) { positive_ = f; }
    bool get_positive() { return positive_; }

    void set_influential(bool f) { influential_ = f; }
    bool get_influential() { return influential_; }

    void set_auto_freezing(bool f) { auto_freezing_ = f; }
    bool get_auto_freezing() { return auto_freezing_; }

    void set_global_force(const vector_type& v) { global_force_ = v; }
    vector_type get_global_force() { return global_force_; }

    void set_force(const vector_type& v) { force_ = v; }
    vector_type get_force() { return force_; }

    void set_drag_factor(real_type x) { drag_ = x; }
    real_type get_drag_factor() { return drag_; }

    void set_internal_dump_factor(real_type x) { dump_ = x; }
    real_type get_internal_dump_factor() { return dump_; }

    bodies_type& get_actual_contact_list() { return actual_contact_list_; }

    bool get_frozen() { return frozen_; }
    void set_frozen(bool f) { frozen_ = f; }

    bool get_defrosting() { return defrosting_; }
    void set_defrosting(bool f) { defrosting_ = f; }

    body_load_type load;

private:
    int    id_;
    vector_type  global_force_;
    vector_type  force_;
    real_type  dump_;
    real_type  drag_;
    bodies_type  actual_contact_list_;
    bool   auto_freezing_;
    bool   frozen_;
    bool   defrosting_;
    bool   alive_;
    bool   positive_;
    bool   influential_;

};

} // namespace partix

#endif // PARTIX_BODY_HPP
