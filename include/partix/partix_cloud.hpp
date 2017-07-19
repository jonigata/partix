/*!
  @file		partix_cloud.hpp
  @brief	<概要>

  <説明>
  $Id: partix_cloud.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_CLOUD_HPP
#define PARTIX_CLOUD_HPP

#include "partix_point.hpp"
#include "partix_math.hpp"

namespace partix {

// Cloud
template < class Traits >
class Cloud {
public:
    typedef typename Traits::vector_traits   vector_traits;
    typedef Point<Traits>       point_type;
    typedef std::vector<point_type>     points_type;
    typedef typename Traits::index_type    index_type;
    typedef std::vector<index_type>     indices_type;
    typedef typename Traits::real_type    real_type;
    typedef typename Traits::vector_type   vector_type;
    typedef typename Traits::matrix_type   matrix_type;
    typedef typename Traits::cloud_load_type  load_type;

public:
    Cloud() {}
    ~Cloud() {}

    void add_point(const vector_type& v, real_type mass) {
        vector_type v0 = math<Traits>::vector_zero();
        point_type p;
        p.id      = int(points_.size());
        p.source_position           = v;
        p.old_position              = v;
        p.new_position              = v;
        p.future_position           = v;
        p.ideal_offset              = v0;
        p.velocity                  = v0;
        p.forces                    = v0;
        p.constraint_pushout        = v0;
        p.active_contact_pushout    = v0;
        p.passive_contact_pushout   = v0;
        p.friction_vector           = v0;
        p.penetration_vector        = v0;
        p.view_vector0              = v0;
        p.view_vector1              = v0;
        p.friction                  = Traits::kinetic_friction();
        p.mass                      = mass;
        p.invmass                   = 1.0f / mass;
        p.collided                  = false;
        p.free_position             = v;
        points_.push_back(p);
    }
    points_type& get_points() { return points_; }

    load_type  load;

public:
    void compute_motion(
        real_type pdt,
        real_type dt,
        real_type idt,
        const vector_type& local_force,
        const vector_type& global_force,
        real_type drag_coefficient,
        real_type dump_factor) {
        // google: "Time-Corrected Verlet"

        // 以下p.velocityは基本的にテンポラリ/ユーザ参照用で
        // 積分の入力としては使っていない

        vector_type v0 = math<Traits>::vector_zero();

        for (typename points_type::iterator i = points_.begin();
             i != points_.end();
             ++i) {

            point_type& p = *i;

            // pushoutのリセット
            p.constraint_pushout = v0;
            p.active_contact_pushout = v0;
            p.passive_contact_pushout = v0;

            // 速度
            p.velocity =(p.new_position - p.old_position)* pdt * idt;
            p.old_position = p.new_position;
            p.energy = math<Traits>::length_sq(p.velocity)* p.mass * 0.5f;

            // 力
            vector_type accel =
                (p.forces + local_force)*(dt * p.invmass)+
                global_force * dt;
            p.forces = v0;
            p.velocity += accel;

            // 圧力dumping(adhoc, 静止に利用)
            //real_type dump_factor = 1.0f;

            // 抗力
            real_type velocity_length_sq =
                math<Traits>::length_sq(p.velocity);
            real_type drag = velocity_length_sq * drag_coefficient;
            real_type drag_factor = 1.0f - drag;
            if (drag_factor <0) { drag_factor = 0; }

            p.velocity *= drag_factor * dump_factor;
            p.tmp_velocity = p.velocity * dt;
            p.new_position = p.old_position + p.tmp_velocity;
        }
    }

    void update_bb(vector_type& bbmin, vector_type& bbmax) {
        for (typename points_type::iterator i = points_.begin();
             i != points_.end();
             ++i) {
            point_type& p = *i;
            math<Traits>::update_bb(bbmin, bbmax, p.new_position);
        }
    }

    void move(const vector_type& v) {
        for (typename points_type::iterator j = points_.begin();
             j != points_.end();
             ++j) {
            point_type& p = *j;
            vector_type d = p.new_position - p.old_position;
            p.new_position += v;
        }
    }

    void teleport(const vector_type& v ) {
        for (typename points_type::iterator j = points_.begin();
             j != points_.end();
             ++j) {
            point_type& p = *j;
            vector_type d = p.new_position - p.old_position;
            p.new_position += v;
            p.old_position = p.new_position - d;
        }
    }

    void set_orientation(const matrix_type& m) {
        for (typename points_type::iterator j = points_.begin();
             j != points_.end();
             ++j) {
            point_type& point = *j;

            point.new_position =
                Traits::transform_vector(m, point.initial_offset)+
                this->current_center_;
        }
    }

    void rotate(const quaternion<real_type>& q, const quaternion<real_type>& rq, const vector_type& center) {
        for (typename points_type::iterator j = points_.begin();
             j != points_.end();
             ++j) {
            point_type& point = *j;

            vector_type v = point.new_position - center;
            quaternion<real_type> t(real_type(1.0), v.x, v.y, v.z);
            t = q * t * rq;
            v = vector_type(t.x, t.y, t.z);
            v += center;

            point.new_position = v;
        }
    }

    void rotate_teleportal(const quaternion<real_type>& q,
                           const quaternion<real_type>& rq,
                           const vector_type& center) {
        for (typename points_type::iterator i = points_.begin();
             i != points_.end();
             ++i) {
            point_type& p = *i;

            vector_type v = p.new_position - center;
            quaternion<real_type> t(real_type(1.0), v.x, v.y, v.z);
            t = q * t * rq;
            v = vector_traits::make_vector(t.x, t.y, t.z);
            v += center;

            vector_type d = p.new_position - p.old_position;
            p.new_position = v;
            p.old_position = v - d;
            p.check();
        }
    }

    void kill_inertia() {
        for (typename points_type::iterator i = points_.begin();
             i != points_.end();
             ++i) {
            point_type& p = *i;
            p.old_position = p.new_position;
        }
    }

    void reset_rotation(const vector_type& current_center, const vector_type& initial_center) {
        for (typename points_type::iterator i = points_.begin();
             i != points_.end();
             ++i) {
            point_type& p = *i;
            p.old_position = p.new_position = current_center - initial_center + p.source_position;
        }
    }

private:
    std::vector<point_type>  points_;

};

} // namespace partix

#endif // PARTIX_CLOUD_HPP
