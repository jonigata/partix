// 2014/09/22 Naoyuki Hirayama

#ifndef TRAITS_HPP_
#define TRAITS_HPP_

#include "Geometry.hpp"
#include "partix/partix.hpp"
#include <sstream>
#include <memory>

const float POINT_MASS = 1.0f;

/*===========================================================================*/
/*!
 * VectorTraits
 *
 *  vector traits
 */
/*==========================================================================*/

struct VectorTraits {
    typedef float   real_type;
    typedef Vector  vector_type;

    static real_type epsilon(){ return real_type( 0.000001f ); }
    static real_type x(const vector_type& v) { return v.x; }
    static real_type y(const vector_type& v) { return v.y; }
    static real_type z(const vector_type& v) { return v.z; }
    static void x(vector_type& v, real_type x) { v.x = x; }
    static void y(vector_type& v, real_type y) { v.y = y; }
    static void z(vector_type& v, real_type z) { v.z = z; }
    static vector_type make_vector(real_type x, real_type y, real_type z) {
        return Vector(x, y, z);
    }
    static real_type length_sq(const vector_type& v) {
        return ::length_sq(v);
    }
    static real_type length(const vector_type& v) {
        return ::length(v);
    }
};


/*===========================================================================*/
/*!
 * PartixTraits
 *
 *  partix用のtraitsクラス
 */
/*==========================================================================*/

struct PartixTraits {
    typedef VectorTraits                vector_traits;
    typedef VectorTraits::real_type     real_type;
    typedef VectorTraits::vector_type   vector_type;
    typedef Matrix                      matrix_type;
    typedef int                         index_type;

    struct body_load_type {
        // FourLegs* four_legs;
    };
    struct block_load_type {};
    struct cloud_load_type {};
    struct point_load_type {
        float front_grip;
        float rear_grip;
        float left_grip;
        float right_grip;
        float accel;
        float jump;
    };

    static real_type speed_drag_coefficient() { return  0.0001f; }
    //static float kinetic_friction() { return 40.0f; }
    static real_type kinetic_friction() { return 0.0f; }

    static real_type freeze_threshold_energy() { return 2.0f; }
    static real_type freeze_duration() { return 0.5f; }

    static real_type tick() { return 0.02f; }
    static void make_matrix(
        matrix_type& d,
        const real_type* s,
        const vector_type& t) {
        d.m00 = s[0]; d.m01 = s[3]; d.m02 = s[6]; d.m03 = 0;
        d.m10 = s[1]; d.m11 = s[4]; d.m12 = s[7]; d.m13 = 0;
        d.m20 = s[2]; d.m21 = s[5]; d.m22 = s[8]; d.m23 = 0;
        d.m30 = t.x;  d.m31 = t.y;  d.m32 = t.z;  d.m33 = 1;
    }
    static vector_type transform_vector(
        const matrix_type& m,
        const vector_type& v) {
        return v * m;
    }
};

/*===========================================================================*/
/*!
 * VehicleParameter
 *
 * 
 */
/*==========================================================================*/

struct VehicleParameter {
    float total_grip_coefficient;
    float front_grip_coefficient;
    float rear_grip_coefficient;
    float sensory_balance_speed;
    float sensory_balance_max;
    float sensory_balance_decrease;
    float balance_angle_max;
    float turning_angle_max;
    float backbend_angle_factor;
    float bank_ratio;
    float balance_ratio;
    float turning_ratio;
    float backbend_ratio;
    float brake_angle;
    float friction_factor;
    float accel_factor;
    float turbo_threshold;
    float turbo_multiplier;
};

/*===========================================================================*/
/*!
 * VehicleAnalyzeData
 *
 * 
 */
/*==========================================================================*/

struct VehicleAnalyzeData {
    float   total_grip;
    float   front_grip;
    float   rear_grip;
    float   left_grip;
    float   right_grip;
    float   bottom_grip;
    float sensory_balance;
    Vector  curr_velocity;
    Vector  prev_velocity;
    float   speed;
    float   front_speed;
    Matrix  com;
    Matrix  pom;
    Vector  right;
    Vector  back;
    Vector  front;
    Vector  old_right;
    Vector  old_back;
    Vector  old_front;

    Vector  bbmin;
    Vector  bbmax;
    float   mass;
};

/*===========================================================================*/
/*!
 * PartixWorld
 *
 * 
 */
/*==========================================================================*/

typedef Vector                                  vector_type;
typedef partix::World<PartixTraits>             world_type;
typedef partix::Point<PartixTraits>             point_type;
typedef partix::Cloud<PartixTraits>             cloud_type;
typedef partix::Block<PartixTraits>             block_type;
typedef partix::Body<PartixTraits>              body_type;
typedef partix::SoftShell<PartixTraits>         softshell_type;
typedef partix::SoftVolume<PartixTraits>        softvolume_type;
typedef typename softvolume_type::mesh_type     mesh_type;
typedef typename softvolume_type::points_type   points_type;
typedef partix::BoundingPlane<PartixTraits>     plane_type;
typedef partix::TetrahedralMesh<PartixTraits>   tetra_type;
typedef partix::Face<PartixTraits>              face_type;
typedef partix::math<PartixTraits>              math_type;

typedef std::shared_ptr<body_type>              body_ptr;
typedef std::shared_ptr<cloud_type>             cloud_ptr;
typedef std::shared_ptr<block_type>             block_ptr;
typedef std::shared_ptr<softvolume_type>        softvolume_ptr;

class PartixWorld {
public:
    PartixWorld() {
        gravity_ = vector_type(0, -9.8f, 0);
        stretch_factor_ = 0.7f;
        restore_factor_ = 1.0f;
        friction_ = 0.8f;

        world_.reset(new world_type);
    }

    void restart() {
        world_->restart();
    }

    void update(float delta) {
        world_->update(delta);
    }

    body_ptr add_plane(const vector_type& location, const vector_type& normal) {
        body_ptr e(new plane_type(location, normal));
        bodies_.push_back(e);
        world_->add_body(e.get());
        return e;
    }

    softvolume_ptr add_softvolume(
        const char* tcf, const vector_type& location, float scale) {
        vector_type o = location;

        // 作成
        softvolume_type* v = make_volume_body(tcf, scale);

        // 硬さ、摩擦
        v->set_stretch_factor(stretch_factor_);
        v->set_restore_factor(restore_factor_);
        for (auto& p: v->get_mesh()->get_points()) {
            p.friction = friction_;
        }
                                
        // 登録
        softvolume_ptr e(v);
        e->teleport(o);
        e->set_auto_freezing(false);
        bodies_.push_back(e);
        models_.push_back(e);
        world_->add_body(e.get());

        e->set_frozen(false);
        e->set_global_force(gravity_);

        return e;
    }

    softvolume_ptr add_vehicle(
        const char* tcf, const vector_type& location, float scale) {
        softvolume_ptr p = add_softvolume(tcf, location, scale);
        setup_vehicle(p.get(), POINT_MASS, scale);
        return p;
    }

    void set_gravity(const vector_type& v) {
        gravity_ = v;
        for (auto body: models_) {
            body->set_frozen(false);
            body->set_global_force(gravity_);
        }
    }

    void set_stretch_factor(float value) {
        stretch_factor_ = value;
        for (auto body: models_) {
            body->set_stretch_factor(value);
        }
        printf("stretch_factor: %f\n", value);
    }

    void set_restore_factor(float value) {
        restore_factor_ = value;
        for (auto body: models_) {
            body->set_restore_factor(value);
        }
        printf("restore_factor: %f\n", value);
    }

    void set_friction(float value) {
        friction_ = value;
        for (auto body: models_) {
            for (auto& p: body->get_mesh()->get_points()) {
                p.friction = value;
            }
        }
        printf("friction: %f\n", value);
    }

    softvolume_ptr pick(const vector_type& s0, const vector_type& s1) {
        body_type* found = world_->pick(s0, s1);
        if (found && found->classid() == partix::BODY_ID_SOFTVOLUME) {
            // ちょっと遅いが仕方ない(mapとか作るのも大げさ)
            for (auto p: models_) {
                if (p.get() == found) {
                    return p;
                }
            }
        }
        return softvolume_ptr();
    }

private:
    softvolume_type* make_volume_body(const char* tcf, float mag) {
        vector_type v0(0, 0, 0);

        tetra_type* e = new tetra_type;

        std::stringstream ss(tcf);

        printf("A\n");
        // .node(頂点座標)読み込み
        {
            int node_count, dummy;
            ss >> node_count >> dummy >> dummy >> dummy;
            for (int i = 0 ; i <node_count ; i++) {
                vector_type v;
                ss >> dummy >> v.x >> v.y >> v.z;
                v.x *= mag;
                v.y *= mag;
                v.z *= mag;
                e->add_point(v, POINT_MASS);
            }
        }

        printf("B\n");
        // .ele(tetrahedron)読み込み
        {
            int node_count, dummy;
            ss >> node_count >> dummy >> dummy;
            for (int i = 0 ; i <node_count ; i++) {
                int i0, i1, i2, i3;
                ss >> dummy >> i0 >> i1 >> i2 >> i3;
                e->add_tetrahedron(i0, i1, i2, i3);
            }
        }

        printf("C\n");
        // .face(外接面)読み込み
        {
            int node_count, dummy;
            ss >> node_count >> dummy;
            for (int i = 0 ; i <node_count ; i++) {
                int i0, i1, i2;
                ss >> dummy >> i0 >> i1 >> i2 >> dummy;
                e->add_face(i0, i1, i2);
            }
        }

        printf("D\n");
        e->setup();
        softvolume_type* v = new softvolume_type;

        printf("E\n");
        v->set_mesh(e);

        printf("F\n");
        v->regularize();

        return v;
    }

    void setup_vehicle(
        softvolume_type* sv,
        float       mass,
        float       scale_factor) {
        mass *= scale_factor;

        const float float_max = (std::numeric_limits<float>::max)();

        // bounding box
        mesh_type*   mesh = sv->get_mesh();
        points_type& points = mesh->get_points();
        Vector bbmin(float_max, float_max, float_max);
        Vector bbmax(-float_max, -float_max, -float_max);
        for (point_type& p: points) {
            const Vector& sp = p.source_position;
            if (sp.x < bbmin.x) { bbmin.x = sp.x; }
            if (sp.y < bbmin.y) { bbmin.y = sp.y; }
            if (sp.z < bbmin.z) { bbmin.z = sp.z; }
            if (bbmax.x < sp.x) { bbmax.x = sp.x; }
            if (bbmax.y < sp.y) { bbmax.y = sp.y; }
            if (bbmax.z < sp.z) { bbmax.z = sp.z; }
        }
        Vector bbw = bbmax - bbmin;

        for (point_type& p: points) {
            p.mass = mass;
            const Vector& sp = p.source_position;
            float threshold = bbw.y * 0.02f;
            bool under_threshold = sp.y < threshold;
            bool front = 0.0f < sp.z;
            bool right = 0.0f < sp.x;

            // グリップは足元のみ
            p.load.front_grip = under_threshold && front ? 1.0f : 0;
            p.load.rear_grip =  under_threshold && !front ? 1.0f : 0;
            p.load.left_grip =  under_threshold && right ? 1.0f : 0;
            p.load.right_grip = under_threshold && !right ? 1.0f : 0;

            // アクセル係数
            auto a = 1.0f - sp.y / bbw.y;
            p.load.accel = a * a * mass * scale_factor; 
            p.load.jump = mass * 500.0f;
        }

        sv->update_mass();
    }


private:
    std::unique_ptr<world_type> world_;
    std::vector<body_ptr>       bodies_;    // 全部
    std::vector<softvolume_ptr> models_;    // figure系だけ

    vector_type gravity_;
    float stretch_factor_;
    float restore_factor_;
    float friction_;
    
};

#endif // TRAITS_HPP_
