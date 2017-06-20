// 2014/09/22 Naoyuki Hirayama

#ifndef TRAITS_HPP_
#define TRAITS_HPP_

#include "Geometry.hpp"
#include "partix/partix.hpp"
#include "FileLogger.hpp"
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
        float weight;
        float friction;
        int fix_target; // boolだとinteroperabilityが怪しい
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
 * EntityFeatures
 *
 * 
 */
/*==========================================================================*/

struct EntityFeatures {
    float stretch_factor;
    float restore_factor;
    int alive;                  // 本当はbool, interoperabilityの都合
    int positive;               // 本当はbool, interoperabilityの都合
    int influential;            // 本当はbool, interoperabilityの都合
    int auto_freezing;          // 本当はbool, interoperabilityの都合
    int frozen;                 // 本当はbool, interoperabilityの都合
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

typedef std::shared_ptr<tetra_type>             tetra_ptr;
typedef std::shared_ptr<body_type>              body_ptr;
typedef std::shared_ptr<cloud_type>             cloud_ptr;
typedef std::shared_ptr<block_type>             block_ptr;
typedef std::shared_ptr<softvolume_type>        softvolume_ptr;
typedef std::shared_ptr<softshell_type>         softshell_ptr;

struct Tetrahedron {
    int i0;
    int i1;
    int i2;
    int i3;
};

struct Triangle {
    int i0;
    int i1;
    int i2;
};

inline
Vector centroid(const face_type& f, const points_type& points) {
    const Vector& v00 = points[f.i0].new_position;
    const Vector& v01 = points[f.i1].new_position;
    const Vector& v02 = points[f.i2].new_position;
    return (v00 + v01 + v02) / 3.0f;
}
    
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
        const char* tcf,
        const vector_type& location, float scale, float mass) {

        // 作成
        softvolume_type* v = make_volume_body(tcf, scale, mass);
        return setup_softvolume(v, location);
    }

    softvolume_ptr add_softvolume(
        int vertexCount, const vector_type* vertices,
        int tetrahedronCount, const Tetrahedron* tetrahedra,
        int faceCount, const Triangle* faces,
        const vector_type& location, float scale, float mass) {

        // 作成
        softvolume_type* v = make_volume_body(
            vertexCount, vertices,
            tetrahedronCount, tetrahedra,
            faceCount, faces,
            scale, mass);
        return setup_softvolume(v, location);
    }

    softshell_ptr add_softshell(
        int             vertex_count,
        const Vector*   vertices,
        int             triangle_count,
        const int*      triangles,
        int             threshold,
        const Vector&   location,
        float           scale,
        float           mass) {
        softshell_type* v = make_shell_body(
            vertex_count, vertices, triangle_count, triangles,
            threshold, scale, mass);
        softshell_ptr e(v);
        e->teleport(location);
        e->set_features(false, false, true);
        bodies_.push_back(e);
        world_->add_body(v);
        return e;
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

    void setup_vehicle(
        softvolume_type* sv,
        float       scale_factor,
        float       mass) {
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
            auto b = sp.z / bbw.z;
            p.load.accel = a * b * scale_factor; 
            p.load.jump = 500.0f;
            p.load.weight = mass;
        }

        sv->update_mass();
    }

private:
    softvolume_type* make_volume_body(
        const char* tcf, float scale, float mass) {
        vector_type v0(0, 0, 0);

        tetra_type* e = new tetra_type;
        meshes_.push_back(tetra_ptr(e));

        std::stringstream ss(tcf);

        printf("A\n");
        // .node(頂点座標)読み込み
        {
            int node_count, dummy;
            ss >> node_count >> dummy >> dummy >> dummy;
            for (int i = 0 ; i <node_count ; i++) {
                vector_type v;
                ss >> dummy >> v.x >> v.y >> v.z;
                v.x *= scale;
                v.y *= scale;
                v.z *= scale;
                e->add_point(v, mass);
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
                e->add_face(i0, i2, i1);
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

    softvolume_type* make_volume_body(
        int vertexCount, const vector_type* vertices,
        int tetrahedronCount, const Tetrahedron* tetrahedra,
        int faceCount, const Triangle* faces,
        float scale, float mass) {

        vector_type v0(0, 0, 0);

        tetra_type* e = new tetra_type;
        tetra_ptr p(e);
        meshes_.push_back(p);
        for (int i = 0 ; i < vertexCount ; i++) {
            vector_type v = vertices[i] * scale;
            e->add_point(v, mass);
        }
        
        for (int i = 0 ; i < tetrahedronCount ; i++) {
            e->add_tetrahedron(
                tetrahedra[i].i0,
                tetrahedra[i].i1,
                tetrahedra[i].i2,
                tetrahedra[i].i3);
        }

        for (int i = 0 ; i < faceCount ; i++) {
            e->add_face(
                faces[i].i0,
                faces[i].i2,
                faces[i].i1);
        }

        e->setup();
        softvolume_type* v = new softvolume_type;
        v->set_stretch_factor(stretch_factor_);
        v->set_mesh(e);
        v->regularize();
        v->set_stretch_factor(stretch_factor_);
        return v;
    }

    softvolume_ptr setup_softvolume(
        softvolume_type* v, const vector_type& location) {
        // 硬さ、摩擦
        v->set_stretch_factor(stretch_factor_);
        v->set_restore_factor(restore_factor_);
        for (auto& p: v->get_mesh()->get_points()) {
            p.friction = friction_;
        }
                                
        // 登録
        softvolume_ptr e(v);
        e->teleport(location);
        e->set_auto_freezing(false);
        bodies_.push_back(e);
        models_.push_back(e);
        world_->add_body(e.get());

        e->set_frozen(false);
        e->set_global_force(gravity_);

        return e;
    }

    softshell_type* make_shell_body(
        int             vertex_count,
        const Vector*   vertices,
        int             triangle_count,
        const int*      triangles,
        int             threshold,
        float           scale,
        float           mass)
    {
        softshell_type* e = new softshell_type;

        // vertex
        cloud_type* c = new cloud_type;
        for (int i = 0 ; i < vertex_count ; i++) {
            const Vector& v = vertices[i];
            c->add_point(v * scale, mass); 
        }
        clouds_.push_back(cloud_ptr(c));
        e->add_cloud(c);

        // index
        block_type* b = new block_type;
        for (int i = 0 ; i < triangle_count ; i++) {
            const Triangle& t = *((const Triangle*)(&triangles[i*3]));
            b->add_face(t.i0, t.i1, t.i2);
        }

        // 分割
        divide_block(threshold, e, c, b);

        return e;
    }

    struct face_compare {
    public:
        face_compare(int a, const cloud_type::points_type& p)
            : axis(a), points(p) {}
        bool operator()(const face_type& f0, const face_type& f1) {
            Vector c0 = centroid(f0, points);
            Vector c1 = centroid(f1, points);

            switch (axis) {
                case 0: return c0.x < c1.x; 
                case 1: return c0.y < c1.y; 
                case 2: return c0.z < c1.z; 
                default: assert(0); return false;
            }
        }
        
        int                             axis;
        const cloud_type::points_type&  points;
    };
    
    void divide_block(
        int             threshold,
        softshell_type* body,
        cloud_type*     c,
        block_type*     b )
    {
        typename block_type::faces_type& faces = b->get_faces();
        
        int n = int(faces.size());

        if (n <threshold) {
            if (b->get_faces().empty()) {
                delete b;
            } else {
                b->set_cloud(c);
                b->set_body(body); 
                b->setup();
                body->add_block(b);

                block_ptr bp(b);
                blocks_.push_back(bp);
            }
            return;
        }
        
        points_type& points = c->get_points();

        // bounding box
        float float_max = (std::numeric_limits<float>::max)();
        Vector bbmin(  float_max,  float_max,  float_max );
        Vector bbmax( -float_max, -float_max, -float_max );
        for (const face_type& face: faces) {
            update_bb(bbmin, bbmax, centroid(face, points));
        }

        // longest axis
        int axis;
        Vector bbw = bbmax - bbmin;
        if( bbw.y <= bbw.x && bbw.z <= bbw.x ) {
            axis = 0;
        } else if( bbw.z <= bbw. y ) {
            axis = 1;
        } else {
            axis = 2;
        }
          
        // sort along axis
        std::sort(
            faces.begin(),
            faces.end(),
            face_compare(axis, c->get_points()));

        // divide
        block_type* b1 = new block_type;

        for (int i = n/2 ; i <n ; i++) {
            b1->add_face(faces[i].i0, faces[i].i1, faces[i].i2); 
        }
        faces.erase(faces.begin()+ n/2, faces.end());

        divide_block(threshold, body, c, b);
        divide_block(threshold, body, c, b1);
    }

    void update_bb(
        Vector& bbmin,
        Vector& bbmax,
        const Vector& v) {
        if (v.x <bbmin.x) { bbmin.x = v.x; }
        if (v.y <bbmin.y) { bbmin.y = v.y; }
        if (v.z <bbmin.z) { bbmin.z = v.z; }
        if (bbmax.x <v.x) { bbmax.x = v.x; }
        if (bbmax.y <v.y) { bbmax.y = v.y; }
        if (bbmax.z <v.z) { bbmax.z = v.z; }
    }

private:
    std::unique_ptr<world_type> world_;
    std::vector<tetra_ptr>      meshes_;
    std::vector<body_ptr>       bodies_;    // 全部
    std::vector<softvolume_ptr> models_;    // figure系だけ 
    std::vector<cloud_ptr>      clouds_;
    std::vector<block_ptr>      blocks_;

    vector_type gravity_;
    float stretch_factor_;
    float restore_factor_;
    float friction_;
    
};

#endif // TRAITS_HPP_
