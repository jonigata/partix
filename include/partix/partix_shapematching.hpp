// 2017/06/20 Naoyuki Hirayama

/*!
	@file	  partix_shapematching.hpp
	@brief	  <概要>

	<説明>
*/

#ifndef PARTIX_SHAPEMATCHING_HPP_
#define PARTIX_SHAPEMATCHING_HPP_

namespace partix {

template <class R, class Traits>
class ShapeMatching {
    typedef typename Traits::vector_traits  vector_traits;
    typedef TetrahedralMesh<Traits>         mesh_type;
    typedef math<Traits>                    mat;
    typedef typename Traits::real_type      real_type;
    typedef typename Traits::vector_type    vector_type;
    typedef typename mesh_type::point_type  point_type;

public:
    static vector_type calculate_center(mesh_type* mesh) {
        vector_type center = mat::vector_zero();
        vector_type e = mat::vector_zero();
        real_type total_mass = 0;
        real_type mass_e = 0;

        for (const point_type& p: mesh->get_points()) {
            if (!p.surface) { continue; }

            vector_type mv = R::retreive_position(p) * p.mass;

            mat::mount(center, e, mv);
            mat::mount(total_mass, mass_e, p.mass);
        }

        center *= real_type(1) / total_mass;
        return center;
    }

private:
    static void calculate_Aqq(mesh_type* mesh, real_type* Aqq) {
        real_type m[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, };
        real_type e[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, };
        for (const point_type& p: mesh->get_points()) {
            if (!p.surface) { continue; }

            const vector_type& q = p.ideal_offset;

            real_type qx = vector_traits::x(q);
            real_type qy = vector_traits::y(q);
            real_type qz = vector_traits::z(q);

            real_type mass = p.mass;

            mat::mount(m[0], e[0], mass * qx * qx);
            mat::mount(m[1], e[1], mass * qx * qy);
            mat::mount(m[2], e[2], mass * qx * qz);
            mat::mount(m[3], e[3], mass * qy * qx);
            mat::mount(m[4], e[4], mass * qy * qy);
            mat::mount(m[5], e[5], mass * qy * qz);
            mat::mount(m[6], e[6], mass * qz * qx);
            mat::mount(m[7], e[7], mass * qz * qy);
            mat::mount(m[8], e[8], mass * qz * qz);
        }

        mat::inverse_matrix(Aqq, m);
    }

    static void calculate_Apq(
        mesh_type* mesh, real_type* Apq, const vector_type& center) {
        real_type e[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        for (const point_type& point: mesh->get_points()) {
            if (!point.surface) { continue; }

            point.check();

            vector_type p = R::retreive_position(point) - center;
            const vector_type& q = point.ideal_offset;

            real_type px = vector_traits::x(p);
            real_type py = vector_traits::y(p);
            real_type pz = vector_traits::z(p);
            real_type qx = vector_traits::x(q);
            real_type qy = vector_traits::y(q);
            real_type qz = vector_traits::z(q);

            real_type m = point.mass;

            mat::mount(Apq[0], e[0], m * px * qx);
            mat::mount(Apq[1], e[1], m * px * qy);
            mat::mount(Apq[2], e[2], m * px * qz);
            mat::mount(Apq[3], e[3], m * py * qx);
            mat::mount(Apq[4], e[4], m * py * qy);
            mat::mount(Apq[5], e[5], m * py * qz);
            mat::mount(Apq[6], e[6], m * pz * qx);
            mat::mount(Apq[7], e[7], m * pz * qy);
            mat::mount(Apq[8], e[8], m * pz * qz);
        }
    }

    static void extract_rotation(
        const real_type A[9], quaternion<real_type>& q) {
        const real_type epsilon = vector_traits::epsilon();
        const real_type r1 = real_type(1);

        // dprintf("iter: ");

        const unsigned int max_iter = 10;
        for (unsigned int i = 0 ; i <max_iter ; i++) {
            real_type R[9];
            q.make_matrix(R);

            vector_type R0 = vector_traits::make_vector(R[0], R[1], R[2]);
            vector_type R1 = vector_traits::make_vector(R[3], R[4], R[5]);
            vector_type R2 = vector_traits::make_vector(R[6], R[7], R[8]);
            vector_type A0 = vector_traits::make_vector(A[0], A[1], A[2]);
            vector_type A1 = vector_traits::make_vector(A[3], A[4], A[5]);
            vector_type A2 = vector_traits::make_vector(A[6], A[7], A[8]);

            vector_type R0xA0 = mat::cross(R0, A0);
            vector_type R1xA1 = mat::cross(R1, A1);
            vector_type R2xA2 = mat::cross(R2, A2);

            float R0dotA0 = mat::dot(R0, A0);
            float R1dotA1 = mat::dot(R1, A1);
            float R2dotA2 = mat::dot(R2, A2);
            float ww = std::abs(R0dotA0 + R1dotA1 + R2dotA2)+ epsilon;

            vector_type omega =(R0xA0 + R1xA1 + R2xA2)*(r1 / ww);

            real_type w = mat::length(omega);
            // dprintf("%f ", w);
            if (w < epsilon) {
                break;
            }
            vector_type o = omega *(r1 / w);
            q = quaternion<real_type>::angle_axis(w, o.x, o.y, o.z)* q;
            q = q.normalize();
        }
        // dprintf("\n");
    }

public:
    static void calculate_deformed_matrices(
        mesh_type* mesh, 
        quaternion<real_type>& q,
        real_type* R,
        real_type* G,
        const vector_type& center,
        const real_type* Aqq) {

        // 「あるべき点」にひっぱられる
        real_type Apq[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        calculate_Apq(mesh, Apq, center);

        extract_rotation(Apq, q);

        q.make_matrix(R);
        
        // real_type stretch_factor = stretch_factor_;
        real_type stretch_factor = 0.5;

        real_type A[9];
        math<Traits>::multiply_matrix(A, Apq, Aqq);
        real_type detA = math<Traits>::determinant_matrix(A);

        real_type cbrt = pow(std::abs(detA), real_type(1.0/3.0));

        real_type Adash[9];
        math<Traits>::multiply_matrix(Adash, A, real_type(1.0) / cbrt);

        real_type G0[9];
        real_type G1[9];
        math<Traits>::multiply_matrix(G0, Adash, stretch_factor);
        math<Traits>::multiply_matrix(G1, R, real_type(1) - stretch_factor);

        math<Traits>::add_matrix(G, G0, G1);
    }
    

};

}

#endif // PARTIX_SHAPEMATCHING_HPP_
