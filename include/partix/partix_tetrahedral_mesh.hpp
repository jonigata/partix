/*!
  @file		partix_tetrahedral_mesh.hpp
  @brief	<äTóv>

  <ê‡ñæ>
  $Id: partix_tetrahedral_mesh.hpp 252 2007-06-23 08:32:33Z naoyuki $
*/
#ifndef PARTIX_TETRAHEDRAL_MESH_HPP
#define PARTIX_TETRAHEDRAL_MESH_HPP

#include "partix_forward.hpp"
#include "partix_cloud.hpp"
#include "partix_geometry.hpp"
#include <map>

namespace partix {

template <class Traits>
class TetrahedralMesh {
public:
    typedef typename Traits::vector_traits vector_traits;
    typedef typename Traits::real_type  real_type;
    typedef typename Traits::vector_type vector_type;
    typedef typename Traits::index_type  index_type;
    typedef std::vector<index_type>  indices_type;
    typedef Volume<Traits>    volume_type;
    typedef Cloud<Traits>     cloud_type;
    typedef Point<Traits>     point_type;
    typedef std::vector<point_type>  points_type;
    typedef Face<Traits>     face_type;
    typedef Tetrahedron<Traits>   tetrahedron_type;

    struct edge_type {
        Edge<Traits> indices;
        bool   border;
        real_type  t; //("not collided"-"collided")ÇÃåWêî
        real_type  u;
        real_type  v;
        real_type  w;
        vector_type  collision_normal;
    };

    typedef std::vector<edge_type>  edges_type;
    typedef std::vector<face_type>  faces_type;
    typedef std::vector<tetrahedron_type> tetrahedra_type;

public:
    TetrahedralMesh() {
        volume_ = NULL;
        cloud_ = new Cloud<Traits>;
    }
    ~TetrahedralMesh() { delete cloud_; }

    void add_point(const vector_type& v, real_type mass) {
        cloud_->add_point(v, mass);
    }
    void add_face(index_type i0, index_type i1, index_type i2) {
        face_type f; f.i0 = i0; f.i1 = i1; f.i2 = i2;
        f.id = int(faces_.size());
        faces_.push_back(f);
    }
    void add_tetrahedron(
        index_type i0,
        index_type i1,
        index_type i2,
        index_type i3) {
        tetrahedron_type t;
        t.i0 = i0;
        t.i1 = i1;
        t.i2 = i2;
        t.i3 = i3; 
        tetrahedra_.push_back(t);
    }
    
    void setup() {
        {
            edges_.clear();
            
            std::map<index_type, std::set<index_type>> s;
            for (typename tetrahedra_type::const_iterator i =
                     tetrahedra_.begin();
                 i != tetrahedra_.end();
                 ++i) {
                const tetrahedron_type& t = *i;
                make_edge(s, t.i0, t.i1);
                make_edge(s, t.i0, t.i2);
                make_edge(s, t.i0, t.i3);
                make_edge(s, t.i1, t.i2);
                make_edge(s, t.i1, t.i3);
                make_edge(s, t.i2, t.i3);
            }
        }
        {
            std::set<index_type> s;
            for (typename tetrahedra_type::const_iterator i =
                     tetrahedra_.begin();
                 i != tetrahedra_.end();
                 ++i) {
                const tetrahedron_type& t = *i;
                s.insert(t.i0);
                s.insert(t.i1);
                s.insert(t.i2);
                s.insert(t.i3);
            }

            indices_.clear();
            for (typename std::set<index_type>::const_iterator i =
                     s.begin();
                 i != s.end();
                 ++i) {
                indices_.push_back(*i);
            }
        }
        {
            points_type& points = cloud_->get_points();

            average_edge_length_ = 0;
            for (typename edges_type::const_iterator i =
                     edges_.begin();
                 i != edges_.end();
                 ++i) {
                const edge_type& e = *i;
                index_type ei0 = e.indices.i0;
                index_type ei1 = e.indices.i1;

#ifdef _WINDOWS
                if (ei0 <0 || int(points.size()) <= ei0) {
                    DebugBreak();
                }
                if (ei1 <0 || int(points.size()) <= ei1) {
                    DebugBreak();
                }
#endif
                const point_type& v0 = points[ei0];
                const point_type& v1 = points[ei1];

                average_edge_length_ += vector_traits::length(
                    v0.source_position - v1.source_position
                    );
            }
            average_edge_length_ /= real_type(edges_.size());
        }
    }

    real_type get_average_edge_length() {
        return average_edge_length_;
    }

    vector_type calculate_center() {
        vector_type center = math<Traits>::vector_zero();
        vector_type e = math<Traits>::vector_zero();
        real_type total_mass = 0;
        real_type mass_e = 0;

        for (const point_type& p: get_points()) {
            if (!p.surface) { continue; }

            vector_type mv = p.new_position * p.mass;

            math<Traits>::mount(center, e, mv);
            math<Traits>::mount(total_mass, mass_e, p.mass);
        }

        center *= real_type(1.0)/ total_mass;
        return center;
    }

    void calculate_Aqq(real_type* Aqq) {
        real_type m[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, };
        real_type e[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, };
        for (const point_type& p: get_points()) {
            if (!p.surface) { continue; }

            const vector_type& q = p.ideal_offset;

            real_type qx = vector_traits::x(q);
            real_type qy = vector_traits::y(q);
            real_type qz = vector_traits::z(q);

            real_type mass = p.mass;

            math<Traits>::mount(m[0], e[0], mass * qx * qx);
            math<Traits>::mount(m[1], e[1], mass * qx * qy);
            math<Traits>::mount(m[2], e[2], mass * qx * qz);
            math<Traits>::mount(m[3], e[3], mass * qy * qx);
            math<Traits>::mount(m[4], e[4], mass * qy * qy);
            math<Traits>::mount(m[5], e[5], mass * qy * qz);
            math<Traits>::mount(m[6], e[6], mass * qz * qx);
            math<Traits>::mount(m[7], e[7], mass * qz * qy);
            math<Traits>::mount(m[8], e[8], mass * qz * qz);
        }

        math<Traits>::inverse_matrix(Aqq, m);
    }

    void calculate_Apq(real_type* Apq, const vector_type& current_center) {
        real_type e[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        for (const point_type& point: get_points()) {
            if (!point.surface) { continue; }

            point.check();

            vector_type p = point.new_position - current_center;
            const vector_type& q = point.ideal_offset;

            real_type px = vector_traits::x(p);
            real_type py = vector_traits::y(p);
            real_type pz = vector_traits::z(p);
            real_type qx = vector_traits::x(q);
            real_type qy = vector_traits::y(q);
            real_type qz = vector_traits::z(q);

            real_type m = point.mass;

            math<Traits>::mount(Apq[0], e[0], m * px * qx);
            math<Traits>::mount(Apq[1], e[1], m * px * qy);
            math<Traits>::mount(Apq[2], e[2], m * px * qz);
            math<Traits>::mount(Apq[3], e[3], m * py * qx);
            math<Traits>::mount(Apq[4], e[4], m * py * qy);
            math<Traits>::mount(Apq[5], e[5], m * py * qz);
            math<Traits>::mount(Apq[6], e[6], m * pz * qx);
            math<Traits>::mount(Apq[7], e[7], m * pz * qy);
            math<Traits>::mount(Apq[8], e[8], m * pz * qz);
        }
    }

    cloud_type*    get_cloud() { return cloud_; }
    points_type&   get_points() { return cloud_->get_points(); }
    edges_type&    get_edges() { return edges_; }
    faces_type&    get_faces() { return faces_; }
    tetrahedra_type&  get_tetrahedra() { return tetrahedra_; }
    indices_type&   get_indices() { return indices_; }

    void   set_volume(volume_type* p) { volume_ = p; }
    volume_type* get_volume() { return volume_; }

private:
    TetrahedralMesh(const TetrahedralMesh&) {}
    void operator =(const TetrahedralMesh&) {}

private:
    void make_edge(
        std::map<index_type, std::set<index_type>>& s,
        index_type i0,
        index_type i1) {
        if (i1 <i0) { std::swap(i0, i1); }

        std::set<index_type>& ss = s[i0];
        if (ss.find(i1) == ss.end()) {
            edge_type e;
            e.indices.i0 = i0;
            e.indices.i1 = i1;
            e.border = false;
            e.collision_normal = math<Traits>::vector_zero();
            edges_.push_back(e);
            ss.insert(i1);
        }
    }

private:
    volume_type* volume_;
    cloud_type*  cloud_;
    edges_type  edges_;
    faces_type  faces_;
    indices_type indices_;
    tetrahedra_type tetrahedra_;
    real_type  average_edge_length_;
		

    //template < class T > friend class SoftVolume;
    //template < class T > friend class VertexTetrahedronSpatialHash;
};

};

#endif // PARTIX_TETRAHEDRAL_MESH_HPP
