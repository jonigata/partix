// Copyright (C) 2004 Naoyuki Hirayama.
// All Rights Reserved.

// $Id: mqoreader.hpp 21 2008-04-28 01:53:54Z Naoyuki.Hirayama $

#if !defined(MQOREADER_HPP)
#define MQOREADER_HPP

// module: 

#include <stdexcept>
#include <string>
#include <map>
#include <vector>
#include <iosfwd>

namespace mqo_reader {

struct mqo_reader_error : public std::runtime_error {
    mqo_reader_error(const std::string& s) : std::runtime_error(s) {}
};

struct color_type {
    float   red;
    float   green;
    float   blue;
    float   alpha;
};

enum shader_type {
    shader_classic  = 0,
    shader_constant = 1,
    shader_lambert  = 2,
    shader_phong    = 3,
    shader_blinn    = 4,
};

enum projection_type {
    projection_uv       = 0,
    projection_plane    = 1,
    projection_cylinder = 2,
    projection_sphere   = 3,
};

struct vector_type {
    float x;
    float y;
    float z;
};

struct projangle_type {
    float heading;
    float pitching;
    float banking;
};

struct material_type {
    std::string     name;
    shader_type     shader;
    bool            vertex_color;
    color_type      color;
    float           diffuse;
    float           ambient;
    float           emissive;
    float           specular;
    float           power;
    std::string     texture;
    std::string     aplane;
    std::string     bump;
    projection_type projection;
    vector_type     proj_pos;
    vector_type     proj_scale;
    projangle_type  proj_angle;
};
typedef std::vector<material_type> materials_type;

enum patch_type {
    patch_plane         = 0,
    patch_spline1       = 1,
    patch_spline2       = 2,
    patch_catmull_clark = 3,
};

enum shading_type {
    shading_flat    = 0,
    shading_gouraud = 1,
};

enum edgecolor_type {
    edgecolor_environment   = 0,
    edgecolor_object        = 1,
};

enum mirror_type {
    mirror_none     = 0,
    mirror_connect  = 1,
    mirror_divide   = 2,
};

enum mirroraxis_type {
    mirroraxis_none = 0,
    mirroraxis_x    = 1,
    mirroraxis_y    = 2,
    mirroraxis_z    = 4,
};

enum lathe_type {
    lathe_none      = 0,
    lathe_double    = 3,
};

enum latheaxis_type {
    latheaxis_x     = 0,
    latheaxis_y     = 1,
    latheaxis_z     = 2,
};

struct vertex_type {
    float x;
    float y;
    float z;
};
typedef std::vector<vertex_type> vertices_type;

struct uv_type {
    float u;
    float v;
};

struct face_type {
    int             vertex_count;
    int             vertex_indices[4];
    int             material_index;
    uv_type         uv[4];
    color_type      colors[4];
};
typedef std::vector<face_type> faces_type;

struct object_type {
    std::string     name;
    int             depth;
    bool            folding;
    vector_type     scale;
    vector_type     rotation;
    vector_type     translation;
    patch_type      patch;
    int             segment;
    bool            visible;
    bool            locking;
    shading_type    shading;
    float           facet;
    color_type      color;
    edgecolor_type  color_type;
    mirror_type     mirror;
    mirroraxis_type mirror_axis;
    float           mirror_dis;
    lathe_type      lathe;
    latheaxis_type  lathe_axis;
    int             lathe_seg;

    vertices_type   vertices;
    faces_type      faces;
};
typedef std::map<std::string, object_type> objdic_type;

struct scene_type {
    color_type      ambient;        
};

struct document_type {
    int major_version;
    int minor_version;

    scene_type      scene;
    objdic_type     objects;
    materials_type  materials;
};

void read_mqo(std::istream&, document_type&);

}

#endif
