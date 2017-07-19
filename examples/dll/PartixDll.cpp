// 2017/03/12 Naoyuki Hirayama

#include "stdafx.h"
#include "PartixDll.hpp"
#include "PartixUser.hpp"
#include "FileLogger.hpp"
#include <fstream>

static debug_log_func_type log_func = nullptr;

namespace {

}

void DebugLog(const char* s) {
    if (log_func != nullptr) {
        log_func(s);
    }
    std::ofstream ofs("t.log", std::ios_base::app);
    ofs << s << std::endl;
}

PARTIX_DLL_API void SetDebugLog(debug_log_func_type f) {
    log_func = f;
}

PARTIX_DLL_API void DebugLogTest() {
    DebugLog("DebugLogTest");
}

PARTIX_DLL_API PartixWorld* CreateWorld() {
    PartixWorld* world = new PartixWorld;
    DebugLog("CreateWorld");
    DebugLogPointer(world);
    world->restart();
    return world;
}

PARTIX_DLL_API void DestroyWorld(PartixWorld* world) {
    delete world;
}

PARTIX_DLL_API void UpdateWorld(PartixWorld* world, float delta) {
    world->update(delta);
}

PARTIX_DLL_API void SetGravity(PartixWorld* world, Vector g) {
    world->set_gravity(g);
}

PARTIX_DLL_API softvolume_type* CreateSoftVolume(
    PartixWorld* world, const char* tcf, Vector v, float scale, float mass) {

    softvolume_ptr b = world->add_softvolume(tcf, v, scale, mass);
    return b.get();
}

PARTIX_DLL_API softvolume_type* CreateVehicle(
    PartixWorld* world, const char* tcf, Vector v, float scale, float mass) {
    
    softvolume_ptr b = world->add_softvolume(tcf, v, scale, mass);
    world->setup_vehicle(b.get(), scale, mass);
    return b.get();
}

PARTIX_DLL_API softvolume_type* CreateSoftVolume2(
    PartixWorld* world,
    int vertexCount, const Vector* vertices,
    int tetrahedronCount, const Tetrahedron* tetrahedra,
    int faceCount, const Triangle* faces,
    Vector position, float scale, float mass) {
    if (world == nullptr) { return nullptr; }
    DebugLog("CreateSoftVolume2");
    DebugLogPointer(world);
    softvolume_ptr b = world->add_softvolume(
        vertexCount, vertices,
        tetrahedronCount, tetrahedra,
        faceCount, faces,
        position, scale, mass);
    DebugLog("B");
    return b.get();
}

PARTIX_DLL_API softvolume_type* CreateVehicle2(
    PartixWorld* world,
    int vertexCount, const Vector* vertices,
    int tetrahedronCount, const Tetrahedron* tetrahedra,
    int faceCount, const Triangle* faces,
    Vector position, float scale, float mass) {
    softvolume_ptr b = world->add_softvolume(
        vertexCount, vertices,
        tetrahedronCount, tetrahedra,
        faceCount, faces,
        position, scale, mass);
    world->setup_vehicle(b.get(), scale, mass);
    return b.get();
}

PARTIX_DLL_API softshell_type* CreateSoftShell(
    PartixWorld* world, 
    int vertex_count, const Vector* vertices, 
    int triangle_count, const int* triangles,
    int threshold, Vector location, float scale, float mass) {
    softshell_ptr b = world->add_softshell(
        vertex_count, vertices, triangle_count, triangles,
        threshold, location, scale, mass);
    return b.get();
    return nullptr;
}
    


PARTIX_DLL_API body_type* CreatePlane(
    PartixWorld* world, Vector position, Vector normal) {
    return world->add_plane(position, normal).get();
}

PARTIX_DLL_API void GetPosition(
    PartixWorld* world, body_type* b, Vector* position) {
    *position = b->get_current_center();
}

PARTIX_DLL_API void GetOrientation(
    PartixWorld* world, softvolume_type* b, Matrix* m) {
    *m = b->get_deformed_matrix();
}

PARTIX_DLL_API void GetInitialPosition(
    PartixWorld* world, body_type* b, Vector* position) {
    *position = b->get_initial_center();
}

PARTIX_DLL_API int GetWireFrameVertexCount(
    PartixWorld* world, softvolume_type* b) {
    return (int)b->get_mesh()->get_points().size();
}

PARTIX_DLL_API int GetWireFrameIndexCount(
    PartixWorld* world, softvolume_type* b) {
    return (int)b->get_mesh()->get_edges().size() * 2;
}


PARTIX_DLL_API void GetWireFrameVertices(
    PartixWorld* world, softvolume_type* b, Vector* buffer) {

#if 0
    auto m = b->get_deformed_matrix();
    Vector center(m.m30, m.m31, m.m32);
    for (const auto& p: b->get_mesh()->get_points()) {
        *buffer++ = p.new_position - center;
    }
#else
    auto m = b->get_deformed_matrix();
    Vector center(m.m30, m.m31, m.m32);
    bool future_avail = b->get_future_avail();
    for (const auto& p: b->get_mesh()->get_points()) {
        if (future_avail) {
            *buffer++ = p.future_position - center;
        } else {
            *buffer++ = p.new_position - center;
        }
    }
#endif
}

PARTIX_DLL_API void GetWireFrameIndices(
    PartixWorld* world, softvolume_type* b, int* buffer) {
    for (const auto& e: b->get_mesh()->get_edges()) {
        *buffer++ = e.indices.i0;
        *buffer++ = e.indices.i1;
    }
}

PARTIX_DLL_API void GetPointLoads(
    PartixWorld* world, softvolume_type* b, 
    PartixTraits::point_load_type* buffer) {
    for (const auto& p: b->get_mesh()->get_points()) {
        *buffer = p.load;
        buffer->weight = p.mass;
        buffer->friction = p.friction;
        buffer++;
    }
}

PARTIX_DLL_API void SetPointLoads(
    PartixWorld* world, softvolume_type* b, 
    const PartixTraits::point_load_type* buffer) {
    for (auto& p: b->get_mesh()->get_points()) {
        p.mass = buffer->weight;
        p.friction = buffer->friction;
        p.load = *buffer;
        buffer++;
    }
    b->update_mass();
    Vector c = b->get_initial_center();

    // char logbuffer[256];
    // sprintf(logbuffer, "before: %f, %f, %f\n", c.x, c.y, c.z);
    // DebugLog(logbuffer);

    b->calculate_initial_center();
    c = b->get_initial_center();
    // sprintf(logbuffer, "after: %f, %f, %f\n", c.x, c.y, c.z);
    // DebugLog(logbuffer);

    for (auto& p: b->get_mesh()->get_points()) {
        if (!p.surface) { continue; }
        Vector c = p.source_position;
        // sprintf(logbuffer, "(%f, %f, %f): %f\n", c.x, c.y, c.z, p.mass);
        // DebugLog(logbuffer);
    }
}

PARTIX_DLL_API void AnalyzeVehicle(
    PartixWorld*        world, 
    softvolume_type*	sv,
    VehicleParameter*	vp,
    float		dt,
    Vector              prev_velocity,
    Matrix              prev_orientaion,
    Matrix              curr_orientaion,
    float		sensory_balance,
    VehicleAnalyzeData*	ad) {
    
    // 姿勢・スピード
    ad->pom = prev_orientaion; normalize_f(ad->pom);
    ad->com = curr_orientaion; normalize_f(ad->com);
    ad->right = ad->com.xaxis();
    ad->back  = ad->com.yaxis();
    ad->front = ad->com.zaxis();
    ad->old_right = ad->pom.xaxis();
    ad->old_back  = ad->pom.yaxis();
    ad->old_front = ad->pom.zaxis();
    ad->prev_velocity = prev_velocity;
    ad->curr_velocity =
        (curr_orientaion.zaxis() - prev_orientaion.zaxis()) / dt;
    ad->front_speed = dot(ad->curr_velocity, ad->front);

    ad->speed = 
        length(prev_velocity + (ad->curr_velocity - prev_velocity) * 0.2f);

    // grip
    float totalcoef = vp->total_grip_coefficient;
    float frontcoef = vp->front_grip_coefficient;
    float rearcoef = vp->rear_grip_coefficient;
    ad->total_grip = 0;
    ad->front_grip = 0;
    ad->rear_grip = 0;
    ad->left_grip = 0;
    ad->right_grip = 0;

    ad->mass = 0;
    ad->bbmin = max_vector();
    ad->bbmax = min_vector();
	
    mesh_type*          mesh     = sv->get_mesh();
    const points_type&  vertices = mesh->get_points();
    for(const auto& p: vertices) {
        const Vector& pn = p.new_position;
        Vector& mn = ad->bbmin;
        Vector& mx = ad->bbmax;
        update_bb(mn, mx, pn);
        ad->mass += p.mass;

        //float pushout = ( p.pushout0 + p.pushout1 ).length() * p.mass;
#if 1
        Vector pushout =
            p.constraint_pushout +
            p.active_contact_pushout +
            p.passive_contact_pushout;
#else
        Vector pushout = p.pushout0 + p.pushout1;
#endif
        float len = float(length(pushout));
        float depth = float(dot(pushout, ad->back));
        
        ad->total_grip += len * p.mass;
        ad->front_grip += depth * p.mass * p.load.front_grip;
        ad->rear_grip += depth * p.mass * p.load.rear_grip;
        ad->left_grip += depth * p.mass * p.load.left_grip;
        ad->right_grip += depth * p.mass * p.load.right_grip;
    }

    ad->front_grip = clamp(ad->front_grip * frontcoef, 0.0f, 1.0f);
    ad->rear_grip = clamp(ad->rear_grip * rearcoef, 0.0f, 1.0f);
    ad->total_grip = clamp(ad->total_grip * totalcoef, 0.0f, 1.0f);

    ad->bottom_grip = (ad->front_grip + ad->rear_grip) * 0.5f;
	
    if (0 == ad->left_grip && 0 < ad->right_grip) {
        sensory_balance += vp->sensory_balance_speed;
    } else if (0 < ad->left_grip && 0 == ad->right_grip) {
        sensory_balance -= vp->sensory_balance_speed;
    } else {
        sensory_balance *= vp->sensory_balance_decrease;
    }

    float sb_max = DEG2RAD(vp->sensory_balance_max);
    ad->sensory_balance = clamp(sensory_balance, -sb_max, sb_max);
}

PARTIX_DLL_API void AccelerateVehicle(
    PartixWorld*        world, 
    softvolume_type*    sv,
    VehicleParameter*   vp,
    Vector              accel) {

    for(auto& p: sv->get_mesh()->get_points()) {
        p.forces += accel * p.load.accel;
    }
}

PARTIX_DLL_API void RotateEntity(
    PartixWorld* world, softvolume_type* b, 
    float w, float x, float y, float z) {
    b->rotate(w, x, y, z);
}

PARTIX_DLL_API void RotateEntityWithPivot(
    PartixWorld* world, softvolume_type* b, 
    float w, float x, float y, float z, Vector pivot) {
    b->rotate(w, x, y, z, pivot);
}

PARTIX_DLL_API void SetEntityFeatures(
    PartixWorld* world, softvolume_type* b, EntityFeatures ef) {
    
    b->set_stretch_factor(ef.stretch_factor);
    b->set_restore_factor(ef.restore_factor);
    b->set_features(ef.alive, ef.positive, ef.influential);
    b->set_frozen(ef.frozen);
}

PARTIX_DLL_API void FixEntity(
    PartixWorld* world, softvolume_type* b, Vector origin) {
    for (auto& p: b->get_mesh()->get_points()) {
        if(p.load.fix_target) {
            p.new_position = origin + p.source_position;
        }
    }
}

PARTIX_DLL_API void AddForce(
    PartixWorld* world, softvolume_type* b, Vector force) {

    for(auto& p: b->get_mesh()->get_points()) {
        p.forces += force;
    }
}

PARTIX_DLL_API int GetContactCount(
    PartixWorld* world, body_type* b) {
    return (int)b->get_actual_contact_list().size();
}

PARTIX_DLL_API void GetContacts(
    PartixWorld* world, body_type* b, body_type** a) {
    
    int i = 0;
    for (const auto& e: b->get_actual_contact_list()) {
        a[i++] = e;
    }
}

PARTIX_DLL_API int GetClassId(PartixWorld* world, body_type* b) {
    return b->classid();
}

PARTIX_DLL_API void BlendPosition(
    PartixWorld* world, softvolume_type* b, Matrix m, float n, float dn) {

    float rn = 1.0f - n;
    for(auto& p: b->get_mesh()->get_points()) {
        Vector ideal_position = p.source_position * m;
        Vector destination = p.new_position * rn + ideal_position * n;
        Vector diff = destination - p.new_position;
        p.old_position += diff * dn;
        p.new_position = destination;
        p.future_position = ideal_position;
    }
    b->set_future_avail(true);
}

PARTIX_DLL_API void EstimateOrientation(
    PartixWorld* world, softvolume_type* b, float deltaTime, Matrix* m) {

    b->estimate(deltaTime, PartixTraits::tick());
    *m = b->get_future_deformed_matrix();
}

PARTIX_DLL_API void Teleport(
    PartixWorld* world, softvolume_type* b,
    Vector v) {
    b->teleport(v);
}
