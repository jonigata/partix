// 2017/03/12 Naoyuki Hirayama

#include "stdafx.h"
#include "PartixDll.hpp"
#include "PartixUser.hpp"

static debug_log_func_type log_func = nullptr;

namespace {

}

void DebugLog(const char* s) {
    if (log_func != nullptr) {
        log_func(s);
    }
}

PARTIX_DLL_API void SetDebugLog(debug_log_func_type f) {
    log_func = f;
}

PARTIX_DLL_API void DebugLogTest() {
    DebugLog("DebugLogTest");
}

PARTIX_DLL_API PartixWorld* CreateWorld() {
    PartixWorld* world = new PartixWorld;
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
    PartixWorld* world, const char* tcf, Vector v, float scale) {

    DebugLog("CreateSoftVolume");
    softvolume_ptr b = world->add_softvolume(tcf, v, scale);
    return b.get();
}

PARTIX_DLL_API softvolume_type* CreateVehicle(
    PartixWorld* world, const char* tcf, Vector v, float scale) {

    DebugLog("CreateVehicle");
    softvolume_ptr b = world->add_vehicle(tcf, v, scale);
    return b.get();
}

PARTIX_DLL_API body_type* CreatePlane(
    PartixWorld* world, Vector position, Vector normal) {
    DebugLog("CreatePlane");
    return world->add_plane(position, normal).get();
}

PARTIX_DLL_API void GetPosition(
    PartixWorld* world, body_type* b, Vector* pos) {
    *pos = b->get_current_center();
}

PARTIX_DLL_API void GetOrientation(
    PartixWorld* world, softvolume_type* b, Matrix* m) {
    *m = b->get_deformed_matrix();
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
    Vector center = b->get_current_center();
    for (const auto& p: b->get_mesh()->get_points()) {
        *buffer++ = p.new_position - center;
    }
}


PARTIX_DLL_API void GetWireFrameIndices(
    PartixWorld* world, softvolume_type* b, int* buffer) {
    for (const auto& e: b->get_mesh()->get_edges()) {
        *buffer++ = e.indices.i0;
        *buffer++ = e.indices.i1;
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

    float accel_factor = vp->accel_factor;

    mesh_type*      mesh     = sv->get_mesh();
    points_type&    vertices = mesh->get_points();
    for(auto& p: vertices) {
        p.forces += accel * p.load.accel * accel_factor;
    }
}

