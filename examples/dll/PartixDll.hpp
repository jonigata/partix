#ifndef PARTIXDLL_HPP_
#define PARTIXDLL_HPP_

#ifdef PARTIX_DLL_EXPORTS
#define PARTIX_DLL_API __declspec(dllexport)
#else
#define PARTIX_DLL_API __declspec(dllimport)
#endif

#include "PartixUser.hpp"

extern "C" {
    using debug_log_func_type = void(*)(const char*);

    PARTIX_DLL_API void SetDebugLog(debug_log_func_type f);
    PARTIX_DLL_API void DebugLogTest();

    PARTIX_DLL_API PartixWorld* CreateWorld();
    PARTIX_DLL_API void DestroyWorld(PartixWorld*);
    PARTIX_DLL_API void UpdateWorld(PartixWorld*, float delta);
    PARTIX_DLL_API void SetGravity(PartixWorld*, Vector g);

    PARTIX_DLL_API softvolume_type* CreateSoftVolume(
        PartixWorld* world, const char* tcf, Vector position, 
        float scale, float mass);
    PARTIX_DLL_API softvolume_type* CreateVehicle(
        PartixWorld* world, const char* tcf, Vector position, 
        float scale, float mass);
    PARTIX_DLL_API softshell_type* CreateSoftShell(
        PartixWorld* world, 
        int vertex_count, Vector* vertices, 
        int triangle_count, int* triangles,
        int threshold, Vector location, float scale, float mass);

    PARTIX_DLL_API body_type* CreatePlane(
        PartixWorld* world, Vector position, Vector normal);
    PARTIX_DLL_API void GetPosition(
        PartixWorld* world, body_type* b, Vector* position);
    PARTIX_DLL_API void GetOrientation(
        PartixWorld* world, softvolume_type* b, Matrix* position);

    PARTIX_DLL_API int GetWireFrameVertexCount(
        PartixWorld* world, softvolume_type* b);
    PARTIX_DLL_API int GetWireFrameIndexCount(
        PartixWorld* world, softvolume_type* b);
    PARTIX_DLL_API void GetWireFrameVertices(
        PartixWorld* world, softvolume_type* b, Vector* buffer);
    PARTIX_DLL_API void GetWireFrameIndices(
        PartixWorld* world, softvolume_type* b, int* buffer);
    PARTIX_DLL_API void GetPointLoads(
        PartixWorld* world, softvolume_type* b, 
        PartixTraits::point_load_type* buffer);
    PARTIX_DLL_API void SetPointLoads(
        PartixWorld* world, softvolume_type* b, 
        const PartixTraits::point_load_type* buffer);
    PARTIX_DLL_API void SetEntityFeatures(
        PartixWorld* world, softvolume_type* b, EntityFeatures ef);
    PARTIX_DLL_API void FixEntity(
        PartixWorld* world, softvolume_type* b, 
        Vector origin);

    PARTIX_DLL_API void AnalyzeVehicle(
        PartixWorld*        world, 
        softvolume_type*    sv,
        VehicleParameter*   vp,
        float               dt,
        Vector              prev_velocity,
        Matrix              prev_orientaion,
        Matrix              curr_orientaion,
        float               sensory_balance,
        VehicleAnalyzeData* ad);
    
    PARTIX_DLL_API void AccelerateVehicle(
        PartixWorld*        world, 
	softvolume_type*    sv,
        VehicleParameter*   vp,
        Vector              accel);

    PARTIX_DLL_API void RotateEntity(
        PartixWorld* world, softvolume_type* b, 
        float w, float x, float y, float z);

}

#endif // PARTIXDLL_HPP_
