#pragma once

#include <Urho3D/Container/Vector.h>

namespace Urho3D
{
    class Context;
    class Geometry;

    enum CSGTask
    {
        CSG_Add,
        CSG_Subtract,
        CSG_Clip,
        CSG_Intersect
    };

    struct URHO3D_API CSGOperand
    {
        Geometry* geometry_;
        CSGTask task_;
        float smoothingPower_ = 0.0f;

        CSGOperand(Geometry* geo, CSGTask task, float smoothing = 0.0f)
        {
            geometry_ = geo;
            task_ = task;
            smoothingPower_ = smoothing;
        }
    };

    URHO3D_API Geometry* Subtract(Context* ctx, Geometry* src, Geometry* subtract, bool encodeTexCoords = false);
    URHO3D_API Geometry* Union(Context* ctx, Geometry* src, Geometry* subtract, bool encodeTexCoords = false);
    URHO3D_API Geometry* Intersect(Context* ctx, Geometry* src, Geometry* subtract, bool encodeTexCoords = false);
    URHO3D_API Geometry* PerformCSG(Context* ctx, PODVector<CSGOperand>& operands, bool encodeTexCoords = false);

    /// Creates a copy of the provided geometry with minimal vertex-data and in canonical form.
    /// For generating occluder and shadow geometry. Only for non-transparents.
    /// Up to 20% a boost (less on stronger GPUs).
    Geometry* CreateShadowGeom(Context* ctx, Geometry* forGeometry);

    /// Combines the provided geometries into one single geometry with minimal vertex-data and in canonical-form.
    /// For generating occluder and shadow geometry, only for non-transparents.
    /// Up to 20% a boost (less on stronger GPUs). This version always uses 32-bit indices.
    Geometry* CreateShadowGeom(Context* ctx, const PODVector<Geometry*>& srcGeoms);
}