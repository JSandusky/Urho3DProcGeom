#pragma once

#include <Urho3D/Math/Matrix3x4.h>
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

    /// Helper for building the call to PerformCSG. Encoding tex-coords will use UDIM-like advancements of U.
    URHO3D_API Geometry* Subtract(Context* ctx, Geometry* src, Geometry* subtract, bool encodeTexCoords = false);
    /// Helper for building the call to PerformCSG. Encoding tex-coords will use UDIM-like advancements of U.
    URHO3D_API Geometry* Union(Context* ctx, Geometry* src, Geometry* subtract, bool encodeTexCoords = false);
    /// Helper for building the call to PerformCSG. Encoding tex-coords will use UDIM-like advancements of U.
    URHO3D_API Geometry* Intersect(Context* ctx, Geometry* src, Geometry* subtract, bool encodeTexCoords = false);

    /// Processes the given CSG-operations in sequence, the first item must be additive (it will be forced to be such if it is not).
    /// Encoding tex-coords advances the U coordinate by 1 for each mesh.
    URHO3D_API Geometry* PerformCSG(Context* ctx, PODVector<CSGOperand>& operands, bool encodeTexCoords = false);
}