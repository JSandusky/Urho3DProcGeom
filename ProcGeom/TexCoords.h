#pragma once

#include <Urho3D/Math/Plane.h>

namespace Urho3D
{
    class Geometry;
    class VertexBuffer;

    /// Calculates LSCM UV chart via Microsoft DX-UVAtlas.
    URHO3D_API Geometry* CalculateUV(Geometry* src, unsigned width, unsigned height, int quality, int maxCharts, float stretch, float gutter, unsigned uvSet = 0);
    
    /// Caculates planar texture coordinates.
    URHO3D_API Geometry* PlanarUV(Geometry* src, const Plane& plane, const Vector2& scale = Vector2::ONE, const Vector2& offset = Vector2::ZERO);
}