#pragma once

#include <Urho3D/Math/Matrix3x4.h>
#include <Urho3D/Math/Vector2.h>
#include <Urho3D/Math/Vector3.h>

namespace Urho3D
{
    class Context;
    class Geometry;
    class Image;

    /// Fits a 2D profile to fit along either the lower or upper portion of some heights.
    /// Use to fit for crown-molding or base-boards.
    URHO3D_API PODVector<Vector2> FitProfileToTrim(const PODVector<Vector2>& profile, float floorHieght, float ceilingHeight, bool anchorLower);

    /// Fits a 2D profile to split values to fill the available area, >50% anchors to top, <50% anchors to bottom.
    /// Use to fit to a wall of unknown height.
    URHO3D_API PODVector<Vector2> FitProfileToBounds(const PODVector<Vector2>& profile, float floorHieght, float ceilingHeight);

    /// Generate a surface of revolution. If hollow then it will not attempt to seal the surface.
    URHO3D_API Geometry* Lathe(Context*, const PODVector<Vector2>& outerPoints, int slices, bool hollow);

    /// Lofts the given geometry, optionally uses specified V coords. This loft is always vertically oriented.
    URHO3D_API Geometry* LoftSpine(Context*, const PODVector<Vector2>& profile, const PODVector<Vector3>& points, const PODVector<Vector3>& pointNormals, const PODVector<float>& profileVCoord, bool squadTails = false);

    /// Crude parallel-transport frame, points with position and orientation.
    URHO3D_API Geometry* Loft(Context*, const PODVector<Vector2>& profile, const PODVector<Matrix3x4>& points, const PODVector<float>& profileUVCoord);

    /// Generates a bitmap reference of the UV edges of a geometry.
    URHO3D_API Image* GenerateUVImage(Context* ctx, Geometry* geo, int width, int height);

    /// Generates a bitmap reference of the UV edges of a geometry.
    URHO3D_API void GenerateUVImage(Image* image, Geometry* geo, int width, int height);

    /// Calculates vertex normals
    URHO3D_API void GenerateNormals(void* vertexData, unsigned vertexSize, 
        const void* indexData, unsigned indexSize, unsigned indexStart, unsigned indexCount,
        unsigned positionOffset, unsigned normalOffset, bool flip = false);
}
