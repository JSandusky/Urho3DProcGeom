#pragma once

#include <Urho3D/Math/Vector3.h>

namespace Urho3D
{

    URHO3D_API float SDFSphere(const Vector3& pt, const Vector3& pos, float radius);
    URHO3D_API float SDFCube(const Vector3& pt, const Vector3& pos, const Vector3& dim);
    URHO3D_API float SDFCylinder(const Vector3& pt, const Vector3& pos, const Vector2& heightRadius);

}