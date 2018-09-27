#include <Urho3D/ProcGeom/SDFImplicits.h>

namespace Urho3D
{

#define MaxElement(VEC) (VEC.x_ > VEC.y_ && VEC.x_ > VEC.z_ ? VEC.x_ : (VEC.y_ > VEC.x_ && VEC.y_ > VEC.z_ ? VEC.y_ : VEC.z_))

float SDFSphere(const Vector3& pt, const Vector3& pos, float radius)
{
    return (pt - pos).Length() - radius;
}

float SDFCube(const Vector3& pt, const Vector3& pos, const Vector3& dim)
{
    Vector3 d = pos.Abs() - dim / 2;
    float f = MaxElement(d);
    return Min(f, 0.0f) + d.Length();
}

float SDFCylinder(const Vector3& pt, const Vector3& pos, const Vector2& heightRadius)
{
    auto p = pt - pos;
    float d = Vector2(p.x_, p.z_).Length() - heightRadius.y_;
    return Max(d, fabsf(p.y_) - heightRadius.x_);
}

}