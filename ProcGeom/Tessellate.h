#pragma once

#include <Urho3D/Urho3D.h>

namespace Urho3D
{
    class Geometry;

    URHO3D_API Geometry* Tessellate(Geometry* geo, int levels, bool smooth);

}