#pragma once

#include <Urho3D/ProcGeom/CSGUtility.h>

namespace Urho3D
{
    class Context;
    class Model;

    URHO3D_API Model* CSGModel(Context* ctx, Model* lhs, Model* rhs, CSGTask operation);

}