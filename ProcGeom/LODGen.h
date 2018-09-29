#pragma once

#include <Urho3D/Container/Vector.h>

namespace Urho3D
{

    class Geometry;
    class Model;

    /// Emits a reduced version of the given geometry.
    URHO3D_API Geometry* GenerateLOD(Geometry* forGeom, float lodPower);

    /// Will generate LODs for all geometries in the model, optionally the VB and IB for each level will be packed together.
    URHO3D_API void GenerateModelLOD(Model* forModel, const PODVector<float>& lodPowers, bool packBuffers);

}