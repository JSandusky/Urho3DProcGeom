#pragma once

#include <Urho3D/Container/Vector.h>
#include <Urho3D/Container/Ptr.h>

namespace Urho3D
{

    class Geometry;
    class Model;

    /// Emits a reduced version of the given geometry.
    URHO3D_API Geometry* GenerateLOD(Geometry* forGeom, float lodPower);

    /// Will generate LODs for all geometries in the model.
    /// TODO: implmenet the option to merge the VB and IB for each level as one single buffer (batch coherence).
    ///     that has interesting implications that may not be realistic, vertex elements must match etc
    URHO3D_API SharedPtr<Model> GenerateModelLOD(Model* forModel, const PODVector<float>& lodPowers, bool packBuffers);

}