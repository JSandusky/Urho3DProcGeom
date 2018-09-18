#pragma once

#include <Urho3D/Container/Vector.h>
#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Math/Vector4.h>

namespace Urho3D
{
    class Geometry;
    class Skeleton;

    struct URHO3D_API BoneWeight
    {
        float weights_[4];
        int boneIdx_[4];

        BoneWeight() { 
            weights_[0] = weights_[1] = weights_[2] = weights_[3] = 0.0f; 
            boneIdx_[0] = boneIdx_[1] = boneIdx_[2] = boneIdx_[3] = -1;
        }

        // Adds a weight, will replace the worst weight.
        void AddWeight(int idx, float w);
    };

    // Uses 
    URHO3D_API bool CalculateBoneWeights(Geometry* forGeom, Skeleton* againstSkeleton, PODVector<Vector4>& weights, PODVector<int>& boneIdx);

    struct URHO3D_API LaplaceHandle
    {
        Vector3 origin_;
        Vector3 replacement_;
    };

    URHO3D_API Geometry* LaplaceDeform(Geometry* forGeom, const PODVector<LaplaceHandle>& handles);
}