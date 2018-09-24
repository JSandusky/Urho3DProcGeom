#pragma once

#include <Urho3D/Container/Vector.h>
#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Math/Vector4.h>

namespace Urho3D
{
    class Geometry;
    class Skeleton;
    class VertexBuffer;

    struct URHO3D_API BoneWeight
    {
        float weights_[4];
        int boneIdx_[4];

        // Construct a clean default state boneweight.
        BoneWeight() { 
            weights_[0] = weights_[1] = weights_[2] = weights_[3] = 0.0f; 
            boneIdx_[0] = boneIdx_[1] = boneIdx_[2] = boneIdx_[3] = -1;
        }

        // Get the normalized bone-weights.
        Vector4 Normalized() const;

        // Adds a weight, will replace the worst weight, or the first unset weight found.
        void AddWeight(int idx, float w);
    };

    // Returns a new vertex-buffer (copied from the source) with all of the original data and the calculated weights.
    URHO3D_API VertexBuffer* CalculateBoneWeights(Geometry* forGeom, Skeleton* againstSkeleton, bool useGlowWeighting = false, bool weightRootBone = false);

    struct URHO3D_API LaplaceHandle
    {
        Vector3 origin_;
        Vector3 replacement_;
        /// Will be calculated during deformation.
        unsigned vertexIndex_;
    };

    // Returns a new vertex-buffer (copied from the source) with all of the original data nd newly calculated positions, normals, and tangents.
    URHO3D_API VertexBuffer* LaplaceDeform(Geometry* forGeom, const PODVector<LaplaceHandle>& handles);
}