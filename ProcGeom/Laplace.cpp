#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/ProcGeom/kdTree.h>
#include <Urho3D/ProcGeom/Laplace.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Graphics/Skeleton.h>
#include <Urho3D/Graphics/VertexBuffer.h>

#include <ThirdParty/igl/cotmatrix.h>
#include <ThirdParty/igl/massmatrix.h>

#define BONE_GLOW_ITERATIONS 6
#define BONE_GLOW_ITERATION_WEIGHT (1.0f / BONE_GLOW_ITERATIONS)

namespace Urho3D
{
    struct BoneSegment
    {
        LineSegment segment_;
        const Bone* base_;
        int baseIndex_;

        BoneSegment(int idx, const Bone* from, const Bone* to) :
            base_(from),
            baseIndex_(idx),
            segment_(from->initialPosition_, to->initialPosition_)
        { 
        }

        static PODVector<BoneSegment> FromSkeleton(Skeleton* skeleton)
        {
            PODVector<BoneSegment> ret;

            auto rootBone = skeleton->GetRootBone();
            Fill(ret, skeleton, rootBone);

            return ret;
        }

        static void Fill(PODVector<BoneSegment>& bones, const Skeleton* skeleton, const Bone* joint)
        {
            auto thisIdx = skeleton->GetBoneIndex(joint);
            for (auto& child : skeleton->GetBones())
            {
                if (child.parentIndex_ == thisIdx)
                {
                    bones.Push(BoneSegment(thisIdx, joint, &child));
                    Fill(bones, skeleton, &child);
                }
            }
        }
    };

    Vector4 BoneWeight::Normalized() const {
        Vector4 ret = Vector4(weights_[0], weights_[1], weights_[2], weights_[3]);
        float len = sqrtf(
            ret.x_ * ret.x_ +
            ret.y_ * ret.y_ +
            ret.z_ * ret.z_ +
            ret.w_ * ret.w_);
        ret /= len;
        return ret;
    }

    void BoneWeight::AddWeight(int idx, float w)
    {
        // find the lowest weight, overwrite that
        int minIdx = -1;
        float minValue = FLT_MAX;
        for (int i = 0; i < 4; ++i)
        {
            // if hitting an unused slot then just use that
            if (boneIdx_[i] == -1)
            {
                minIdx = i;
                minValue = 0;
                break;
            }
            
            // Select the lowest
            if (weights_[i] < minValue)
            {
                minValue = weights_[i];
                minIdx = i;
            }
        }
        if (minIdx != -1 && w > minValue)
        {
            weights_[minIdx] = w;
            boneIdx_[minIdx] = idx;
        }
    }

    bool ExtractVertexNormalCanonicals(const unsigned char* vertexData, unsigned vertexCount, unsigned vertexSize,
        const unsigned char* indexData, unsigned indexCount, unsigned indexSize, 
        unsigned posOffset, unsigned normOffset, 
        PODVector<Vector3>& positions, PODVector<Vector3>& normals, PODVector<unsigned>& indices, unsigned& indexStartOffset, const Matrix3x4& transform, PODVector<unsigned>* remapping)
    {
        // NOTE: Written to assume it may be used to concatenate multiple geometries.

        HashMap<Vector3, unsigned> canonicals;
        PODVector<unsigned> localVertexRemap;
        PODVector<unsigned>* vertexRemap = remapping != nullptr ? remapping : &localVertexRemap;
        vertexRemap->Resize(vertexCount);
        const bool largeIndices = indexSize == sizeof(unsigned);

        // Require a position semantic
        for (size_t i = 0; i < vertexCount; ++i)
        {
            Vector3 vertPos = *((Vector3*)(vertexData + vertexSize*i + posOffset));
            Vector3 vertNor = *((Vector3*)(vertexData + vertexSize*i + normOffset));
            vertPos = transform * vertPos;
            vertNor = transform.RotationMatrix() * vertNor;
            const unsigned tentativeIdx = canonicals.Size();
            auto existing = canonicals.Find(vertPos);
            if (existing == canonicals.End())
            {
                positions.Push(vertPos);
                normals.Push(vertNor);
                canonicals.Insert(MakePair(vertPos, tentativeIdx));
                (*vertexRemap)[i] = tentativeIdx;
            }
            else
            {
                (*vertexRemap)[i] = existing->second_;
                normals[existing->second_] += vertNor;
            }
        }

        // Make sure there's enough index data space
        if (indices.Size() < indexStartOffset + indexCount)
            indices.Resize(indices.Size() + indexCount);

        for (size_t idx = 0; idx < indexCount; idx += 3)
        {
            unsigned indexTable[] = {
                largeIndices ? ((unsigned*)indexData)[idx] : ((unsigned short*)indexData)[idx],
                largeIndices ? ((unsigned*)indexData)[idx + 1] : ((unsigned short*)indexData)[idx + 1],
                largeIndices ? ((unsigned*)indexData)[idx + 2] : ((unsigned short*)indexData)[idx + 2],
            };

            indices[indexStartOffset + idx] = indexTable[0] + indexStartOffset;
            indices[indexStartOffset + idx + 1] = indexTable[1] + indexStartOffset;
            indices[indexStartOffset + idx + 2] = indexTable[2] + indexStartOffset;
        }

        // advance index starting value
        indexStartOffset += indexCount;
        return true;
    }

    /// Equalize the bone weights for each vertex
    static void BalanceBoneWeights(Eigen::SparseMatrix<float>& PP)
    {
        for (int i = 0; i < PP.rows(); ++i)
        {
            const float sum = PP.row(i).sum();
            const float div = 1.0f / sum;
            for (int b = 0; b < PP.cols(); ++b)
                PP.coeffRef(i, b) *= div;
        }
    }

    /// Pinnochio style direct access weights
    static Pair<float, float> CalculateHeatWeight(Eigen::SparseMatrix<float>& PP, Eigen::MatrixXf& pos, Eigen::MatrixXf& norm, int vertIndIdx, int vertexIdx, int boneIdx, kdTree& octree, PODVector<BoneSegment>& bones)
    {
        const auto vertexPos = Vector3(pos.coeff(vertexIdx, 0), pos.coeff(vertexIdx, 1), pos.coeff(vertexIdx, 2));
        const auto vertexNor = Vector3(norm.coeff(vertexIdx, 0), norm.coeff(vertexIdx, 1), norm.coeff(vertexIdx, 2));
        const auto& currentBone = bones[boneIdx];

        bool visible = false;
        float weightSum = 0.0f;
        float minDist = FLT_MAX;

        auto onBone = currentBone.segment_.ClosestPoint(vertexPos);
        for (float i = 0; i < BONE_GLOW_ITERATIONS + 1.0f; ++i)
        {
            const float fraction = (i + 1) * (1.0f / BONE_GLOW_ITERATIONS);

            const auto currentBonePos = currentBone.segment_.a_.Lerp(currentBone.segment_.b_, fraction);
            const auto toVertex = vertexPos - currentBonePos;
            if (toVertex.LengthSquared() > currentBone.segment_.LengthSquared() * 2)
                return MakePair<float, float>(0.0f, FLT_MAX);

            Vec3 hitPos;
            if (!octree.Hit(LineSegment(currentBonePos, vertexPos), vertexIdx))
            {
                const float dist = currentBone.segment_.Distance(vertexPos);
                minDist = std::min(minDist, dist);
            }
        }

        return MakePair(weightSum, minDist);
    }

    /// Bone glow weights, bone is handled like a lighting element
    /// Return value as <WEIGHT, DISTANCE>
    static Pair<float, float> CalculateGlowWeight(Eigen::SparseMatrix<float>& PP, Eigen::MatrixXf& pos, Eigen::MatrixXf& norm, int vertIndIdx, int vertexIdx, int boneIdx, kdTree& octree, PODVector<BoneSegment>& bones)
    {
        const auto vertexPos = Vector3(pos.coeff(vertexIdx, 0), pos.coeff(vertexIdx, 1), pos.coeff(vertexIdx, 2));
        const auto vertexNor = Vector3(norm.coeff(vertexIdx, 0), norm.coeff(vertexIdx, 1), norm.coeff(vertexIdx, 2));
        const auto& currentBone = bones[boneIdx];

        bool visible = false;
        float weightSum = 0.0f;
        float minDist = FLT_MAX;

        auto onBone = currentBone.segment_.ClosestPoint(vertexPos);
        for (float i = 0; i < BONE_GLOW_ITERATIONS + 1.0f; ++i)
        {
            const float fraction = (i + 1) * (1.0f / BONE_GLOW_ITERATIONS);

            const auto currentBonePos = currentBone.segment_.a_.Lerp(currentBone.segment_.b_, fraction);
            const auto toVertex = vertexPos - currentBonePos;
            const auto toVertexNor = toVertex.Normalized();

            Vector3 hitPos;
            if (!octree.Hit(LineSegment(currentBonePos, vertexPos), vertexIdx))
            {
                if (fabsf(currentBone.segment_.Direction().DotProduct(toVertex.Normalized())) < 0.45)
                    continue;

                minDist = Min(minDist, currentBone.segment_.Distance(vertexPos));
                float lambert = fabsf(toVertexNor.DotProduct(vertexNor));
                lambert = Max(lambert, 0.0f);
                weightSum += (lambert / toVertex.LengthSquared()) * (toVertexNor.CrossProduct(currentBone.segment_.Direction()).Length()) * BONE_GLOW_ITERATION_WEIGHT * currentBone.segment_.Length();
            }
        }

        PP.coeffRef(vertIndIdx, currentBone.baseIndex_) = weightSum;
        return MakePair(weightSum, minDist);
    }

    typedef Pair<float,float>(*WEIGHTING_FUNCTION)(Eigen::SparseMatrix<float>& PP, Eigen::MatrixXf& pos, Eigen::MatrixXf& norm, int vertIndIdx, int vertexIdx, int boneIdx, kdTree& octree, PODVector<BoneSegment>& bones);

    VertexBuffer* CalculateBoneWeights(Geometry* forGeom, Skeleton* skeleton, bool useGlow, bool weightRoot)
    {
        if (!forGeom || !skeleton)
        {
            URHO3D_LOGERROR("Cannot calculate bone weights for a mesh whose data does not exist");
            return nullptr;
        }

        const unsigned char* vertexData;
        const unsigned char* indexData;
        unsigned vertexSize, indexSize;
        const PODVector<VertexElement>* elements;
        forGeom->GetRawData(vertexData, vertexSize, indexData, indexSize, elements);
        
        const unsigned posOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);
        const unsigned normOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_NORMAL);
        const unsigned boneIdxOffset = VertexBuffer::GetElementOffset(*elements, TYPE_UBYTE4, SEM_BLENDINDICES);
        const unsigned boneWeightOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR4, SEM_BLENDWEIGHTS);

        const unsigned rawVertexCt = forGeom->GetVertexCount();
        const unsigned rawNormalCt = normOffset != M_MAX_UNSIGNED ? forGeom->GetVertexCount() : 0;
        const unsigned rawIndexCt = forGeom->GetIndexCount();

        if (rawVertexCt == 0 || rawNormalCt == 0 || rawIndexCt == 0 || posOffset == M_MAX_UNSIGNED || normOffset == M_MAX_UNSIGNED)
        {
            URHO3D_LOGERROR("Cannot calculate bone weights for a mesh without vertices: SEM_POSITION & SEM_NORMAL required");
            return nullptr;
        }

        if (boneIdxOffset == -1 || boneWeightOffset == -1)
        {
            URHO3D_LOGERROR("Cannot calculate bone weights for a mesh without weight/index data");
            return nullptr;
        }

        PODVector<BoneSegment> bones = BoneSegment::FromSkeleton(skeleton);
        const unsigned numBones = bones.Size();
        auto joints = skeleton->GetBones();
        const unsigned numJoints = joints.Size();

        Eigen::MatrixXf V;
        Eigen::MatrixXf N;
        Eigen::MatrixXi F;
        Eigen::SparseMatrix<float> L, M;

        // Extract and construct canonicals
        // It's necessary to work with canonicals so that weights properly diffuse across seam-edges,
        // Otherwise weights will not propogate (the cotangent weights will be foul and nullify everything).
        PODVector<Vector3> canonicalPos;
        PODVector<Vector3> canonicalNorm;
        PODVector<unsigned> canonicalIdx;
        PODVector<unsigned> canonRemapping;
        unsigned idxStartOffset = 0;
        ExtractVertexNormalCanonicals(vertexData, rawVertexCt, vertexSize,
            indexData, rawIndexCt, indexSize, posOffset, normOffset,
            canonicalPos, canonicalNorm, canonicalIdx, idxStartOffset, Matrix3x4::IDENTITY, &canonRemapping);

        V.resize(canonicalPos.Size(), 3);
        N.resize(canonicalNorm.Size(), 3);
        F.resize(canonicalIdx.Size() / 3, 3);

        for (unsigned i = 0; i < canonicalPos.Size(); ++i)
        {
            V.coeffRef(i, 0) = canonicalPos[i].x_;
            V.coeffRef(i, 1) = canonicalPos[i].y_;
            V.coeffRef(i, 2) = canonicalPos[i].z_;

            auto norm = canonicalNorm[i].Normalized();
            N.coeffRef(i, 0) = norm.x_;
            N.coeffRef(i, 1) = norm.y_;
            N.coeffRef(i, 2) = norm.z_;
        }

        // build face-matrix
        for (unsigned i = 0; i < canonicalIdx.Size(); i += 3)
        {
            F.coeffRef(i / 3, 0) = canonicalIdx[i];
            F.coeffRef(i / 3, 1) = canonicalIdx[i + 1];
            F.coeffRef(i / 3, 2) = canonicalIdx[i + 2];
        }

        igl::cotmatrix(V, F, L);
        igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_DEFAULT, M);

        Eigen::VectorXf H;
        H.resize(canonicalIdx.Size());
        H.setZero();
        Eigen::SparseMatrix<float> P;
        P.resize(canonicalPos.Size(), numJoints);

        kdTreeConstructionData kdTreeData;
        kdTreeData.positionBuffer_ = canonicalPos.Buffer();
        kdTreeData.positionBufferLength_ = canonicalPos.Size();
        kdTreeData.indexBuffer_ = canonicalIdx.Buffer();
        kdTreeData.indexBufferLength_ = canonicalIdx.Size();
        kdTreeData.Pack();
        kdTree kdTree(kdTreeData);

        WEIGHTING_FUNCTION weightingMethod = useGlow ? CalculateGlowWeight : CalculateHeatWeight;

        for (unsigned vertIndIdx = 0; vertIndIdx < canonicalIdx.Size(); ++vertIndIdx)
        {
            const unsigned vertexIdx = canonicalIdx[vertIndIdx];

            int nearestJoint = -1;
            float minDist = FLT_MAX;
            for (unsigned boneIdx = 0; boneIdx < numBones; ++boneIdx)
            {
                const auto& currentBone = bones[boneIdx];
                if (currentBone.baseIndex_ == 0 && !weightRoot) // don't weight the root
                    continue;
                auto weight = weightingMethod(P, V, N, vertIndIdx, vertexIdx, boneIdx, kdTree, bones);
                if (weight.second_ < minDist)
                {
                    minDist = weight.second_;
                    nearestJoint = skeleton->GetBoneIndex(bones[boneIdx].base_);
                }
            }
            if (minDist != FLT_MAX)
                H.coeffRef(vertIndIdx) = 1.0f / (minDist*minDist);
            if (nearestJoint != -1)
            {
                float& dist = P.coeffRef(vertIndIdx, nearestJoint);
                dist = Max(1.0f, dist);
            }

            // TODO: port cancellation callback?
            //if (callback && !callback())
            //{
            //    delete heMesh;
            //    return;
            //}
        }

        // Glow weights are a lighting integral, requires normalization
        if (weightingMethod == CalculateGlowWeight)
            BalanceBoneWeights(P);

        auto HH = H.asDiagonal();
        Eigen::SimplicialLLT<Eigen::SparseMatrix<float>, Eigen::Upper > solver;
        solver.compute(-L + M*HH);
        if (solver.info() != Eigen::Success)
        {
            if (solver.info() == Eigen::InvalidInput)
                URHO3D_LOGERROR("Bone weight inputs are invaid");
            else
                URHO3D_LOGERROR("Numerical issue encountered in solving bone weights");
            return nullptr;
        }

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> diffusedWeights = solver.solve(HH/**M*/*P);
        if (solver.info() == Eigen::Success)
        {
            PODVector<BoneWeight> boneWeights(canonicalPos.Size());
            for (unsigned j = 0; j < numJoints; ++j)
            {
                for (unsigned v = 0; v < canonicalIdx.Size(); ++v)
                {
                    float w = diffusedWeights.coeffRef(v, j);
                    auto vertIdx = canonicalIdx[v];
                    boneWeights[canonicalIdx[v]].AddWeight(j, w);
                }
            }

            VertexBuffer* newVtxBuffer = new VertexBuffer(forGeom->GetContext());
            newVtxBuffer->SetSize(rawVertexCt, *elements);
            unsigned char* newData = new unsigned char[vertexSize * rawVertexCt];
            memcpy(newData, vertexData, vertexSize * rawVertexCt);

            // Copy over weights on the original vertex data
            for (unsigned v = 0; v < rawVertexCt; ++v)
            {
                // TODO: port over response-curve based remapping of bone-weights?

                auto remapped = canonRemapping[v];
                auto ind = boneWeights[remapped].boneIdx_;

                *(Vector4*)(newData + v * vertexSize + boneWeightOffset) = boneWeights[remapped].Normalized();
                *(unsigned char*)(newData + v * vertexSize + boneIdxOffset) = ind[0];
                *(unsigned char*)(newData + v * vertexSize + boneIdxOffset + 1) = ind[1];
                *(unsigned char*)(newData + v * vertexSize + boneIdxOffset + 2) = ind[2];
                *(unsigned char*)(newData + v * vertexSize + boneIdxOffset + 3) = ind[3];
            }

            newVtxBuffer->SetData(newData);
            delete[] newData;
            return newVtxBuffer;
        }
        else
        {
            URHO3D_LOGERROR("Failed to find a solution for skin weighting");
            return nullptr;
        }
    }

    Geometry* LaplaceDeform(Geometry* forGeom, const PODVector<LaplaceHandle>& handles)
    {

#if 0
        Eigen::MatrixXf positions;
        Eigen::MatrixXi indices;

        FillIGLData(forGeom, positions, indices);

        Eigen::MatrixXf canonPositions;
        Eigen::MatrixXi canonIndices;
        Eigen::MatrixXi canonTable;
        igl::remove_duplicate_vertices(positions, M_EPSILON, canonPositions, canonIndices, canonTable);

        Eigen::SparseMatrix<float> cotMat, massMat;
        igl::cotmatrix(canonPositions, canonIndices, cotMat);
        igl::massmatrix(canonPositions, canonIndices, igl::MASSMATRIX_TYPE_DEFAULT, massMat);

        Eigen::VectorXf H;
        H.resize(canonPositions.rows());
        H.setZero();
        Eigen::SparseMatrix<float> P;
#endif
        return nullptr;
    }

}