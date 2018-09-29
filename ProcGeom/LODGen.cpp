#include <Urho3D/ProcGeom/LODGen.h>

#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/VertexBuffer.h>

#include <igl/decimate.h>

namespace Urho3D
{

    Geometry* GenerateLOD(Geometry* src, float lodPower)
    {
        const unsigned char* data;
        const unsigned char* indexData;
        unsigned vertexSize;
        unsigned indexSize;
        const PODVector<VertexElement>* elements;

        src->GetRawData(data, vertexSize, indexData, indexSize, elements);
        const auto vertexStart = src->GetVertexStart();
        const auto vertexCt = src->GetVertexCount();
        const auto indexStart = src->GetIndexStart();
        const auto indexCt = src->GetIndexCount();
        const bool largeIndices = indexSize == sizeof(unsigned);

        const unsigned posOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);

        Eigen::MatrixXd positions;
        positions.resize(vertexCt, 3);
        Eigen::MatrixXi indices;
        indices.resize(indexCt / 3, 3);

        for (unsigned i = vertexStart; i < vertexStart + vertexCt; ++i)
        {
            Vector3 pos = *(Vector3*)(data + vertexSize * i + posOffset);
            positions.coeffRef(i - vertexStart, 0) = pos.x_;
            positions.coeffRef(i - vertexStart, 1) = pos.y_;
            positions.coeffRef(i - vertexStart, 2) = pos.z_;
        }

        for (unsigned i = indexStart; i < indexCt; i += 3)
        {
            unsigned idx[] = {
                largeIndices ? ((unsigned*)indexData)[i] : ((unsigned short*)indexData)[i],
                largeIndices ? ((unsigned*)indexData)[i + 1] : ((unsigned short*)indexData)[i + 1],
                largeIndices ? ((unsigned*)indexData)[i + 2] : ((unsigned short*)indexData)[i + 2],
            };

            indices.coeffRef(i / 3, i) = idx[0] - vertexStart;
            indices.coeffRef(i / 3, i + 1) = idx[1] - vertexStart;
            indices.coeffRef(i / 3, i + 2) = idx[2] - vertexStart;
        }

        Eigen::MatrixXd newPos;
        Eigen::MatrixXi newTris;
        Eigen::VectorXi birthFaces;
        Eigen::VectorXi birthVerts;
        if (igl::decimate(positions, indices, (size_t)(indexCt * lodPower), newPos, newTris, birthFaces, birthVerts))
        {
            Geometry* ret = new Geometry(src->GetContext());

            unsigned char* newVtxData = new unsigned char[vertexSize * newPos.rows()];
            unsigned char* newIndexData = new unsigned char[indexSize * newTris.rows()];

            for (unsigned i = 0; i < newTris.rows(); ++i)
            {
                unsigned indices[3];
                if (indexSize == sizeof(unsigned))
                {
                    indices[0] = newTris.coeff(i, 0);
                    indices[1] = newTris.coeff(i, 1);
                    indices[2] = newTris.coeff(i, 2);
                    *(unsigned*)(i * indexSize + i) = indices[0];
                    *(unsigned*)(i * indexSize + i) = indices[1];
                    *(unsigned*)(i * indexSize + i) = indices[2];
                }
                else
                {
                    indices[0] = newTris.coeff(i, 0);
                    indices[1] = newTris.coeff(i, 1);
                    indices[2] = newTris.coeff(i, 2);
                    *(unsigned short*)(i * indexSize + i) = indices[0];
                    *(unsigned short*)(i * indexSize + i) = indices[1];
                    *(unsigned short*)(i * indexSize + i) = indices[2];
                }
            }

            for (unsigned i = 0; i < newPos.rows(); ++i)
            {
                unsigned birthIdx = vertexStart + birthVerts.coeff(i);

                *(Vector3*)(newVtxData + i * vertexSize + posOffset) = Vector3(newPos.coeff(i, 0), newPos.coeff(i, 1), newPos.coeff(i, 2));
                for (auto elem : *elements)
                {
                    switch (elem.type_)
                    {
                    case TYPE_VECTOR2:
                        *(Vector2*)(newVtxData + i * vertexSize + elem.offset_) = *(Vector2*)(data + birthIdx * vertexSize + elem.offset_);
                        break;
                    case TYPE_VECTOR3:
                        *(Vector3*)(newVtxData + i * vertexSize + elem.offset_) = *(Vector3*)(data + birthIdx * vertexSize + elem.offset_);
                        break;
                    case TYPE_VECTOR4:
                        *(Vector4*)(newVtxData + i * vertexSize + elem.offset_) = *(Vector4*)(data + birthIdx * vertexSize + elem.offset_);
                        break;
                    case TYPE_UBYTE4:
                    case TYPE_UBYTE4_NORM:
                        *(unsigned*)(newVtxData + i * vertexSize + elem.offset_) = *(unsigned*)(data + birthIdx * vertexSize + elem.offset_);
                        break;
                    case TYPE_INT:
                        *(int*)(newVtxData + i * vertexSize + elem.offset_) = *(int*)(data + birthIdx * vertexSize + elem.offset_);
                        break;
                    }
                }
            }

            VertexBuffer* newVertBuffer = new VertexBuffer(src->GetContext());
            newVertBuffer->SetShadowed(true);
            newVertBuffer->SetSize(newPos.rows(), *elements);
            newVertBuffer->SetData(newVtxData);
            delete[] newVtxData;

            IndexBuffer* newIdxBuffer = new IndexBuffer(src->GetContext());
            newIdxBuffer->SetShadowed(true);
            newIdxBuffer->SetSize(newTris.rows(), largeIndices);
            newIdxBuffer->SetData(newIndexData);

            ret->SetNumVertexBuffers(1);
            ret->SetVertexBuffer(0, newVertBuffer);
            ret->SetIndexBuffer(newIdxBuffer);

            return ret;
        }
        else
        {
            URHO3D_LOGERROR("Failed to determine LOD");
            return nullptr;
        }
    }

    /// This function will only work if the vertex data is identical ... that could be a problem
    void GenerateLOD_Incremental(Geometry* src, float lodPower, PODVector<unsigned char>& outVertexData, PODVector<unsigned int>& outIndexData)
    {
        const unsigned char* data;
        const unsigned char* indexData;
        unsigned vertexSize;
        unsigned indexSize;
        const PODVector<VertexElement>* elements;

        src->GetRawData(data, vertexSize, indexData, indexSize, elements);
        const auto vertexStart = src->GetVertexStart();
        const auto vertexCt = src->GetVertexCount();
        const auto indexStart = src->GetIndexStart();
        const auto indexCt = src->GetIndexCount();
        const bool largeIndices = indexSize == sizeof(unsigned);

        const unsigned posOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);

        Eigen::MatrixXd positions;
        positions.resize(vertexCt, 3);
        Eigen::MatrixXi indices;
        indices.resize(indexCt / 3, 3);

        for (unsigned i = vertexStart; i < vertexStart + vertexCt; ++i)
        {
            Vector3 pos = *(Vector3*)(data + vertexSize * i + posOffset);
            positions.coeffRef(i - vertexStart, 0) = pos.x_;
            positions.coeffRef(i - vertexStart, 1) = pos.y_;
            positions.coeffRef(i - vertexStart, 2) = pos.z_;
        }

        for (unsigned i = indexStart; i < indexCt; i += 3)
        {
            unsigned idx[] = {
                largeIndices ? ((unsigned*)indexData)[i] : ((unsigned short*)indexData)[i],
                largeIndices ? ((unsigned*)indexData)[i + 1] : ((unsigned short*)indexData)[i + 1],
                largeIndices ? ((unsigned*)indexData)[i + 2] : ((unsigned short*)indexData)[i + 2],
            };

            indices.coeffRef(i / 3, i) = idx[0] - vertexStart;
            indices.coeffRef(i / 3, i + 1) = idx[1] - vertexStart;
            indices.coeffRef(i / 3, i + 2) = idx[2] - vertexStart;
        }

        Eigen::MatrixXd newPos;
        Eigen::MatrixXi newTris;
        Eigen::VectorXi birthFaces;
        Eigen::VectorXi birthVerts;
        if (igl::decimate(positions, indices, (size_t)(indexCt * lodPower), newPos, newTris, birthFaces, birthVerts))
        {
            outIndexData.Reserve(outIndexData.Size() + newTris.rows() * 3);
            for (unsigned i = 0; i < newTris.rows(); ++i)
            {
                outIndexData.Push(newTris.coeff(i, 0));
                outIndexData.Push(newTris.coeff(i, 1));
                outIndexData.Push(newTris.coeff(i, 2));
            }

            unsigned startOffset = outVertexData.Size();
            outVertexData.Resize(outVertexData.Size() + newPos.rows() * vertexSize);
            unsigned char* buffer = outVertexData.Buffer();
            for (unsigned i = 0; i < newPos.rows(); ++i)
            {
                unsigned birthIdx = vertexStart + birthVerts.coeff(i);

                *(Vector3*)(buffer + i * vertexSize + posOffset + startOffset) = Vector3(newPos.coeff(i, 0), newPos.coeff(i, 1), newPos.coeff(i, 2));
                for (auto elem : *elements)
                {
                    switch (elem.type_)
                    {
                    case TYPE_VECTOR2:
                        *(Vector2*)(buffer + i * vertexSize + elem.offset_ + startOffset) = *(Vector2*)(data + birthIdx * vertexSize + elem.offset_);
                        break;
                    case TYPE_VECTOR3:
                        *(Vector3*)(buffer + i * vertexSize + elem.offset_ + startOffset) = *(Vector3*)(data + birthIdx * vertexSize + elem.offset_);
                        break;
                    case TYPE_VECTOR4:
                        *(Vector4*)(buffer + i * vertexSize + elem.offset_ + startOffset) = *(Vector4*)(data + birthIdx * vertexSize + elem.offset_);
                        break;
                    case TYPE_UBYTE4:
                    case TYPE_UBYTE4_NORM:
                        *(unsigned*)(buffer + i * vertexSize + elem.offset_ + startOffset) = *(unsigned*)(data + birthIdx * vertexSize + elem.offset_);
                        break;
                    case TYPE_INT:
                        *(int*)(buffer + i * vertexSize + elem.offset_ + startOffset) = *(int*)(data + birthIdx * vertexSize + elem.offset_);
                        break;
                    }
                }
            }
        }
        else
        {
            URHO3D_LOGERROR("Failed to generate LOD");
        }
    }

    SharedPtr<Model> GenerateModelLOD(Model* forModel, const PODVector<float>& lodPowers, bool packBuffers)
    {
        auto ret = forModel->Clone();
        for (unsigned i = 0; i < ret->GetNumGeometries(); ++i)
        {
            auto srcGeom = ret->GetGeometry(i, 0);
            ret->SetNumGeometryLodLevels(i, lodPowers.Size() + 1);
            for (unsigned l = 0; l < lodPowers.Size(); ++l)
            {
                if (auto lodGeom = GenerateLOD(srcGeom, lodPowers[l]))
                    ret->SetGeometry(i, l + 1, lodGeom);
            }
        }
        return ret;
    }

}