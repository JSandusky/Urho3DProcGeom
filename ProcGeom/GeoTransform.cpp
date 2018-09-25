#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/ProcGeom/GeoTransform.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Tangent.h>
#include <Urho3D/Graphics/VertexBuffer.h>
#include <Urho3D/Graphics/VertexDeclaration.h>

namespace Urho3D
{

    
    VertexBuffer* Transform(Geometry* src, const Matrix3x4& transform, const Matrix3& uvTransform, bool recalcTangents)
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

        const unsigned posOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);
        const unsigned normOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_NORMAL, 0);
        const unsigned tangentOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR4, SEM_TANGENT, 0);
        const unsigned uvOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR2, SEM_TEXCOORD, 0);
        if (posOffset == M_MAX_UNSIGNED)
        {
            URHO3D_LOGERROR("Cannot transform a mesh without the appropriate position semantic");
            return nullptr;
        }

        unsigned char* newData = new unsigned char[vertexSize * vertexCt];
        const auto rotMat = transform.RotationMatrix();
        for (unsigned i = vertexStart; i < vertexCt; ++i)
        {
            Vector3 pos = *(Vector3*)(data + vertexSize * i + posOffset);
            pos = transform * pos;
            *(Vector3*)(newData + vertexSize * i + normOffset) = pos;

            if (normOffset != M_MAX_UNSIGNED)
            {
                Vector3 norm = *(Vector3*)(data + vertexSize * i + normOffset);
                norm = rotMat * norm;
                *(Vector3*)(newData + vertexSize * i + normOffset) = norm.Normalized();
            }

            if (uvOffset != M_MAX_UNSIGNED && uvTransform != Matrix3::IDENTITY)
            {
                Vector2 uvVal = *(Vector2*)(data + vertexSize * i + uvOffset);
                Vector2 newUVVal(uvVal.x_ * transform.m00_ + uvVal.x_ * transform.m01_ + transform.m02_,
                    uvVal.y_ * transform.m10_ + uvVal.y_ * transform.m11_ + transform.m12_);
                *(Vector2*)(newData + vertexSize * i + uvOffset) = newUVVal;
            }
        }

        if (tangentOffset != M_MAX_UNSIGNED  && normOffset != M_MAX_UNSIGNED && uvOffset != M_MAX_UNSIGNED && recalcTangents)
            GenerateTangents(newData, vertexSize, indexData, indexSize, indexStart, indexCt, normOffset, uvOffset, tangentOffset);

        VertexBuffer* newVtxBuffer = new VertexBuffer(src->GetContext());
        newVtxBuffer->SetShadowed(true);
        newVtxBuffer->SetSize(vertexCt, *elements, false);
        newVtxBuffer->SetData(newData);
        delete[] newData;

        return newVtxBuffer;
    }

    
    VertexBuffer* TransformUV(Geometry* src, const Matrix3& transform, int uvSet, bool recalcTangents)
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

        const unsigned normOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_NORMAL, 0);
        const unsigned tangentOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR4, SEM_TANGENT, 0);
        const unsigned uvOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR2, SEM_TEXCOORD, 0);
        if (uvOffset == M_MAX_UNSIGNED)
        {
            URHO3D_LOGERROR("Cannot TransformUV a mesh without the appropriate UV set");
            return nullptr;
        }

        unsigned char* newData = new unsigned char[vertexSize * vertexCt];
        for (unsigned i = vertexStart; i < vertexCt; ++i)
        {
            Vector2 uvVal = *(Vector2*)(data + vertexSize * i + uvOffset);
            Vector2 newUVVal(uvVal.x_ * transform.m00_ + uvVal.x_ * transform.m01_ + transform.m02_,
                uvVal.y_ * transform.m10_ + uvVal.y_ * transform.m11_ + transform.m12_);
            *(Vector2*)(newData + vertexSize * i + uvOffset) = newUVVal;
        }

        if (tangentOffset != M_MAX_UNSIGNED  && normOffset != M_MAX_UNSIGNED && uvOffset != M_MAX_UNSIGNED && recalcTangents)
            GenerateTangents(newData, vertexSize, indexData, indexSize, indexStart, indexCt, normOffset, uvOffset, tangentOffset);

        VertexBuffer* newVtxBuffer = new VertexBuffer(src->GetContext());
        newVtxBuffer->SetShadowed(true);
        newVtxBuffer->SetSize(vertexCt, *elements, false);
        newVtxBuffer->SetData(newData);
        delete[] newData;

        return newVtxBuffer;
    }

    PODVector<Vector3> CalculateTriangleCenters(Geometry* src)
    {
        if (src->GetPrimitiveType() != TRIANGLE_LIST)
        {
            URHO3D_LOGERROR("Can only compute triangle centers on a TRIANGLE_LIST geometry");
            return PODVector<Vector3>();
        }

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

        const unsigned posOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);
        if (posOffset == M_MAX_UNSIGNED)
        {
            URHO3D_LOGERROR("Cannot calculate triangle centers without the appropriate position semantic");
            return PODVector<Vector3>();
        }

        PODVector<Vector3> triCenters;

        for (unsigned i = indexStart; i < indexStart + indexCt; i += 3)
        {
            unsigned indices[] = {
                indexSize == 2 ? *(unsigned short*)(indexData)[i] : *(unsigned*)(indexData)[i],
                indexSize == 2 ? *(unsigned short*)(indexData)[i + 1] : *(unsigned*)(indexData)[i + 1],
                indexSize == 2 ? *(unsigned short*)(indexData)[i + 2] : *(unsigned*)(indexData)[i + 2]
            };

            Vector3 a = *(Vector3*)(data + vertexSize * indices[0] + posOffset);
            Vector3 b = *(Vector3*)(data + vertexSize * indices[0] + posOffset);
            Vector3 c = *(Vector3*)(data + vertexSize * indices[0] + posOffset);

            triCenters.Push((a + b + c) / 3);
        }

        return triCenters;
    }

    PODVector<Vector3> CalculateTriangleNormals(Geometry* src)
    {
        if (src->GetPrimitiveType() != TRIANGLE_LIST)
        {
            URHO3D_LOGERROR("Can only compute triangle normals on a TRIANGLE_LIST geometry");
            return PODVector<Vector3>();
        }

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
        if (posOffset == M_MAX_UNSIGNED)
        {
            URHO3D_LOGERROR("Cannot calculate triangle normals without the appropriate position semantic");
            return PODVector<Vector3>();
        }

        PODVector<Vector3> triNorms;

        for (unsigned i = indexStart; i < indexStart + indexCt; i += 3)
        {
            unsigned indices[] = {
                largeIndices ? ((unsigned*)indexData)[i] : ((unsigned short*)indexData)[i],
                largeIndices ? ((unsigned*)indexData)[i + 1] : ((unsigned short*)indexData)[i + 1],
                largeIndices ? ((unsigned*)indexData)[i + 2] : ((unsigned short*)indexData)[i + 2],
            };

            Vector3 a = *(Vector3*)(data + vertexSize * indices[0] + posOffset);
            Vector3 b = *(Vector3*)(data + vertexSize * indices[0] + posOffset);
            Vector3 c = *(Vector3*)(data + vertexSize * indices[0] + posOffset);

            auto ab = b - a;
            auto ac = c - a;
            triNorms.Push(ac.CrossProduct(ab).Normalized());
        }

        return triNorms;
    }

    Geometry* MakeVerticesUnique(Geometry* src)
    {
        if (src->GetPrimitiveType() != TRIANGLE_LIST)
        {
            URHO3D_LOGERROR("Can only make the vertices unique in a triangle list at present");
            return nullptr;
        }

        auto vtxBuffer = src->GetVertexBuffer(0);
        auto idxBuffer = src->GetIndexBuffer();
        auto elements = vtxBuffer->GetElements();
        unsigned vertexSize = vtxBuffer->GetVertexSize();
        unsigned vertexCount = vtxBuffer->GetVertexCount();

        unsigned indexSize = idxBuffer->GetIndexSize();
        unsigned indexCount = idxBuffer->GetIndexCount();
        const bool largeIndices = indexSize == sizeof(unsigned);

        auto vertexData = vtxBuffer->GetShadowData();
        auto indexData = idxBuffer->GetShadowData();

        unsigned char* newVertexData = new unsigned char[vertexSize * indexCount];
        for (unsigned idx = 0; idx < indexCount; idx += 3)
        {
            const unsigned indices[] = {
                largeIndices ? ((unsigned*)indexData)[idx] : ((unsigned short*)indexData)[idx],
                largeIndices ? ((unsigned*)indexData)[idx + 1] : ((unsigned short*)indexData)[idx + 1],
                largeIndices ? ((unsigned*)indexData)[idx + 2] : ((unsigned short*)indexData)[idx + 2],
            };

            memcpy(newVertexData + vertexSize * idx, vertexData + vertexSize * indices[0], vertexSize);
            memcpy(newVertexData + vertexSize * (idx+1), vertexData + vertexSize * indices[1], vertexSize);
            memcpy(newVertexData + vertexSize * (idx+2), vertexData + vertexSize * indices[2], vertexSize);
        }

        Geometry* resultGeom = new Geometry(src->GetContext());

        PODVector<unsigned short> indices;
        indices.Reserve(idxBuffer->GetIndexCount());
        for (int i = 0; i < idxBuffer->GetIndexCount(); ++i)
            indices.Push(i);

        VertexBuffer* newVtxBuffer = new VertexBuffer(src->GetContext());
        newVtxBuffer->SetShadowed(true);
        newVtxBuffer->SetSize(indexCount, elements);
        newVtxBuffer->SetData(newVertexData);
        resultGeom->SetNumVertexBuffers(1);
        resultGeom->SetVertexBuffer(0, newVtxBuffer);
        delete[] newVertexData;

        IndexBuffer* newIdxBuffer = new IndexBuffer(src->GetContext());
        newIdxBuffer->SetShadowed(true);
        newIdxBuffer->SetSize(indices.Size(), false);
        newIdxBuffer->SetData(indices.Buffer());
        resultGeom->SetIndexBuffer(newIdxBuffer);

        resultGeom->SetDrawRange(TRIANGLE_LIST, 0, indices.Size());

        return resultGeom;
    }

    SharedPtr<Model> MakeModelVerticesUnique(Model* src)
    {
        auto resultModel = src->Clone();
        for (unsigned i = 0; i < src->GetNumGeometries(); ++i)
        {
            auto geometry = src->GetGeometry(i, 0);
            auto uniqueGeometry = MakeVerticesUnique(geometry);
            resultModel->SetGeometry(i, 0, uniqueGeometry);
        }

        return resultModel;
    }

    void ProcessVertices(Geometry* src, GeoVertexFilter processor)
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

        for (unsigned i = 0; i < vertexCt; ++i)
            processor(data + vertexSize * i, elements);
    }

    void ProcessEdges(Geometry* src, GeoEdgeFilter processor)
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

        for (unsigned i = indexStart; i < indexStart + indexCt; i += 3)
        {
            unsigned indices[] = {
                largeIndices ? ((unsigned*)indexData)[i] : ((unsigned short*)indexData)[i],
                largeIndices ? ((unsigned*)indexData)[i + 1] : ((unsigned short*)indexData)[i + 1],
                largeIndices ? ((unsigned*)indexData)[i + 2] : ((unsigned short*)indexData)[i + 2],
            };

            processor(data + vertexSize * indices[0], data + vertexSize * indices[1], elements);
            processor(data + vertexSize * indices[1], data + vertexSize * indices[2], elements);
            processor(data + vertexSize * indices[2], data + vertexSize * indices[0], elements);
        }
    }

    void ProcessFaces(Geometry* src, GeoTriangleFilter processor)
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

        for (unsigned i = indexStart; i < indexStart + indexCt; i += 3)
        {
            unsigned indices[] = {
                largeIndices ? ((unsigned*)indexData)[i] : ((unsigned short*)indexData)[i],
                largeIndices ? ((unsigned*)indexData)[i + 1] : ((unsigned short*)indexData)[i + 1],
                largeIndices ? ((unsigned*)indexData)[i + 2] : ((unsigned short*)indexData)[i + 2],
            };

            processor(data + vertexSize * indices[0], data + vertexSize * indices[1], data + vertexSize * indices[1], elements);
        }
    }

    Geometry* ConvertVertexData(Geometry* src, const PODVector<VertexElement>& vertElements, bool recalcTangents, GeoVertexConverter conversion)
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

        const auto newVertSize = VertexBuffer::GetVertexSize(vertElements);
        unsigned char* newData = new unsigned char[vertexCt * newVertSize];
        for (unsigned i = 0; i < vertexCt; ++i)
            conversion(newData + i * newVertSize, &vertElements, data + vertexSize * i, elements);

        if (recalcTangents)
        {
            auto normOffset = VertexBuffer::GetElementOffset(vertElements, TYPE_VECTOR3, SEM_NORMAL);
            auto texOffset = VertexBuffer::GetElementOffset(vertElements, TYPE_VECTOR2, SEM_TEXCOORD);
            auto tanOffset = VertexBuffer::GetElementOffset(vertElements, TYPE_VECTOR4, SEM_TANGENT);
            if (normOffset != M_MAX_UNSIGNED && texOffset != M_MAX_UNSIGNED && tanOffset != M_MAX_UNSIGNED)
                GenerateTangents(newData, newVertSize, indexData, indexSize, indexStart, indexCt, normOffset, texOffset, tanOffset);
        }

        VertexBuffer* newVtxBuffer = new VertexBuffer(src->GetContext());
        newVtxBuffer->SetShadowed(true);
        newVtxBuffer->SetSize(vertexCt, vertElements, false);
        newVtxBuffer->SetData(newData);
        delete[] newData;

        Geometry* ret = new Geometry(src->GetContext());
        ret->SetNumVertexBuffers(1);
        ret->SetVertexBuffer(0, newVtxBuffer);
        ret->SetIndexBuffer(src->GetIndexBuffer());
        ret->SetDrawRange(src->GetPrimitiveType(), src->GetIndexStart(), src->GetIndexCount());
        return ret;
    }

}