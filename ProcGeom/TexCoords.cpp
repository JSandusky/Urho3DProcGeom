#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/ProcGeom/GeoTransform.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Graphics/Tangent.h>
#include <Urho3D/ProcGeom/TexCoords.h>
#include <Urho3D/Graphics/VertexBuffer.h>

#include <Urho3D/ProcGeom/UVAtlas/UVAtlas.h>
#include <Urho3D/ProcGeom/UVAtlas/Adjacency.h>

namespace Urho3D
{

    Geometry* CalculateUV(Geometry* src, unsigned width, unsigned height, int quality, int charts, float stretch, float gutter, unsigned uvSet)
    {
        std::vector<unsigned> indexBuffer;

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
        const unsigned texOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR2, SEM_TEXCOORD, uvSet);

        if (posOffset == M_MAX_UNSIGNED || texOffset == M_MAX_UNSIGNED)
        {
            URHO3D_LOGERROR("Position and tex-coord are required to calculate tex-coords");
            return nullptr;
        }

        for (unsigned i = indexStart; i < indexStart + indexCt; i += 3)
        {
            unsigned indices[] = {
                largeIndices ? ((unsigned*)indexData)[i] : ((unsigned short*)indexData)[i],
                largeIndices ? ((unsigned*)indexData)[i + 1] : ((unsigned short*)indexData)[i + 1],
                largeIndices ? ((unsigned*)indexData)[i + 2] : ((unsigned short*)indexData)[i + 2],
            };

            indexBuffer.push_back(indices[0]);
            indexBuffer.push_back(indices[1]);
            indexBuffer.push_back(indices[2]);
        }

        std::vector<Vector3> positionBuffer;
        for (unsigned i = 0; i < vertexCt; ++i)
            positionBuffer.push_back(*(Vector3*)(data + i * vertexSize + posOffset));

        std::vector<unsigned> adjacency;
        adjacency.resize(indexBuffer.size());

#define CLAMP(a,b,c) ((a < b) ? b : (a > c ? c : a))
        quality = CLAMP(quality, 0, 2);
        stretch = CLAMP(stretch, 0.0f, 1.0f);
        charts = CLAMP(charts, 0, 100);
        gutter = CLAMP(gutter, 0.0f, 50.0f);

        if (!FAILED(DirectX::GenerateAdjacencyAndPointReps(indexBuffer.data(), indexBuffer.size() / 3, (DirectX::XMFLOAT3*)positionBuffer.data(), positionBuffer.size(), M_EPSILON, 0, adjacency.data())))
        {
            std::vector<DirectX::UVAtlasVertex> outMesh;
            std::vector<unsigned char> outIndices;
            std::vector<unsigned> remappedVertices;

            HRESULT result = DirectX::UVAtlasCreate(
                (DirectX::XMFLOAT3*)positionBuffer.data() /*vertex positions*/,
                positionBuffer.size() /*position count*/,
                indexBuffer.data() /*indices*/,
                DXGI_FORMAT_R32_UINT /*index type*/,
                indexBuffer.size() / 3 /*N Faces*/,
                charts /*max charts*/,
                stretch /*max stretch*/,
                width /*width*/,
                height /*height*/,
                gutter /*gutter*/,
                adjacency.data() /*adjacency data*/,
                0x0 /*reps*/,
                0x0 /*IMT array*/,
                nullptr /*callback*/,
                DirectX::UVATLAS_DEFAULT_CALLBACK_FREQUENCY /*callback frequency*/,
                quality /* options */,
                outMesh /* output */,
                outIndices /* output indices */,
                0x0,
                &remappedVertices
            );

            if (!FAILED(result))
            {
                unsigned char* newData = new unsigned char[outMesh.size() * vertexSize];
                for (unsigned i = 0; i < outMesh.size(); ++i)
                {
                    auto oVert = outMesh[i];

                    // first copy the data from the remapped vertex, then apply the UV-coord
                    memcpy(newData + vertexSize * i, data + vertexSize * remappedVertices[i], vertexSize);
                    *(Vector2*)(newData + vertexSize * i + texOffset) = Vector2(oVert.uv.x, oVert.uv.y);
                }

                // if we need to then calculate tangents, always using uv-set 0 for the tex-offset
                auto normOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_NORMAL);
                auto texOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR2, SEM_TEXCOORD, 0);
                auto tanOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR4, SEM_TANGENT);
                if (uvSet == 0 && normOffset != M_MAX_UNSIGNED && texOffset != M_MAX_UNSIGNED && tanOffset != M_MAX_UNSIGNED)
                    GenerateTangents(newData, vertexSize, indexData, indexSize, indexStart, indexCt, normOffset, texOffset, tanOffset);

                Geometry* result = new Geometry(src->GetContext());
                VertexBuffer* vtxBuffer = new VertexBuffer(src->GetContext());
                vtxBuffer->SetShadowed(true);
                vtxBuffer->SetSize(outMesh.size(), *elements);
                vtxBuffer->SetData(newData);
                
                IndexBuffer* idxBuffer = new IndexBuffer(src->GetContext());
                idxBuffer->SetShadowed(true);
                idxBuffer->SetSize(outIndices.size(), true);
                idxBuffer->SetData(outIndices.data());

                result->SetNumVertexBuffers(1);
                result->SetVertexBuffer(0, vtxBuffer);
                result->SetIndexBuffer(idxBuffer);
                result->SetDrawRange(TRIANGLE_LIST, indexStart, indexCt);

                URHO3D_LOGDEBUG("Finished generating UV coordinates");
                return result;
            }
            else
                URHO3D_LOGERROR("Failed to generate UV coordinates");
        }
        else
        {
            URHO3D_LOGERROR("Failed to generate adjacency data for UV coordinate generation");
        }
        return nullptr; // failure got us here
    }

    Geometry* PlanarUV(Geometry* src, const Plane& plane, const Vector2& scale, const Vector2& offset)
    {
        auto elements = src->GetVertexBuffer(0)->GetElements();

        // statics cheat around requiring a capturing lambda
        static Plane localPlane;
        static Vector3 planeRight;
        static Vector3 planeUp;
        static unsigned texCoordOffset;
        static unsigned posOffset;
        static Vector2 scl;
        static Vector2 ofs;

        localPlane = plane;
        planeRight = localPlane.normal_.CrossProduct(Vector3::UP);
        if (localPlane.normal_ == Vector3::UP)
            planeRight = Vector3::RIGHT;
        planeUp = localPlane.normal_.CrossProduct(planeRight);
        texCoordOffset = VertexBuffer::GetElementOffset(elements, TYPE_VECTOR2, SEM_TEXCOORD);
        posOffset = VertexBuffer::GetElementOffset(elements, TYPE_VECTOR3, SEM_POSITION);
        scl = scale;
        ofs = offset;

        if (texCoordOffset == M_MAX_UNSIGNED || posOffset == M_MAX_UNSIGNED)
        {
            URHO3D_LOGERROR("Vertex must have tex-coord and position to calculate UV coordinates");
            return nullptr;
        }

        auto ret = ConvertVertexData(src, elements, true, [](unsigned char* newData, const PODVector<VertexElement>* newElems, const unsigned char* oldData, const PODVector<VertexElement>* oldElems) {
            memcpy(newData, oldData, VertexBuffer::GetVertexSize(*newElems));
            auto vec = *(Vector3*)(newData + posOffset);
            auto onPlane = localPlane.Project(vec);
            float x = onPlane.DotProduct(planeRight);
            float y = onPlane.DotProduct(planeUp);

            *(Vector2*)(newData + texCoordOffset) = Vector2(x,y) * scl + ofs;
        });
    }

}