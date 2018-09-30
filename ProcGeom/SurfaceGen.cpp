#include <Urho3D/ProcGeom/SurfaceGen.h>

#include <Urho3D/Math/Color.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Math/Vector4.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Resource/Image.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Graphics/Tangent.h>
#include <Urho3D/Graphics/VertexBuffer.h>

#define PAR_SHAPES_IMPLEMENTATION
#include <ThirdParty/par/par_shapes.h>

namespace Urho3D
{

    Geometry* ParMeshToGeometry(Urho3D::Context* ctx, par_shapes_mesh* mesh)
    {
        // Get index data
        PODVector<unsigned short> indexData;
        indexData.Resize(mesh->ntriangles * 3);
        for (int i = 0; i < indexData.Size(); ++i)
            indexData[i] = mesh->triangles[i];

        struct VertexData
        {
            Vector3 position;
            Vector3 normal;
            Vector4 tangent;
            Vector2 texcoord;
        };

        PODVector<VertexData> vertexData;
        vertexData.Resize(mesh->npoints);
        for (int i = 0; i < mesh->npoints; ++i)
        {
            vertexData[i].position = ((Vector3*)(mesh->points))[i];
            vertexData[i].texcoord = ((Vector2*)(mesh->tcoords))[i];
            vertexData[i].normal = ((Vector3*)(mesh->normals))[i];
        }

        GenerateTangents(vertexData.Buffer(), sizeof(VertexData), indexData.Buffer(), sizeof(unsigned short), 0, indexData.Size(),
            offsetof(VertexData, normal), offsetof(VertexData, texcoord), offsetof(VertexData, tangent));

        Geometry* ret = new Geometry(ctx);
        ret->SetNumVertexBuffers(1);

        VertexBuffer* vbo = new VertexBuffer(ctx);
        vbo->SetShadowed(true);
        vbo->SetSize(vertexData.Size(), {
            VertexElement(TYPE_VECTOR3, SEM_POSITION),
            VertexElement(TYPE_VECTOR3, SEM_NORMAL),
            VertexElement(TYPE_VECTOR4, SEM_TANGENT),
            VertexElement(TYPE_VECTOR2, SEM_TEXCOORD)
        });
        vbo->SetData(vertexData.Buffer());

        IndexBuffer* ibo = new IndexBuffer(ctx);
        ibo->SetShadowed(true);
        ibo->SetSize(indexData.Size(), false);
        ibo->SetData(indexData.Buffer());

        ret->SetVertexBuffer(0, vbo);
        ret->SetIndexBuffer(ibo);
        ret->SetDrawRange(TRIANGLE_LIST, 0, ibo->GetIndexCount());

        return ret;
    }

    PODVector<Vector2> FitProfileToTrim(const PODVector<Vector2>& curvePts, float floorHeight, float ceilingHeight, bool anchorLower)
    {
        PODVector<Vector2> ret;

        float multiplier = anchorLower ? 1 : -1;
        float height = anchorLower ? floorHeight : ceilingHeight;
        float maxY = FLT_MIN;
        float minY = FLT_MAX;
        for (auto pt : curvePts)
        {
            minY = Min(minY, pt.y_);
            maxY = Max(maxY, pt.y_);
        }
        float rangeY = maxY - minY;

#define NORMALIZE(a, b, c) ((a - b) / (c - a))
        for (int i = 0; i < curvePts.Size(); ++i)
        {
            float normY = NORMALIZE(curvePts[i].y_, minY, maxY);
            ret.Push(Vector2(curvePts[i].x_,
                anchorLower ?
                height + rangeY * normY :
                height - (rangeY - rangeY * normY)
            ));
        }

        return ret;
    }

    PODVector<Vector2> FitProfileToBounds(const PODVector<Vector2>& curvePts, float floorHeight, float ceilingHeight)
    {
        PODVector<Vector2> ret;
        float maxY = FLT_MIN;
        float minY = FLT_MAX;
        for (auto pt : curvePts)
        {
            minY = Min(minY, pt.y_);
            maxY = Max(maxY, pt.y_);
        }
        float rangeY = maxY - minY;
        float halfRange = rangeY / 2;

        for (int i = 0; i < curvePts.Size(); ++i)
        {
            float y = curvePts[i].y_;
            y = y > halfRange ? y = ceilingHeight - (maxY - y) : y = floorHeight + (y - minY);
            ret.Push(Vector2(curvePts[i].x_, y));
        }

        return ret;
    }

    Geometry* Lathe(Context* ctx, const PODVector<Vector2>& outerPoints, int slices, bool hollow)
    {
        if (outerPoints.Size() == 0)
        {
            URHO3D_LOGWARNING("Lathe: attempted to lathe an empty set of points");
            return nullptr;
        }

        PODVector<Vector2> modPoints = outerPoints;
        if (!hollow || modPoints.Size() == 1)
        {
            // Enforce a closed surface of revolution by inserting cap points
            if (outerPoints.Front().x_ != 0.0f)
                modPoints.Insert(modPoints.Begin(), outerPoints.Front() * Vector2(0, 1));
            if (outerPoints.Back().x_ != 0.0f)
                modPoints.Push(outerPoints.Back() * Vector2(0, 1));
        }

        auto meshResult = par_shapes_create_parametric([](float const* uv, const int* sliceStack, float* xyz, void* userData) {
            const PODVector<Vector2>& modPoints = *(PODVector<Vector2>*)userData;
            float theta = uv[1] * 2 * PAR_PI;
            xyz[0] = sinf(theta) * modPoints[sliceStack[0]].x_;
            xyz[1] = cosf(theta) * modPoints[sliceStack[0]].x_;
            xyz[2] = modPoints[sliceStack[0]].y_;// uv[0];
        }, slices, modPoints.Size()-1, false, &modPoints);

        if (meshResult == nullptr)
        {
            URHO3D_LOGERROR("Lathe: failed to evaluate surface");
            return nullptr;
        }

        auto weldedMesh = meshResult;// par_shapes_weld(meshResult, M_EPSILON, 0x0);
        //par_shapes_free_mesh(meshResult);
        if (weldedMesh == nullptr)
        {
            URHO3D_LOGERROR("Lathe: failed to weld mesh surface");
            return nullptr;
        }
        par_shapes_compute_normals(weldedMesh);

        Geometry* ret = ParMeshToGeometry(ctx, weldedMesh);
        par_shapes_free_mesh(weldedMesh);
        return ret;
    }

    Geometry* LoftSpine(Context* ctx, const PODVector<Vector2>& profile, const PODVector<Vector3>& points, const PODVector<Vector3>& pointNormals, const PODVector<float>& profileUV, bool squashTails)
    {
        auto loftNormals = pointNormals;
        auto loftPoints = points;
        if (pointNormals.Size() == 0)
        {
            for (int i = 0; i < points.Size(); ++i)
            {
                int prev = i - 1;
                int next = i + 1;
                Vector3 pt = points[i];
                Vector3 cur;
                Vector3 backup;
                if (prev >= 0)
                {
                    auto prevPt = points[prev];
                    prevPt = Quaternion(-90, Vector3::UP) * (pt - prevPt);
                    prevPt.Normalize();
                    backup = prevPt;
                    cur += prevPt;
                }
                else if (i == 0 && points.Front() == points.Back())
                {
                    auto prevPt = points[points.Size() - 2];
                    prevPt = Quaternion(-90, Vector3::UP) * (pt - prevPt);
                    prevPt.Normalize();
                    backup = prevPt;
                    cur += prevPt;
                }
                if (next < points.Size())
                {
                    auto nextPt = points[next];
                    nextPt = Quaternion(-90, Vector3::UP) * (nextPt - pt);
                    nextPt.Normalize();
                    backup = nextPt;
                    cur += nextPt;
                }
                else if (i == points.Size() - 1 && points.Front() == points.Back())
                {
                    auto nextPt = points[1];
                    nextPt = Quaternion(-90, Vector3::UP) * (nextPt - pt);
                    nextPt.Normalize();
                    backup = nextPt;
                    cur += nextPt;
                }
                cur.Normalize();
                if (isnan(cur.x_) || isnan(cur.y_))
                    cur = backup;
                loftNormals.Push(cur);
            }

            PODVector<Vector3> corrected;
            for (unsigned i = 0; i < loftNormals.Size(); ++i)
            {
                if (i == 0 || i == loftNormals.Size() - 1)
                {
                    corrected.Push(loftNormals[i]);
                    continue;
                }
                auto cur = loftNormals[i];
                auto prevNorm = loftNormals[i - 1];
                auto nextNorm = loftNormals[i + 1];

                auto newCur = cur * (1.0f + (1.0f - cur.AbsDotProduct(prevNorm)) + (1.0f - cur.AbsDotProduct(nextNorm)));
                corrected.Push(newCur);
            }
            loftNormals = corrected;
        }

        if (squashTails)
        {
            loftPoints.Insert(0, loftPoints.Front());
            loftPoints.Push(loftPoints.Back());
            loftNormals.Insert(0, loftNormals.Front());
            loftNormals.Push(loftNormals.Back());
        }

        int slices = profile.Size() - 1;
        int edges = loftPoints.Size() - 1;
        PODVector<Vector3> vertices;
        PODVector<Vector2> uv;
        PODVector<unsigned short> indexData;

        PODVector<Vector3> StartingProfile;
        PODVector<Vector3> EndingProfile;

        for (int u = 0; u < edges + 1; ++u)
        {
            float uCoord = (u / (float)(edges));
            Vector3 cornerPt = loftPoints[u];
            Vector3 cornerNorm = loftNormals[u];
            for (int v = 0; v < slices + 1; ++v)
            {
                float vCoord = (v / (float)(slices));
                Vector2 profPt = profile[v];
                Vector3 pt = cornerPt;
                pt.y_ += profPt.y_;
                if ((u == 0 || u == edges) && !squashTails)
                    pt += cornerNorm * profPt.x_;
                else if (!squashTails)
                    pt += cornerNorm * profPt.x_;

                if (u == 0)
                    StartingProfile.Push(pt);
                else if (u == edges)
                    EndingProfile.Push(pt);

                vertices.Push(pt);
                uv.Push(profileUV.Size() ? 
                    Vector2(uCoord, profileUV[v]) :
                    Vector2(uCoord, vCoord));
            }
        }
        int vv = 0;
        int ti = 0;
        for (int stack = 0; stack < edges; stack++)
        {
            for (int slice = 0; slice < slices; slice++)
            {
                int next = slice + 1;
                int faceA = vv + slice + slices + 1;
                int faceB = vv + next;
                int faceC = vv + slice;
                int faceD = vv + slice + slices + 1;
                int faceE = vv + next + slices + 1;
                int faceF = vv + next;
                indexData.Push(faceA);
                indexData.Push(faceB);
                indexData.Push(faceC);

                indexData.Push(faceD);
                indexData.Push(faceE);
                indexData.Push(faceF);
            }
            vv += slices + 1;
        }

        // Convert temporary data to final vertex data
        struct VertexData
        {
            Vector3 position;
            Vector3 normal;
            Vector4 tangent;
            Vector2 texcoord;
        };

        PODVector<VertexData> vertexData;
        vertexData.Resize(vertices.Size());
        for (int i = 0; i < vertices.Size(); ++i)
        {
            vertexData[i].position = vertices[i];
            vertexData[i].texcoord = uv[i];
        }

        // Calculate normals and tangents
        GenerateNormals(vertexData.Buffer(), sizeof(VertexData), indexData.Buffer(), sizeof(unsigned short), 0, indexData.Size(),
            offsetof(VertexData, position), offsetof(VertexData, normal));
        GenerateTangents(vertexData.Buffer(), sizeof(VertexData), indexData.Buffer(), sizeof(unsigned short), 0, indexData.Size(),
            offsetof(VertexData, normal), offsetof(VertexData, texcoord), offsetof(VertexData, tangent));

        // Construct final geometry
        Geometry* ret = new Geometry(ctx);
        ret->SetNumVertexBuffers(1);

        VertexBuffer* vbo = new VertexBuffer(ctx);
        vbo->SetShadowed(true);
        vbo->SetSize(vertexData.Size(), {
            VertexElement(TYPE_VECTOR3, SEM_POSITION),
            VertexElement(TYPE_VECTOR3, SEM_NORMAL),
            VertexElement(TYPE_VECTOR4, SEM_TANGENT),
            VertexElement(TYPE_VECTOR2, SEM_TEXCOORD)
        });
        vbo->SetData(vertexData.Buffer());

        IndexBuffer* ibo = new IndexBuffer(ctx);
        ibo->SetShadowed(true);
        ibo->SetSize(indexData.Size(), false);
        ibo->SetData(indexData.Buffer());

        ret->SetVertexBuffer(0, vbo);
        ret->SetIndexBuffer(ibo);
        ret->SetDrawRange(TRIANGLE_LIST, 0, ibo->GetIndexCount());

        return ret;
    }

    Geometry* Loft(Context* ctx, const PODVector<Vector2>& profile, const PODVector<Matrix3x4>& points, const PODVector<float>& profileUV)
    {
        auto loftPoints = points;

        int slices = profile.Size() - 1;
        int edges = loftPoints.Size() - 1;
        PODVector<Vector3> vertices;
        PODVector<Vector2> uv;
        PODVector<unsigned short> indexData;

        PODVector<Vector3> StartingProfile;
        PODVector<Vector3> EndingProfile;

        for (int u = 0; u < edges + 1; ++u)
        {
            float uCoord = (u / (float)(edges));
            Vector3 cornerPt = loftPoints[u].Translation();
            Vector4 cornerUpV = loftPoints[u].Row(1);
            Vector4 cornerRightV = loftPoints[u].Row(0);
            Vector3 cornerUp(cornerUpV.x_, cornerUpV.y_,cornerUpV.z_);
            Vector3 cornerRight(cornerRightV.x_, cornerRightV.y_, cornerRightV.z_);
            
            for (int v = 0; v < slices + 1; ++v)
            {
                float vCoord = (v / (float)(slices));
                Vector2 profPt = profile[v];
                Vector3 pt = cornerPt;
                pt += cornerUp * profPt.y_ + cornerRight * profPt.x_;

                if (u == 0)
                    StartingProfile.Push(pt);
                else if (u == edges)
                    EndingProfile.Push(pt);

                vertices.Push(pt);
                uv.Push(profileUV.Size() ?
                    Vector2(uCoord, profileUV[v]) :
                    Vector2(uCoord, vCoord));
            }
        }
        int vv = 0;
        int ti = 0;
        for (int stack = 0; stack < edges; stack++)
        {
            for (int slice = 0; slice < slices; slice++)
            {
                int next = slice + 1;
                int faceA = vv + slice + slices + 1;
                int faceB = vv + next;
                int faceC = vv + slice;
                int faceD = vv + slice + slices + 1;
                int faceE = vv + next + slices + 1;
                int faceF = vv + next;
                indexData.Push(faceA);
                indexData.Push(faceB);
                indexData.Push(faceC);

                indexData.Push(faceD);
                indexData.Push(faceE);
                indexData.Push(faceF);
            }
            vv += slices + 1;
        }

        // Convert temporary data to final vertex data
        struct VertexData
        {
            Vector3 position;
            Vector3 normal;
            Vector4 tangent;
            Vector2 texcoord;
        };

        PODVector<VertexData> vertexData;
        vertexData.Resize(vertices.Size());
        for (int i = 0; i < vertices.Size(); ++i)
        {
            vertexData[i].position = vertices[i];
            vertexData[i].texcoord = uv[i];
        }

        // Calculate normals and tangents
        GenerateNormals(vertexData.Buffer(), sizeof(VertexData), indexData.Buffer(), sizeof(unsigned short), 0, indexData.Size(),
            offsetof(VertexData, position), offsetof(VertexData, normal));
        GenerateTangents(vertexData.Buffer(), sizeof(VertexData), indexData.Buffer(), sizeof(unsigned short), 0, indexData.Size(),
            offsetof(VertexData, normal), offsetof(VertexData, texcoord), offsetof(VertexData, tangent));

        // Construct final geometry
        Geometry* ret = new Geometry(ctx);
        ret->SetNumVertexBuffers(1);

        VertexBuffer* vbo = new VertexBuffer(ctx);
        vbo->SetShadowed(true);
        vbo->SetSize(vertexData.Size(), {
            VertexElement(TYPE_VECTOR3, SEM_POSITION),
            VertexElement(TYPE_VECTOR3, SEM_NORMAL),
            VertexElement(TYPE_VECTOR4, SEM_TANGENT),
            VertexElement(TYPE_VECTOR2, SEM_TEXCOORD)
        });
        vbo->SetData(vertexData.Buffer());

        IndexBuffer* ibo = new IndexBuffer(ctx);
        ibo->SetShadowed(true);
        ibo->SetSize(indexData.Size(), false);
        ibo->SetData(indexData.Buffer());

        ret->SetVertexBuffer(0, vbo);
        ret->SetIndexBuffer(ibo);
        ret->SetDrawRange(TRIANGLE_LIST, 0, ibo->GetIndexCount());

        return ret;
    }

    Image* GenerateUVImage(Context* ctx, Geometry* geo, int width, int height)
    {
        Image* ret = new Image(ctx);
        if (!ret->SetSize(width, height, 3))
            return ret;
        ret->Clear(Color::BLACK);

        GenerateUVImage(ret, geo);

        return ret;
    }

    void GenerateUVImage(Image* image, Geometry* geo)
    {
        const auto width = image->GetWidth();
        const auto height = image->GetHeight();

        const unsigned char* vertexData;
        const unsigned char* indexData;
        unsigned vertexSize;
        unsigned indexSize;
        const PODVector<VertexElement>* elements;
        geo->GetRawData(vertexData, vertexSize, indexData, indexSize, elements);
        const bool largeIndices = indexSize == sizeof(unsigned);

        if (!vertexData || !elements || VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR2, SEM_TEXCOORD) == -1)
        {
            URHO3D_LOGERROR("GenerateUVImage requires texture coordinates");
            return;
        }

        unsigned uvOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR2, SEM_TEXCOORD);

        for (int idx = 0; idx < geo->GetIndexCount(); idx += 3)
        {
            const unsigned indices[] = {
                largeIndices ? ((unsigned*)indexData)[idx] : ((unsigned short*)indexData)[idx],
                largeIndices ? ((unsigned*)indexData)[idx + 1] : ((unsigned short*)indexData)[idx + 1],
                largeIndices ? ((unsigned*)indexData)[idx + 2] : ((unsigned short*)indexData)[idx + 2],
            };

            Vector2 a = *((Vector2*)(vertexData + vertexSize * indices[0] + uvOffset));
            Vector2 b = *((Vector2*)(vertexData + vertexSize * indices[1] + uvOffset));
            Vector2 c = *((Vector2*)(vertexData + vertexSize * indices[2] + uvOffset));
            image->DrawLine(a, b, Color::WHITE);
            image->DrawLine(a, c, Color::WHITE);
            image->DrawLine(b, c, Color::WHITE);
        }
    }

    void GenerateNormals(void* vertexData, unsigned vertexSize, const void* indexData, unsigned indexSize, unsigned indexStart, unsigned indexCount, unsigned positionOffset, unsigned normalOffset, bool flip)
    {
        static auto GetIndex = [](void*& indexPointer, unsigned indexSize) -> unsigned
        {
            if (indexSize == sizeof(unsigned short))
            {
                auto& p = (unsigned short*&)indexPointer;
                return *p++;
            }
            else
            {
                auto& p = (unsigned*&)indexPointer;
                return *p++;
            }
        };

        unsigned minVertex = M_MAX_UNSIGNED;
        unsigned maxVertex = 0;
        auto* vertices = (unsigned char*)vertexData;

        auto* indexPointer = const_cast<void*>(indexData);
        for (unsigned i = indexStart; i < indexStart + indexCount; ++i)
        {
            unsigned v = GetIndex(indexPointer, indexSize);
            if (v < minVertex)
                minVertex = v;
            if (v > maxVertex)
                maxVertex = v;
        }

        unsigned vertexCount = maxVertex + 1;
        PODVector<Vector3> normals;
        normals.Resize(vertexCount);

        indexPointer = const_cast<void*>(indexData);
        for (unsigned i = indexStart; i < indexStart + indexCount; i += 3)
        {
            unsigned i1 = GetIndex(indexPointer, indexSize);
            unsigned i2 = GetIndex(indexPointer, indexSize);
            unsigned i3 = GetIndex(indexPointer, indexSize);

            auto v1 = *(Vector3*)(vertices + vertexSize * i1 + positionOffset);
            auto v2 = *(Vector3*)(vertices + vertexSize * i2 + positionOffset);
            auto v3 = *(Vector3*)(vertices + vertexSize * i3 + positionOffset);

            auto ac = v3 - v1;
            auto ab = v2 - v1;
            ab.Normalize();
            ac.Normalize();
            auto norm = ac.CrossProduct(ab).Normalized();
            normals[i1] += norm;
            normals[i2] += norm;
            normals[i3] += norm;
        }

        for (auto& v : normals)
            v = flip ? -v.Normalized() : v.Normalized();

        for (unsigned i = minVertex; i <= maxVertex; i++)
            *(Vector3*)(vertices + i * vertexSize + normalOffset) = normals[i];
    }

}
