#include <Urho3D/ProcGeom/CSGUtility.h>

#include <Urho3D/Core/Context.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Container/Ptr.h>
#include <Urho3D/Graphics/Tangent.h>
#include <Urho3D/Math/Vector2.h>
#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Graphics/VertexBuffer.h>

#include <Carve/carve.hpp>
#include <Carve/csg.hpp>
#include <Carve/csg_triangulator.hpp>
#include <Carve/interpolator.hpp>
#include <Carve/mesh.hpp>
#include <Carve/mesh_simplify.hpp>

#include <Urho3D/DebugNew.h>

namespace Urho3D
{

    Geometry* Subtract(Context* ctx, Geometry* src, Geometry* subtract, bool encodeTexCoords)
    {
        PODVector<CSGOperand> operands;
        operands.Push(CSGOperand(src, CSG_Add, 0.0f));
        operands.Push(CSGOperand(subtract, CSG_Subtract, 0.0f));
        return PerformCSG(ctx, operands, encodeTexCoords);
    }
    Geometry* Union(Context* ctx, Geometry* src, Geometry* add, bool encodeTexCoords)
    {
        PODVector<CSGOperand> operands;
        operands.Push({ src, CSG_Add, 0.0f });
        operands.Push({ add, CSG_Add, 0.0f });
        return PerformCSG(ctx, operands, encodeTexCoords);
    }
    Geometry* Intersect(Context* ctx, Geometry* src, Geometry* intersect, bool encodeTexCoords)
    {
        PODVector<CSGOperand> operands;
        operands.Push({ src, CSG_Add, 0.0f });
        operands.Push({ intersect, CSG_Intersect, 0.0f });
        return PerformCSG(ctx, operands, encodeTexCoords);
    }

    struct CSGVertex
    {
        Vector3 pos;
        Vector3 nor;
        Vector2 uv;

        inline bool operator<(const CSGVertex& rhs) const
        {
            return std::tuple<Vector3, Vector3, Vector2>(pos, nor, uv) <  std::tuple<Vector3, Vector3, Vector2>(rhs.pos, rhs.nor, rhs.uv);
        }
    };

    #define MESH_DIM 3
    typedef carve::mesh::MeshSet<MESH_DIM> CSGMeshType;
    typedef std::map<CSGVertex, unsigned> CSGColocalMap;

    class CSGMeshData : public RefCounted
    {
    public:
        CSGMeshData(CSGMeshType* mesh) : mesh_(mesh)
        {

        }
        virtual ~CSGMeshData()
        {
            if (mesh_)
                delete mesh_;
        }
        CSGMeshType* mesh_;
    };

    struct CSGAttributes
    {
        carve::interpolate::FaceVertexAttr<Vector3> NormalsAttr;
        carve::interpolate::FaceVertexAttr<Vector2> UVAttr;

        void Hook(carve::csg::CSG& csg)
        {
            NormalsAttr.installHooks(csg);
            UVAttr.installHooks(csg);
        }
    };

    SharedPtr<CSGMeshData> FillMesh(CSGOperand& operand, CSGAttributes& attrs, Vector2 uvCoordOffset)
    {
        SharedPtr<CSGMeshData> ret;
        if (Geometry* geo = operand.geometry_)
        {
            std::vector<CSGMeshType::face_t*> faces = std::vector<CSGMeshType::face_t *>();
            std::vector<CSGMeshType::vertex_t*> vertexStorage(geo->GetVertexCount());

            const unsigned char* vertexData;
            const unsigned char* indexData;
            unsigned vertexSize;
            unsigned indexSize;
            const PODVector<VertexElement>* elements;
            geo->GetRawData(vertexData, vertexSize, indexData, indexSize, elements);
            const bool largeIndices = indexSize == sizeof(unsigned);

            if (!vertexData || !elements || VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION) != 0)
                return nullptr;

            unsigned posOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);
            unsigned uvOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR2, SEM_TEXCOORD);
            unsigned normOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_NORMAL);

            for (int vertIdx = 0; vertIdx < geo->GetVertexCount(); ++vertIdx)
            {
                auto position = *((Vector3*)(&vertexData[vertIdx * vertexSize + posOffset]));
                CSGMeshType::vertex_t* vertData = new CSGMeshType::vertex_t(carve::geom::VECTOR(position.x_, position.y_, position.z_));
                vertexStorage[vertIdx] = vertData;
            }


            for (int idx = 0; idx < geo->GetIndexCount(); idx += 3)
            {
                const unsigned indices[] = {
                    largeIndices ? ((unsigned*)indexData)[idx] : ((unsigned short*)indexData)[idx],
                    largeIndices ? ((unsigned*)indexData)[idx + 1] : ((unsigned short*)indexData)[idx + 1],
                    largeIndices ? ((unsigned*)indexData)[idx + 2] : ((unsigned short*)indexData)[idx+2],
                };

                auto a = vertexStorage[indices[0]];
                auto b = vertexStorage[indices[1]];
                auto c = vertexStorage[indices[2]];

                auto face = new CSGMeshType::face_t(a, b, c);

                // normals
#define WRITE_ATTRIBUTE(ATTR, BUFFER, INDEX, OINDEX, SEM, TYPE, OFFSET) ATTR.setAttribute(face, OINDEX, *((TYPE*)(&vertexData)[indices[INDEX] * vertexSize + SEM + INDEX]) + OFFSET)

                WRITE_ATTRIBUTE(attrs.NormalsAttr, vertexData, 0, 0, normOffset, Vector3, Vector3::ZERO);
                WRITE_ATTRIBUTE(attrs.NormalsAttr, vertexData, 1, 1, normOffset, Vector3, Vector3::ZERO);
                WRITE_ATTRIBUTE(attrs.NormalsAttr, vertexData, 2, 2, normOffset, Vector3, Vector3::ZERO);
                WRITE_ATTRIBUTE(attrs.UVAttr, vertexData, 0, 0, uvOffset, Vector2, uvCoordOffset);
                WRITE_ATTRIBUTE(attrs.UVAttr, vertexData, 1, 1, uvOffset, Vector2, uvCoordOffset);
                WRITE_ATTRIBUTE(attrs.UVAttr, vertexData, 2, 2, uvOffset, Vector2, uvCoordOffset);
                faces.push_back(face);
            }

            auto retMesh = new CSGMeshType(faces);
            ret = new CSGMeshData(retMesh);
            //retMesh->canonicalize();
        }
        return ret;
    }

    CSGVertex CSGGetFaceVertex(CSGMeshType::face_t* face, unsigned vertIdx, CSGMeshType::vertex_t* vert, CSGAttributes* attr)
    {
        CSGVertex ret;
        ret.pos = Vector3(vert->v[0], vert->v[1], vert->v[2]);
        Vector3 nor = attr->NormalsAttr.getAttribute(face, vertIdx);
        if (nor.DotProduct(Vector3(face->plane.N[0], face->plane.N[1], face->plane.N[2]).Normalized()) < 0)
            nor *= -1;
        ret.nor = nor.Normalized();
        ret.uv = attr->UVAttr.getAttribute(face, vertIdx);
        return ret;
    }

    struct CSGOutVertex
    {
        Vector3 position_;
        Vector3 normal_;
        Vector4 tangent_;
        Vector2 texcoord_;
    };

    unsigned CSGAddVertex(CSGVertex& vert, CSGColocalMap& colocalTable, PODVector<CSGVertex>& allVerts, PODVector<CSGOutVertex>& meshData)
    {
        auto found = colocalTable.find(vert);
        if (found == colocalTable.end())
        {
            unsigned idx = allVerts.Size();
            allVerts.Push(vert);
            colocalTable[vert] = idx;

            CSGOutVertex v;
            v.position_ = vert.pos;
            v.normal_ = vert.nor;
            v.texcoord_ = vert.uv;
            meshData.Push(v);

            return idx;
        }
        return found->second;
    }

    Geometry* FromCSGMeshCompact(Context* context, CSGMeshType* meshData, CSGAttributes* attr)
    {
        Geometry* outData = new Geometry(context);
        CSGColocalMap vertexTable;
        PODVector<CSGVertex> outVertices;
        if (meshData)
        {
            auto mesh = meshData->meshes[0];
            PODVector<CSGOutVertex> vertexData;
            PODVector<unsigned short> indexData;
            for (auto faceIter = meshData->faceBegin(); faceIter != meshData->faceEnd(); ++faceIter)
            {
                CSGMeshType::face_t* face = *faceIter;
                std::vector<CSGMeshType::vertex_t *> verts;
                face->getVertices(verts);

                // construct a vertex for each face vertex
                CSGVertex va = CSGGetFaceVertex(face, 0, verts[0], attr);
                CSGVertex vb = CSGGetFaceVertex(face, 1, verts[1], attr);
                CSGVertex vc = CSGGetFaceVertex(face, 2, verts[2], attr);

                // attempt to add the vertices to our output mesh
                // or get lookup IDs for colocal vertices with matching attributes
                unsigned iA = CSGAddVertex(va, vertexTable, outVertices, vertexData);
                unsigned iB = CSGAddVertex(vb, vertexTable, outVertices, vertexData);
                unsigned iC = CSGAddVertex(vc, vertexTable, outVertices, vertexData);

                indexData.Push(iA);
                indexData.Push(iB);
                indexData.Push(iC);
            }

            GenerateTangents(vertexData.Buffer(), sizeof(CSGOutVertex), indexData.Buffer(), 
                sizeof(unsigned short), 0, indexData.Size(), 
                offsetof(CSGOutVertex, normal_), 
                offsetof(CSGOutVertex, texcoord_), 
                offsetof(CSGOutVertex, tangent_));

            VertexBuffer* vertBuffer = new VertexBuffer(context);
            IndexBuffer* idxBuffer = new IndexBuffer(context);
            vertBuffer->SetSize(vertexData.Size(), {
                { TYPE_VECTOR3, SEM_POSITION },
                { TYPE_VECTOR3, SEM_NORMAL },
                { TYPE_VECTOR4, SEM_TANGENT },
                { TYPE_VECTOR2, SEM_TEXCOORD }
            }, false);
            vertBuffer->SetData(vertexData.Buffer());
            idxBuffer->SetSize(indexData.Size(), false);
            idxBuffer->SetData(indexData.Buffer());

            outData->SetNumVertexBuffers(1);
            outData->SetVertexBuffer(0, vertBuffer);
            outData->SetIndexBuffer(idxBuffer);
            outData->SetDrawRange(TRIANGLE_LIST, 0, idxBuffer->GetIndexCount());
        }

        return outData;
    }

    Geometry* PerformCSG(Context* ctx, PODVector<CSGOperand>& meshes, bool encodeTexCoords)
    {
        CSGMeshType* workingMesh = nullptr;
        SharedPtr<CSGMeshData> firstData = nullptr;
        PODVector<SharedPtr<CSGMeshData>> csgMeshList;
        CSGAttributes csgAttributes;
        for (int i = 0; i < meshes.Size(); ++i)
        {
            auto meshOperand = meshes[i];
            auto meshData = FillMesh(meshOperand, csgAttributes, Vector2(encodeTexCoords ? i : 0, 0));
            bool operate = false;
            if (meshData && meshData->mesh_)
            {
                if (firstData.Null())
                    firstData = meshData;
                else
                    operate = true;

                csgMeshList.Push(meshData);
            }

            if (operate)
            {
                carve::csg::CSG csg;
                csg.hooks.registerHook(new carve::csg::CarveTriangulatorWithImprovement(), carve::csg::CSG::Hooks::PROCESS_OUTPUT_FACE_BIT);
                csgAttributes.Hook(csg);
                carve::csg::V2Set sharedEdges;
                std::set<CSGMeshType::vertex_t*> sharedVertsSet;
                std::map<Vector3, Vector3> sharedVertNormals;
                std::map<Vector3, Vector2> smoothFactors;
                CSGMeshType* resultMesh = 0x0;
                try
                {
                    resultMesh = csg.compute(firstData->mesh_, meshData->mesh_, carve::csg::CSG::A_MINUS_B_ONLY_A/*ConvertToCarveOp(meshOperand->Task)*/, 0x0);
                }
                catch (carve::exception except)
                {
                    URHO3D_LOGERROR("Failed to compute CSG mesh");
                }
                if (resultMesh != 0x0)
                {
                    SharedPtr<CSGMeshData> resultMeshData(new CSGMeshData(nullptr));
                    // Smooth the shared edges of the CSG result
                    if (meshOperand.smoothingPower_ > 0)
                    {
                        const float smoothPower = meshOperand.smoothingPower_;
                        for (auto sharedEdge : sharedEdges)
                        {
                            auto a = Vector3(sharedEdge.first->v[0], sharedEdge.first->v[1], sharedEdge.first->v[2]);
                            auto b = Vector3(sharedEdge.second->v[0], sharedEdge.second->v[1], sharedEdge.second->v[2]);
                            sharedVertsSet.insert(sharedEdge.first);
                            sharedVertsSet.insert(sharedEdge.second);

                            // use this to calculate smoothing power
                            Vector2 smoothFactor(1, a.DistanceToPoint(b));
                            smoothFactors[a] += smoothFactor;
                            smoothFactors[b] += smoothFactor;
                        }

                        for (auto faceIter = resultMesh->faceBegin(); faceIter != resultMesh->faceEnd(); ++faceIter)
                        {
                            auto face = *faceIter;
                            std::vector<CSGMeshType::vertex_t*> vertices;
                            face->getVertices(vertices);

                            for (int i = 0; i < vertices.size(); ++i)
                            {
                                if (sharedVertsSet.find(vertices[i]) != sharedVertsSet.end())
                                {
                                    Vector3 v(vertices[i]->v[0], vertices[i]->v[1], vertices[i]->v[2]);
                                    sharedVertNormals[v] += Vector3(face->plane.N[0], face->plane.N[1], face->plane.N[2]);
                                }
                            }

                            std::unordered_set<CSGMeshType::vertex_t*> closedList;
                            for (auto& vert : resultMesh->vertex_storage)
                            {
                                // don't multiprocess
                                if (closedList.find(&vert) != closedList.end())
                                    continue;

                                if (sharedVertsSet.find(&vert) != sharedVertsSet.end())
                                {
                                    const Vector3 v(vert.v[0], vert.v[1], vert.v[2]);
                                    const Vector3 normal = sharedVertNormals[v].Normalized();
                                    const auto sourceFactorData = smoothFactors[v];
                                    const float baseSmoothPower = sqrtf(sourceFactorData.y_ / sourceFactorData.x_);

                                    vert.v[0] += normal.x_ * baseSmoothPower * smoothPower;
                                    vert.v[1] += normal.y_ * baseSmoothPower * smoothPower;
                                    vert.v[2] += normal.z_ * baseSmoothPower * smoothPower;
                                    closedList.insert(&vert);
                                }
                            }

                            for (auto mesh : resultMesh->meshes)
                                mesh->recalc();
                        }
                    }

                    resultMeshData->mesh_ = resultMesh;
                    firstData = resultMeshData;
                    csgMeshList.Push(resultMeshData);
                }

            }
        }

        if (firstData.NotNull() && firstData->mesh_)
            return FromCSGMeshCompact(ctx, firstData->mesh_, &csgAttributes);
        return nullptr;
    }

    /// Writes out the canonical (seamless) vertex data and the remapped indices. Returns false if there's a failure.
    bool ExtractCanonicalPositions(Geometry* forGeometry, PODVector<Vector3>& positions, PODVector<unsigned>& indices, unsigned& indexStartOffset, const Matrix3x4& transform, PODVector<unsigned>* remapping)
    {
        // NOTE: Written to assume it may be used to concatenate multiple geometries.

        HashMap<Vector3, unsigned> canonicals;
        PODVector<unsigned> localVertexRemap;
        PODVector<unsigned>* vertexRemap = remapping != nullptr ? remapping : &localVertexRemap;
        vertexRemap->Resize(forGeometry->GetVertexCount());

        const unsigned char* vertexData;
        const unsigned char* indexData;
        unsigned vertexSize;
        unsigned indexSize;
        const PODVector<VertexElement>* elements;
        forGeometry->GetRawData(vertexData, vertexSize, indexData, indexSize, elements);
        const bool largeIndices = indexSize == sizeof(unsigned);

        // Require a position semantic
        if (!vertexData || !elements || VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION) != 0)
        {
            URHO3D_LOGERROR("Encountered illegal geometry (no SEM_POSITION) in CreateShadowGeom");
            return false;
        }

        unsigned posOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);
        for (size_t i = 0; i < forGeometry->GetVertexCount(); ++i)
        {
            Vector3 vertPos = *((Vector3*)(vertexData + vertexSize*i + posOffset));
            vertPos = transform * vertPos;
            const unsigned tentativeIdx = canonicals.Size();
            auto existing = canonicals.Find(vertPos);
            if (existing == canonicals.End())
            {
                positions.Push(vertPos);
                canonicals.Insert(MakePair(vertPos, tentativeIdx));
                (*vertexRemap)[i] = tentativeIdx;
            }
            else
                (*vertexRemap)[i] = existing->second_;
        }

        // Make sure there's enough index data space
        if (indices.Size() < indexStartOffset + forGeometry->GetIndexCount())
            indices.Resize(indices.Size() + forGeometry->GetIndexCount());

        for (size_t idx = 0; idx < forGeometry->GetIndexCount(); idx += 3)
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
        indexStartOffset += forGeometry->GetIndexCount();
        return true;
    }

    Geometry* CreateShadowGeom(Context* ctx, Geometry* forGeometry)
    {  
        PODVector<Vector3> newVertexData;
        newVertexData.Reserve(forGeometry->GetVertexCount());
        PODVector<unsigned> newIndexData;
        newIndexData.Resize(forGeometry->GetIndexCount());

        unsigned idxStartOffset = 0;
        if (!ExtractCanonicalPositions(forGeometry, newVertexData, newIndexData, idxStartOffset, Matrix3x4::IDENTITY, nullptr))
            return nullptr;

        // Create the new geometry
        Geometry* ret = new Geometry(ctx);

        // generate new vertex buffer, position only ... could include UVs if transparent versions are needed
        auto vertBuffer = new VertexBuffer(ctx);
        vertBuffer->SetSize(newVertexData.Size(), ELEMENT_POSITION, false);
        vertBuffer->SetData(newVertexData.Buffer());

        // generate new index buffer, keep as small as possible
        auto idxBuffer = new IndexBuffer(ctx);
        idxBuffer->SetDataOptimal(newIndexData);

        ret->SetNumVertexBuffers(1);
        ret->SetVertexBuffer(0, vertBuffer);
        ret->SetIndexBuffer(idxBuffer);
        ret->SetDrawRange(TRIANGLE_LIST, 0, idxBuffer->GetIndexCount());

        return ret;
    }

    Geometry* CreateShadowGeom(Context* ctx, const PODVector<Geometry*>& srcGeoms, const PODVector<Matrix3x4>& transforms)
    {
        unsigned totalVertCt = 0;
        unsigned totalIdxCt = 0;
        for (unsigned i = 0; i < srcGeoms.Size(); ++i)
        {
            totalVertCt += srcGeoms[i]->GetVertexCount();
            totalIdxCt += srcGeoms[i]->GetIndexCount();
        }

        PODVector<Vector3> newVertexData;
        newVertexData.Reserve(totalVertCt);

        PODVector<unsigned> newIndexData;
        newIndexData.Resize(totalIdxCt);

        unsigned newIndexStartPos = 0;
        for (unsigned i = 0; i < srcGeoms.Size(); ++i)
        {
            auto forGeometry = srcGeoms[i];
            if (!ExtractCanonicalPositions(forGeometry, newVertexData, newIndexData, newIndexStartPos, transforms[i], nullptr))
                return nullptr; // already logged an error
        }

        // Create the new geometry
        Geometry* ret = new Geometry(ctx);

        // generate new vertex buffer, position only ... could include UVs if transparent versions are needed
        auto vertBuffer = new VertexBuffer(ctx);
        vertBuffer->SetSize(newVertexData.Size(), ELEMENT_POSITION, false);
        vertBuffer->SetData(newVertexData.Buffer());

        // generate new index buffer, keep as small as possible
        auto idxBuffer = new IndexBuffer(ctx);
        idxBuffer->SetDataOptimal(newIndexData);

        ret->SetNumVertexBuffers(1);
        ret->SetVertexBuffer(0, vertBuffer);
        ret->SetIndexBuffer(idxBuffer);
        ret->SetDrawRange(TRIANGLE_LIST, 0, idxBuffer->GetIndexCount());

        return ret;
    }
}
