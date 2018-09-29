#include <Urho3D/ProcGeom/MarchingTriangles.h>

#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Math/Quaternion.h>
#include <Urho3D/Graphics/VertexBuffer.h>

#include <MathGeoLib.h>

#define INTERIOR_ANGLE 60
#define OUTER_ANGLE 120

#define TOVEC(a) vec(a.x_, a.y_, a.z_)
#define FROMVEC(a) Vector3(a.x, a.y, a.z)

namespace Urho3D
{
    /// Compute the `natural` tangent space for a normal.
    void ImplicitTangentSpace(const Vector3& norm, Vector3& tangent, Vector3& binormal)
    {
        if (norm.x_ > 0.5f || norm.y_ > 0.5f || norm.x_ < -0.5f || norm.y_ < -0.5f)
            tangent = Vector3(norm.y_, -norm.x_, 0.0f);
        else 
            tangent = Vector3(-norm.z_, 0.0f, norm.x_);
        binormal = norm.CrossProduct(tangent);
        binormal.Normalize();
        tangent.Normalize();
    }

    MTVertex* MTVertex::Clone()
    {
        MTVertex* ret = new MTVertex();
        memcpy(ret, this, sizeof(MTVertex));
        return ret;
    }

    void MTVertex::CalculateAngle() 
    {        
        // The purpose of this code is to get a 360-degree outer angle at the vertex in the 2d plane
        // oriented with the normal. To do this the edge-vectors are dotted with the basis vectors to get 2d coords.
        // Then standard atan2.

        auto thisToPrev = (prev_->position_ - position_).Normalized();
        auto thisToRight = (next_->position_ - position_).Normalized();

        Vector3 t, b;
        ImplicitTangentSpace(normal_, t, b);

        Vector2 dx = Vector2(t.DotProduct(thisToPrev), b.DotProduct(thisToPrev));
        Vector2 dy = Vector2(t.DotProduct(thisToRight), b.DotProduct(thisToRight));

        float dot = dx.x_ * dy.x_ + dx.y_ * dy.y_;
        float det = dx.x_ * dy.y_ - dx.y_ * dy.x_;
        angle_ = atan2(det, dot);
        angle_ *= M_RADTODEG;
        // atan2 goes the wrong way, so subtract the value from 360
        // want to go clockwise for intuitive reading of the values when debugging
        angle_ = 360 - ((int)(angle_ + 720) % 360);

        angleDirty_ = false;
    }

    bool MTVertex::InSameLoop(const MTVertex* other) const
    {
        auto current = this;
        auto sentinal = current->prev_;
        do {
            if (current == other)
                return true;
            current = current->next_;
        } while (current != sentinal);
        return false;
    }

    MTFront::~MTFront()
    {
        for (auto v : vertices_)
            delete v;
    }

    Pair<MTVertex*, float> MTFront::TightestAngle() const
    {
        float minAng = FLT_MAX;
        MTVertex* best = nullptr;

        #pragma omp parallel for
        for (unsigned i = 0; i < vertices_.Size(); ++i)
        {
            const auto v = vertices_[i];
            if (v->angle_ < minAng)
            {
                // don't actually care about any OMP dangers, we just want to be a `good enough` vertex.
                best = v;
                minAng = v->angle_;
            }
        }
        return MakePair(best, minAng);
    }

    Pair<MTVertex*, float> MTFront::Nearest(const MTVertex* other, const Vector3& candidateExpansionDir) const
    {
        float minDist2 = FLT_MAX;
        MTVertex* best = nullptr;
        for (auto v : vertices_)
        {
            if (v == other)
                continue;

            // don't bother if we're outside of our candidate expansion direction
            const Vector3 fromOtherToThis = other->position_ - v->position_;
            if (candidateExpansionDir.DotProduct(fromOtherToThis.Normalized()) < 0.5f)
                continue;

            float d2 = fromOtherToThis.LengthSquared();
            if (d2 < minDist2 && v->normal_.DotProduct(other->normal_) > 0.3f)
            {
                minDist2 = d2;
                best = v;
            }
        }
        return MakePair(best, minDist2);
    }
    
    
    void MTFront::FillWithLoop(MTVertex* a)
    {
        vertices_.Clear();
        auto current = a;
        do {
            current->front_ = this;
            vertices_.Push(current);
            current = current->next_;
        } while (current != a);
    }

    MTFront* MTFront::SplitFront(MTVertex* vA, MTVertex* vB)
    {
        if (vA->IsDenegerate())
        {
            FillWithLoop(vB);
            delete vA->prev_;
            delete vA;
#ifdef DEBUG
            SanityCheck();
#endif
            return nullptr;
        }
        else if (vB->IsDenegerate())
        {
            FillWithLoop(vA);
            delete vB->prev_;
            delete vB;
#ifdef DEBUG
            SanityCheck();
#endif
            return nullptr;
        }

        FillWithLoop(vA);

        MTFront* newFront = new MTFront();
        newFront->FillWithLoop(vB);

#ifdef DEBUG
        SanityCheck();
        newFront->SanityCheck();
#endif
        return newFront;
    }

    void MTFront::MergeFronts(MTFront* a)
    {
        auto start = vertices_.Front();
        a->vertices_.Clear();
        vertices_.Clear();

        auto current = start;
        while (current->next_ != start)
        {
            current->front_ = this;
            vertices_.Push(current);
            current = current->next_;
        }

#ifdef DEBUG
        SanityCheck();
#endif
    }

    void MTFront::SanityCheck()
    {
        for (auto v : vertices_)
        {
            assert(v != (MTVertex*)0xdddddddddddddddd);
            assert(v->next_ != (MTVertex*)0xdddddddddddddddd);
            assert(v->prev_ != (MTVertex*)0xdddddddddddddddd);
            assert(v != v->next_);
            assert(v != v->prev_);
        }
    }

    PODVector<Pair<Vector3, Vector3> > MTFront::CaptureFrontData() const
    {
        PODVector<Pair<Vector3, Vector3> > edges;
        edges.Reserve(vertices_.Size());

        auto current = vertices_[0];
        const auto start = current;
        do {
            edges.Push(MakePair(current->position_, current->next_->position_));
            current = current->next_;
        } while (current != start);

        return edges;
    }

    MarchingTriangles::MarchingTriangles(float edgeLength)
    {
        nextVertexIndex_ = 0;
        edgeLength_ = edgeLength;
        frontProximityEdgeLength_ = edgeLength_ * edgeLength_;
    }
    
    MarchingTriangles::MarchingTriangles(float edgeLength, float tolerance)
    {
        nextVertexIndex_ = 0;
        edgeLength_ = edgeLength; 
        frontProximityEdgeLength_ = tolerance;
    }

    
    MarchingTriangles::~MarchingTriangles()
    {
        ClearFronts();
    }

    void MarchingTriangles::ClearFronts()
    {
        for (auto f : fronts_)
            delete f;
        fronts_.Clear();
    }

    void MarchingTriangles::Initialize(SurfacingFunction sdfFunc, const Vector3& seedPoint, unsigned maxTris)
    {
        nextVertexIndex_ = 0;
        sdfFunc_ = sdfFunc;
        maxTris_ = maxTris;

        SeedSurface(seedPoint);
    }

    void MarchingTriangles::GenerateSurface(SurfacingFunction sdfFunc, const Vector3& seedPoint, unsigned maxTris)
    {
        nextVertexIndex_ = 0;
        sdfFunc_ = sdfFunc;
        maxTris_ = maxTris;

        SeedSurface(seedPoint);

        InternalSurfaceGeneration();
    }

    void MarchingTriangles::InternalSurfaceGeneration()
    {
        while (fronts_.Size() > 0)
        {
            if (Advance())
                break;
        }
        ClearFronts();
    }

    bool MarchingTriangles::Advance()
    {
        // hit our escape point
        if (indices_.Size() / 3 > maxTris_)
            return true;

        CalculateAngles();

        // Front selection can be interesting:
        //  - Could always go from the front/back 
        //  - or select based on criteria like fewest vertices or greatest perimeter
        //  - shouldn't have much of a real bearing on the final result though, maybe memory pressure
        //      - ie: eliminate large fronts ASAP
        MTFront* activeFront = fronts_.Back();
#ifdef DEBUG
        activeFront->SanityCheck();
#endif
        auto bestVertex = activeFront->TightestAngle();
        if (activeFront->vertices_.Size() == 3)
        {
            // Trivial case, we're a triangle!
            auto a = bestVertex.first_;
            auto b = bestVertex.first_->prev_;
            auto c = bestVertex.first_->next_;

            indices_.Push(a->index_);
            indices_.Push(b->index_);
            indices_.Push(c->index_);

            fronts_.Remove(activeFront);
            delete activeFront;
        }
        else
        {
            // take the outer vector in the middle of our angle as our candidate direction
            float middleAngle = bestVertex.first_->angle_ / 2;
            auto candidateDir = (bestVertex.first_->prev_->position_ - bestVertex.first_->position_).Normalized();
            candidateDir = Quaternion(-middleAngle, bestVertex.first_->normal_) * candidateDir;
            candidateDir *= edgeLength_;

            int numAngles = (int)floorf(3 * (bestVertex.second_ * M_DEGTORAD) / M_PI);
            auto nearest = Nearest(bestVertex.first_, candidateDir, bestVertex.first_->normal_, numAngles);

            // If the nearest is the `sentinel` then force an outer edge close.
            // TODO: to improve triangulation should move to the next or previous vertex to form a better edge close.
            //      doing that should be less skinny
            if (nearest.first_ == MTVertex::Sentinel())
                numAngles = 0;

            if (numAngles > 0 && nearest.first_ != nullptr && nearest.second_ < Max(frontProximityEdgeLength_, Max(bestVertex.first_->front_->tolerance_, nearest.first_->front_->tolerance_)))
            {
                // a lot of the contents in here can probably go away now, have to go through a number of things to be sure
                // Checklist:
                //  - Genus 0 GOOD
                //  - Genus 1-9 unknown

                auto current = bestVertex.first_;
                auto bestPrev = current->prev_;
                auto bestNext = current->next_;
                auto nearestNext = nearest.first_->next_;
                auto nearestPrev = nearest.first_->prev_;

                assert(!nearest.first_->IsTriangle());
                assert(!nearest.first_->IsQuad());
                bool nearestNextFormsQuad = nearest.first_->next_->IsConnectedTo(current->prev_);
                bool nearestPrevFormsQuad = nearest.first_->prev_->IsConnectedTo(current->next_);

                // single triangle cases
                if (bestPrev->IsConnectedTo(nearest.first_))
                {
                    // our prev -> nearest
                    indices_.Push(bestPrev->index_);
                    indices_.Push(nearest.first_->index_);
                    indices_.Push(current->index_);

                    current->front_->vertices_.Remove(bestPrev);
                    delete bestPrev;
                    nearest.first_->SetNext(current);

                    return fronts_.Empty();
                }
                else if (bestNext->IsConnectedTo(nearest.first_))
                {
                    // our next -> nearest
                    indices_.Push(current->index_);
                    indices_.Push(nearest.first_->index_);
                    indices_.Push(bestNext->index_);

                    current->front_->vertices_.Remove(bestNext);
                    delete bestNext;
                    current->SetNext(nearest.first_);

                    return fronts_.Empty();
                }

                // triangulate for shortest edge
                // which side of the nearest->current edge appears to have no bearing on edge flipping
                if ((bestPrev->position_ - nearest.first_->position_).LengthSquared() < (current->position_ - nearestNext->position_).LengthSquared())
                {
                    indices_.Push(current->index_);
                    indices_.Push(bestPrev->index_);
                    indices_.Push(nearest.first_->index_);
                        
                    indices_.Push(nearest.first_->index_);
                    indices_.Push(bestPrev->index_);
                    indices_.Push(nearestNext->index_);
                }
                else
                {
                    indices_.Push(current->index_);
                    indices_.Push(bestPrev->index_);
                    indices_.Push(nearestNext->index_);
                        
                    indices_.Push(current->index_);
                    indices_.Push(nearestNext->index_);
                    indices_.Push(nearest.first_->index_);
                }
                    
                nearest.first_->SetNext(current);
                bestPrev->SetNext(nearestNext);

                if (nearest.first_->front_ == current->front_)
                {
                    // same front, split it
                    nearest.first_->front_->SanityCheck();
                    assert(!nearest.first_->InSameLoop(nearestNext));

                    // This can probably all be cleaned up.
                    auto newFront = activeFront->SplitFront(nearest.first_, nearestNext);
                    if (newFront && newFront->vertices_.Size() < 3)
                        delete newFront;
                    else if (newFront)
                        fronts_.Push(newFront);
                    if (activeFront->vertices_.Size() < 3)
                    {
                        fronts_.Remove(activeFront);
                        delete activeFront;
                    }
                }
                else
                {
                    // different fronts, merge them
                    assert(nearest.first_->InSameLoop(nearestNext));
                    auto f = nearest.first_->front_;
                    fronts_.Remove(f);
                    current->front_->MergeFronts(f);
                    delete f;
                }
                
                return fronts_.Empty();
            }

            if (numAngles < 1)
            {
                // simple fill
                // close an interior triangle
                auto a = bestVertex.first_;
                auto b = bestVertex.first_->prev_;
                auto c = bestVertex.first_->next_;

                indices_.Push(a->index_);
                indices_.Push(b->index_);
                indices_.Push(c->index_);

                // remove and relink
                activeFront->vertices_.Remove(a);
                b->SetNext(c);
                c->SetPrev(b);
                delete a;
            }
            else
            {
                // radial fill
                float anglesPerStep = bestVertex.second_ / (numAngles + 1);
                auto prevVert = bestVertex.first_->prev_;
                auto nextVert = bestVertex.first_->next_;
                        
                // setup our radial chain
                MTVertex* newPoints[8]; // bigger than actually required
                memset(newPoints, 0, sizeof(MTVertex*)*8);
                newPoints[0] = prevVert;
                newPoints[numAngles + 1] = nextVert;

                auto baseVector = prevVert->position_ - bestVertex.first_->position_;
                baseVector.Normalize();
                auto rotationNorm = bestVertex.first_->normal_;

                float curAng = 0.0f;
                for (int ang = 1; ang < numAngles + 1; ++ang)
                {
                    curAng += anglesPerStep;
                    auto reachVector = Quaternion(-curAng, rotationNorm) * baseVector;
                    reachVector *= edgeLength_;

                    auto newVertexPos = bestVertex.first_->position_ + reachVector;
                    auto normal = CalculateNormal(newVertexPos);
                    for (int i = 0; i < 3; ++i)
                    {
                        float dist = sdfFunc_(newVertexPos);
                        newVertexPos += normal * -dist;
                        normal = CalculateNormal(newVertexPos);
                    }

                    MTVertex* newVertex = new MTVertex{
                        nextVertexIndex_++,
                        newVertexPos,
                        normal,
                        0,
                        activeFront,
                        nullptr,
                        nullptr
                    };

                    positions_.Push(newVertexPos);
                    normals_.Push(normal);

                    indices_.Push(bestVertex.first_->index_);
                    indices_.Push(newPoints[ang-1]->index_);
                    indices_.Push(newVertex->index_);

                    newPoints[ang] = newVertex;
                    activeFront->vertices_.Push(newVertex);
                }

                // add the last triangle
                indices_.Push(bestVertex.first_->index_);
                indices_.Push(newPoints[numAngles]->index_);
                indices_.Push(newPoints[numAngles + 1]->index_);
                        
                for (int i = 0; i < numAngles + 1; ++i)
                {
                    newPoints[i]->SetNext(newPoints[i + 1]);
                    if (i > 0)
                        newPoints[i]->SetPrev(newPoints[i - 1]);
                }
                nextVert->SetPrev(newPoints[numAngles]);

                activeFront->vertices_.Remove(bestVertex.first_);
                delete bestVertex.first_;
                return fronts_.Empty();
            }
        }

        return fronts_.Empty();
    }

    void MarchingTriangles::GenerateSurface(SurfacingFunction sdfFunc, MTFront* startFrom, unsigned maxTris)
    {
        sdfFunc_ = sdfFunc;
        maxTris_ = maxTris;

        if (fronts_.Remove(startFrom))
        {
            fronts_.Insert(0, startFrom);
            InternalSurfaceGeneration();
        }
        else
            GenerateSurface(sdfFunc, Vector3::ZERO, maxTris);
    }

    /// For finding the open-loop in a custom mesh that's inserted.
    struct MTEdgePair 
    {
        /// a_ and b_ must follow a consisted ordering so that mirrored edges pass.
        Vector3 a_;
        Vector3 b_;
        /// Indices of where the vertices actually came from, this is required for the loop-winding order.
        unsigned canonA_;
        unsigned canonB_;

        bool operator==(const MTEdgePair& rhs) const { return a_ == rhs.a_ && b_ == rhs.b_; }
        bool operator!=(const MTEdgePair& rhs) const { return a_ != rhs.a_ || b_ != rhs.b_; }
    };
    
    MTFront* MarchingTriangles::AddCustomGeometry(Geometry* geometry, const Matrix3x4& transform, float reachTolerance)
    {
        const unsigned char* data;
        const unsigned char* indexData;
        unsigned vertexSize;
        unsigned indexSize;
        const PODVector<VertexElement>* elements;

        geometry->GetRawData(data, vertexSize, indexData, indexSize, elements);
        const auto vertexStart = geometry->GetVertexStart();
        const auto vertexCt = geometry->GetVertexCount();
        const auto indexStart = geometry->GetIndexStart();
        const auto indexCt = geometry->GetIndexCount();
        const bool largeIndices = indexSize == sizeof(unsigned);

        PODVector<MTEdgePair> edgePairs;
        static auto MakeEdgePair = [](const Vector3& a, const Vector3& b, unsigned aIdx, unsigned bIdx) -> MTEdgePair
        {
            int ct = a.x_ < b.x_;
            ct += a.y_ < b.y_;
            ct += a.z_ < b.z_;
            int oCt = 3 - ct;

            return ct > oCt ? MTEdgePair { a, b, aIdx, bIdx } : MTEdgePair { b, a, aIdx, bIdx };
        };

        const unsigned posOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);
        const unsigned normOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_NORMAL);
        for (unsigned i = indexStart; i < indexStart + indexCt; i += 3)
        {
            unsigned int indices[] = {
                largeIndices ? ((unsigned*)indexData)[i] : ((unsigned short*)indexData)[i],
                largeIndices ? ((unsigned*)indexData)[i + 1] : ((unsigned short*)indexData)[i + 1],
                largeIndices ? ((unsigned*)indexData)[i + 2] : ((unsigned short*)indexData)[i + 2]
            };

            indices_.Push(indices[0] - indexStart);
            indices_.Push(indices[1] - indexStart);
            indices_.Push(indices[2] - indexStart);
            
            auto a = *(Vector3*)(data + indices[0] * vertexSize + posOffset);
            auto b = *(Vector3*)(data + indices[1] * vertexSize + posOffset);
            auto c = *(Vector3*)(data + indices[2] * vertexSize + posOffset);

            auto ab = MakeEdgePair(a, b, indices[0] - indexStart + nextVertexIndex_, indices[1] - indexStart + nextVertexIndex_);
            auto bc = MakeEdgePair(b, c, indices[1] - indexStart + nextVertexIndex_, indices[2] - indexStart + nextVertexIndex_);
            auto ca = MakeEdgePair(c, a, indices[2] - indexStart + nextVertexIndex_, indices[0] - indexStart + nextVertexIndex_);
            
            if (!edgePairs.Contains(ab))
                edgePairs.Push(ab);
            else
                edgePairs.Remove(ab);
            
            if (!edgePairs.Contains(bc))
                edgePairs.Push(bc);
            else
                edgePairs.Remove(bc);

            if (!edgePairs.Contains(ca))
                edgePairs.Push(ca);
            else
                edgePairs.Remove(ca);
        }

        auto rotMat = transform.RotationMatrix();
        for (unsigned i = vertexStart; i < vertexStart + vertexCt; ++i)
        {
            positions_.Push(transform * (*(Vector3*)(data + i * vertexSize + posOffset)));
            normals_.Push(rotMat * (*(Vector3*)(data + i * vertexSize + posOffset)));
        }

        nextVertexIndex_ = positions_.Size();
        if (edgePairs.Size() > 0)
        {
            MTFront* front = new MTFront();
            front->tolerance_ = reachTolerance;

            auto current = edgePairs[0];
            MTVertex* lastVert = new MTVertex {
                current.canonA_,
                positions_[current.canonA_],
                normals_[current.canonA_],
                0,
                front,
                nullptr, nullptr
            };
            front->vertices_.Push(lastVert);
            edgePairs.Remove(current);
            for (int i = 0; i < edgePairs.Size(); ++i)
            {
                if (edgePairs[i].canonA_ == current.canonB_)
                {
                    MTVertex* nextVert = new MTVertex {
                        edgePairs[i].canonA_,
                        positions_[edgePairs[i].canonA_],
                        normals_[edgePairs[i].canonA_],
                        0,
                        front,
                        lastVert, nullptr
                    };
                    lastVert->SetNext(nextVert);
                    front->vertices_.Push(nextVert);
                    current = edgePairs[i];
                    edgePairs.Remove(edgePairs[i]);
                    i = -1;
                }
            }
            front->vertices_.Front()->SetPrev(front->vertices_.Back());
            front->vertices_.Back()->SetNext(front->vertices_.Front());
            return front;
        }
        else
        {
            URHO3D_LOGERROR("No outer-border edges in mesh inserted into MarchingTriangles");
            return nullptr;
        }
    }

    Geometry* MarchingTriangles::ExtractGeometry(Context* ctx)
    {
        PODVector<VertexElement> elements;
        elements.Push(VertexElement(TYPE_VECTOR3, SEM_POSITION));
        elements.Push(VertexElement(TYPE_VECTOR3, SEM_NORMAL));

        VertexBuffer* vertexBuffer = new VertexBuffer(ctx);
        vertexBuffer->SetShadowed(true);
        vertexBuffer->SetSize(positions_.Size(), elements);
        elements = vertexBuffer->GetElements();
        const unsigned vertSize = VertexBuffer::GetVertexSize(elements);
        const unsigned posOffset = VertexBuffer::GetElementOffset(elements, TYPE_VECTOR3, SEM_POSITION);
        const unsigned normOffset = VertexBuffer::GetElementOffset(elements, TYPE_VECTOR3, SEM_NORMAL);
        unsigned char* data = new unsigned char[positions_.Size() * vertSize];
        for (unsigned i = 0; i < positions_.Size(); ++i)
        {
            *((Vector3*)(data + vertSize * i + posOffset)) = positions_[i];
            *((Vector3*)(data + vertSize * i + normOffset)) = normals_[i];
        }
        vertexBuffer->SetData(data);

        IndexBuffer* indexBuffer = new IndexBuffer(ctx);
        indexBuffer->SetShadowed(true);
        indexBuffer->SetSize(indices_.Size(), true);
        indexBuffer->SetData(indices_.Buffer());

        Geometry* geometry = new Geometry(ctx);
        geometry->SetNumVertexBuffers(1);
        geometry->SetVertexBuffer(0, vertexBuffer);
        geometry->SetIndexBuffer(indexBuffer);
        geometry->SetDrawRange(TRIANGLE_LIST, 0, indices_.Size());
        return geometry;
    }

    void MarchingTriangles::SeedSurface(const Vector3& seedPoint)
    {
        Vector3 samplingPoint = seedPoint;
        auto norm = CalculateNormal(samplingPoint);
        if (norm.Length() == 0)
            norm = Vector3::UP;
        float density = sdfFunc_(samplingPoint);

        // walk along the normal 6 times to get close to an accurate depth
        for (int i = 0; i < 6 && density != 0; ++i)
        {
            samplingPoint += norm * -density;
            norm = CalculateNormal(samplingPoint);
            density = sdfFunc_(samplingPoint);
        }

        Vector3 tan, binorm;
        ImplicitTangentSpace(norm, tan, binorm);

        // Emit the seeding hexagon wavefront
        positions_.Push(samplingPoint);
        normals_.Push(norm);
        unsigned rootIndex = nextVertexIndex_;
        nextVertexIndex_ += 1;
        
        MTFront* hexFront = new MTFront();
        float degreesPerHexEdge = 360.0f / 6.0f;
        unsigned indices[] = { 0, 0, 0, 0, 0, 0 };
        for (unsigned i = 0; i < 6; ++i)
        {
            Vector3 outerDir = Quaternion(-degreesPerHexEdge * i, norm) * tan;
            outerDir = outerDir.Normalized() * edgeLength_;

            auto outerPoint = samplingPoint + outerDir;
            float density = sdfFunc_(outerPoint);
            auto normal = CalculateNormal(outerPoint);

            outerPoint += normal * -density;
            normal = CalculateNormal(samplingPoint);
            
            positions_.Push(outerPoint);
            normals_.Push(normal);

            MTVertex* newVert = new MTVertex {
                nextVertexIndex_++,
                outerPoint,
                normal,
                0,          // angle
                hexFront,   // front
                nullptr,    // prev
                nullptr     // next
            };
            indices[i] = newVert->index_;
            hexFront->vertices_.Push(newVert);
        }

        hexFront->vertices_[0]->next_ = hexFront->vertices_[1];
        hexFront->vertices_[1]->next_ = hexFront->vertices_[2];
        hexFront->vertices_[2]->next_ = hexFront->vertices_[3];
        hexFront->vertices_[3]->next_ = hexFront->vertices_[4];
        hexFront->vertices_[4]->next_ = hexFront->vertices_[5];
        hexFront->vertices_[5]->next_ = hexFront->vertices_[0];

        hexFront->vertices_[0]->prev_ = hexFront->vertices_[5];
        hexFront->vertices_[1]->prev_ = hexFront->vertices_[0];
        hexFront->vertices_[2]->prev_ = hexFront->vertices_[1];
        hexFront->vertices_[3]->prev_ = hexFront->vertices_[2];
        hexFront->vertices_[4]->prev_ = hexFront->vertices_[3];
        hexFront->vertices_[5]->prev_ = hexFront->vertices_[4];

        indices_.Push(rootIndex); indices_.Push(indices[0]); indices_.Push(indices[1]);
        indices_.Push(rootIndex); indices_.Push(indices[1]); indices_.Push(indices[2]);
        indices_.Push(rootIndex); indices_.Push(indices[2]); indices_.Push(indices[3]);
        indices_.Push(rootIndex); indices_.Push(indices[3]); indices_.Push(indices[4]);
        indices_.Push(rootIndex); indices_.Push(indices[4]); indices_.Push(indices[5]);
        indices_.Push(rootIndex); indices_.Push(indices[5]); indices_.Push(indices[0]);

        fronts_.Push(hexFront);
    }

    void MarchingTriangles::CalculateAngles()
    {
        for (auto front : fronts_)
        {
            #pragma omp parallel for
            for (unsigned i = 0; i < front->vertices_.Size(); ++i)
            {
                if (front->vertices_[i]->angleDirty_)
                    front->vertices_[i]->CalculateAngle();
            }
        }
    }

    Vector3 MarchingTriangles::CalculateNormal(const Vector3& f) const
    {
        // sometimes called the gradient when referred to in signed distance fields
        // that's technically more accurate.
        const float H = 0.001f;
        const float dx = sdfFunc_(f + Vector3(H, 0.f, 0.f)) - sdfFunc_(f - Vector3(H, 0.f, 0.f));
        const float dy = sdfFunc_(f + Vector3(0.f, H, 0.f)) - sdfFunc_(f - Vector3(0.f, H, 0.f));
        const float dz = sdfFunc_(f + Vector3(0.f, 0.f, H)) - sdfFunc_(f - Vector3(0.f, 0.f, H));

        return Vector3(dx, dy, dz).Normalized();
    }

    // This function is a problem, use a kdTree or nanoflann for search?
    // Most kdTrees involve rebuilding, and we'll have to rebuild every single pass
    // Fronts should be small though, is building a kdtree cheaper than evaluating this?
    Pair<MTVertex*, float> MarchingTriangles::Nearest(const MTVertex* other, const Vector3& candidateExpansionDir, const Vector3& normal, int numAngles) const
    {
#define dp_norm(A,B,C) ((A - B)/(C - B))
        float minDist2 = FLT_MAX;
        float bestDP = 0;
        MTVertex* best = nullptr;

        // calculate the dot-product for the opening angle
        const float a = -((other->angle_ > 180 ? 360 - other->angle_ : other->angle_) / 180.0f);
        const float dpTolerance = dp_norm(a, -1.0f, 1.0f);
        const auto normExpansionDir = candidateExpansionDir.Normalized();
        const auto distCand = candidateExpansionDir.Length()*0.5f;

        const Vector3 extentVectors[] = {
            other->position_ + candidateExpansionDir*0.5f,
            other->position_ + candidateExpansionDir
        };

        // not doing it at the present, but if necessary could use substeps
        for (int i = 0; i < 1; ++i)
        {
            for (auto f : fronts_)
            {
                for (auto v : f->vertices_)
                {
                    if (v == other || v->IsConnectedTo(other))
                        continue;

                    // don't bother if we're outside of our candidate expansion direction
                    const Vector3 fromOtherToThis = v->position_ - extentVectors[i];
                    float d2 = fromOtherToThis.Length();
                    if (d2 > distCand)
                        continue;

                    // do we form an outer triangle? leave that to simple-fill
                    if (v->next_->IsConnectedTo(other) || v->prev_->IsConnectedTo(other))
                    {
                        // Undocumented exceptional case, emitting a 2-tri fan will corrupt the surface.
                        if (numAngles <= 1)
                            return MakePair(MTVertex::Sentinel(), FLT_MAX);
                        continue;
                    }

                    // are we within the code
                    float thisDP = normExpansionDir.DotProduct(fromOtherToThis.Normalized());
                    if (thisDP < dpTolerance)
                        continue;

                    if (d2 < minDist2 && v->normal_.DotProduct(other->normal_) > 0.25f)
                    {
                        bestDP = thisDP;
                        minDist2 = d2;
                        best = v;
                    }
                }
            }
        }
        return MakePair(best, minDist2*minDist2);
    }

    PODVector < PODVector<Pair<Vector3, Vector3> > > MarchingTriangles::GetFrontData()
    {
        PODVector < PODVector<Pair<Vector3, Vector3> > > ret;
        for (auto f : fronts_)
            ret.Push(f->CaptureFrontData());
        return ret;
    }

    void MarchingTriangles::DrawFrontData(DebugRenderer* debugRender, const PODVector < PODVector<Pair<Vector3, Vector3> > >& data)
    {
        for (auto& front : data)
        {
            for (auto& line : front)
                debugRender->AddLine(line.first_, line.second_, Color::MAGENTA);
        }
    }
}