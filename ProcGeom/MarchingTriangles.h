#pragma once

#include <Urho3D/Math/Matrix3x4.h>
#include <Urho3D/Container/Pair.h>
#include <Urho3D/Container/Vector.h>
#include <Urho3D/Math/Vector2.h>
#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Math/Plane.h>

namespace Urho3D
{
    class DebugRenderer;
    class Context;
    class Geometry;
    struct MTFront;

    void ImplicitTangentSpace(const Vector3& norm, Vector3& tangent, Vector3& binormal);

    /// A vertex on the advancing front.
    struct MTVertex 
    {
        /// Index of the vertex in the stored position+normal data.
        unsigned index_;
        /// Position of the vertex.
        Vector3 position_;
        /// Normal of the vertex.
        Vector3 normal_;
        /// Opening angle of the vertex.
        float angle_;
        /// Front to which this vertex belongs.
        MTFront* front_;
        /// Previous vertex in the loop.
        MTVertex* prev_;
        /// Next vertex in the loop.
        MTVertex* next_;
        /// Marker flag for angle recalc. Angle calculation is expensive, don't want to do it on the whole front.
        bool angleDirty_ = true;

        /// Construct.
        MTVertex() { }
        /// Construct, for initializer list.
        MTVertex(unsigned index, const Vector3& position, const Vector3& normal, float angle, MTFront* front, MTVertex* prev, MTVertex* next) :
            index_(index), position_(position), normal_(normal), angle_(angle),
            front_(front), prev_(prev), next_(next_), angleDirty_(true)
        {

        }
        ~MTVertex() {
            
        }

        /// Clones this vertex.
        MTVertex* Clone();

        /// Calculates the `opening angle`, which is a full 360-degree angle betweent the edges at this vertex.
        void CalculateAngle();

        /// Helper for safely setting the next vertex.
        inline void SetNext(MTVertex* next) { next_ = next; next_->prev_ = this; angleDirty_ = true; next_->angleDirty_ = true; }
        /// Helper for safely setting the previous vertex.
        inline void SetPrev(MTVertex* prev) { prev_ = prev; prev_->next_ = this; angleDirty_ = true; prev_->angleDirty_ = true; }
        /// Returns true if this vertex is connected to the other one.
        inline bool IsConnectedTo(const MTVertex* other) const { return other == prev_ || other == next_; }

        /// Tests if two vertices are in the same loop.
        bool InSameLoop(const MTVertex* other) const;

        /// forms a prev -> center -> next linkage, reduced error risk
        static void LinkTriplet(MTVertex* a, MTVertex* b, MTVertex* c) {
            a->next_ = b;
            b->prev_ = a;
            b->next_ = c;
            c->prev_ = a;
            a->angleDirty_ = b->angleDirty_ = c->angleDirty_ = true;
        }

        /// Returns true if this edge is degenerate.
        bool IsDenegerate() const { return prev_ == next_; }
        /// Returns true if this vertex is part of a triangle.
        bool IsTriangle() const { return next_->next_ == prev_; }
        /// Returns true if this vertex is part of a quad.
        bool IsQuad() const { return next_->next_ == prev_->prev_; }

        /// Get the direction vector to the previous vertex.
        inline Vector3 GetPrevDir() const { return (prev_->position_ - position_).Normalized(); }
        /// Get the direction vector to the next vertex.
        inline Vector3 GetNextDir() const { return (next_->position_ - position_).Normalized(); }

        /// Magic value for 
        static MTVertex* Sentinel() { 
#ifdef URHO3D_64BIT
            return (MTVertex*)((unsigned long long)-1);
#else
            return (MTVertex*)((unsigned)-1);
#endif
        }
    };

    /// An advancing front from which triangles will be emitted.
    struct MTFront 
    {
        /// List of vertices in the front, they're not necessarilly in sequence.
        PODVector<MTVertex*> vertices_;
        /// Manually added fronts may desire to have greater tolerances for merging/splitting.
        float tolerance_ = 0.0f;

        /// Returns vertex and the angle.
        Pair<MTVertex*, float> TightestAngle() const;
        /// Returns vertex and the dist2.
        Pair<MTVertex*, float> Nearest(const MTVertex* other, const Vector3& candidateExpansionDir) const;
        /// Returns the resulting split front.
        MTFront* SplitFront(MTVertex* vA, MTVertex* vB);
        /// Merges two fronts into one.
        void MergeFronts(MTFront* a);
        /// Merges two fronts into one.
        void FillWithLoop(MTVertex* a);

        /// Verify that the data is okay.
        void SanityCheck();

        /// Destruct and delete contained vertices.
        ~MTFront();

        /// Captures the edges of the front in its' current form. For debug rendering.
        PODVector<Pair<Vector3, Vector3> > CaptureFrontData() const;
    };

    typedef float (*SurfacingFunction)(const Urho3D::Vector3&);

    /// Uses a surface walking algorithm to mesh an implicit surface.
    /// Pros over voxel based primal/dual methods:
    ///     - fairly regular triangles
    ///     - modest memory requirements
    ///         - reducing voxel memory footprint means performing the same SDF evaluations over and over again
    ///     - can mesh arbitrary authored geometries into the surface (requires an open-loop)
    ///     - vertex-sharing is trivial
    /// Downsides: 
    ///     - risk of the surface never finishing from inconsistent sdf functions
    ///     - independent meshes need to be independent, `floating balls` are non-trivial to mesh
    /// Technical:
    ///     - This implementation is roughly based on `Curvature Dependent Polygonization of Implicit Surfaces`
    ///         - de Araujo; Jorge: http://www.cs.toronto.edu/~brar/blobmaker/ISpoligonization.pdf
    ///     - Curvature adaptation is not present
    ///         - a crude (and faster) method would be to incrementally walk the candidate 
    ///           direction and measure the deviation to determine a desired edge-length.
    class MarchingTriangles
    {
        friend struct MTVertex;
        friend struct MTFront;
    public:
        /// Construct. Uses default tolerance of edgeLength^2 * 1.5.
        MarchingTriangles(float edgeLength);
        /// Construct, with a custom tolerance.
        MarchingTriangles(float edgeLength, float tolerance);
        /// Destruct.
        virtual ~MarchingTriangles();

        /// Uses seed-point finding to mesh an implicit surface.
        void GenerateSurface(SurfacingFunction sdfFunc, const Vector3& seedPoint = Vector3::ZERO, unsigned maxTris = 10000);
        /// Only to be used when custom geometry has been added, to allow starting from an existing front and not clearing the existing data.
        void GenerateSurface(SurfacingFunction sdfFunc, MTFront* startFrom, unsigned maxTris = 10000);
        /// Injects a custom mesh into the data and prepares it for
        MTFront* AddCustomGeometry(Geometry* geometry, const Matrix3x4& transform, float reachTolerance = 0.0f);
        
        /// Converts the data into a triangle mesh.
        Geometry* ExtractGeometry(Context* ctx);
        /// Returns true if any geometry has been written.
        bool HasGeometry() const { return indices_.Size() > 0; }

        /// For Debugging: starts incremental execution
        virtual void Initialize(SurfacingFunction sdfFunc, const Vector3& seedPoint = Vector3::ZERO, unsigned maxTris = 10000);
        /// For Debugging: Advances incremental execution, returns true if the surface is complete.
        virtual bool Advance();

        /// Grabs A->B edges for all of the fronts.
        PODVector < PODVector<Pair<Vector3, Vector3> > > GetFrontData();
        /// Helper to draw those A->B front edges to a debug renderer.
        static void DrawFrontData(DebugRenderer* debugRender, const PODVector < PODVector<Pair<Vector3, Vector3> > >& data);

    protected:
        /// Runs the surface generation algorithm. An adaptive implementation may wish to override.
        virtual void InternalSurfaceGeneration();
        /// Finds a starting-point on the surface by evaluating the SDF beginning at the seed point.
        void SeedSurface(const Vector3& seedPoint);
        /// Erases all fronts.
        void ClearFronts();
        /// Updates the angles of vertices on the fronts.
        void CalculateAngles();
        /// Calculates the normal via the SDF at the given point.
        Vector3 CalculateNormal(const Vector3& norm) const;
        /// Gets the nearest vertex along the candidate expansion direction. Returned as vert, distance-squared.
        Pair<MTVertex*, float> Nearest(const MTVertex* other, const Vector3& candidateExpansionDir, const Vector3& normal, int numAngles) const;

        /// Index to use for the next vertex, ensures optimal surface reuse of vertices.
        unsigned nextVertexIndex_;
        /// Once this number is exceed the surfacing algorithm will exit, assuming that it has failed to close the fronts.
        unsigned maxTris_ = 500;
        /// List of active fronts.
        PODVector<MTFront*> fronts_;
        /// List of vertex positions.
        PODVector<Vector3> positions_;
        /// List of vertex normals.
        PODVector<Vector3> normals_;
        /// Triangle indices.
        PODVector<unsigned> indices_;
        /// Function being used currently.
        SurfacingFunction sdfFunc_;
        /// Desired triangle edge length.
        float edgeLength_;
        /// Squared-distance limit for splitting fronts.
        float frontProximityEdgeLength_;
    };
}