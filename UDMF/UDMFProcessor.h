#pragma once

#include <Urho3D/Container/Vector.h>
#include <Urho3D/Math/Vector2.h>
#include <Urho3D/Math/Vector3.h>

#include <MathGeoLib.h>

namespace UDMF
{
    class SideDef;
    class LineDef;
    class Sector;

    namespace MapProcessing
    {
        /// Gets the rightmost side of a sector, will belong to the outer-most loop.
        SideDef* GetRightMost(const Sector* s);
        /// Return next side that is also part of the given sector.
        SideDef* GetNextSide(SideDef* current, Sector* sector);
        
        /// Fills vertices contiguously starting at a specific side.
        void FillVertices(const Sector* sector, SideDef* start, Urho3D::PODVector<Urho3D::Vector2>& holder);
        /// Returns the 2d vertices of the outer-edge of a sector.
        Urho3D::PODVector<Urho3D::Vector2> GetSectorVerts(const Sector* sector);

        /// Converts a chain of side-defs into a series of points (without duplicates).
        Urho3D::PODVector<Urho3D::Vector2> ToPointList(const Urho3D::PODVector<SideDef*>& list, bool asLoop);
        /// Converts the given chain of points into pseudo-normals.
        Urho3D::PODVector<Urho3D::Vector3>  ToNormals(const Urho3D::PODVector<Urho3D::Vector2>& list);
        /// Converts the collection of 2d points to 3d coorindates as X,0,Y.
        Urho3D::PODVector<Urho3D::Vector3> To3D(const Urho3D::PODVector<Urho3D::Vector2>& list);

        typedef bool(*ChainFilter)(SideDef*);
        /// Returns a list of contiguous chains of solid lines.
        Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetSectorSolidChains(Sector* s, ChainFilter filterFunc);
        /// Returns a list of contiguous chains of lower lines.
        Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetSectorLowerChains(Sector* s);
        /// Returns a list of contiguous chains of upper lines.
        Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetSectorUpperChains(Sector* s);
        /// Gets all chains of this sector, regardless
        Urho3D::Vector<Urho3D::PODVector<SideDef*>> GetAllSectorChains(Sector* s);

        /// An edge that has passed some criteria.
        struct EdgeFitment {
            Urho3D::Vector2 a;
            Urho3D::Vector2 b;
            SideDef* side;
            const Sector* sector;

            inline float EdgeLength() const { return (a - b).Length(); }
        };

        /// A corner whose outer-angle passes some criteria.
        struct HardCorner {
            MapVertex* vertex;
            Urho3D::Vector2 normal;

            bool operator==(const HardCorner& rhs) const { return vertex == rhs.vertex || normal == rhs.normal; }
            bool operator!=(const HardCorner& rhs) const { return vertex != rhs.vertex || normal != rhs.normal; }
        };

        /// Returns all of the edge fitments of a sector.
        Urho3D::PODVector<EdgeFitment> GetSectorEdgeFitments(const Sector* sector);
        /// Returns all of the hard|sharp-corners of a sector.
        Urho3D::PODVector<HardCorner> GetSectorHardCorners(const Sector* sector, float angleTolerance);

        // WARNING: polygon returned is in Doom coordinates X,Y,Z==0
        math::Polygon GetSectorPolygon(const Sector* sector);
        /// Gets a list of all polygons, starting with the outer and the inner afterwards.
        Urho3D::Vector<math::Polygon> GetSectorPolygons(const Sector* sector);
        /// Returns true if the two maps overlap.
        bool Overlaps(const UDMFMap* lhs, const UDMFMap* rhs);
        // Shrinks the given polygon by the edges.
        math::Polygon PolyShrink(const math::Polygon& poly, float distance);
        // Shrinks the given polygon by vertex normals.
        math::Polygon VertexShrink(const math::Polygon& poly, float distance);

        /// Generates a line-intersection based trim-of-cards around the outer edge of the sector, use for detritus along edges.
        void SkirtSector(const Sector* sector, float skirtDist, Urho3D::PODVector<Urho3D::Vector3>& pts, Urho3D::PODVector<unsigned>& indice, bool flipCulling);
        /// Triangulates the sector, all vertices are unique.
        void PolygonizeSector(const Sector* sector, float height, Urho3D::PODVector<Urho3D::Vector3>& pts, Urho3D::PODVector<unsigned>& indice, bool flipCulling);
        void PolygonizeSectorWithHoles(const Sector* sector, float height, Urho3D::PODVector<Urho3D::Vector3>& pts, Urho3D::PODVector<unsigned>& indice, bool flipCulling);
    }

}