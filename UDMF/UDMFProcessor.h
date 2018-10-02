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
        SideDef* GetRightMost(const Sector* s);
        SideDef* GetNextSide(SideDef* current, Sector* sector);
        
        void FillVertices(const Sector* sector, SideDef* start, Urho3D::PODVector<Urho3D::Vector2>& holder);
        Urho3D::PODVector<Urho3D::Vector2> GetSectorVerts(const Sector* sector);

        Urho3D::PODVector<Urho3D::Vector2> ToPointList(const Urho3D::PODVector<SideDef*>& list);
        Urho3D::PODVector<Urho3D::Vector3>  ToPseudoNormals(const Urho3D::PODVector<Urho3D::Vector2>& list);
        Urho3D::PODVector<Urho3D::Vector3>  ToNormals(const Urho3D::PODVector<Urho3D::Vector2>& list);
        Urho3D::PODVector<Urho3D::Vector3> To3D(const Urho3D::PODVector<Urho3D::Vector2>& list);

        typedef bool(*ChainFilter)(SideDef*);
        Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetSectorSolidChains(Sector* s, ChainFilter filterFunc);
        Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetSectorLowerChains(Sector* s);
        Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetSectorUpperChains(Sector* s);

        struct EdgeFitment {
            Urho3D::Vector2 a;
            Urho3D::Vector2 b;
            SideDef* side;
            const Sector* sector;

            inline float EdgeLength() const { return (a - b).Length(); }
        };

        struct HardCorner {
            MapVertex* vertex;
            Urho3D::Vector2 normal;

            bool operator==(const HardCorner& rhs) const { return vertex == rhs.vertex || normal == rhs.normal; }
            bool operator!=(const HardCorner& rhs) const { return vertex != rhs.vertex || normal != rhs.normal; }
        };

        Urho3D::PODVector<EdgeFitment> GetSectorEdgeFitments(const Sector* sector);
        Urho3D::PODVector<HardCorner> GetSectorHardCorners(const Sector* sector, float angleTolerance);

        // WARNING: polygon returned is in Doom coordinates X,Y,Z==0
        math::Polygon GetSectorPolygon(const Sector* sector);
        /// Returns true if the two maps overlap.
        bool Overlaps(const UDMFMap* lhs, const UDMFMap* rhs);
        // Shrinks the given polygon by the edges.
        math::Polygon PolyShrink(const math::Polygon& poly, float distance);
        // Shrinks the given polygon by vertex normals.
        math::Polygon VertexShrink(const math::Polygon& poly, float distance);

        void SkirtSector(const Sector* sector, float skirtDist, Urho3D::PODVector<Urho3D::Vector3>& pts, Urho3D::PODVector<unsigned>& indice, bool flipCulling);
        void PolygonizeSector(const Sector* sector, float height, Urho3D::PODVector<Urho3D::Vector3>& pts, Urho3D::PODVector<unsigned>& indice, bool flipCulling);
    }

}