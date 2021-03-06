#pragma once

#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Math/Matrix3.h>
#include <Urho3D/Math/Matrix3x4.h>

namespace Urho3D
{

    class Geometry;
    class Model;

    /// Calculates a transformed vertex-buffer. Honors vertex-range in the case of a shared VBO.
    URHO3D_API VertexBuffer* Transform(Geometry* src, const Matrix3x4& transform, const Matrix3& uvTransform = Matrix3::IDENTITY, bool recalculateTangents = true);
    
    /// Calculates a transformed vertex-buffer. Honors vertex-range in the case of a shared VBO.
    URHO3D_API VertexBuffer* TransformUV(Geometry* src, const Matrix3& transform, int uvSet = 0, bool recalculateTangents = true);

    /// Calculates the transformation of the given raw data, does not deal with tangents.
    URHO3D_API void Transform_Raw(const Matrix3x4& transform, unsigned vertexStart, unsigned vertexCt, const unsigned char* srcData, unsigned char* destData, unsigned srcVertexSize, unsigned destVertexSize, unsigned srcPosOffset, unsigned destPosOffset, unsigned srcNormOFfset, unsigned destNormOffset);

    /// Operates on the raw data. Sizes and offsets between the src and dest vertices may differ but the vertex count cannot.
    URHO3D_API void TransformUV_Raw(const Matrix3& uvTransform, unsigned vertexStart, unsigned vertexCt, const unsigned char* srcData, unsigned char* destData, unsigned srcVertexSize, unsigned destVertexSize, unsigned srcUVOffset, unsigned destUVOffset);

    /// Determines the centroid of each triangle in a TRIANGLE_LIST geometry.
    URHO3D_API PODVector<Vector3> CalculateTriangleCenters(Geometry* src);

    /// Determines the triangle normal of each triangle in a TRIANGLE_LIST geometry.
    URHO3D_API PODVector<Vector3> CalculateTriangleNormals(Geometry* src);

    /// Duplicates shared vertices so that each triangle has unique vertices.
    URHO3D_API Geometry* MakeVerticesUnique(Geometry* src);
    /// Duplicates shared vertices so that each triangle's vertices are unique to it.
    URHO3D_API SharedPtr<Model> MakeModelVerticesUnique(Model* src);

    typedef void(*GeoVertexFilter)(const unsigned char* vertexData, const PODVector<VertexElement>* elements);
    typedef void(*GeoEdgeFilter)(const unsigned char* startVertexData, const unsigned char* endVertexData, const PODVector<VertexElement>* elements);
    typedef void(*GeoTriangleFilter)(const unsigned char* aVertexData, const unsigned char* bVertexData, const unsigned char* cVertexData, const PODVector<VertexElement>* elements);

    /// Runs the given filter-function on all vertices.
    URHO3D_API void ProcessVertices(Geometry* geom, GeoVertexFilter processor);
    /// Runs the given filter-function on all edges (as vertex pairs).
    URHO3D_API void ProcessEdges(Geometry* geom, GeoEdgeFilter processor);
    /// Runs the given filter-function on all triangles (as vertex triplets)
    URHO3D_API void ProcessFaces(Geometry* geom, GeoTriangleFilter processor);

    typedef void(*GeoVertexConverter)(unsigned char* newVertexData, const PODVector<VertexElement>* newVertexElements, const unsigned char* oldVertexData, const PODVector<VertexElement>* oldVertexElements);
    
    /// Loops through the vertex elements and attempts to remap when possible. Not optimal, but handy if you don't care (offline).
    URHO3D_API void GenericVertexConverter(unsigned char* newVertexData, const PODVector<VertexElement>* newVertexElements, const unsigned char* oldVertexData, const PODVector<VertexElement>* oldVertexElements);

    /// General helper for converting beween vertex-data types.
    URHO3D_API Geometry* ConvertVertexData(Geometry* geom, const PODVector<VertexElement>& vertElements, bool recalcTangents, GeoVertexConverter conversion);

    typedef Geometry* (*ModelGeoConverter)(Geometry* srcGeom, unsigned index, unsigned lodLevel, Model* target);
    /// Runs the given conversion helper on all geometries in the model, the converter may return null to do its own setting of data (such as an LOD generator might).
    URHO3D_API SharedPtr<Model> ProcessModelGeometries(Model* model, ModelGeoConverter processor);

    /// Creates a copy of the provided geometry with minimal vertex-data and in canonical form.
    /// For generating occluder and shadow geometry. Only for non-transparents.
    /// Up to 20% a boost (less on stronger GPUs).
    URHO3D_API Geometry* CreateShadowGeom(Context* ctx, Geometry* forGeometry);

    /// Combines the provided geometries into one single geometry with minimal vertex-data and in canonical-form.
    /// For generating occluder and shadow geometry, only for non-transparents.
    /// Up to 20% a boost (less on stronger GPUs). This version always uses 32-bit indices.
    URHO3D_API Geometry* CreateShadowGeom(Context* ctx, const PODVector<Geometry*>& srcGeoms, const PODVector<Matrix3x4>& transforms);
}