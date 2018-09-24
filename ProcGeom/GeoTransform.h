#pragma once

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

    /// Determines the centroid of each triangle in a TRIANGLE_LIST geometry.
    URHO3D_API PODVector<Vector3> CalculateTriangleCenters(Geometry* src);

    /// Determines the triangle normal of each triangle in a TRIANGLE_LIST geometry.
    URHO3D_API PODVector<Vector3> CalculateTriangleNormals(Geometry* src);

    URHO3D_API Geometry* MakeVerticesUnique(Geometry* src);

    URHO3D_API SharedPtr<Model> MakeModelVerticesUnique(Model* src);

    typedef void(*GeoVertexFilter)(const unsigned char* vertexData, const PODVector<VertexElement>* elements);
    typedef void(*GeoEdgeFilter)(const unsigned char* startVertexData, const unsigned char* endVertexData, const PODVector<VertexElement>* elements);
    typedef void(*GeoTriangleFilter)(const unsigned char* aVertexData, const unsigned char* bVertexData, const unsigned char* cVertexData, const PODVector<VertexElement>* elements);

    URHO3D_API void ProcessVertices(Geometry* geom, GeoVertexFilter processor);
    URHO3D_API void ProcessEdges(Geometry* geom, GeoEdgeFilter processor);
    URHO3D_API void ProcessFaces(Geometry* geom, GeoTriangleFilter processor);

}