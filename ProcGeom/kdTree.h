#pragma once

#include <Urho3D/Math/BoundingBox.h>
#include <Urho3D/Math/Ray.h>
#include <Urho3D/Container/Vector.h>
#include <Urho3D/Math/Vector3.h>

namespace Urho3D
{

    struct LineSegment
    {
        Vector3 a_, b_;

        LineSegment(const Ray& ray, float length);
        LineSegment(const Vector3& a, const Vector3& b) {
            a_ = a;
            b_ = b;
        }

        inline Vector3 Direction() const { return (b_ - a_).Normalized(); }
        inline float Length() const { return (b_ - a_).Length(); }
        inline float LengthSquared() const { return (b_ - a_).LengthSquared(); }
        inline Ray ToRay() const { return Ray(a_, (b_ - a_).Normalized()); }
        bool Intersects(const BoundingBox& bounds) const;
        float Distance(const Vector3& point) const;
        Vector3 ClosestPoint(const Vector3& point) const;
    };

    struct kdTreeConstructionData
    {
        Vector3* positionBuffer_;
        unsigned positionBufferLength_;
        uint32_t* indexBuffer_;
        unsigned indexBufferLength_;
        PODVector<uint32_t> shortForm_;

        void Pack() {
            shortForm_.Reserve(indexBufferLength_ / 3);
            for (int i = 0; i < indexBufferLength_ / 3; ++i)
                shortForm_.Push(i);
        }
    };

    struct kdTree
    {
        BoundingBox bounds_;
        PODVector<uint32_t> contents_;
        bool isRoot_ = false;
        kdTree* left_ = 0x0;
        kdTree* right_ = 0x0;
        kdTreeConstructionData& data_;

        kdTree(kdTreeConstructionData& data);
        kdTree(kdTreeConstructionData& data, const BoundingBox& bounds, PODVector<uint32_t> selectedIndices, int depth);
        ~kdTree();

        void Build(kdTreeConstructionData& data, const BoundingBox& bounds, PODVector<uint32_t> shortForm, int depth);

        inline bool kdTree::Hit(const LineSegment& ray, uint32_t testIdex = -1) const
        {
            return Hit(this, ray, testIdex);
        }

        inline int kdTree::Winding(const Ray& ray) const
        {
            return Winding(this, ray, 0);
        }

        bool Hit(const kdTree* tree, const LineSegment& ray, uint32_t testIndex = -1) const;
        int Winding(const kdTree* tree, const Ray& ray, int currentWinding) const;
    };

}