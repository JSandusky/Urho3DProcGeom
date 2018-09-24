#include "kdTree.h"

namespace Urho3D
{

    bool LineSegment::Intersects(const BoundingBox& bounds) const
    {
        float hitDist = ToRay().HitDistance(bounds);
        if (hitDist == M_INFINITY)
            return false;
        return hitDist < Length();
    }

    float LineSegment::Distance(const Vector3& point) const
    {
        return ClosestPoint(point).DistanceToPoint(point);
    }
    
    Vector3 LineSegment::ClosestPoint(const Vector3& point) const
    {
        auto dir = b_ - a_;
        float d = (point - a_).DotProduct(dir) / dir.LengthSquared();
        d = Max(0, Min(d, 1));
        return a_ + d * dir;
    }

    struct kdTreeTriangle
    {
        mutable Vector3 a, b, c;

        kdTreeTriangle() { }
        kdTreeTriangle(const Vector3& a, const Vector3& b, const Vector3& c) : a(a), b(b), c(c) { }

        BoundingBox GetBounds() const {
            BoundingBox bb;
            bb.min_ = bb.max_ = a;
            bb.Merge(b);
            bb.Merge(c);
            return bb;
        }

        void Flip() const { Swap(a, c); }

        bool IntersectRayEitherSide(const Ray& ray, float* dist = 0x0, Vector3* hitPos = 0x0, Vector3* outBary = 0x0) const
        {
            Vector3 e1(b - a);
            Vector3 e2(c - a);

            Vector3 normal = e1.CrossProduct(e2);
            if (normal.Normalized().DotProduct(ray.direction_) > 0.0f)
            {
                Flip();
                e1 = (b - a);
                e2 = (c - a);
                //return false;
            }

            Vector3 p(ray.direction_.CrossProduct(e2));
            float det = e1.DotProduct(p);
            if (det < M_EPSILON)
                return false;

            Vector3 t(ray.origin_ - a);
            float u = t.DotProduct(p);
            if (u < 0.0f || u > det)
                return false;

            Vector3 q(t.CrossProduct(e1));
            float v = ray.direction_.DotProduct(q);
            if (v < 0.0f || u + v > det)
                return false;

            float distance = e2.DotProduct(q) / det;
            if (distance >= 0.0f)
            {
                if (outBary)
                {
                    *outBary = Vector3(1 - (u / det) - (v / det), u / det, v / det);
                    if (hitPos)
                        *hitPos = a * outBary->x_ + b * outBary->y_ + c * outBary->z_;
                }
                if (dist)
                    *dist = distance;
                return true;
            }
            return false;
        }

        bool ContainedBy(const BoundingBox& bounds) const
        {
            return bounds.IsInside(GetBounds()) == INSIDE;
        }

        inline Vector3 Normal() const {
            return (c - a).CrossProduct(b - a).Normalized();
        }

        inline Vector3 NormalCCW() const {
            return (b - a).CrossProduct(c - a).Normalized();
        }
    };

    kdTree::kdTree(kdTreeConstructionData& data) :
        isRoot_(true),
        data_(data)
    {
        BoundingBox bb;
        for (unsigned i = 0; i < data.positionBufferLength_; ++i)
            if (i != 0)
                bb.Merge(data.positionBuffer_[i]);
            else
                bb.min_ = bb.max_ = data.positionBuffer_[i];
        Build(data, bb, data.shortForm_, 0);
    }

    kdTree::kdTree(kdTreeConstructionData& data, const BoundingBox& bounds, PODVector<uint32_t> selectedIndices, int depth) :
        data_(data),
        bounds_(bounds),
        isRoot_(false)
    {
        Build(data, bounds, selectedIndices, depth);
    }

    kdTree::~kdTree()
    {
        if (left_)
            delete left_;
        if (right_)
            delete right_;
        left_ = right_ = 0x0;
    }

    static int MaxElementIndex(const Vector3& v)
    {
        auto maxVal = v.Abs();
        if (maxVal.x_ >= maxVal.y_ && maxVal.x_ >= maxVal.z_)
            return 0;
        if (maxVal.y_ >= maxVal.x_ && maxVal.y_ >= maxVal.z_)
            return 1;
        if (maxVal.z_ >= maxVal.y_ && maxVal.z_ >= maxVal.x_)
            return 2;
        return 0;
    }

    void kdTree::Build(kdTreeConstructionData& data, const BoundingBox& bounds, PODVector<uint32_t> shortForm, int depth)
    {
        bounds_ = bounds;
        contents_ = shortForm;

        if (contents_.Size() == 0)
            return;

        if (contents_.Size() == 1)
            return;

        const int maxAxis = MaxElementIndex(bounds_.Size());
        BoundingBox leftBounds = bounds;
        BoundingBox rightBounds = bounds;
        switch (maxAxis)
        {
        case 0:
            leftBounds.max_.x_ = bounds.Center().x_;
            rightBounds.min_.x_ = bounds.Center().x_;
            break;
        case 1:
            leftBounds.max_.y_ = bounds.Center().y_;
            rightBounds.min_.y_ = bounds.Center().y_;
            break;
        case 2:
            leftBounds.max_.z_ = bounds.Center().z_;
            rightBounds.min_.z_ = bounds.Center().z_;
            break;
        }

        PODVector<uint32_t> leftList, rightList;
        for (unsigned i = 0; i < shortForm.Size(); ++i)
        {
            const int indices[] = {
                data_.indexBuffer_[shortForm[i] * 3],
                data_.indexBuffer_[shortForm[i] * 3 + 1],
                data_.indexBuffer_[shortForm[i] * 3 + 2],
            };
            auto tri = kdTreeTriangle(data_.positionBuffer_[indices[0]], data_.positionBuffer_[indices[1]], data_.positionBuffer_[indices[2]]);
            if (tri.ContainedBy(leftBounds))
                leftList.Push(i);
            if (tri.ContainedBy(rightBounds))
                rightList.Push(i);
        }

        int matches = 0;
        for (int i = 0; i < leftList.Size(); ++i)
        {
            for (int j = 0; j < rightList.Size(); ++j)
            {
                if (leftList[i] == rightList[j])
                    matches++;
            }
        }

        if (depth < 32 || ((float)matches / leftList.Size() < 0.5f && (float)matches / rightList.Size() < 0.5f))
        {
            contents_.Clear();
            left_ = new kdTree(data, leftBounds, leftList, depth + 1);
            right_ = new kdTree(data, rightBounds, rightList, depth + 1);
        }
    }

    bool kdTree::Hit(const kdTree* tree, const LineSegment& ray, uint32_t testIndex) const
    {
        if (ray.Intersects(bounds_))
        {
            if ((left_ != 0x0 && right_ != 0x0))// && (left_->contents_.size() || right_->contents_.size()))
            {
                bool hitLeft = left_->Hit(ray, testIndex);
                if (hitLeft)
                    return true;
                bool hitRight = right_->Hit(ray, testIndex);
                if (hitRight)
                    return hitRight;
                return false;
            }
            if (!contents_.Empty())
            {
                float len = ray.Length();
                auto asRay = ray.ToRay();
                bool hitIndex = false;
                for (const auto& index : contents_)
                {
                    const int indices[] = {
                        data_.indexBuffer_[index * 3],
                        data_.indexBuffer_[index * 3 + 1],
                        data_.indexBuffer_[index * 3 + 2],
                    };

                    bool isTarget = false;
                    if (testIndex != -1 && (indices[0] == testIndex || indices[1] == testIndex || indices[2] == testIndex))
                        isTarget = true;

                    float dist = FLT_MAX;
                    if (kdTreeTriangle(data_.positionBuffer_[indices[0]], data_.positionBuffer_[indices[1]], data_.positionBuffer_[indices[2]]).IntersectRayEitherSide(asRay, &dist))
                    {
                        if (dist < ray.Length() - FLT_EPSILON)
                        {
                            return !isTarget;
                        }
                    }
                }
            }
        }
        return false;
    }

    int kdTree::Winding(const kdTree* tree, const Ray& ray, int currentWinding) const
    {
        int winding = currentWinding;
        if (ray.HitDistance(bounds_) != M_INFINITY)
        {
            if ((left_ != 0x0 && right_ != 0x0) && (left_->contents_.Size() || right_->contents_.Size()))
            {
                winding = left_->Winding(left_, ray, winding);
                winding = right_->Winding(right_, ray, winding);
            }
            if (!contents_.Empty())
            {
                for (const auto& index : contents_)
                {
                    const int indices[] = {
                        data_.indexBuffer_[index],
                        data_.indexBuffer_[index + 1],
                        data_.indexBuffer_[index + 2],
                    };

                    float dist = FLT_MAX;
                    kdTreeTriangle t(data_.positionBuffer_[indices[0]], data_.positionBuffer_[indices[1]], data_.positionBuffer_[indices[2]]);
                    if (t.IntersectRayEitherSide(ray))
                    {
                        if (t.NormalCCW().DotProduct(ray.direction_) > 0.0f)
                            ++winding;
                        else
                            --winding;
                    }
                }
            }
        }
        return winding;
    }
}