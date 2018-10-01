#include <Urho3D/UDMF/UDMFMap.h>
#include <Urho3D/UDMF/UDMFProcessor.h>

using namespace Urho3D;

namespace UDMF { 

extern Vector2 Rotate2D(Vector2 v, float rotateBy);
    
namespace MapProcessing {

    float DotPerp(float3 v1, float3 v2)
    {
        return v1.x*v2.y - v1.y*v2.x;
    }
    float3 IntersectionPoint(Line self, Line other, double dotThresh = 0.0f)
    {
        // see IntrLine2Line2 for explanation of algorithm
        float3 diff = other.pos - self.pos;
        double D0DotPerpD1 = DotPerp(self.dir, other.dir);
        if (abs(D0DotPerpD1) > dotThresh) {                    // Lines intersect in a single point.
            double invD0DotPerpD1 = ((double)1) / D0DotPerpD1;
            double diffDotPerpD1 = DotPerp(diff, other.dir);
            double s = diffDotPerpD1 * invD0DotPerpD1;
            return self.pos + s * self.dir;
        }
        // Lines are parallel.
        return float3(FLT_MAX, FLT_MAX, FLT_MAX);
    }

    SideDef* GetRightMost(const Sector* sector)
    {
        SideDef* curBest = sector->sides_[0];
        float x = FLT_MIN;
        for (SideDef* s : sector->sides_)
        {
            if (s->line_->start_->position_.x_ > x)
            {
                curBest = s;
                x = s->line_->start_->position_.x_;
            }
            if (s->line_->end_->position_.x_ > x)
            {
                curBest = s;
                x = s->line_->end_->position_.x_;
            }
        }
        return curBest;
    }

    SideDef* GetNextSide(SideDef* current, Sector* sector)
    {
        if (current->isFront_)
        {
            for (SideDef* side : sector->sides_)
            {
                if (side == current)
                    continue;
                if (side->isFront_)
                {
                    if (side->line_->start_ == current->line_->end_)
                        return side;
                }
                else if (side->line_->end_ == current->line_->end_)
                    return side;
            }
        }
        else
        {
            for (SideDef* side : sector->sides_)
            {
                if (side == current)
                    continue;
                if (side->isFront_)
                {
                    if (side->line_->start_ == current->line_->start_)
                        return side;
                }
                else if (side->line_->end_ == current->line_->start_)
                    return side;
            }
        }
        return nullptr;
    }

    void FillVertices(const Sector* sector, SideDef* start, Urho3D::PODVector<Urho3D::Vector2>& pts)
    {
        PODVector<SideDef*> hit;
        SideDef* current = start;
        hit.Push(start);
        if (start->isFront_)
        {
            pts.Push(start->line_->start_->position_);
            pts.Push(start->line_->end_->position_);
        }
        else
        {
            pts.Push(start->line_->end_->position_);
            pts.Push(start->line_->start_->position_);
        }
        int tries = 0;
        do
        {
            current = GetNextSide(current, (Sector*)sector);
            if (current != nullptr && hit.Contains(current) == false)
            {
                hit.Push(current);
                if (current->isFront_)
                    pts.Push(current->line_->end_->position_);
                else
                    pts.Push(current->line_->start_->position_);
                tries = 0;
            }
            else
                ++tries;
            if (tries > 10)
                break;
        } while (current != nullptr && current != start);
        pts.Remove(pts.Back());
    }

    Urho3D::PODVector<Urho3D::Vector2> GetSectorVerts(const Sector* sector)
    {
        PODVector<Urho3D::Vector2> ret;
        FillVertices(sector, GetRightMost(sector), ret);
        return ret;
    }

    Urho3D::PODVector<Urho3D::Vector2> ToPointList(const Urho3D::PODVector<SideDef*>& chain)
    {
        PODVector<Vector2> ret;
        for (int i = 0; i < chain.Size(); ++i)
        {
            if (chain[i]->isFront_)
                ret.Push(chain[i]->line_->start_->position_);
            else
                ret.Push(chain[i]->line_->end_->position_);
        }
        if (chain.Size() > 0)
        {
            if (chain[chain.Size() - 1]->isFront_)
                ret.Push(chain[chain.Size() - 1]->line_->end_->position_);
            else
                ret.Push(chain[chain.Size() - 1]->line_->start_->position_);
        }
        return ret;
    }

    Urho3D::PODVector<Urho3D::Vector3> ToPseudoNormals(const Urho3D::PODVector<Urho3D::Vector2>& list)
    {
        PODVector<Line> lines;
        for (int i = 0; i < list.Size() - 1; ++i)
        {
            auto cur = list[i];
            auto next = list[i+1];
            auto dir = (next - cur).Normalized();
            Line l(float3(cur.x_, 0, cur.y_), float3(dir.x_, 0, dir.y_));
            lines.Push(l);
        }

        PODVector<Vector3> points;
        for (int i = 0; i < lines.Size() - 1; ++i)
        {
            auto& thisLine = lines[i];
            auto& nextLine = lines[i + 1];
            auto hitPt = IntersectionPoint(thisLine, nextLine);
            points.Push(Vector3(hitPt.x, hitPt.y, hitPt.z));
        }
        return points;
    }

    Urho3D::PODVector<Urho3D::Vector3>  ToNormals(const Urho3D::PODVector<Urho3D::Vector2>& vec)
    {
        PODVector<Vector3> ret;
        for (int i = 0; i < vec.Size(); ++i)
        {
            int prev = i - 1;
            int next = i + 1;
            Vector2 pt = vec[i];
            Vector2 cur;
            Vector2 backup;
            if (prev >= 0)
            {
                auto prevPt = vec[prev];
                prevPt = Rotate2D((pt - prevPt), -90);
                prevPt.Normalize();
                backup = prevPt;
                cur += prevPt;
            }
            else if (i == 0 && vec.Front() == vec.Back())
            {
                auto prevPt = vec[vec.Size() - 2];
                prevPt = Rotate2D((pt - prevPt), -90);
                prevPt.Normalize();
                backup = prevPt;
                cur += prevPt;
            }
            if (next < vec.Size())
            {
                auto nextPt = vec[next];
                nextPt = Rotate2D((nextPt - pt), -90);
                nextPt.Normalize();
                backup = nextPt;
                cur += nextPt;
            }
            else if (i == vec.Size() - 1 && vec.Front() == vec.Back())
            {
                auto nextPt = vec[1];
                nextPt = Rotate2D((nextPt - pt), -90);
                nextPt.Normalize();
                backup = nextPt;
                cur += nextPt;
            }
            cur.Normalize();
            if (isnan(cur.x_) || isnan(cur.y_))
                cur = backup;
            ret.Push(Vector3(cur.x_, 0, cur.y_));
        }
        return ret;
    }

    Urho3D::PODVector<Urho3D::Vector3> To3D(const Urho3D::PODVector<Urho3D::Vector2>& vec)
    {
        PODVector<Vector3> ret;
        for (int i = 0; i < vec.Size(); ++i)
            ret.Push(Vector3(vec[i].x_, 0, vec[i].y_));
        return ret;
    }

    Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetSectorSolidChains(Sector* s, ChainFilter filterFunc)
    {
        PODVector<PODVector<SideDef*>> ret;

        PODVector<SideDef*> sides = s->sides_;

        static auto sidesWhere = [](const PODVector<SideDef*>& sides, ChainFilter filter) {
            PODVector<SideDef*> r;
            for (auto s : sides)
                if (filter(s))
                    r.Push(s);
            return r;
        };

        sides = sidesWhere(sides, filterFunc);
        while (sides.Size() > 0)
        {
            PODVector<SideDef*> lineChain;
            for (int i = 0; i < sides.Size(); ++i)
            {
                if (lineChain.Size() == 0)
                {
                    lineChain.Push(sides[i]);
                    sides.Erase(i);
                    i = -1;
                    continue;
                }

                auto first = lineChain.Front();
                auto last = lineChain.Back();
                if (last->line_->end_ == sides[i]->line_->start_)
                {
                    lineChain.Push(sides[i]);
                    sides.Erase(i);
                    i = -1;
                }
                else if (first->line_->start_ == sides[i]->line_->end_)
                {
                    lineChain.Insert(0, sides[i]);
                    sides.Erase(i);
                    i = -1;
                }
            }
            if (lineChain.Size() > 0)
                ret.Push(lineChain);
        }

        return ret;
    }

    Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetSectorLowerChains(Sector* s)
    {
        PODVector<PODVector<SideDef*>> ret;

        bool hitImpassable = false;
        PODVector<SideDef*> working;
        SideDef* lastSide = nullptr;
        for (int z = 0; z < s->sides_.Size(); ++z)
        {
            for (auto side : s->sides_)
            {
                if (side->line_->metadata_.impassable_)
                {
                    if (hitImpassable == false)
                    {
                        hitImpassable = true;
                        lastSide = side;
                    }
                    else if (SideDef::IsConnected(lastSide, side))
                    {
                        if (working.Size() > 0)
                            ret.Push(working);
                        working.Clear();
                        hitImpassable = false;
                    }
                }
                else if (hitImpassable && SideDef::IsConnected(lastSide, side)
                    && side->sector_->floorHeight_ != side->GetOpposite()->sector_->floorHeight_)
                {
                    if (!working.Contains(side))
                        working.Push(side);
                    lastSide = side;
                }
            }
        }

        return ret;
    }

    Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetSectorUpperChains(Sector* s)
    {
        PODVector<PODVector<SideDef*>> ret;

        bool hitImpassable = false;
        PODVector<SideDef*> working;
        SideDef* lastSide = nullptr;
        for (int z = 0; z < s->sides_.Size(); ++z)
        {
            for (auto side : s->sides_)
            {
                if (side->line_->metadata_.impassable_)
                {
                    if (hitImpassable == false)
                    {
                        hitImpassable = true;
                        lastSide = side;
                    }
                    else if (SideDef::IsConnected(lastSide, side))
                    {
                        if (working.Size() > 0)
                            ret.Push(working);
                        working.Clear();
                        hitImpassable = false;
                    }
                }
                else if (hitImpassable && SideDef::IsConnected(lastSide, side)
                    && side->sector_->ceilingHeight_ != side->GetOpposite()->sector_->ceilingHeight_)
                {
                    if (!working.Contains(side))
                        working.Push(side);
                    lastSide = side;
                }
            }
        }

        return ret;
    }

    Urho3D::PODVector<EdgeFitment> GetSectorEdgeFitments(const Sector* sector)
    {
        PODVector<EdgeFitment> ret;
        for (auto side : sector->sides_)
        {
            if (side->line_->metadata_.impassable_)
            {
                ret.Push(EdgeFitment
                {
                    side->line_->start_->position_,
                    side->line_->end_->position_,
                    side,
                    sector // possibly redundant, but that depends on how the function is being used
                });
            }
        }
        return ret;
    }

    Urho3D::PODVector<HardCorner> GetSectorHardCorners(const Sector* sector, float angleTolerance)
    {
        Urho3D::PODVector<HardCorner> ret;
        for (auto line : sector->lines_)
        {
            for (auto vertex : sector->vertices_)
            {
                bool done = false;
                for (auto line : vertex->lines_)
                {
                    if (!line->metadata_.impassable_)
                        continue;
                    for (auto otherLine : vertex->lines_)
                    {
                        if (otherLine == line || otherLine->metadata_.impassable_)
                            continue;
                        if (abs(line->faceNormal_.DotProduct(otherLine->faceNormal_)) < angleTolerance)
                        {
                            MapVertex* vert = line->GetSharedVertex(otherLine);
                            if (vert != nullptr)
                            {
                                auto hc = HardCorner { vert, (line->faceNormal_ + otherLine->faceNormal_).Normalized() };
                                if (!ret.Contains(hc))
                                    ret.Push(hc);
                            }
                            done = true;
                            break;
                        }
                    }
                    if (done)
                        break;
                }
            }
            //if (line->metadata_.impassable_)
            //{
            //    auto nextLine = line->GetNextSolid(sector);
            //    if (nextLine != nullptr && abs(line->faceNormal_.DotProduct(nextLine->faceNormal_)) < angleTolerance)
            //    {
            //        MapVertex* vert = line->GetSharedVertex(nextLine);
            //        if (vert != nullptr)
            //        {
            //            auto hc = HardCorner{ vert, (line->faceNormal_ + nextLine->faceNormal_).Normalized() };
            //            if (!ret.Contains(hc))
            //                ret.Push(hc);
            //        }
            //    }
            //}
        }
        return ret;
    }

    Polygon GetSectorPolygon(const Sector* sector)
    {
        auto verts = GetSectorVerts(sector);
        Polygon poly;
        for (auto v : verts)
            poly.p.push_back(float3(v.x_, v.y_, 0));
        return poly;
    }

    bool Overlaps(const UDMFMap* lhs, const UDMFMap* rhs)
    {
        for (auto sector : lhs->sectors_)
        {
            auto poly = VertexShrink(GetSectorPolygon(sector), -0.01f);
            for (auto otherSector : rhs->sectors_)
            {
                auto otherPoly = VertexShrink(GetSectorPolygon(otherSector), -0.01f);
                if (poly.Intersects(otherPoly))
                    return true;
            }
        }
        return false;
    }

    float3 Perp(float3 v)
    {
        return float3(v.y_, -v.x_, v.z_);
    }

    float3 GetNormal_FaceAvg(const Polygon& p, int i)
    {
        float3 next = p.p[(i + 1) % p.p.size()];
        float3 prev = p.p[i == 0 ? p.p.size() - 1 : i - 1];
        next -= p.p[i]; next.Normalize();
        prev -= p.p[i]; prev.Normalize();

        float3 n = Perp(next) - Perp(prev);
        float len = n.Normalize();
        if (len == 0)
            return (next + prev).Normalized();   // this gives right direction for degenerate angle
        else
            return n;
    }

    Polygon PolyShrink(const Polygon& poly, float dist)
    {
        std::vector<float3> newv;
        newv.resize(poly.p.size());
        for (int k = 0; k < poly.p.size(); ++k) {
            auto v = poly.p[k];
            auto next = poly.p[(k + 1) % poly.p.size()];
            auto prev = poly.p[k == 0 ? poly.p.size() - 1 : k - 1];
            auto dn = (next - v).Normalized();
            auto dp = (prev - v).Normalized();
            Line ln = Line(v + dist * Perp(dn), dn);
            Line lp = Line(v - dist * Perp(dp), dp);

            newv[k] = IntersectionPoint(ln, lp);
            if (newv[k] == float3(FLT_MAX, FLT_MAX, FLT_MAX))
                newv[k] = poly.p[k] + dist * GetNormal_FaceAvg(poly, k);
        }

        Polygon newPoly;
        for (int k = 0; k < poly.p.size(); ++k)
            newPoly.p.push_back(newv[k]);
        return newPoly;
    }

    Polygon VertexShrink(const Polygon& poly, float distance)
    {
        Polygon r;
        for (int i = 0; i < poly.p.size(); ++i)
        {
            vec thisNorm;
            vec prevNorm;
            if (i > poly.NumEdges())
                thisNorm = poly.EdgeNormal(i - poly.NumEdges());
            else
                thisNorm = poly.EdgeNormal(i);
            if (i - 1 < 0)
                prevNorm = poly.EdgeNormal(poly.NumEdges() - 1);
            else
                prevNorm = poly.EdgeNormal(i - 1);

            auto offset = (thisNorm + prevNorm).Normalized();
            r.p.push_back(poly.p[i] + offset * distance);
        }
        return r;
    }

    void SkirtSector(const Sector* sector, float skirtDist, Urho3D::PODVector<Urho3D::Vector3>& mesh, Urho3D::PODVector<unsigned>& indices, bool flipCulling)
    {
        auto poly = GetSectorPolygon(sector);
        auto shrunk = PolyShrink(poly, skirtDist);

        int idx = 0;
        for (int i = 0; i < poly.p.size(); ++i)
        {
            int next = i + 1;
            if (next >= poly.p.size())
                next = 0;

            auto thisPos = poly.p[i];
            auto nextPos = poly.p[next];

            auto thisOuter = shrunk.p[i];
            auto nextOuter = shrunk.p[next];

            const auto floorHeight = sector->floorHeight_;
            mesh.Push(Vector3(thisPos.x, floorHeight, thisPos.y));
            mesh.Push(Vector3(nextPos.x, floorHeight, nextPos.y));
            mesh.Push(Vector3(thisOuter.x, floorHeight, thisOuter.y));
            mesh.Push(Vector3(nextOuter.x, floorHeight, nextOuter.y));

            if (flipCulling)
            {
                indices.Push(idx + 0);
                indices.Push(idx + 3);
                indices.Push(idx + 2);

                indices.Push(idx + 0);
                indices.Push(idx + 1);
                indices.Push(idx + 3);
            }
            else
            {
                indices.Push(idx + 0);
                indices.Push(idx + 2);
                indices.Push(idx + 3);

                indices.Push(idx + 0);
                indices.Push(idx + 3);
                indices.Push(idx + 1);
            }
            idx += 4;
        }
    }

    void PolygonizeSector(const Sector* sector, float height, Urho3D::PODVector<Urho3D::Vector3>& mesh, Urho3D::PODVector<unsigned>& indice, bool flipCulling)
    {
        auto poly = GetSectorPolygon(sector);
        auto tris = poly.Triangulate();
        int idx = 0;
        for (unsigned i = 0; i < tris.size(); ++i, idx += 3)
        {
            mesh.Push(Vector3(tris[i].v0.x_, height, tris[i].v0.y_));
            mesh.Push(Vector3(tris[i].v1.x_, height, tris[i].v1.y_));
            mesh.Push(Vector3(tris[i].v2.x_, height, tris[i].v2.y_));

            if (flipCulling)
            {
                indice.Push(idx + 0);
                indice.Push(idx + 2);
                indice.Push(idx + 1);
            }
            else
            {
                indice.Push(idx + 0);
                indice.Push(idx + 1);
                indice.Push(idx + 2);
            }
        }
    }
} }