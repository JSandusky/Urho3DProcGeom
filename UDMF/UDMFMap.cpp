#include <Urho3D/UDMF/UDMFMap.h>

#include <Urho3D/Core/StringUtils.h>
#include <Urho3D/IO/VectorBuffer.h>
#include <Urho3D/Resource/Image.h>

#include <string>

using namespace Urho3D;

namespace UDMF
{

    Vector2 Rotate2D(Vector2 v, float rotateBy)
    {
        Vector2 newVec = v;
        float sn = sin(rotateBy * M_DEGTORAD);
        float cs = cos(rotateBy * M_DEGTORAD);

        newVec.x_ = v.x_ * cs - v.y_ * sn;
        newVec.y_ = v.x_ * sn + v.y_ * cs;
        return newVec;
    }

    void MapObject::ReadFlags(Urho3D::XMLElement elem)
    {
        auto flagNodes = elem.Select(".//flag");
        for (int i = 0; i < flagNodes.Size(); ++i)
        {
            auto flagElem = flagNodes[i];
            String flagName = flagElem.GetAttribute("name");
            String flagValue = flagElem.GetAttribute("value");
            flags_[flagName] = flagValue;
        }
    }

    void MapObject::ReadFields(Urho3D::XMLElement elem)
    {
        auto flagNodes = elem.Select(".//field");
        for (int i = 0; i < flagNodes.Size(); ++i)
        {
            auto flagElem = flagNodes[i];
            String flagName = flagElem.GetAttribute("name");
            String flagValue = flagElem.GetAttribute("value");
            properties_[flagName] = flagValue;
        }
    }

    SideDef* SideDef::GetOpposite() const
    {
        return line_->front_ == this ? line_->back_ : line_->front_;
    }

    bool SideDef::IsConnected(SideDef* from, SideDef* other)
    {
        if (from == nullptr || other == nullptr)
            return false;
        if (from->isFront_)
        {
            if (other->isFront_)
                return other->line_->start_ == from->line_->end_;
            else
                return other->line_->end_ == from->line_->end_;
        }
        else
        {
            if (other->isFront_)
                return other->line_->start_ == from->line_->start_;
            else
                return other->line_->end_ == from->line_->start_;
        }
    }

    SideDef* SideDef::GetNext(UDMFMap* map, std::set<SideDef*>& ignore)
    {
        for (int i = 0; i < map->sides_.Size(); ++i)
        {
            SideDef* s = map->sides_[i];
            if (s == this)
                continue;
            if (IsConnected(s, this) && ignore.find(s) == ignore.end())
                return s;
        }
        return nullptr;
    }

    void LineDef::NormalizeSides()
    {
        // If only 1 side then swap if it's the back-face
        if (front_ == nullptr && back_ != nullptr)
            front_ = back_;
    }

    LineDef* LineDef::GetNextSolid() const
    {
        for (int i = 0; i < end_->lines_.Size(); ++i)
        {
            LineDef* l = end_->lines_[i];
            if (l->GetSharedVertex(this) == end_ && l->metadata_.impassable_)
                return l;
        }
        return nullptr;
    }

    LineDef* LineDef::GetPrevSolid() const
    {
        for (int i = 0; i < start_->lines_.Size(); ++i)
        {
            LineDef* l = start_->lines_[i];
            if (l->GetSharedVertex(this) == start_ && l->metadata_.impassable_)
                return l;
        }
        return nullptr;
    }

    LineDef* LineDef::GetNextSolid(const Sector* s) const
    {
        for (auto l : end_->lines_)
        {
            if (l->start_ == end_ && l->metadata_.impassable_ && l->front_->sector_ == s)
                return l;
        }
        return nullptr;
    }

    LineDef* LineDef::GetPrevSolid(const Sector* s) const
    {
        for (auto l : end_->lines_)
        {
            if (l->end_ == start_ && l->metadata_.impassable_ && l->front_->sector_ == s)
                return l;
        }
        return nullptr;
    }

    MapVertex* LineDef::GetSharedVertex(const LineDef* rhs) const
    {
        if (rhs->start_ == start_ || rhs->end_ == start_)
            return start_;
        if (rhs->end_ == end_ || rhs->start_ == end_)
            return end_;
        return nullptr;
    }

    bool LineDef::IsSameAs(const LineDef* other) const
    {
        if (start_ == other->start_ && end_ == other->end_)
            return true;
        if (start_ == other->end_ && end_ == other->start_)
            return true;
        return false;
    }

    void Sector::CalculateLineNormals()
    {
        for (auto l : lines_)
        {
            Vector2 lineVec = l->GetLineVec().Normalized();
            lineVec = Rotate2D(lineVec, 90);
            l->faceNormal_ = lineVec;
        }
    }

    void Sector::ClearLineMarks()
    {
        for (auto l : lines_)
            l->metadata_.taskMark_ = false;
    }

    PODVector<PODVector<SideDef*>> Sector::GetLoops() const
    {
        PODVector<PODVector<SideDef*>> ret;
        std::set<SideDef*> taken;

        while (taken.size() != sides_.Size())
        {
            PODVector<SideDef*> working;
            SideDef* cur = sides_[0];
            taken.insert(cur);
            working.Push(cur);
            SideDef* next = nullptr;

            do {
                static auto NextSide = [&]() -> SideDef* {
                    for (auto s : this->sides_)
                        if (s->line_->GetSharedVertex(cur->line_) && taken.find(s) == taken.end())
                            return s;
                    return nullptr;
                };
                next = NextSide();
                if (next != nullptr)
                {
                    taken.insert(next);
                    working.Push(next);
                    cur = next;
                }

            } while (next != nullptr);
            if (working.Size() > 0)
                ret.Push(working);
        }

        return ret;
    }

    UDMFMap::~UDMFMap()
    {
        for (auto v : vertices_)
            delete v;
        for (auto s : sides_)
            delete s;
        for (auto l : lines_)
            delete l;
        for (auto t : things_)
            delete t;
        for (auto s : sectors_)
            delete s;
    }

    void UDMFMap::Reindex()
    {
#define RIDX(LIST) for (unsigned i = 0; i < LIST.Size(); ++i) LIST[i]->index_ = i;

        RIDX(vertices_);
        RIDX(sides_);
        RIDX(lines_);
        RIDX(things_);
        RIDX(sectors_);
    }

    void UDMFMap::MergeIntoThis(UDMFMap* rhs)
    {
        PODVector<MapVertex*> newVerts = rhs->vertices_;
        PODVector<LineDef*> newLines = rhs->lines_;

        for (unsigned i = 0; i < rhs->vertices_.Size(); ++i)
        {
            static auto findVert = [](PODVector<MapVertex*>& target, MapVertex* v) {
                for (auto it = target.Begin(); it != target.End(); ++it)
                {
                    if ((*it)->position_ == v->position_)
                        return *it;
                }
                return v;
            };

            auto found = findVert(vertices_, rhs->vertices_[i]);
            if (found != rhs->vertices_[i])
            {
                delete rhs->vertices_[i];
                newVerts[i] = vertices_[i];
            }
            else
                vertices_.Push(found);
        }

        for (unsigned i = 0; i < rhs->lines_.Size(); ++i)
        {
            auto rhsLine = rhs->lines_[i];
            unsigned startIdx = rhs->vertices_.IndexOf(rhsLine->start_);
            unsigned endIdx = rhs->vertices_.IndexOf(rhsLine->end_);
            rhsLine->start_ = newVerts[startIdx];
            rhsLine->end_ = newVerts[endIdx];

            static auto findLine = [](PODVector<LineDef*>& target, LineDef* l) {
                for (auto it = target.Begin(); it != target.End(); ++it)
                    if ((*it)->IsSameAs(l))
                        return *it;
                return l;
            };

            auto found = findLine(lines_, rhsLine);
            if (found != rhsLine)
            {
                newLines[i] = found;
                delete rhsLine;
            }
            else
                lines_.Push(rhsLine);
        }
        
        for (auto s : rhs->sides_)
        {
            unsigned lineIdx = rhs->lines_.IndexOf(s->line_);
            if (s->line_ != newLines[lineIdx])
            {
                s->line_ = newLines[lineIdx];
                if (s->line_->front_ == nullptr)
                    s->line_->front_ = s;
                else if (s->line_->back_ == nullptr)
                    s->line_->back_ = s;
                else
                {
                    // this is bad ... this should not happen right?
                    delete s;
                    continue;
                }
            }
            sides_.Push(s);
        }

        for (auto s : rhs->sectors_)
        {
            for (unsigned v = 0; v < s->vertices_.Size(); ++v)
            {
                unsigned oldIdx = rhs->vertices_.IndexOf(s->vertices_[v]);
                rhs->vertices_[v] = newVerts[oldIdx];
            }

            for (unsigned l = 0; l < s->lines_.Size(); ++l)
            {
                unsigned oldIdx = rhs->lines_.IndexOf(s->lines_[l]);
                s->lines_[l] = newLines[oldIdx];
            }

            // todo: get the new sides? according to the above they can't actually change

            sectors_.Push(s);
        }

        rhs->vertices_.Clear();
        rhs->lines_.Clear();
        rhs->sectors_.Clear();
        rhs->sides_.Clear();

        Reindex();
    }

    bool UDMFMap::AlignSides(SideDef* thisMapsSide, UDMFMap* otherMap, SideDef* otherMapSide)
    {
        // can't align two sided lines
        if (thisMapsSide->line_->HasTwoFaces() && otherMapSide->line_->HasTwoFaces())
            return false;

        // sides need to be the same length to align
        if (thisMapsSide->line_->GetLineVec().Length() != otherMapSide->line_->GetLineVec().Length())
            return false;
        
        auto thisLine = thisMapsSide->line_;
        auto otherLine = otherMapSide->line_;

        auto thisLineNorm = -thisLine->faceNormal_;
        auto otherLineNorm = otherLine->faceNormal_;

        // rotate so the lines are aligned
        float angleBetween = thisLineNorm.Angle(otherLineNorm);
        otherMap->Transform(Vector2(), -angleBetween);
        
        // translate so the lines are on top of each-other
        auto otherLineCenter = otherLine->GetLineVec() * 0.5f;
        auto thisLineCenter = thisLine->GetLineVec() * 0.5f;
        auto delta = thisLineCenter - otherLineCenter;
        otherMap->Transform(delta, 0);
        
        otherLine->start_->position_ = thisLine->end_->position_;
        otherLine->end_->position_ = thisLine->start_->position_;

        // once aligned, MergeIntoThis() will take care of things
        return true;
    }

    void UDMFMap::PostLoad()
    {
        for (auto l : lines_)
        {
            if (l->back_ == nullptr)
                l->metadata_.impassable_ = true;
            else
                l->metadata_.impassable_ = l->GetBoolFlag("blockmonsters") || l->GetBoolFlag("1");
            l->metadata_.doubleSided_ = l->GetBoolFlag("twosided") || l->GetBoolFlag("4");
            l->metadata_.isHidden_ = l->GetBoolFlag("dontdraw") || l->GetBoolFlag("128");
            l->metadata_.isSecret_ = l->GetBoolFlag("secret") || l->GetBoolFlag("32");
            l->metadata_.dontPegTop_ = l->GetBoolFlag("dontpegtop") || l->GetBoolFlag("8");
            l->metadata_.dontPegBottom_ = l->GetBoolFlag("dontpegbottom") || l->GetBoolFlag("16");
        }

        Reindex();
        LinkVerticesToLines();
        CalculateGeometricData();
    }

    void UDMFMap::CalculateGeometricData()
    {
        CalculateCentroid();
        CalculateFaceNormals();
    }

    void UDMFMap::LinkVerticesToLines()
    {
        for (auto v : vertices_)
            v->lines_.Clear();
        for (auto l : lines_)
        {
            l->start_->lines_.Push(l);
            l->end_->lines_.Push(l);
        }
        bool needsReindex = false;
        for (auto v : vertices_)
        {
            if (v->lines_.Size() == 0)
            {
                vertices_.Remove(v);
                delete v;
                needsReindex = true;
            }
        }
        if (needsReindex)
            Reindex();
    }

    void UDMFMap::CalculateCentroid()
    {
        Vector2 c = Vector2(0, 0);
        for (int i = 0; i < vertices_.Size(); ++i)
            c += vertices_[i]->position_;
        if (vertices_.Size() > 0)
            centroid_ = c / vertices_.Size();
        else
            centroid_ = c;
    }

    void UDMFMap::CalculateFaceNormals()
    {
        for (auto l : lines_)
            l->faceNormal_ = Vector2(0, 0);
        for (auto s : sectors_)
            s->CalculateLineNormals();
        for (auto l : lines_)
            l->faceNormal_.Normalize();
    }

    void UDMFMap::Transform(Vector2 moveBy, float rotateBy)
    {
        for (int i = 0; i < vertices_.Size(); ++i)
        {
            if (rotateBy != 0.0f)
            {
                Vector2 centroidRel = vertices_[i]->position_ - centroid_;
                Vector2 centroidBackup = centroidRel;

                float sn = sin(rotateBy * M_DEGTORAD);
                float cs = cos(rotateBy * M_DEGTORAD);
                
                centroidRel.x_ = centroidBackup.x_ * cs - centroidBackup.y_ * sn;
                centroidRel.y_ = centroidBackup.x_ * sn + centroidBackup.y_ * cs;

                vertices_[i]->position_ = centroidRel + centroid_;
            }

            if (moveBy != Vector2::ZERO)
                vertices_[i]->position_ += moveBy;
        }

        for (int i = 0; i < things_.Size(); ++i)
        {
            if (rotateBy != 0.0f)
            {
                Vector2 centroidRel = vertices_[i]->position_ - centroid_;
                Vector2 centroidBackup = centroidRel;

                float sn = sin(rotateBy * M_DEGTORAD);
                float cs = cos(rotateBy * M_DEGTORAD);

                centroidRel.x_ = centroidBackup.x_ * cs - centroidBackup.y_ * sn;
                centroidRel.y_ = centroidBackup.x_ * sn + centroidBackup.y_ * cs;

                auto p = centroidRel + centroid_;
                things_[i]->position_ = Vector3(p.x_, p.y_, things_[i]->position_.z_);

                Vector2 v(1, 0);
                Vector2 r = Rotate2D(v, rotateBy);
                things_[i]->angle_ = r.Angle(v);
            }

            if (moveBy != Vector2::ZERO)
                things_[i]->position_ += Vector3(moveBy.x_, moveBy.y_, 0);
        }

        CalculateGeometricData();
    }

    void UDMFMap::Flip(bool vertical, bool horizontal)
    {
        Vector2 mulvec = Vector2(horizontal ? -1 : 1, vertical ? -1 : 1);
        for (int i = 0; i < vertices_.Size(); ++i)
        {
            Vector2 centroidRel = vertices_[i]->position_ - centroid_;
            centroidRel *= mulvec;
            vertices_[i]->position_ = centroid_ + centroidRel;
        }

        for (int i = 0; i < things_.Size(); ++i)
        {
            auto thing = things_[i];
            Vector2 centroidRel = Vector2(thing->position_.x_, thing->position_.y_) - centroid_;
            centroidRel *= mulvec;
            Vector2 adj = centroid_ + centroidRel;

            thing->position_ = Vector3(adj.x_, adj.y_, thing->position_.z_);
            Vector2 thingAng = Rotate2D(Vector2(1, 0), thing->angle_);
            thingAng *= mulvec;
            thing->angle_ = thingAng.Angle(Vector2(1, 0));
        }

        CalculateGeometricData();
    }

    void UDMFMap::RemoveSector(Sector* s, bool recalcData)
    {
        int i = sectors_.IndexOf(s);

        for (auto side : s->sides_)
        {
            if (side->line_->front_ == side)
                side->line_->front_ == nullptr;
            if (side->line_->back_ == side)
                side->line_->back_ == nullptr;
            if (side->line_->front_ == nullptr && side->line_->back_ == nullptr)
            {
                lines_.Remove(side->line_);
                delete side->line_;
            }
            sides_.Remove(side);
            delete side;
        }

        sectors_.Remove(s);
        delete s;
        if (recalcData)
        {
            Reindex();
            LinkVerticesToLines();
            CalculateGeometricData();
        }
    }

    void UDMFMap::RemoveSectors(const Urho3D::PODVector<Sector*>& s)
    {
        for (auto ss : s)
            RemoveSector(ss, false);
        Reindex();
        LinkVerticesToLines();
        CalculateGeometricData();
    }

    void UDMFMap::CenterMap()
    {
        Vector2 min(FLT_MAX,FLT_MAX), max(FLT_MIN, FLT_MIN);
        for (auto v : vertices_)
        {
            min.x_ = Min(min.x_, v->position_.x_);
            min.y_ = Min(min.y_, v->position_.y_);
            max.x_ = Max(max.x_, v->position_.x_);
            max.y_ = Max(max.y_, v->position_.y_);
        }
        auto center = (max - min) * 0.5f;
        for (auto v : vertices_)
            v->position_ -= center;
    }

    UDMFMap* UDMFMap::ExtractSectors(const Urho3D::PODVector<Sector*>& sectors)
    {
        if (sectors.Size() == 0)
            return nullptr;

        for (auto sector : sectors)
            sectors_.Remove(sector);
        
        UDMFMap* ret = new UDMFMap();
        for (auto sector : sectors)
        {
            ret->sectors_.Push(sector);
            for (auto l : sector->lines_)
            {
                ret->lines_.Push(l);
                lines_.Remove(l);
            }

            for (auto side : sides_)
            {
                ret->sides_.Push(side);
                sides_.Remove(side);
            }

            for (auto v : vertices_)
            {
                ret->vertices_.Push(v);
                vertices_.Remove(v);
            }
        }

        PostLoad();
        ret->PostLoad();
        
        return ret;
    }

    Lexer::Token Lexer::Peek()
    {
        unsigned i = index_;
        while (i < data_.Length())
        {
            if (data_[i] == ' ' || data_[i] == '\r' || data_[i] == '\n')
                ++i;
            else if (data_[i] == '/' && data_[i + 1] == '/')
            {
                i += 2;
                while (data_[i] != '\r')
                    ++i;
                if (data_[i] == '\n')
                    ++i;
            }
            else if (data_[i] == '{')
                return Open;
            else if (data_[i] == '}')
                return Close;
            else if (data_[i] == '=')
                return Assign;
            else if (data_[i] == ';')
                return Semi;
            else if (memcmp(data_.CString() + i, "true", 4) == 0)
                return Boolean;
            else if (memcmp(data_.CString() + i, "false", 5) == 0)
                return Boolean;
            else if (isalpha(data_[i]) || data_[i] == '"')
                return Text;
            else if (isalnum(data_[i]))
                return Number;
        }
        return End;
    }
    
    Lexer::Lexer(const String& data)
    {
        data_ = data;
        index_ = 0;
    }

    void Lexer::Advance()
    {
        unsigned i = index_;
        while (i < data_.Length())
        {
            if (data_[i] == ' ' || data_[i] == '\r' || data_[i] == '\n')
            {
                ++i;
                break;
            }
            else if (data_[i] == '=')
            {
                ++i;
                break;
            }
            else if (data_[i] == '/' && data_[i + 1] == '/')
            {
                i += 2;
                while (data_[i] != '\r')
                    ++i;
                if (data_[i] == '\n')
                    ++i;
                break;
            }
            else if (data_[i] == '{')
            {
                ++i;
                break;
            }
            else if (data_[i] == '}')
            {
                ++i;
                break;
            }
            else if (memcmp(data_.CString() + i, "true", 4) == 0)
            {
                i += 4;
                break;
            }
            else if (memcmp(data_.CString() + i, "false", 5) == 0)
            {
                i += 5;
                break;
            }
            else if (isalpha(data_[i]) || data_[i] == '"')
            {
                while ((isalnum(data_[i]) || ispunct(data_[i])) && (data_[i] != ';' && data_[i] != ' '))
                    ++i;
                break;
            }
            else if (isdigit(data_[i]))
            {
                while (isdigit(data_[i]) || data_[i] == '.')
                    ++i;
                break;
            }
        }
        index_ = i;
    }

    bool Lexer::GetBool()
    {
        while (data_[index_] == ' ')
            ++index_;
        if (memcmp(data_.CString() + index_, "true", 4) == 0)
            return true;
        return false;
    }

    float Lexer::GetFloat()
    {
        unsigned i = index_;
        String ret;
        while (i < data_.Length())
        {
            if (data_[i] == ' ')
            {
                ++i;
                continue;
            }
            if (isdigit(data_[i]) || data_[i] == '.')
                ret += data_[i];
            else
                break;
        }
        index_ = i;
        return ToFloat(ret);
    }

    Urho3D::String Lexer::GetWord()
    {
        unsigned i = index_;
        String ret;
        if (data_[i] == '"')
            ++i;
        while (i < data_.Length())
        {
            if (data_[i] == ' ')
            {
                ++i;
                continue;
            }
            if (isalnum(data_[i]) || ispunct(data_[i]) && data_[i] != ';' && data_[i] != '"')
                ret += data_[i];
            else
                break;
        }
        if (data_[i] == '"')
            ++i;
        index_ = i;
        return ret;
    }

    Urho3D::Vector4 UDMFMap::GetCoords() const
    {
        Vector2 min(FLT_MAX, FLT_MAX), max(FLT_MIN, FLT_MIN);
        for (auto v : vertices_)
        {
            min.x_ = Min(min.x_, v->position_.x_);
            min.y_ = Min(min.y_, v->position_.y_);
            max.x_ = Max(max.x_, v->position_.x_);
            max.y_ = Max(max.y_, v->position_.y_);
        }
        return Vector4(min.x_, min.y_, max.x_, max.y_);
    }
    void UDMFMap::Render(Image* image)
    {
        auto v = GetCoords();
        auto min = Vector2(v.x_, v.y_);
        auto r = Vector2(v.z_, v.w_) - Vector2(v.x_, v.y_);

        // resize to fit sensibly
        if (r.x_ > r.y_)
            image->SetSize(image->GetWidth(), image->GetHeight() * r.y_ / r.x_, 4);
        else if (r.y_ > r.x_)
            image->SetSize(image->GetWidth() * r.x_ / r.y_, image->GetHeight(), 4);
        image->Clear(Color::BLACK);

        for (auto l : lines_)
        {
            if (l->metadata_.isHidden_)
                continue;

            auto a = ((l->start_->position_ - min) / r);
            auto b = ((l->end_->position_ - min) / r);
            a.y_ = 1 - a.y_;
            b.y_ = 1 - b.y_;
            if (l->actionCode_ != 0)
                image->DrawLine(a, b, Color::YELLOW);
            else if (l->IsSolid())
                image->DrawLine(a, b, Color::RED);
            else if (l->metadata_.isSecret_)
                image->DrawLine(a, b, Color::MAGENTA);
            else if (l->front_->sector_->floorHeight_ != l->back_->sector_->floorHeight_ || 
                l->front_->sector_->ceilingHeight_ != l->back_->sector_->ceilingHeight_)
                image->DrawLine(a, b, Color::GRAY);
        }
    }
}