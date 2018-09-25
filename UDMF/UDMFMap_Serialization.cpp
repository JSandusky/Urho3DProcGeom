#include <Urho3D/UDMF/UDMFMap.h>

#include <Urho3D/Core/StringUtils.h>
#include <Urho3D/IO/VectorBuffer.h>

#include <string>

using namespace Urho3D;

namespace UDMF
{

    void MapObject::Serialize(Urho3D::Serializer* stream, int version) const
    {
        stream->WriteInt(flags_.Size());
        for (auto flag : flags_)
        {
            stream->WriteStringHash(flag.first_);
            stream->WriteVariant(flag.second_);
        }
        stream->WriteInt(properties_.Size());
        for (auto flag : properties_)
        {
            stream->WriteStringHash(flag.first_);
            stream->WriteVariant(flag.second_);
        }
    }

    void MapObject::Deserialize(Urho3D::Deserializer* stream, int version)
    {
        int ct = stream->ReadInt();
        for (int i = 0; i < ct; ++i)
        {
            auto h = stream->ReadStringHash();
            auto v = stream->ReadVariant();
            flags_[h] = v;
        }
        ct = stream->ReadInt();
        for (int i = 0; i < ct; ++i)
        {
            auto h = stream->ReadStringHash();
            auto v = stream->ReadVariant();
            properties_[h] = v;
        }
    }

    void MapVertex::Serialize(Urho3D::Serializer* stream, int version) const
    {
        stream->WriteVector2(position_);
        MapObject::Serialize(stream, version);
    }

    void MapVertex::Deserialize(Urho3D::Deserializer* stream, int version)
    {
        position_ = stream->ReadVector2();
        MapObject::Deserialize(stream, version);
    }

    void SideDef::Serialize(Urho3D::Serializer* stream, int version) const
    {
        stream->WriteInt(line_->index_);
        stream->WriteInt(sector_->index_);
        stream->WriteString(upper_.textureName_);
        stream->WriteVector2(upper_.textureOffset_);
        stream->WriteString(middle_.textureName_);
        stream->WriteVector2(middle_.textureOffset_);
        stream->WriteString(lower_.textureName_);
        stream->WriteVector2(lower_.textureOffset_);
        stream->WriteBool(isFront_);
        MapObject::Serialize(stream, version);
    }

    void SideDef::Deserialize(Urho3D::Deserializer* stream, int version)
    {
        line_ = new LineDef();
        line_->index_ = stream->ReadInt();
        sector_ = new Sector();
        sector_->index_ = stream->ReadInt();

        upper_.textureName_ = stream->ReadString();
        upper_.textureOffset_ = stream->ReadVector2();
        middle_.textureName_ = stream->ReadString();
        middle_.textureOffset_ = stream->ReadVector2();
        lower_.textureName_ = stream->ReadString();
        lower_.textureOffset_ = stream->ReadVector2();

        isFront_ = stream->ReadBool();
        MapObject::Deserialize(stream, version);
    }

    void LineDef::Serialize(Urho3D::Serializer* stream, int version) const
    {
        stream->WriteBool(front_ != nullptr);
        stream->WriteInt(front_->index_);
        stream->WriteBool(back_ != nullptr);
        stream->WriteInt(back_->index_);
        stream->WriteInt(start_->index_);
        stream->WriteInt(end_->index_);
        stream->WriteInt(actionCode_);
        stream->WriteInt(actionParams_[0]);
        stream->WriteInt(actionParams_[1]);
        stream->WriteInt(actionParams_[2]);
        stream->WriteInt(actionParams_[3]);
        stream->WriteInt(actionParams_[4]);
        stream->WriteInt(tag_);
        MapObject::Serialize(stream, version);
    }
    void LineDef::Deserialize(Urho3D::Deserializer* stream, int version)
    {
        if (stream->ReadBool())
        {
            front_ = new SideDef();
            front_->index_ = stream->ReadInt();
        }
        if (stream->ReadBool())
        {
            back_ = new SideDef();
            back_->index_ = stream->ReadInt();
        }
        start_ = new MapVertex(); end_ = new MapVertex();
        start_->index_ = stream->ReadInt();
        end_->index_ = stream->ReadInt();
        actionCode_ = stream->ReadInt();
        actionParams_[0] = stream->ReadInt();
        actionParams_[1] = stream->ReadInt();
        actionParams_[2] = stream->ReadInt();
        actionParams_[3] = stream->ReadInt();
        actionParams_[4] = stream->ReadInt();
        tag_ = stream->ReadInt();
        MapObject::Deserialize(stream, version);
    }

    void MapThing::Serialize(Urho3D::Serializer* stream, int version) const
    {
        stream->WriteInt(thingTypeID_);
        stream->WriteVector3(position_);
        stream->WriteFloat(angle_);
        stream->WriteInt(tag_);
        stream->WriteInt(action_);
        MapObject::Serialize(stream, version);
    }
    void MapThing::Deserialize(Urho3D::Deserializer* stream, int version)
    {
        thingTypeID_ = stream->ReadInt();
        position_ = stream->ReadVector3();
        angle_ = stream->ReadFloat();
        tag_ = stream->ReadInt();
        action_ = stream->ReadInt();
        MapObject::Deserialize(stream, version);
    }

    void Sector::Serialize(Serializer* stream, int version) const
    {
        stream->WriteInt(floorHeight_);
        stream->WriteInt(ceilingHeight_);
        stream->WriteInt(brightness_);
        stream->WriteInt(tag_);
        stream->WriteInt(special_);
        stream->WriteString(floorTextureName_);
        stream->WriteString(ceilingTextureName_);
        stream->WriteBool(hasFloorSlope_);
        if (hasFloorSlope_)
        {
            stream->WriteFloat(floorSlope_.d_);
            stream->WriteVector3(floorSlope_.normal_);
        }
        stream->WriteBool(hasCeilingSlope_);
        if (hasCeilingSlope_)
        {
            stream->WriteFloat(ceilingSlope_.d_);
            stream->WriteVector3(ceilingSlope_.normal_);
        }
        stream->WriteInt(sides_.Size());
        for (auto s : sides_)
            stream->WriteInt(s->index_);
        stream->WriteInt(lines_.Size());
        for (auto s : lines_)
            stream->WriteInt(s->index_);
        stream->WriteInt(vertices_.Size());
        for (auto s : vertices_)
            stream->WriteInt(s->index_);
        MapObject::Serialize(stream, version);
    }
    void Sector::Deserialize(Deserializer* stream, int version)
    {
        floorHeight_ = stream->ReadInt();
        ceilingHeight_ = stream->ReadInt();
        brightness_ = stream->ReadInt();
        tag_ = stream->ReadInt();
        special_ = stream->ReadInt();
        floorTextureName_ = stream->ReadString();
        ceilingTextureName_ = stream->ReadString();
        hasFloorSlope_ = stream->ReadBool();
        if (hasFloorSlope_)
        {
            float d = stream->ReadFloat();
            auto v = stream->ReadVector3();
            floorSlope_ = Urho3D::Plane(v, d * v);
        }
        hasCeilingSlope_ = stream->ReadBool();
        if (hasCeilingSlope_)
        {
            float d = stream->ReadFloat();
            auto v = stream->ReadVector3();
            ceilingSlope_ = Urho3D::Plane(v, d * v);
        }

#define READSECTORDEP(TARGET, CTOR) { int ct = stream->ReadInt(); for (int i = 0; i < ct; ++i) { auto v = new CTOR(); v->index_ = stream->ReadInt(); TARGET.Push(v); } }

        READSECTORDEP(sides_, SideDef);
        READSECTORDEP(lines_, LineDef);
        READSECTORDEP(vertices_, MapVertex);

        MapObject::Deserialize(stream, version);
    }

    void UDMFMap::Read(Urho3D::XMLElement element)
    {
        auto result = element.Select("//v");
        for (int i = 0; i < result.Size(); ++i)
        {
            auto node = result[i];
            if (node.NotNull())
            {
                MapVertex* vert = new MapVertex();
                vert->index_ = ToInt(node.GetAttributeCString("id"));
                vert->position_.x_ = ToInt(node.GetAttributeCString("x"));
                vert->position_.y_ = ToInt(node.GetAttributeCString("y"));
                vert->ReadFlags(node);
                vert->ReadFields(node);
                vertices_.Push(vert);
            }
        }

        result = element.Select("//line");
        for (int i = 0; i < result.Size(); ++i)
        {
            auto node = result[i];
            if (node.NotNull())
            {
                LineDef* l = new LineDef();
                l->index_ = ToInt(node.GetAttribute("id"));
                int aVert = ToInt(node.GetAttribute("vert-start"));
                int bVert = ToInt(node.GetAttribute("vert-end"));
                l->start_ = vertices_[aVert];
                l->end_ = vertices_[bVert];
                l->tag_ = ToInt(node.GetAttribute("tag"));
                l->actionCode_ = ToInt(node.GetAttribute("action"));
                auto actionArgs = node.GetAttribute("action-args");
                auto args = actionArgs.Split(',');
                if (args.Size() > 0)
                {
                    for (int i = 0; i < args.Size(); ++i)
                        l->actionParams_[i] = ToInt(args[i]);
                }
                else
                    l->actionParams_[0] = l->actionParams_[1] = l->actionParams_[2] = l->actionParams_[3] = 0;

                l->ReadFlags(node);
                l->ReadFields(node);
                lines_.Push(l);
            }
        }

        result = element.Select("//side");
        for (int i = 0; i < result.Size(); ++i)
        {
            auto node = result[i];
            if (node.NotNull())
            {
                SideDef* s = new SideDef();
                s->index_ = ToInt(node.GetAttribute("id"));
                s->line_ = lines_[ToInt(node.GetAttribute("line"))];
                s->isFront_ = ToBool(node.GetAttribute("front"));
                if (s->isFront_)
                    s->line_->front_ = s;
                else
                    s->line_->back_ = s;

                s->upper_.textureName_ = node.GetAttribute("high-tex");
                s->middle_.textureName_ = node.GetAttribute("middle-tex");
                s->lower_.textureName_ = node.GetAttribute("low-tex");

                float xOfs = ToFloat(node.GetAttribute("offset-x"));
                float yOfs = ToFloat(node.GetAttribute("offset-y"));
                s->upper_.textureOffset_.x_ = s->middle_.textureOffset_.x_ = s->lower_.textureOffset_.x_ = xOfs;
                s->upper_.textureOffset_.y_ = s->middle_.textureOffset_.y_ = s->lower_.textureOffset_.y_ = yOfs;
                s->ReadFlags(node);
                s->ReadFields(node);

                sides_.Push(s);
            }
        }

        result = element.Select("//sector");
        for (int i = 0; i < result.Size(); ++i)
        {
            auto node = result[i];
            if (node.NotNull())
            {
                Sector* s = new Sector();
                s->index_ = sectors_.Size();
                s->floorHeight_ = ToInt(node.GetAttribute("floor-height"));
                s->ceilingHeight_ = ToInt(node.GetAttribute("ceiling-height"));
                s->floorTextureName_ = node.GetAttribute("floor-tex");
                s->ceilingTextureName_ = node.GetAttribute("ceiling-tex");
                s->brightness_ = ToInt(node.GetAttribute("lighting"));
                s->tag_ = ToInt(node.GetAttribute("tag"));
                s->special_ = ToInt(node.GetAttribute("special"));
                s->ReadFlags(node);
                s->ReadFields(node);

                XMLElement sidesElem = node.GetChild("sides");
                if (sidesElem.NotNull())
                {
                    auto sides = sidesElem.GetValue().Split(',');
                    for (auto sideID : sides)
                    {
                        SideDef* side = sides_[ToInt(sideID)];
                        side->sector_ = s;
                        s->sides_.Push(side);
                        if (!s->lines_.Contains(side->line_))
                            s->lines_.Push(side->line_);
                    }
                }

                XMLElement verticesElem = node.GetChild("vertices");
                if (verticesElem.NotNull())
                {
                    auto vertices = sidesElem.GetValue().Split(',');
                    for (auto vertID : vertices)
                        s->vertices_.Push(vertices_[ToInt(vertID)]);
                }

                sectors_.Push(s);
            }
        }

        PostLoad();
    }

    UDMFMap* UDMFMap::Clone() const
    {
        VectorBuffer buffer;
        Serialize(&buffer);

        buffer.Seek(0);
        UDMFMap* newMap = new UDMFMap();
        newMap->Deserialize(&buffer);
        return newMap;
    }

    void UDMFMap::Serialize(Urho3D::Serializer* stream) const
    {
        const int version = 1;

        stream->WriteFileID("UDMF");
        stream->WriteInt(version);
        stream->WriteUInt(vertices_.Size());
        for (auto v : vertices_)
            v->Serialize(stream, version);
        stream->WriteUInt(sides_.Size());
        for (auto s : sides_)
            s->Serialize(stream, version);
        stream->WriteUInt(lines_.Size());
        for (auto l : lines_)
            l->Serialize(stream, version);
        stream->WriteUInt(things_.Size());
        for (auto t : things_)
            t->Serialize(stream, version);
        stream->WriteUInt(sectors_.Size());
        for (auto s : sectors_)
            s->Serialize(stream, version);
    }

    void UDMFMap::Deserialize(Urho3D::Deserializer* stream)
    {
        if (stream->ReadFileID() != "UDMF")
            return;

        int version = stream->ReadInt();

        unsigned ct = 0;
#define DODESERIAL(TARGET, CTOR) ct = stream->ReadUInt(); { for (unsigned i = 0; i < ct; ++i) { auto c = new CTOR(); c->index_ = TARGET.Size(); c->Deserialize(stream, version); TARGET.Push(c); } }

        DODESERIAL(vertices_, MapVertex);
        DODESERIAL(sides_, SideDef);
        DODESERIAL(lines_, LineDef);
        DODESERIAL(things_, MapThing);
        DODESERIAL(sectors_, Sector);

        for (auto s : sides_)
        {
            auto l = s->line_;
            s->line_ = lines_[l->index_];
            delete l;

            auto ss = s->sector_;
            s->sector_ = sectors_[ss->index_];
            delete ss;
        }

        for (auto l : lines_)
        {
            auto ls = l->start_;
            auto le = l->end_;
            auto lf = l->front_;
            auto lb = l->back_;

            l->start_ = vertices_[ls->index_];
            l->end_ = vertices_[le->index_];

            if (lf != nullptr)
            {
                l->front_ = sides_[lf->index_];
                delete lf;
            }

            if (lb != nullptr)
            {
                l->back_ = sides_[lb->index_];
                delete lb;
            }

            delete ls;
            delete le;
        }

        for (auto s : sectors_)
        {
            for (int i = 0; i < s->lines_.Size(); ++i)
            {
                auto l = s->lines_[i];
                s->lines_[i] = lines_[l->index_];
                delete l;
            }
            for (int i = 0; i < s->vertices_.Size(); ++i)
            {
                auto l = s->vertices_[i];
                s->vertices_[i] = vertices_[l->index_];
                delete l;
            }
            for (int i = 0; i < s->sides_.Size(); ++i)
            {
                auto l = s->sides_[i];
                s->sides_[i] = sides_[l->index_];
                delete l;
            }
        }

        PostLoad();
    }
}