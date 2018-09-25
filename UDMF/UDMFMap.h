#pragma once

#include <Urho3D/IO/Deserializer.h>
#include <Urho3D/Container/HashMap.h>
#include <Urho3D/Math/Plane.h>
#include <Urho3D/IO/Serializer.h>
#include <Urho3D/Math/StringHash.h>
#include <Urho3D/Core/Variant.h>
#include <Urho3D/Container/Vector.h>
#include <Urho3D/Math/Vector2.h>
#include <Urho3D/Math/Vector3.h>
#include <Urho3D/Resource/XMLElement.h>

#include <set>

namespace Urho3D
{
    class XMLElement;
    class Image;
}

namespace UDMF
{
    class Lexer;
    class LineDef;
    struct SideDefPart;
    class SideDef;
    class Sector;
    class UDMFMap;

    class MapObject
    {
    protected:
        Urho3D::VariantMap properties_;
        Urho3D::VariantMap flags_;
    public:
        int index_ = -1;

        Urho3D::VariantMap& GetProperties() { return properties_; }
        Urho3D::VariantMap& GetFlags() { return flags_; }
        const Urho3D::VariantMap& GetProperties() const { return properties_; }
        const Urho3D::VariantMap& GetFlags() const { return flags_; }

        bool GetBoolFlag(const Urho3D::StringHash& key, bool defVal = false) const {
            if (flags_.Contains(key))
                return flags_[key]->GetBool();
            return defVal;
        }

        void ReadFlags(Urho3D::XMLElement);
        void ReadFields(Urho3D::XMLElement);

        virtual void Serialize(Urho3D::Serializer* stream, int version) const;
        virtual void Deserialize(Urho3D::Deserializer* stream, int version);
    };

    class MapVertex : public MapObject
    {
    public:
        Urho3D::Vector2 position_;
        Urho3D::PODVector<LineDef*> lines_;

        virtual void Serialize(Urho3D::Serializer* stream, int version) const override;
        virtual void Deserialize(Urho3D::Deserializer* stream, int version) override;
    };

    struct SideDefPart
    {
        Urho3D::Vector2 textureOffset_;
        Urho3D::String textureName_;
    };

    class SideDef : public MapObject
    {
    public:
        LineDef* line_;
        Sector* sector_;
        SideDefPart upper_;
        SideDefPart middle_;
        SideDefPart lower_;
        bool isFront_;

        SideDef* GetOpposite() const;
        static bool IsConnected(SideDef* lhs, SideDef* rhs);
        SideDef* GetNext(UDMFMap* map, std::set<SideDef*>& ignore);

        virtual void Serialize(Urho3D::Serializer* stream, int version) const override;
        virtual void Deserialize(Urho3D::Deserializer* stream, int version) override;
    };

    class LineDef : public MapObject
    {
    public:
        SideDef* front_ = nullptr;
        SideDef* back_ = nullptr;
        MapVertex* start_;
        MapVertex* end_;
        Urho3D::Vector2 faceNormal_;
        int actionCode_;
        int actionParams_[5];
        int tag_ = 0;

        struct Meta {
            bool impassable_;
            bool doubleSided_;
            bool isHidden_;
            bool isSecret_;
            bool dontPegTop_;
            bool dontPegBottom_;
            bool taskMark_;
        } metadata_;

        bool IsSolid() const { return metadata_.impassable_ || back_ == nullptr; }
        bool IsHidden() const { return metadata_.isHidden_; }
        bool IsSecret() const { return metadata_.isSecret_; }

        void NormalizeSides();
        LineDef* GetNextSolid() const;
        LineDef* GetPrevSolid() const;
        LineDef* GetNextSolid(const Sector* forSector) const;
        LineDef* GetPrevSolid(const Sector* forSector) const;
        MapVertex* GetSharedVertex(const LineDef* other) const;

        bool HasTwoFaces() const { return front_ != nullptr && back_ != nullptr; }
        bool IsSameAs(const LineDef* other) const;

        Urho3D::Vector2 GetLineVec() const { return end_->position_ - start_->position_; }

        virtual void Serialize(Urho3D::Serializer* stream, int version) const override;
        virtual void Deserialize(Urho3D::Deserializer* stream, int version) override;
    };

    class MapThing : public MapObject
    {
    public:
        int thingTypeID_;
        Urho3D::Vector3 position_;
        float angle_;
        int tag_;
        int action_;

        virtual void Serialize(Urho3D::Serializer* stream, int version) const override;
        virtual void Deserialize(Urho3D::Deserializer* stream, int version) override;
    };

    class Sector : public MapObject
    {
    public:
        int floorHeight_;
        int ceilingHeight_;
        int brightness_;
        int tag_;
        int special_;
        Urho3D::Plane floorSlope_;
        Urho3D::Plane ceilingSlope_;
        Urho3D::String ceilingTextureName_;
        Urho3D::String floorTextureName_;
        bool hasFloorSlope_ = false;
        bool hasCeilingSlope_ = false;

        Urho3D::PODVector<SideDef*> sides_;
        Urho3D::PODVector<LineDef*> lines_;
        Urho3D::PODVector<MapVertex*> vertices_;

        inline Urho3D::Plane GetFloorPlane() const { hasFloorSlope_ ? floorSlope_ : Urho3D::Plane(Urho3D::Vector3::UP, Urho3D::Vector3(0, floorHeight_, 0)); }
        inline Urho3D::Plane GetCeilingPlane() const { hasCeilingSlope_ ? ceilingSlope_ : Urho3D::Plane(Urho3D::Vector3::DOWN, Urho3D::Vector3(0, ceilingHeight_, 0)); }

        void CalculateLineNormals();
        void ClearLineMarks();
        Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetLoops() const;

        virtual void Serialize(Urho3D::Serializer* stream, int version) const override;
        virtual void Deserialize(Urho3D::Deserializer* stream, int version) override;
    };

    class UDMFMap
    {
    public:
        UDMFMap() { }
        ~UDMFMap();

        Urho3D::Vector2 centroid_;
        Urho3D::PODVector<MapVertex*> vertices_;
        Urho3D::PODVector<SideDef*> sides_;
        Urho3D::PODVector<LineDef*> lines_;
        Urho3D::PODVector<MapThing*> things_;
        Urho3D::PODVector<Sector*> sectors_;

        void PostLoad();
        void CalculateGeometricData();
        void LinkVerticesToLines();
        void CalculateCentroid();
        
        void Reindex();
        void MergeIntoThis(UDMFMap*);
        
        // Transforms
        bool AlignSides(SideDef* thisMapsSide, UDMFMap* otherMap, SideDef* otherMapSide);
        void Transform(Urho3D::Vector2 moveBy, float rotateBy);
        void FlipVertical() { Flip(true, false); }
        void FlipHorizontal() { Flip(false, true); }
        void Flip(bool vertical, bool horizontal);
        /// Removes the given sector, it's sides, and if necessary it's lines
        void RemoveSector(Sector* s, bool recalcData = false);
        void RemoveSectors(const Urho3D::PODVector<Sector*>& s);
        void CenterMap();
        // WARNING: does not account for shared data, intended for use with `tagged islands` of data
        UDMFMap* ExtractSectors(const Urho3D::PODVector<Sector*>& sectors);

        void CalculateFaceNormals();

        void Read(Urho3D::XMLElement element);

        void Serialize(Urho3D::Serializer* stream) const;
        void Deserialize(Urho3D::Deserializer* stream);
        UDMFMap* Clone() const;

        Urho3D::Vector4 GetCoords() const;
        void Render(Urho3D::Image* image);
    };

    class Lexer {
        Urho3D::String data_;
        unsigned index_ = 0;
    public:
        Lexer(const Urho3D::String& text);

        enum Token {
            Open,
            Close,
            Assign,
            Semi,
            Boolean,
            Number,
            Text,
            End
        };
        
        bool IsEOF() const { return index_ >= data_.Length(); }
        Token Peek();
        void Advance();
        Urho3D::String GetWord();
        bool GetBool();
        float GetFloat();
        void Reset() { index_ = 0; }
    };
}