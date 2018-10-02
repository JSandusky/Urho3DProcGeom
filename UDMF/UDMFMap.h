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

    /// Baseclass for items of a UDMF map, deals with properties and flags.
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
        /// Map-space position (XY)
        Urho3D::Vector2 position_;
        /// List of all lines that use this vertex.
        Urho3D::PODVector<LineDef*> lines_;

        virtual void Serialize(Urho3D::Serializer* stream, int version) const override;
        virtual void Deserialize(Urho3D::Deserializer* stream, int version) override;
    };

    /// The half-edge `side-defs` are divided into upper, middle, and lower portions that are used
    /// based on the heights of the sectors on both sides of a two sided line.
    struct SideDefPart
    {
        Urho3D::Vector2 textureOffset_;
        Urho3D::String textureName_;
    };

    /// One half-edge of a line.
    class SideDef : public MapObject
    {
    public:
/*
--------|
        | <-- Upper
        |
--------  <-- ceiling height discontinuity
        |
        | <-- Middle
        | 
--------   <-- floor height discontinuity
        |
        | <-- Lower
        |
*/
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
            /// Special tag for iteration so that this line will not be revisisted.
            bool taskMark_;
        } metadata_;

        bool IsSolid() const { return metadata_.impassable_ || back_ == nullptr; }
        bool IsHidden() const { return metadata_.isHidden_; }
        bool IsSecret() const { return metadata_.isSecret_; }

        /// Ensures that if this line has become single-sided that it's only side becomes the front-one.
        void NormalizeSides();
        /// Get the next `solid` line that is connected to this line in clockwise order.
        LineDef* GetNextSolid() const;
        /// Get the previous `solid` line that is connected to this line in clockwise order.
        LineDef* GetPrevSolid() const;
        /// Get the next solid that is referenced by the given sector (non-manifold and sector tracing).
        LineDef* GetNextSolid(const Sector* forSector) const;
        /// Get the previous solid that is referenced by the given sector (non-manifold and sector tracing).
        LineDef* GetPrevSolid(const Sector* forSector) const;
        /// Return the vertex (or null) that this line and the other line share.
        MapVertex* GetSharedVertex(const LineDef* other) const;

        /// Returns true if this line has two sides.
        inline bool HasTwoFaces() const { return front_ != nullptr && back_ != nullptr; }
        /// Returns true if the two lines are equivalent.
        bool IsSameAs(const LineDef* other) const;

        /// Returns the direction-vector of the line.
        Urho3D::Vector2 GetLineVec() const { return end_->position_ - start_->position_; }
        /// Returns the centerpoint on the line.
        Urho3D::Vector2 GetLineCenter() const { return start_->position_ + GetLineVec() * 0.5f; }

        virtual void Serialize(Urho3D::Serializer* stream, int version) const override;
        virtual void Deserialize(Urho3D::Deserializer* stream, int version) override;
    };

    /// A `Thing` is a `point entity` placed in the map.
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

    /// A sector is a polygonal region of a map formed from vertices and sides (which reference lines).
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

        /// Return the floor-plane of this sector (for 3d floors).
        inline Urho3D::Plane GetFloorPlane() const { hasFloorSlope_ ? floorSlope_ : Urho3D::Plane(Urho3D::Vector3::UP, Urho3D::Vector3(0, floorHeight_, 0)); }
        /// Return the ceiling-plane of this sector (for 3d ceilings).
        inline Urho3D::Plane GetCeilingPlane() const { hasCeilingSlope_ ? ceilingSlope_ : Urho3D::Plane(Urho3D::Vector3::DOWN, Urho3D::Vector3(0, ceilingHeight_, 0)); }

        /// Calculates the line normals for lines this sector references.
        void CalculateLineNormals();
        /// Removes the special marker bit (used in iterations) from lines this sector references.
        void ClearLineMarks();
        /// Returns the collection of continuous loops of sides for this sector. First loop is the outer loop, the others are interior loops.
        Urho3D::PODVector<Urho3D::PODVector<SideDef*>> GetLoops() const;

        virtual void Serialize(Urho3D::Serializer* stream, int version) const override;
        virtual void Deserialize(Urho3D::Deserializer* stream, int version) override;
    };

    /// The Map contains a collection of vertices, sides, lines, things, and sectors. Maps can be transformed, filtered, and trimmed.
    class UDMFMap
    {
    public:
        /// Construct.
        UDMFMap() { }
        /// Destruct.
        ~UDMFMap();

        Urho3D::Vector2 centroid_;
        Urho3D::PODVector<MapVertex*> vertices_;
        Urho3D::PODVector<SideDef*> sides_;
        Urho3D::PODVector<LineDef*> lines_;
        Urho3D::PODVector<MapThing*> things_;
        Urho3D::PODVector<Sector*> sectors_;

        /// Performs operations necessary when a map has been loaded, merged, or had sectors removed.
        void PostLoad();
        /// Helper for updating centroid and face normals in one call.
        void CalculateGeometricData();
        /// Clears and fills the lists of lines connected to each vertex.
        void LinkVerticesToLines();
        /// Determines where the center of the map is (based on vertices).
        void CalculateCentroid();
        
        /// Redetermines the indices of the contents of the map.
        void Reindex();
        /// Merges the given map into this one, coalescing vertices and lines as necessary.
        void MergeIntoThis(UDMFMap*);
        
        /// Transforms
        /// Aligns another map to this one so that the two sides are in alignment (assumed to both be single-sided lines).
        bool AlignSides(SideDef* thisMapsSide, UDMFMap* otherMap, SideDef* otherMapSide, bool force);
        /// Translates and/or rotates the map.
        void Transform(Urho3D::Vector2 moveBy, float rotateBy);
        /// Scales the map (XY only)
        void Scale(float fraction) { Scale(Vector2(fraction, fraction)); }
        /// Scales the map (XY only)
        void Scale(const Urho3D::Vector2& scaleBy);
        /// Flips the map vertically.
        void FlipVertical() { Flip(true, false); }
        /// Flips the map horizontally.
        void FlipHorizontal() { Flip(false, true); }
        /// Flips the map vertically, horizontally, or both.
        void Flip(bool vertical, bool horizontal);
        /// Removes the given sector, it's sides, and if necessary it's lines
        void RemoveSector(Sector* s, bool recalcData = false);
        /// Removes the list of sectors.
        void RemoveSectors(const Urho3D::PODVector<Sector*>& s);
        /// Centers the map around 0,0
        void CenterMap();
        // WARNING: does not account for shared data, intended for use with `tagged islands` of data
        UDMFMap* ExtractSectors(const Urho3D::PODVector<Sector*>& sectors);

        /// Determines the front-side normals for each line.
        void CalculateFaceNormals();

        /// Reads an XML UDMF dump.
        void Read(Urho3D::XMLElement element);
        /// Writes map to a binary format (smaller than XML).
        void Serialize(Urho3D::Serializer* stream) const;
        /// Reads map from the above binary format.
        void Deserialize(Urho3D::Deserializer* stream);
        /// Clones this map into a new instance.
        UDMFMap* Clone() const;

        /// Gets the dimensions in XY = min, ZW = max.
        Urho3D::Vector4 GetCoords() const;
        /// Renders the map to an image (the image will be resized), the initial dimensions are a guide for the *longest* side.
        void Render(Urho3D::Image* image);
    };

    /// Token lexer for reading true UDMF data.
    /// Usage:
    ///     while (!lexer->IsEOF())
    ///     {
    ///         switch (lexer->Peek())
    ///         {
    ///         case Open:
    ///             break;
    ///         case Number:
    ///             myValue = lexer->GetFloat();
    ///             break;
    ///         ... etc ...
    ///         }
    ///         lexer->Advance();
    ///     }
    class Lexer {
        Urho3D::String data_;
        unsigned index_ = 0;
    public:
        /// Construct from a string.
        Lexer(const Urho3D::String& text);

        /// This lexer only understands a limited set of types.
        enum Token {
            Open,       // {
            Close,      // }
            Assign,     // =
            Semi,       // ;
            Boolean,    // True|False
            Number,     // 50|5.0
            Text,       // "My name is Roger"
            End         // EOF
        };
        
        /// Returns true if the lexer is finished.
        bool IsEOF() const { return index_ >= data_.Length(); }
        /// Returns the expected next token.
        Token Peek();
        /// Steps past the current token.
        void Advance();
        /// Gets the next token as a string.
        Urho3D::String GetWord();
        /// Gets the next boolean token.
        bool GetBool();
        /// Gets the next float token.
        float GetFloat();
        /// Returns the lexer index to the start of data.
        void Reset() { index_ = 0; }

    private:
        /// Index at which the last peek was done, enables reusing equivalent results.
        unsigned lastPeekIndex_ = -1;
        /// Token result from the last peek, used with lastPeekIndex_ for caching peek results.
        Token lastPeekToken_;
    };
}