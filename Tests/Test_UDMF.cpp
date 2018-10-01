#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/UDMF/UDMFMap.h>
#include <Urho3D/UDMF/UDMFProcessing.h>

namespace Urho3D
{

void Test_UDMFLoadRender(Context* ctx)
{
    auto udmfMapFile = ctx->GetSubsystem<ResourceCache>()->GetResource<XMLFile>("Maps/Doom2_MAP01.xml");
    UDMF::UDMFMap* map = new UDMF::UDMFMap();
    map->Read(udmfMapFile->GetRoot());
    Image* img = new Image(ctx);
    img->SetSize(512, 512, 4);
    map->Render(img);
    img->SavePNG("Test_UDMFLoadRender.png");
    delete img;
    delete map;
}

void Test_UDMFSkirtSectors(Context* ctx)
{
    // it's assumed that this is a dump of Doom2 Map01 (Entry)
    auto udmfMapFile = ctx->GetSubsystem<ResourceCache>()->GetResource<XMLFile>("Maps/Doom2_MAP01.xml");
    UDMF::UDMFMap* map = new UDMF::UDMFMap();
    map->Read(udmfMapFile->GetRoot());
    
    PODVector<Vector3> pts; 
    PODVector<unsigned> indices;
    // Skirt the sector with wonky cutout towards the end of the map.    
    UDMF::MapProcessing::SkirtSector(map->sectors_[18], 20.0f, pts, indices, false);

    Geometry* g = new Geometry(ctx);
    VertexBuffer* v = new VertexBuffer(ctx);
    v->SetShadowed(true);
    v->SetSize(pts.Size(), { VertexElement(TYPE_VECTOR3, SEM_POSITION) });
    v->SetData(pts.Buffer());
    IndexBuffer* i = new IndexBuffer(ctx);
    i->SetShadowed(true);
    i->SetSize(indices.Size(), true);
    i->SetData(indices.Buffer());
    g->SetNumVertexBuffers(1);
    g->SetVertexBuffer(0, v);
    g->SetIndexBuffer(i);
    g->SetDrawRange(TRIANGLE_LIST, 0, indices.Size());
    auto outFile = SharedPtr<File>(new File(ctx, "Test_SkirtSector.obj", Urho3D::FILE_WRITE));
    WriteGeometryToOBJ({ g }, outFile, false, true);
    delete g;
}

}