#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/UDMF/UDMFMap.h>

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

}