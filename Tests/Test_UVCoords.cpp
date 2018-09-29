#include <Urho3D/IO/File.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/ProcGeom/TexCoords.h>

namespace Urho3D
{

void Test_DXUVAtlas(Context* ctx)
{
    auto mdl = ctx->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Cylinder.mdl");
    auto geo = mdl->GetGeometry(0, 0);
    auto newGeo = CalculateUV(geo, 512, 512, 2, 10, 0.25f, 4.0, 0);
    auto outFile = SharedPtr<File>(new File(ctx, "Test_DXUVAtlas.obj", Urho3D::FILE_WRITE));
    WriteGeometryToOBJ({ newGeo }, outFile, false, true, false);
    delete mdl;
}

void Test_PlanarUV(Context* ctx)
{
    auto mdl = ctx->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Cylinder.mdl");
    auto geo = mdl->GetGeometry(0, 0);
    auto newGeo = PlanarUV(geo, Urho3D::Plane(Vector3::FORWARD, Vector3::ZERO);
    auto outFile = SharedPtr<File>(new File(ctx, "Test_PlanarUV.obj", Urho3D::FILE_WRITE));
    WriteGeometryToOBJ({ newGeo }, outFile, false, true, false);
    delete mdl;
}

}
