#include <Urho3D/Core/Context.h>
#include <Urho3D/ProcGeom/MarchingTriangles.h>
#include <Urho3D/ProcGeom/SDFImplicits.h>


namespace Urho3D
{

void Test_MarchingTriangles(Context* context)
{
    MarchingTriangles mt(4);

    mt.GenerateSurface([](const Vector3& pos) -> float {
        float a = SDFSphere(pos, Vector3(0, 0, 0), 20);
        float b = SDFSphere(pos, Vector3(10, 15, 0), 20);
        float c = SDFSphere(pos, Vector3(30, 30, 0), 20);
        float e = SDFSphere(pos, Vector3(25, 25, 0), 20);
        float d = SDFSphere(pos, Vector3(20, 20, 0), 20);
        float v = Min(a, Min(b, Min(c, Min(e, d))));
        return v;
    }, Vector3::ZERO, 7000);
    
    Geometry* extractGeo = mt.ExtractGeometry(ctx);
    outFile = SharedPtr<File>(new File(ctx, "Test_MarchingTriangles.obj", Urho3D::FILE_WRITE));
    WriteGeometryToOBJ({ extractGeo }, outFile, false, true);
    delete extractGeo;
}

}