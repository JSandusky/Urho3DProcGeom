#include <Urho3D/ProcGeom/SurfaceGen.h>

void Test_Lathe(Context* ctx)
{
    if (auto pointList = ctx->GetSubsystem<ResourceCache>()->GetFile("PointList.txt"))
    {
        PODVector<Vector2> points;
        while (!pointList->IsEof())
            points.Push(FromString<Vector2>(pointList->ReadLine()));
        Geometry* geo = Lathe(ctx, points, 6, false);
        Model* model = new Model(ctx);
        model->SetNumGeometries(1);
        model->SetGeometry(0, 0, geo);

        auto outFile = SharedPtr<File>(new File(ctx, "Test_Lathe.obj", Urho3D::FILE_WRITE));
        WriteGeometryToOBJ({ geo }, outFile.Get(), false, false, false);
        
        delete model;
    }
}
