#include <Urho3D/ProcGeom/SurfaceGen.h>

void Test_Lathe(Context* ctx)
{
    if (auto pointList = ctx->GetSubsystem<ResourceCache>()->GetFile("PointList.txt"))
    {
        PODVector<Vector2> points;
        while (!pointList->IsEof())
            points.Push(FromString<Vector2>(pointList->ReadLine()));
        
        Geometry* geo = Lathe(ctx, points, 6, false);
        auto outFile = SharedPtr<File>(new File(ctx, "Test_Lathe.obj", Urho3D::FILE_WRITE));
        WriteGeometryToOBJ({ geo }, outFile.Get(), false, false, false);
        
        delete geo;
    }
}

void Test_LoftSpine(Context* ctx)
{
    if (auto pointList = ctx->GetSubsystem<ResourceCache>()->GetFile("PointList.txt"))
    {
        PODVector<Vector2> points;
        while (!pointList->IsEof())
            points.Push(FromString<Vector2>(pointList->ReadLine()));
        
        Geometry* geo = LoftSpine(ctx, points, { Vector3(0,0,0), Vector3(5,0,0), Vector3(7,0,3) }, {}, {});
        auto outFile = SharedPtr<File>(new File(ctx, "Test_LoftSpine.obj", Urho3D::FILE_WRITE));
        WriteGeometryToOBJ({ geo }, outFile.Get(), false, false, false);
        
        delete geo;
    }
}

void Test_Loft(Context* ctx)
{
    if (auto pointList = ctx->GetSubsystem<ResourceCache>()->GetFile("PointList.txt"))
    {
        PODVector<Vector2> points;
        while (!pointList->IsEof())
            points.Push(FromString<Vector2>(pointList->ReadLine()));
        
        Matrix3x4 pointA = Matrix3x4(Vector3(0, 0, 0), Quaternion(), Vector3::ONE);
        Matrix3x4 pointB = Matrix3x4(Vector3(5, 0, 0), Quaternion(45, Vector3::UP), Vector3::ONE);
        Matrix3x4 pointC = Matrix3x4(Vector3(7, 0, 3), Quaternion(45, Vector3::RIGHT), Vector3::ONE);
        Matrix3x4 pointD = Matrix3x4(Vector3(7, 0, 8), Quaternion(45, Vector3::FORWARD), Vector3::ONE);
        auto geo = Loft(ctx, points, { pointA, pointB, pointC, pointD }, {});
        auto outFile = SharedPtr<File>(new File(ctx, "Test_Loft.obj", Urho3D::FILE_WRITE));
        WriteGeometryToOBJ({ geo }, outFile.Get(), false, false, false);
        
        delete geo;
    }
}
