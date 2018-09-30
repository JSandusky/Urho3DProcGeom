namespace Urho3D
{

void Test_LODGen(Context* ctx)
{
    // NOTE: this test does not actually function in total
    //      - it will first generate LOD for model #1
    //      - then it will fail because the next one is non-manifold
    //      - this is considered OK for this test (as long as when the success case is checked it's a semi-valid model)
    auto jack = ctx->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Jack.mdl");
    auto newJackMdl = GenerateModelLOD(jack, { 0.5f, 0.25f }, false);
    auto outFile = SharedPtr<File>(new File(ctx, "Test_ModelLOD.obj", Urho3D::FILE_WRITE));
    WriteGeometryToOBJ({ newJackMdl->GetGeometry(0, 2) }, outFile, true, true);
    delete jack;
}

}