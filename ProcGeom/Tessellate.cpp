#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/Graphics/IndexBuffer.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/ProcGeom/Tessellate.h>
#include <Urho3D/Graphics/VertexBuffer.h>

#include <ThirdParty/igl/upsample.h>
#include <ThirdParty/igl/loop.h>

namespace Urho3D
{

    void FillIGLData(Geometry* geo, Eigen::MatrixXf& positions, Eigen::MatrixXi& indices)
    {
        if (geo->GetPrimitiveType() != TRIANGLE_LIST)
        {
            URHO3D_LOGERROR("IGL data can only be filled from triangle lists");
            return;
        }

        const unsigned char* vertexData;
        const unsigned char* indexData;
        unsigned vertexSize;
        unsigned indexSize;
        const PODVector<VertexElement>* elements;
        geo->GetRawData(vertexData, vertexSize, indexData, indexSize, elements);
        const bool largeIndices = indexSize == sizeof(unsigned);

        indices.resize(geo->GetIndexCount() / 3, 3);
        for (int i = 0; i < geo->GetIndexCount(); i += 3)
        {
            unsigned idxValues[] = {
                largeIndices ? *(((unsigned*)indexData) + i) : *(((unsigned short*)indexData) + i),
                largeIndices ? *(((unsigned*)indexData) + i) : *(((unsigned short*)indexData) + i),
                largeIndices ? *(((unsigned*)indexData) + i) : *(((unsigned short*)indexData) + i)
            };

            indices.coeffRef(i / 3, 0) = idxValues[0];
            indices.coeffRef(i / 3, 1) = idxValues[0];
            indices.coeffRef(i / 3, 2) = idxValues[0];
        }

        positions.resize(geo->GetVertexCount(), 3);
        const auto posOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);
        for (int i = 0; i < geo->GetVertexCount(); ++i)
        {
            auto pos = *((Vector3*)(vertexData + i * vertexSize + posOffset));
            positions.coeffRef(i, 0) = pos.x_;
            positions.coeffRef(i, 1) = pos.y_;
            positions.coeffRef(i, 2) = pos.z_;
        }
    }

    Geometry* FromIGLData(Context* ctx, Eigen::MatrixXf& positions, Eigen::MatrixXi& indices)
    {
        Geometry* ret = new Geometry(ctx);

        VertexBuffer* vtxBuffer = new VertexBuffer(ctx);
        vtxBuffer->SetSize(positions.rows(), ELEMENT_POSITION);

        PODVector<Vector3> points;
        for (unsigned i = 0; i < positions.rows(); ++i)
            points.Push(Vector3(
                positions.coeff(i, 0),
                positions.coeff(i, 1),
                positions.coeff(i, 2)));
        vtxBuffer->SetData(points.Buffer());
        
        IndexBuffer* idxBuffer = new IndexBuffer(ctx);
        idxBuffer->SetSize(indices.rows() * 3, true);
        PODVector<unsigned> idx;
        for (unsigned i = 0; i < indices.rows(); ++i)
        {
            idx.Push(indices.coeff(i, 0));
            idx.Push(indices.coeff(i, 1));
            idx.Push(indices.coeff(i, 2));
        }
        idxBuffer->SetData(idx.Buffer());

        ret->SetNumVertexBuffers(1);
        ret->SetVertexBuffer(0, vtxBuffer);
        ret->SetIndexBuffer(idxBuffer);
        ret->SetDrawRange(TRIANGLE_LIST, 0, idxBuffer->GetIndexCount());

        return ret;
    }

    Geometry* Tessellate(Geometry* geo, int levels, bool smooth)
    {

        Eigen::MatrixXf positions;
        Eigen::MatrixXi indices;

        FillIGLData(geo, positions, indices);

        Eigen::MatrixXf newPositions;
        Eigen::MatrixXi newIndices;

        if (smooth)
            igl::loop(positions, indices, newPositions, newIndices, levels);
        else
            igl::upsample(positions, indices, newPositions, newIndices, levels);

        return FromIGLData(geo->GetContext(), newPositions, newIndices);
    }

}