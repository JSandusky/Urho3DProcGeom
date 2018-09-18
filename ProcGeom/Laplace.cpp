#include <Urho3D/ProcGeom/Laplace.h>
#include <Urho3D/IO/Log.h>

#include <ThirdParty/igl/cotmatrix.h>
#include <ThirdParty/igl/massmatrix.h>
#include <ThirdParty/igl/mat_min.h>
#include <ThirdParty/igl/normalize_row_sums.h>
//#include <ThirdParty/igl/remove_duplicate_vertices.h>

namespace Urho3D
{

    extern void FillIGLData(Geometry* geo, Eigen::MatrixXf& positions, Eigen::MatrixXi& indices);
    extern Geometry* FromIGLData(Context* ctx, Eigen::MatrixXf& positions, Eigen::MatrixXi& indices);

    bool CalculateBoneWeights(Geometry* forGeom, Skeleton* againstSkeleton, PODVector<Vector4>& weights, PODVector<int>& boneIdx)
    {
        return true;
    }

    Geometry* LaplaceDeform(Geometry* forGeom, const PODVector<LaplaceHandle>& handles)
    {
#if 0
        Eigen::MatrixXf positions;
        Eigen::MatrixXi indices;

        FillIGLData(forGeom, positions, indices);

        Eigen::MatrixXf canonPositions;
        Eigen::MatrixXi canonIndices;
        Eigen::MatrixXi canonTable;
        igl::remove_duplicate_vertices(positions, M_EPSILON, canonPositions, canonIndices, canonTable);

        Eigen::SparseMatrix<float> cotMat, massMat;
        igl::cotmatrix(canonPositions, canonIndices, cotMat);
        igl::massmatrix(canonPositions, canonIndices, igl::MASSMATRIX_TYPE_DEFAULT, massMat);

        Eigen::VectorXf H;
        H.resize(canonPositions.rows());
        H.setZero();
        Eigen::SparseMatrix<float> P;
#endif
        return nullptr;
    }

}