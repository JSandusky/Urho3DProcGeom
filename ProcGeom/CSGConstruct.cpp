#include <Urho3D/ProcGeom/CSGConstruct.h>
#include <Urho3D/Graphics/Geometry.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Graphics/Model.h>

namespace Urho3D
{

    Model* CSGModel(Context* ctx, Model* lhs, Model* rhs, CSGTask operation)
    {
        Model* ret = nullptr;
        if (lhs == nullptr || rhs == nullptr || lhs->GetNumGeometries() == 0)
        {
            URHO3D_LOGERROR("Attempted to perform CSG without valid inputs");
            return ret;
        }

        if (lhs->GetNumGeometries() > 1)
            URHO3D_LOGWARNING("CSG can only use an LHS model with a single geometry");

        PODVector<CSGOperand> operands;
        auto targetGeom = lhs->GetGeometry(0, 0);
        operands.Push(CSGOperand{ targetGeom, CSG_Add });
        for (unsigned i = 0; i < rhs->GetNumGeometries(); ++i)
            operands.Push(CSGOperand{ rhs->GetGeometry(i, 0), operation });
        if (operands.Size() == 1)
        {
            URHO3D_LOGDEBUG("No RHS geometry to perform CSG with");
            return ret;
        }

        if (auto newGeometry = PerformCSG(ctx, operands, true))
        {
            ret = new Model(ctx);
            ret->SetNumGeometries(1);
            ret->SetGeometry(0, 0, newGeometry);
        }
        return ret;
    }

}