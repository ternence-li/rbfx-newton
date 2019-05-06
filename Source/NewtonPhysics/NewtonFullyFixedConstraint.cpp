#include "NewtonFullyFixedConstraint.h"
#include "NewtonPhysicsWorld.h"
#include "UrhoNewtonConversions.h"
#include "NewtonRigidBody.h"


#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Core/Context.h"

#include "Newton.h"
#include "dMatrix.h"
#include "dCustomFixDistance.h"



namespace Urho3D {


    NewtonFullyFixedConstraint::NewtonFullyFixedConstraint(Context* context) : NewtonConstraint(context)
    {

    }

    NewtonFullyFixedConstraint::~NewtonFullyFixedConstraint()
    {

    }

    void NewtonFullyFixedConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonFullyFixedConstraint>(DEF_PHYSICS_CATEGORY.c_str());

        URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);
    }

    void NewtonFullyFixedConstraint::buildConstraint()
    {

        Matrix3x4 ownFrame = GetOwnNewtonBuildWorldFrame();
        Matrix3x4 otherFrame = GetOtherNewtonBuildWorldFrame();

        //GetSubsystem<VisualDebugger>()->AddFrame(ownFrame, 1.0f, false)->SetLifeTimeMs(100000);
        //GetSubsystem<VisualDebugger>()->AddFrame(otherFrame, 1.0f, false)->SetLifeTimeMs(100000);

        newtonJoint_ = new dCustom6dof(UrhoToNewton(ownFrame), UrhoToNewton(otherFrame), GetOwnNewtonBody(), GetOtherNewtonBody());

    }

}
