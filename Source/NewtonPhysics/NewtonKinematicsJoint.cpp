#include "NewtonPhysicsWorld.h"
#include "NewtonKinematicsJoint.h"
#include "UrhoNewtonConversions.h"
#include "NewtonRigidBody.h"
#include "NewtonPhysicsEvents.h"
#include "NewtonConstraint.h"

#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/IO/Log.h"
#include "Urho3D/Core/Context.h"

#include "dCustomKinematicController.h"






namespace Urho3D {




    NewtonKinematicsControllerConstraint::NewtonKinematicsControllerConstraint(Context* context) : NewtonConstraint(context)
    {
        SubscribeToEvent(E_NEWTON_PHYSICSPOSTSTEP, URHO3D_HANDLER(NewtonKinematicsControllerConstraint, HandlePhysicsPreStep));
    }

    NewtonKinematicsControllerConstraint::~NewtonKinematicsControllerConstraint()
    {

    }

    void NewtonKinematicsControllerConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonKinematicsControllerConstraint>(DEF_PHYSICS_CATEGORY.c_str());

        URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);
    }

    void NewtonKinematicsControllerConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        NewtonConstraint::DrawDebugGeometry(debug, depthTest);
    }

    void NewtonKinematicsControllerConstraint::SetLinearFrictionalAcceleration(float friction)
    {
        if (linearFrictionalAcceleration != friction) {
            linearFrictionalAcceleration = friction;
            if(newtonJoint_)
                updateFrictions();
        }
    }

    void NewtonKinematicsControllerConstraint::SetAngularFrictionalAcceleration(float friction)
    {
        if (angularFrictionalAcceleration != friction) {
            angularFrictionalAcceleration = friction;
            if (newtonJoint_)
                updateFrictions();
        }
    }

    void NewtonKinematicsControllerConstraint::SetConstrainRotation(bool enable)
    {
        if (constrainRotation_ != enable)
        {
            constrainRotation_ = enable;
            if (newtonJoint_)
                static_cast<dCustomKinematicController*>(newtonJoint_)->SetPickMode(constrainRotation_);
        }
    }

    void NewtonKinematicsControllerConstraint::SetLimitRotationalVelocity(bool enable)
    {
        if (limitRotationalVelocity_ != enable)
        {
            limitRotationalVelocity_ = enable;
            if(newtonJoint_)
                static_cast<dCustomKinematicController*>(newtonJoint_)->SetLimitRotationVelocity(limitRotationalVelocity_);
        }
    }


    void NewtonKinematicsControllerConstraint::SetOtherPosition(const Vector3& position)
    {
		otherPosition_ = position;

    }

    void NewtonKinematicsControllerConstraint::SetOtherRotation(const Quaternion& rotation)
    {
		otherRotation_ = rotation;
    }

  

    void NewtonKinematicsControllerConstraint::buildConstraint()
    {
        newtonJoint_ = new dCustomKinematicController(GetOwnNewtonBody(), UrhoToNewton(GetOwnBuildWorldFrame()));
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetPickMode(constrainRotation_);//#todo support all pick modes
        updateFrictions();
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetLimitRotationVelocity(limitRotationalVelocity_);
    }

    void NewtonKinematicsControllerConstraint::updateTarget()
    {
        if (newtonJoint_) {
            static_cast<dCustomKinematicController*>(newtonJoint_)->SetTargetMatrix(UrhoToNewton(GetOtherWorldFrame()));
        }
    }

    void NewtonKinematicsControllerConstraint::updateFrictions()
    {

        dFloat Ixx;
        dFloat Iyy;
        dFloat Izz;
        dFloat mass;
        NewtonBodyGetMass(ownBody_->GetNewtonBody(), &mass, &Ixx, &Iyy, &Izz);


        const dFloat inertia = dMax(Izz, dMax(Ixx, Iyy));


        static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxLinearFriction(mass * linearFrictionalAcceleration);
        static_cast<dCustomKinematicController*>(newtonJoint_)->SetMaxAngularFriction(inertia * angularFrictionalAcceleration);
    }

    void NewtonKinematicsControllerConstraint::HandlePhysicsPreStep(StringHash event, VariantMap& eventData)
    {
        updateTarget();
    }

}
