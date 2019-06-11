#include "NewtonPhysicsWorld.h"
#include "NewtonBallAndSocketConstraint.h"
#include "UrhoNewtonConversions.h"
#include "NewtonRigidBody.h"
#include "NewtonDebugDrawing.h"


#include "Urho3D/Core/Context.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Graphics/DebugRenderer.h"


#include "Newton.h"
#include "dMatrix.h"
#include "dCustomBallAndSocket.h"
#include "dCustomJoint.h"
#include "dCustomSixdof.h"



namespace Urho3D {

    NewtonBallAndSocketConstraint::NewtonBallAndSocketConstraint(Context* context) : NewtonConstraint(context)
    {

    }

    void NewtonBallAndSocketConstraint::SetConeAngle(float angle)
    {
        if (coneAngle_ != angle) {
            coneAngle_ = angle;
            if (newtonJoint_)
            {
                static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetConeLimits(coneAngle_ * dDegreeToRad);
            }
            else
                MarkDirty();
        }
    }

    float NewtonBallAndSocketConstraint::GetConeAngle() const
    {
        return coneAngle_;
    }

    void NewtonBallAndSocketConstraint::SetTwistLimits(float minAngle, float maxAngle)
    {
        if (twistMaxAngle_ != maxAngle || twistMinAngle_ != minAngle) {
            twistMinAngle_ = minAngle;
            twistMaxAngle_ = maxAngle;
            if (newtonJoint_)
            {
                static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetTwistLimits(twistMinAngle_* dDegreeToRad, twistMaxAngle_ * dDegreeToRad);
            }
            else
                MarkDirty();
        }
    }

    void NewtonBallAndSocketConstraint::SetTwistLimitMin(float minAngle)
    {
        if (twistMinAngle_ != minAngle) {
            twistMinAngle_ = minAngle;
            if (newtonJoint_)
            {
                static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetTwistLimits(twistMinAngle_* dDegreeToRad, twistMaxAngle_ * dDegreeToRad);
            }
            else
                MarkDirty();
        }
    }

    void NewtonBallAndSocketConstraint::SetTwistLimitMax(float maxAngle)
    {
        if (twistMaxAngle_ != maxAngle) {
            twistMaxAngle_ = maxAngle;
            if (newtonJoint_)
            {
                static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetTwistLimits(twistMinAngle_* dDegreeToRad, twistMaxAngle_ * dDegreeToRad);
            }
            else
                MarkDirty();
        }
    }

    float NewtonBallAndSocketConstraint::GetTwistLimitMin() const
    {
        return twistMinAngle_;
    }

    float NewtonBallAndSocketConstraint::GetTwistLimitMax() const
    {
        return twistMaxAngle_;
    }

    Urho3D::Vector2 NewtonBallAndSocketConstraint::GetTwistLimits() const
    {
        return Vector2(twistMinAngle_, twistMaxAngle_);
    }

    void NewtonBallAndSocketConstraint::SetConeEnabled(bool enabled /*= true*/)
    {
        if (coneEnabled_ != enabled) {
            coneEnabled_ = enabled;
            if (newtonJoint_)
            {
                static_cast<dCustomBallAndSocket*>(newtonJoint_)->EnableCone(coneEnabled_);
            }
            else
                MarkDirty();
        }
    }

    bool NewtonBallAndSocketConstraint::GetConeEnabled() const
    {
        return coneEnabled_;
    }

    void NewtonBallAndSocketConstraint::SetTwistLimitsEnabled(bool enabled /*= false*/)
    {
        if (twistLimitsEnabled_ != enabled) {
            twistLimitsEnabled_ = enabled;
            if (newtonJoint_)
            {
                static_cast<dCustomBallAndSocket*>(newtonJoint_)->EnableTwist(twistLimitsEnabled_);
            }
            else
                MarkDirty();
        }
    }

    bool NewtonBallAndSocketConstraint::GetTwistLimitsEnabled() const
    {
        return twistLimitsEnabled_;
    }

    void NewtonBallAndSocketConstraint::SetConeFriction(float frictionTorque)
    {
        if (frictionTorque != coneFriction_) {
            coneFriction_ = frictionTorque;
            if (newtonJoint_)
            {
                static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetConeFriction((coneFriction_));
            }
            else
                MarkDirty();
        }
    }

    float NewtonBallAndSocketConstraint::GetConeFriction() const
    {
        return coneFriction_;
    }

    void NewtonBallAndSocketConstraint::SetTwistFriction(float frictionTorque)
    {
        if (twistFriction_ != frictionTorque) {
            twistFriction_ = frictionTorque;
            if (newtonJoint_)
            {
                static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetTwistFriction((twistFriction_));
            }
            else
                MarkDirty();
        }
    }

    float NewtonBallAndSocketConstraint::GetTwistFriction() const
    {
        return twistFriction_;
    }

    void NewtonBallAndSocketConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        NewtonConstraint::DrawDebugGeometry(debug, depthTest);
    }



    Urho3D::NewtonBallAndSocketConstraint::~NewtonBallAndSocketConstraint()
    {

    }

    void Urho3D::NewtonBallAndSocketConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonBallAndSocketConstraint>(DEF_PHYSICS_CATEGORY.c_str());


        URHO3D_COPY_BASE_ATTRIBUTES(NewtonConstraint);

        URHO3D_ACCESSOR_ATTRIBUTE("Cone Enabled", GetConeEnabled, SetConeEnabled, bool, true, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Cone Angle", GetConeAngle, SetConeAngle, float, 20.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Cone Friction", GetConeFriction, SetConeFriction, float, 0.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Twist Limits Enabled", GetTwistLimitsEnabled, SetTwistLimitsEnabled, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Twist Angle Min", GetTwistLimitMin, SetTwistLimitMin, float, -45.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Twist Angle Max", GetTwistLimitMax, SetTwistLimitMax, float, 45.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Twist Friction", GetTwistFriction, SetTwistFriction, float, 0.0f, AM_DEFAULT);

    }

    void Urho3D::NewtonBallAndSocketConstraint::buildConstraint()
    {
        // Create a dCustomBallAndSocket
        newtonJoint_ = new dCustomBallAndSocket(UrhoToNewton(GetOwnBuildWorldFrame()), UrhoToNewton(GetOtherBuildWorldFrame()), GetOwnNewtonBody(), GetOtherNewtonBody());

        
    }
    bool NewtonBallAndSocketConstraint::applyAllJointParams()
    {
        if (!NewtonConstraint::applyAllJointParams())
            return false;

        static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetConeLimits(coneAngle_ * dDegreeToRad);
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->EnableCone(coneEnabled_);
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->EnableTwist(twistLimitsEnabled_);
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetTwistLimits(twistMinAngle_* dDegreeToRad, twistMaxAngle_ * dDegreeToRad);
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetConeFriction((coneFriction_));
        static_cast<dCustomBallAndSocket*>(newtonJoint_)->SetTwistFriction((twistFriction_));


        return true;
    }

}
