#pragma once
#include "NewtonConstraint.h"


namespace Urho3D {
    class Context;
    class URHONEWTON_API NewtonFixedDistanceConstraint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonFixedDistanceConstraint, NewtonConstraint);

    public:

        NewtonFixedDistanceConstraint(Context* context);
        ~NewtonFixedDistanceConstraint();

        static void RegisterObject(Context* context);


    protected:

        virtual void buildConstraint() override;
    };


}
