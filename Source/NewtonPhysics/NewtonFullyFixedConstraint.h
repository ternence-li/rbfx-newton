#pragma once



#include "NewtonConstraint.h"


namespace Urho3D {
    class Context;
    class URHONEWTON_API NewtonFullyFixedConstraint : public NewtonConstraint
    {
        URHO3D_OBJECT(NewtonFullyFixedConstraint, NewtonConstraint);

    public:

        NewtonFullyFixedConstraint(Context* context);
        ~NewtonFullyFixedConstraint();

        static void RegisterObject(Context* context);


    protected:

        virtual void buildConstraint() override;
    };


}
