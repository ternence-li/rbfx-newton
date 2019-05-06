#pragma once

#include "UrhoNewtonApi.h"

#include "Urho3D/Resource/Resource.h"
#include "Urho3D/Core/Context.h"
#include "Urho3D/Core/Object.h"

namespace Urho3D {

    ///resource providing physics model data 
    class URHONEWTON_API PhysicsModel : public ResourceWithMetadata
    {
        URHO3D_OBJECT(PhysicsModel, ResourceWithMetadata);

    public:
        PhysicsModel(Context* context) : ResourceWithMetadata(context)
        {

        }
        virtual ~PhysicsModel() {}

        static void RegisterObject(Context* context) {
            context->RegisterFactory< PhysicsModel>();
        }

    protected:

        //friction, restitution, etc..
    };

}
