//
// Copyright (c) 2008-2019 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
#include "Urho3D/Core/Context.h"
#include "Urho3D/Core/Mutex.h"
#include "Urho3D/Core/Profiler.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/Core/Context.h"
#include "Urho3D/Core/CoreEvents.h"
#include "Urho3D/Core/Object.h"
#include "Urho3D/Scene/Component.h"
#include "Urho3D/Scene/Scene.h"
#include "Urho3D/Scene/Node.h"
#include "Urho3D/Math/Sphere.h"
#include "Urho3D/Core/Thread.h"
#include "Urho3D/Engine/Engine.h"
#include "Urho3D/Math/Ray.h"
#include "Urho3D/Scene/SceneEvents.h"
#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Core/Profiler.h"
#include "Urho3D/IO/Log.h"


#include "UrhoNewtonApi.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonCollisionShape.h"
#include "NewtonCollisionShapesDerived.h"
#include "NewtonRigidBody.h"
#include "UrhoNewtonConversions.h"
#include "NewtonConstraint.h"
#include "NewtonFixedDistanceConstraint.h"
#include "NewtonPhysicsEvents.h"
#include "NewtonBallAndSocketConstraint.h"
#include "NewtonKinematicsJoint.h"
#include "NewtonDebugDrawing.h"
#include "NewtonFullyFixedConstraint.h"
#include "NewtonHingeConstraint.h"
#include "NewtonSliderConstraint.h"
#include "Newton6DOFConstraint.h"




#include "Newton.h"
#include "NewtonMeshObject.h"
#include "dgMatrix.h"
#include "dCustomJoint.h"
#include "dMatrix.h"

#include "EASTL/sort.h"


namespace Urho3D {


    static const Vector3 DEFAULT_GRAVITY = Vector3(0.0f, -9.81f, 0.0f);

    NewtonPhysicsWorld::NewtonPhysicsWorld(Context* context) : Component(context)
    {

        SubscribeToEvent(E_SCENESUBSYSTEMUPDATE, URHO3D_HANDLER(NewtonPhysicsWorld, HandleSceneUpdate));


        contactEntryPool_.clear();
        for (int i = 0; i < contactEntryPoolSize_; i++)
        {
            contactEntryPool_.insert(contactEntryPool_.begin(), context->CreateObject<RigidBodyContactEntry>());
        }

        //set timestep target to max fps
		timeStepTarget_ = 1.0f / GetSubsystem<Engine>()->GetMaxFps();
    }

    NewtonPhysicsWorld::~NewtonPhysicsWorld()
    {

        freeWorld();
    }

    void NewtonPhysicsWorld::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonPhysicsWorld>(DEF_PHYSICS_CATEGORY.c_str());
        URHO3D_COPY_BASE_ATTRIBUTES(Component);
        URHO3D_ACCESSOR_ATTRIBUTE("Gravity", GetGravity, SetGravity, Vector3, DEFAULT_GRAVITY, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Time Scale", GetTimeScale, SetTimeScale, float, 1.0f, AM_DEFAULT);
    }




	void NewtonPhysicsWorld::SerializeNewtonWorld(eastl::string fileName)
    {
        NewtonSerializeToFile(newtonWorld_, fileName.c_str(), nullptr, nullptr);
    }



    
	eastl::string NewtonPhysicsWorld::GetSolverPluginName()
    {
        void* plugin = NewtonCurrentPlugin(newtonWorld_);
		return eastl::string(NewtonGetPluginString(newtonWorld_, plugin));
    }



    void NewtonPhysicsWorld::SetGravity(const Vector3& force)
    {
        gravity_ = force;
    }


    Urho3D::Vector3 NewtonPhysicsWorld::GetGravity() const
{
        return gravity_;
    }


    void NewtonPhysicsWorld::WaitForUpdateFinished()
    {
        NewtonWaitForUpdateToFinish(newtonWorld_);
        isUpdating_ = false;
    }

    void NewtonPhysicsWorld::SetIterationCount(int numIterations /*= 8*/)
    {
        iterationCount_ = numIterations;
        applyNewtonWorldSettings();
    }


    int NewtonPhysicsWorld::GetIterationCount() const
    {
        return iterationCount_;
    }

    void NewtonPhysicsWorld::SetSubstepFactor(int factor)
    {
        subSteps_ = factor;
        applyNewtonWorldSettings();
    }

    int NewtonPhysicsWorld::GetSubstepFactor() const
    {
        return subSteps_;
    }

    void NewtonPhysicsWorld::SetThreadCount(int numThreads)
    {
        newtonThreadCount_ = numThreads;
        applyNewtonWorldSettings();
    }

    int NewtonPhysicsWorld::GetThreadCount() const
    {
        return newtonThreadCount_;
    }

    void NewtonPhysicsWorld::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        DrawDebugGeometry(debug, true, true, true, depthTest);
    }

    void NewtonPhysicsWorld::DrawDebugGeometry(DebugRenderer* debug, bool drawConstraints, bool drawContacts, bool drawRigidBodies, bool depthTest)
    {
        URHO3D_PROFILE_FUNCTION();
        if (debug)
        {
            WaitForUpdateFinished();

            if (drawConstraints) {
                //draw debug geometry on constraints.
                for (NewtonConstraint* constraint : constraintList)
                {
                    constraint->DrawDebugGeometry(debug, depthTest);
                }
            }

            if (drawContacts) {
                //draw debug geometry for contacts
                for (int i = 0; i < contactEntryPool_.size(); i++)
                {
                    if (!contactEntryPool_[i]->expired_)
                        contactEntryPool_[i]->DrawDebugGeometry(debug, depthTest);
                }
            }

            if (drawRigidBodies)
            {
                //draw debug geometry on rigid bodies.
                for (NewtonRigidBody* body : rigidBodyComponentList) {
                    body->DrawDebugGeometry(debug, depthTest);
                }

                //draw debug geometry on static scene.
                if (sceneBody_)
                    sceneBody_->DrawDebugGeometry(debug, depthTest);
            }
        }
    }

    Urho3D::RigidBodyContactEntry* NewtonPhysicsWorld::GetCreateContactEntry(NewtonRigidBody* body0, NewtonRigidBody* body1)
    {
        IntVector2 key(body0->GetID(), body1->GetID());
        //look through existing
        RigidBodyContactEntry* entry = contactEntries_[key.ToHash()];

        if (!entry)
        {
            //find a good one to grab from the physics world pool - if none available - grow the pool.
            int startingIdx = contactEntryPoolCurIdx_;
            while (!contactEntryPool_[contactEntryPoolCurIdx_]->expired_) {

                contactEntryPoolCurIdx_++;

                if (contactEntryPoolCurIdx_ > contactEntryPool_.size() - 1) {
                    contactEntryPoolCurIdx_ = 0;
                }
                if (contactEntryPoolCurIdx_ == startingIdx)
                {

                    //grow the pool
                    int prevSize = contactEntryPool_.size();
                    for (int i = 0; i < contactEntryPoolSize_; i++) {
                        contactEntryPool_.insert(contactEntryPool_.size(), context_->CreateObject<RigidBodyContactEntry>());
                    }

                    URHO3D_LOGINFO("PhysicsWorld Contact Entry Pool Grow To: " + eastl::to_string(contactEntryPool_.size()));

                    contactEntryPoolCurIdx_ = prevSize;
                }



            }
            entry = contactEntryPool_[contactEntryPoolCurIdx_];


            //contactEntries_.InsertNew(otherBody->GetID(), newEntry);

            //form key based on both bodies ids.

  

            contactEntries_.insert_or_assign(key.ToHash(), entry);
        }






        return entry;
    }

    void NewtonPhysicsWorld::CleanContactEntries()
    {
		eastl::vector<unsigned int> keys = contactEntries_.keys();
        for (int i = 0; i < keys.size(); i++) {

            if (contactEntries_[keys[i]]->expired_)
                contactEntries_.erase(keys[i]);
        }
    }

    void NewtonPhysicsWorld::OnSceneSet(Scene* scene)
    {
        if (scene) {
            //create the newton world
            if (newtonWorld_ == nullptr) {
                newtonWorld_ = NewtonCreate();
                NewtonWorldSetUserData(newtonWorld_, (void*)this);
                applyNewtonWorldSettings();



                //resolve root scene body.
                if (!sceneBody_) {
                    sceneBody_ = GetScene()->GetOrCreateComponent<NewtonRigidBody>();
                    sceneBody_->SetIsSceneRootBody(true);
                    sceneBody_->SetTemporary(true);
                }


                NewtonMaterialSetCollisionCallback(newtonWorld_, 0, 0, Newton_AABBOverlapCallback, Newton_ProcessContactsCallback);
                //NewtonMaterialSetCompoundCollisionCallback(newtonWorld_, 0, 0, Newton_AABBCompoundOverlapCallback);
                NewtonSetPostUpdateCallback(newtonWorld_, Newton_PostUpdateCallback);
                NewtonWorldSetCreateDestroyContactCallback(newtonWorld_, nullptr, Newton_DestroyContactCallback);


            }
        }
        else
        {
            //wait for update to finish if in async mode so we can safely clean up.
            NewtonWaitForUpdateToFinish(newtonWorld_);

            freeWorld();
        }
    }

    void NewtonPhysicsWorld::addCollisionShape(NewtonCollisionShape* collision)
    {
		collisionComponentList.push_front(eastl::weak_ptr<NewtonCollisionShape>(collision));
    }

    void NewtonPhysicsWorld::removeCollisionShape(NewtonCollisionShape* collision)
    {
		collisionComponentList.erase(collisionComponentList.index_of(eastl::weak_ptr<NewtonCollisionShape>(collision)));
    }

    void NewtonPhysicsWorld::addRigidBody(NewtonRigidBody* body)
    {
		rigidBodyComponentList.push_front(eastl::weak_ptr<NewtonRigidBody>(body));
    }

    void NewtonPhysicsWorld::removeRigidBody(NewtonRigidBody* body)
    {

		rigidBodyComponentList.erase(rigidBodyComponentList.index_of(eastl::weak_ptr<NewtonRigidBody>(body)));
    }

    void NewtonPhysicsWorld::addConstraint(NewtonConstraint* constraint)
    {
		constraintList.push_front(eastl::weak_ptr<NewtonConstraint>(constraint));



    }

    void NewtonPhysicsWorld::removeConstraint(NewtonConstraint* constraint)
    {
		constraintList.erase(constraintList.index_of(eastl::weak_ptr<NewtonConstraint>(constraint)));
    }



    void NewtonPhysicsWorld::freeWorld()
    {
        //wait for update to finish if in async mode so we can safely clean up.
        if (newtonWorld_)
            NewtonWaitForUpdateToFinish(newtonWorld_);


        //free any joints
        for (NewtonConstraint* constraint : constraintList)
        {
            constraint->freeInternal();
        }
        constraintList.clear();

        //free any collision shapes currently in the list
        for (NewtonCollisionShape* col : collisionComponentList)
        {
            col->freeInternalCollision();
        }
        collisionComponentList.clear();



        //free internal bodies for all rigid bodies.
        for (NewtonRigidBody* rgBody : rigidBodyComponentList)
        {
            rgBody->freeBody();
        }
        rigidBodyComponentList.clear();



        //free meshes in mesh cache
        newtonMeshCache_.clear();

        //free the actual memory
        freePhysicsInternals();



        //destroy newton world.
        if (newtonWorld_ != nullptr) {
            NewtonDestroy(newtonWorld_);
            newtonWorld_ = nullptr;
        }


    }




    void NewtonPhysicsWorld::addToFreeQueue(NewtonBody* newtonBody)
    {
        freeBodyQueue_.push_front(newtonBody);
    }

    void NewtonPhysicsWorld::addToFreeQueue(dCustomJoint* newtonConstraint)
    {
        freeConstraintQueue_.push_front(newtonConstraint);
    }

    void NewtonPhysicsWorld::addToFreeQueue(NewtonCollision* newtonCollision)
    {
        freeCollisionQueue_.push_front(newtonCollision);
    }

    void NewtonPhysicsWorld::applyNewtonWorldSettings()
    {
        NewtonSetSolverIterations(newtonWorld_, iterationCount_);
        NewtonSetNumberOfSubsteps(newtonWorld_, subSteps_);
        NewtonSetThreadsCount(newtonWorld_, newtonThreadCount_);
        NewtonSelectBroadphaseAlgorithm(newtonWorld_, 1);//persistent broadphase.

        
    }







    void NewtonPhysicsWorld::ParseContacts()
    {
        eastl::vector<unsigned int> removeKeys;
        VariantMap eventData;
        eventData[PhysicsCollisionStart::P_WORLD] = this;


        for (eastl::hash_map<unsigned int, RigidBodyContactEntry*>::iterator i = contactEntries_.begin(); i != contactEntries_.end(); i++)
        {
            RigidBodyContactEntry* entry = i->second;

            if (entry->expired_)
                continue;

            eventData[PhysicsCollisionStart::P_BODYA] = entry->body0;
            eventData[PhysicsCollisionStart::P_BODYB] = entry->body1;

            eventData[PhysicsCollisionStart::P_CONTACT_DATA] = entry;

            if (entry->body0.expired() || entry->body1.expired())//check expired
            {
                entry->expired_ = true;
            }
            else if (entry->wakeFlag_ && !entry->wakeFlagPrev_)//begin contact
            {
                if (entry->body0->collisionEventMode_ &&entry->body1->collisionEventMode_) {
                    SendEvent(E_PHYSICSCOLLISIONSTART, eventData);
                }

                if (entry->body0->collisionEventMode_) {
                    if (entry->body0.expired() || entry->body1.expired()) break; //it is possible someone deleted a body in the previous event.

                    eventData[NodeCollisionStart::P_OTHERNODE] = entry->body1->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = entry->body1;
                    entry->body0->GetNode()->SendEvent(E_NODECOLLISIONSTART, eventData);
                }


                if (entry->body1->collisionEventMode_) {
                    if (entry->body0.expired() || entry->body1.expired()) break;

                    eventData[NodeCollisionStart::P_OTHERNODE] = entry->body0->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = entry->body0;
                    entry->body1->GetNode()->SendEvent(E_NODECOLLISIONSTART, eventData);
                }


                if (entry->body0->collisionEventMode_ &&entry->body1->collisionEventMode_) {
                    if (entry->body0.expired() || entry->body1.expired()) break;

                    //also send the E_NODECOLLISION event
                    SendEvent(E_PHYSICSCOLLISION, eventData);
                }

                if (entry->body0->collisionEventMode_) {

                    if (entry->body0.expired() || entry->body1.expired()) break;

                    eventData[NodeCollisionStart::P_OTHERNODE] = entry->body1->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = entry->body1;
                    entry->body0->GetNode()->SendEvent(E_NODECOLLISION, eventData);
                }


                if (entry->body1->collisionEventMode_) {
                    if (entry->body0.expired() || entry->body1.expired()) break;

                    eventData[NodeCollisionStart::P_OTHERNODE] = entry->body0->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = entry->body0;
                    entry->body1->GetNode()->SendEvent(E_NODECOLLISION, eventData);
                }


            }
            else if (!entry->wakeFlag_ && entry->wakeFlagPrev_)//end contact
            {
                if (entry->body0->collisionEventMode_ && entry->body1->collisionEventMode_) {
                    SendEvent(E_PHYSICSCOLLISIONEND, eventData);
                }

                if (entry->body0->collisionEventMode_) {

                    if (entry->body0.expired() || entry->body1.expired()) break;
                    eventData[NodeCollisionStart::P_OTHERNODE] = entry->body1->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = entry->body1;
                    entry->body0->GetNode()->SendEvent(E_NODECOLLISIONEND, eventData);
                }

                if (entry->body1->collisionEventMode_) {
                    if (entry->body0.expired() || entry->body1.expired()) break;
                    eventData[NodeCollisionStart::P_OTHERNODE] = entry->body0->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = entry->body0;
                    entry->body1->GetNode()->SendEvent(E_NODECOLLISIONEND, eventData);
                }
            }
            else if (entry->wakeFlag_ && entry->wakeFlagPrev_)//continued contact
            {
                if (entry->body0->collisionEventMode_ == NewtonRigidBody::COLLISION_ALL || entry->body1->collisionEventMode_ == NewtonRigidBody::COLLISION_ALL) {
                    SendEvent(E_PHYSICSCOLLISION, eventData);
                }


                if (entry->body0->collisionEventMode_ == NewtonRigidBody::COLLISION_ALL) {
                    if (entry->body0.expired() || entry->body1.expired()) break;

                    eventData[NodeCollisionStart::P_OTHERNODE] = entry->body1->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = entry->body1;
                    entry->body0->GetNode()->SendEvent(E_NODECOLLISION, eventData);
                }

                if (entry->body1->collisionEventMode_ == NewtonRigidBody::COLLISION_ALL) {

                    if (entry->body0.expired() || entry->body1.expired()) break;

                    eventData[NodeCollisionStart::P_OTHERNODE] = entry->body0->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = entry->body0;
                    entry->body1->GetNode()->SendEvent(E_NODECOLLISION, eventData);
                }
            }
            else if (!entry->wakeFlag_ && !entry->wakeFlagPrev_)//no contact for one update. (mark for removal from the map)
            {
                entry->expired_ = true;
            }

            //move on..
            entry->wakeFlagPrev_ = entry->wakeFlag_;

            if (entry->newtonJoint_)
                entry->wakeFlag_ = NewtonJointIsActive(entry->newtonJoint_);
            else
                entry->wakeFlag_ = false;
            
        }

        if(contactEntries_.size() > 10)
            CleanContactEntries();

    }




    void NewtonPhysicsWorld::HandleSceneUpdate(StringHash eventType, VariantMap& eventData)
    {


       float timeStep = eventData[SceneSubsystemUpdate::P_TIMESTEP].GetFloat();
       timeStepTarget_ = timeStep;

       //URHO3D_LOGINFO(String(timeStep));


       //do the update.
       Update(timeStep, true);
    }



    void NewtonPhysicsWorld::Update(float timestep, bool isRootUpdate)
    {

        URHO3D_PROFILE_FUNCTION();
        float physicsTimeStep = timestep*GetScene()->GetTimeScale()*timeScale_;

 



        if (simulationStarted_) {
            URHO3D_PROFILE("Wait For ASync Update To finish.");
            
            NewtonWaitForUpdateToFinish(newtonWorld_);
            isUpdating_ = false;
            // Send post-step event
            VariantMap& eventData = GetEventDataMap();
            eventData[PhysicsPostStep::P_WORLD] = this;
            eventData[PhysicsPostStep::P_TIMESTEP] = physicsTimeStep;
            SendEvent(E_PHYSICSPOSTSTEP, eventData);

        }
        

        if (simulationStarted_) {

            {
                URHO3D_PROFILE("Apply Node Transforms");

                {
                    URHO3D_PROFILE("Rigid Body Order Pre Sort");
                    //sort the rigidBodyComponentList by scene depth.
                    if (rigidBodyListNeedsSorted) {

						eastl::sort(rigidBodyComponentList.begin(), rigidBodyComponentList.end(), RigidBodySceneDepthCompare);
                        rigidBodyListNeedsSorted = false;
                    }
                }
            }
        }



        ParseContacts();

        //rebuild stuff.
        rebuildDirtyPhysicsComponents();

        freePhysicsInternals();




        
        for (NewtonRigidBody* rigBody : rigidBodyComponentList)
        {
            if (rigBody == sceneBody_)
                continue;



            //if the node local transform has been changed since last update - move the body.
            //this is done so that a change to the scene graph will have a resulting change in the rigid body's transformation.
            //if contraints are connected dont do this.
            if (rigBody->node_->GetWorldTransform() != rigBody->lastSetNodeWorldTransform_ && (rigBody->connectedConstraints_.size() == 0)) {





                rigBody->SetWorldTransformToNode();
                rigBody->MarkInternalTransformDirty(true);

                if (rigBody->isKinematic_) {
                    //get translational matrix and apply it as velocity to the kinematic body.  this can be overridden if velocities of body are set in the physics pre-step event.
                    Matrix3x4 deltaTransform = rigBody->node_->GetWorldTransform().Inverse() * rigBody->lastSetNodeWorldTransform_;

                    Quaternion rotationalDelta = deltaTransform.Rotation();
                    Vector3 translationalDelta = deltaTransform.Translation();

                    rigBody->SetAngularVelocity(-rotationalDelta.EulerAngles()/physicsTimeStep);
                    rigBody->SetLinearVelocityHard(translationalDelta);
                }
            }

            //wake bodies around kinematic bodies
            if (rigBody->isKinematic_) {
                dVector p0, p1;
                NewtonBodyGetAABB(rigBody->newtonBody_, &p0[0], &p1[0]);

                //p0 = p0 - dVector(5, 5, 5);
                //p1 = p1 + dVector(5, 5, 5);

                NewtonWorldForEachBodyInAABBDo(newtonWorld_, &p0[0], &p1[0], Newton_WakeBodiesInAABBCallback, nullptr);
            }

            //apply the transform of all rigid body components to their respective nodes.
            if (rigBody->GetInternalTransformDirty()) {

                rigBody->ApplyTransformToNode();

                //if (rigBody->InterpolationWithinRestTolerance())
                rigBody->MarkInternalTransformDirty(false);
            }
        }




        {
            // Send pre-step event
            VariantMap& eventData = GetEventDataMap();
            eventData[PhysicsPreStep::P_WORLD] = this;
            eventData[PhysicsPreStep::P_TIMESTEP] = physicsTimeStep;
            SendEvent(E_PHYSICSPRESTEP, eventData);


            //use target time step to give newton constant time steps. 
            NewtonUpdateAsync(newtonWorld_, physicsTimeStep);
            isUpdating_ = true;
            simulationStarted_ = true;
        }
    }

    void NewtonPhysicsWorld::rebuildDirtyPhysicsComponents()
    {
        URHO3D_PROFILE_FUNCTION();


        //rebuild dirty collision shapes
        for (NewtonCollisionShape* colShape : collisionComponentList)
        {
            if (colShape->GetDirty()) {
                colShape->updateBuild();
                colShape->MarkDirty(false);
            }
        }
        

        //then rebuild rigid bodies if they need rebuilt (dirty) from root nodes up.
        for (NewtonRigidBody* rigBody : rigidBodyComponentList)
        {
            if (!rigBody->GetDirty())
                continue;

            //mark all connected constraints as needing rebuilt.
            for (NewtonConstraint* constraint : rigBody->connectedConstraints_) {
                constraint->MarkDirty(true);
            }

            rigBody->reBuildBody();

            rigBody->ApplyTransformToNode();
            rigBody->MarkDirty(false);
        }


        //rebuild contraints if they need rebuilt (dirty)
        for (NewtonConstraint* constraint : constraintList)
        {
            if (constraint->dirty_)
                constraint->reEvalConstraint();
        }


        
        for (NewtonRigidBody* rigBody : rigidBodyComponentList)
        {
            //apply properties
            rigBody->applyDefferedProperties();

            //apply deferred actions (like impulses/velocity sets etc.) that were waiting for a real body to be built.
            rigBody->applyDefferedActions();
        }
    }





    Urho3D::StringHash NewtonPhysicsWorld::NewtonMeshKey(eastl::string modelResourceName, int modelLodLevel, eastl::string otherData)
    {
        return modelResourceName + eastl::to_string(modelLodLevel) + otherData;
    }

    NewtonMeshObject* NewtonPhysicsWorld::GetCreateNewtonMesh(StringHash urhoNewtonMeshKey)
    {
        if (newtonMeshCache_.contains(urhoNewtonMeshKey)) {
            return newtonMeshCache_[urhoNewtonMeshKey];
        }
        else
        {
            NewtonMesh* mesh = NewtonMeshCreate(newtonWorld_);
            ea::shared_ptr<NewtonMeshObject> meshObj = context_->CreateObject<NewtonMeshObject>();
            meshObj->mesh = mesh;
            newtonMeshCache_[urhoNewtonMeshKey] = meshObj;
            return meshObj;
        }
    }

    NewtonMeshObject* NewtonPhysicsWorld::GetNewtonMesh(StringHash urhoNewtonMeshKey)
    {
        if (newtonMeshCache_.contains(urhoNewtonMeshKey)) {
            return newtonMeshCache_[urhoNewtonMeshKey];
        }
        return nullptr;
    }

    void NewtonPhysicsWorld::freePhysicsInternals()
    {
        for (dCustomJoint* constraint : freeConstraintQueue_)
        {
            delete constraint;
        }
        freeConstraintQueue_.clear();


        for (NewtonCollision* col : freeCollisionQueue_)
        {
            NewtonDestroyCollision(col);
        }
        freeCollisionQueue_.clear();


        for (NewtonBody* body : freeBodyQueue_)
        {
            NewtonDestroyBody(body);
        }
        freeBodyQueue_.clear();




    }
 

    void PrintPhysicsRayCastIntersection(PhysicsRayCastIntersection& intersection)
    {
        URHO3D_LOGINFO("body_: " + Urho3D::ToString((void*)intersection.body_));
        URHO3D_LOGINFO("collision_: " + Urho3D::ToString((void*)intersection.collision_));
        URHO3D_LOGINFO("subCollision_: " + Urho3D::ToString((void*)intersection.subCollision_));
        URHO3D_LOGINFO("rayIntersectParameter_: " + ea::to_string(intersection.rayIntersectParameter_));
        URHO3D_LOGINFO("rigBody_: " + Urho3D::ToString((void*)intersection.rigBody_));
        URHO3D_LOGINFO("collisionShape_: " + Urho3D::ToString((void*)intersection.collisionShape_));
        URHO3D_LOGINFO("rayIntersectWorldPosition_: " + intersection.rayIntersectWorldPosition_.ToString());
        URHO3D_LOGINFO("rayIntersectWorldNormal_: " + intersection.rayIntersectWorldNormal_.ToString());
        URHO3D_LOGINFO("rayDistance_: " + eastl::to_string(intersection.rayDistance_));
        URHO3D_LOGINFO("rayOriginWorld_: " + intersection.rayOriginWorld_.ToString());
    }

    eastl::string NewtonThreadProfilerString(int threadIndex)
    {
        return ("Newton_Thread" + ea::to_string(threadIndex));
    }

    
    //called when the newton update finished.
    void Newton_PostUpdateCallback(const NewtonWorld* const world, dFloat timestep)
    {
        NewtonPhysicsWorld* physicsWorld = (NewtonPhysicsWorld*)NewtonWorldGetUserData(world);
        physicsWorld->isUpdating_ = false;
    }

    //add rigid bodies to the list as the function recurses from node to root. the last rigidbody in rigidBodies is the most root. optionally include the scene as root.
    void GetRootRigidBodies(ea::vector<NewtonRigidBody*>& rigidBodies, Node* node, bool includeScene)
    {
        NewtonRigidBody* body = node->GetComponent<NewtonRigidBody>();


        if (body)
            rigidBodies.push_back(body);

        //recurse on parent
        if(node->GetParent() && ((node->GetScene() != node->GetParent()) || includeScene))
            GetRootRigidBodies(rigidBodies, node->GetParent(), includeScene);
    }




	URHONEWTON_API NewtonRigidBody* GetRigidBody(Node* node, bool includeScene)
    {
        Node* curNode = node;

        while (curNode) {
            if (curNode == curNode->GetScene() && !includeScene)
            {
                return nullptr;
            }

            NewtonRigidBody* body = curNode->GetComponent<NewtonRigidBody>();

            if (body)
                return body;

            curNode = curNode->GetParent();
        }

        return nullptr;
    }

    //returns first occuring child rigid bodies.
	URHONEWTON_API void GetNextChildRigidBodies(ea::vector<NewtonRigidBody*>& rigidBodies, Node* node)
    {

		ea::vector<Node*> immediateChildren;
        node->GetChildren(immediateChildren, false);

        for (Node* child : immediateChildren) {
            if (child->HasComponent<NewtonRigidBody>())
                rigidBodies.push_back(child->GetComponent<NewtonRigidBody>());
            else
                GetNextChildRigidBodies(rigidBodies, child);
        }

    }

    //recurses up the scene tree starting at the starting node,  continuing up every branch adding collision shapes to the array until a rigid body is encountered in which case the algorithm stops traversing that branch.
    void GetAloneCollisionShapes(ea::vector<NewtonCollisionShape*>& colShapes, Node* startingNode, bool includeStartingNodeShapes)
    {

        if (includeStartingNodeShapes)
        {
            startingNode->GetDerivedComponents<NewtonCollisionShape>(colShapes, false, false);
        }



		ea::vector<Node*> immediateChildren;
        startingNode->GetChildren(immediateChildren, false);

        for (Node* child : immediateChildren) {
            if (child->HasComponent<NewtonRigidBody>() && child->GetComponent<NewtonRigidBody>()->IsEnabled())
                continue;
            else
            {
                child->GetDerivedComponents<NewtonCollisionShape>(colShapes, false, false);
                GetAloneCollisionShapes(colShapes, child, false);
            }
        }
    }









    void RebuildPhysicsNodeTree(Node* node)
    {
        //trigger a rebuild on the root of the new tree.
		ea::vector<NewtonRigidBody*> rigBodies;
        GetRootRigidBodies(rigBodies, node, false);
        if (rigBodies.size()) {
            NewtonRigidBody* mostRootRigBody = rigBodies.back();
            if (mostRootRigBody)
                mostRootRigBody->MarkDirty(true);
        }
    }


    unsigned CollisionLayerAsBit(unsigned layer)
    {
        if (layer == 0)
            return M_MAX_UNSIGNED;

        return (0x1 << layer - 1);
    }

    void RegisterNewtonPhysicsLibrary(Context* context)
    {
        NewtonPhysicsWorld::RegisterObject(context);

        NewtonCollisionShape::RegisterObject(context);
        NewtonCollisionShape_Box::RegisterObject(context);
        NewtonCollisionShape_Sphere::RegisterObject(context);
        NewtonCollisionShape_Cylinder::RegisterObject(context);
        NewtonCollisionShape_ChamferCylinder::RegisterObject(context);
        NewtonCollisionShape_Capsule::RegisterObject(context);
        NewtonCollisionShape_Cone::RegisterObject(context);
        NewtonCollisionShape_Geometry::RegisterObject(context);
        NewtonCollisionShape_ConvexHull::RegisterObject(context);
        NewtonCollisionShape_ConvexHullCompound::RegisterObject(context);
        NewtonCollisionShape_ConvexDecompositionCompound::RegisterObject(context);
        NewtonCollisionShape_TreeCollision::RegisterObject(context);
        NewtonCollisionShape_HeightmapTerrain::RegisterObject(context);

        NewtonRigidBody::RegisterObject(context);
        NewtonMeshObject::RegisterObject(context);
        NewtonConstraint::RegisterObject(context);
        NewtonFixedDistanceConstraint::RegisterObject(context);
        NewtonBallAndSocketConstraint::RegisterObject(context);
        NewtonSixDofConstraint::RegisterObject(context);
        NewtonHingeConstraint::RegisterObject(context);
        NewtonSliderConstraint::RegisterObject(context);
        NewtonFullyFixedConstraint::RegisterObject(context);
        KinematicsControllerConstraint::RegisterObject(context);
        RigidBodyContactEntry::RegisterObject(context);

    }












    RigidBodyContactEntry::RigidBodyContactEntry(Context* context) : Object(context)
    {

    }

    RigidBodyContactEntry::~RigidBodyContactEntry()
    {

    }

    void RigidBodyContactEntry::RegisterObject(Context* context)
    {
        context->RegisterFactory<RigidBodyContactEntry>();
    }

    void RigidBodyContactEntry::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        //draw contact points
        if (!expired_)
        {
            for (int i = 0; i < numContacts; i++)
            {
                debug->AddLine(contactPositions[i], (contactPositions[i] + contactNormals[i]), Color::GREEN, depthTest);
            }
        }
    }



}
