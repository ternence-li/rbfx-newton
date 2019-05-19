#pragma once
#include "UrhoNewtonApi.h"
#include "NewtonCollisionShape.h"

namespace Urho3D {



    class Geometry;
    class StringHash;



    class URHONEWTON_API NewtonCollisionShape_Box : public NewtonCollisionShape {

        URHO3D_OBJECT(NewtonCollisionShape_Box, NewtonCollisionShape);

    public:
        NewtonCollisionShape_Box(Context* context);
        virtual ~NewtonCollisionShape_Box();

        static void RegisterObject(Context* context);

        /// Set the size of the box
        void SetSize(const Vector3& size) { size_ = size; MarkDirty(true); }

        /// Get the size of the box
        Vector3 GetSize() const { return size_; }

    protected:
        Vector3 size_ = Vector3::ONE;

        virtual bool buildNewtonCollision() override;

    };

    class URHONEWTON_API NewtonCollisionShape_Sphere : public NewtonCollisionShape {

        URHO3D_OBJECT(NewtonCollisionShape_Sphere, NewtonCollisionShape);

    public:
        NewtonCollisionShape_Sphere(Context* context);
        virtual ~NewtonCollisionShape_Sphere();

        static void RegisterObject(Context* context);


        void SetRadius(float radius) { radius_ = radius; MarkDirty(true); }

        float GetRadius() const { return radius_; }

    protected:
        float radius_ = 0.5f;

        virtual bool buildNewtonCollision() override;

    };

    class URHONEWTON_API NewtonCollisionShape_Capsule : public NewtonCollisionShape {

        URHO3D_OBJECT(NewtonCollisionShape_Capsule, NewtonCollisionShape);

    public:
        NewtonCollisionShape_Capsule(Context* context);
        virtual ~NewtonCollisionShape_Capsule();

        static void RegisterObject(Context* context);

        void SetLength(float length) { length_ = length; MarkDirty(true); }
        float GetLength() const { return length_; }

        void SetRadius1(float radius) { radius1_ = radius; MarkDirty(true); }
        float GetRadius1() const { return radius1_; }


        void SetRadius2(float radius) { radius2_ = radius; MarkDirty(true); }
        float GetRadius2() const { return radius2_; }

        /// Set both radi
        void SetRadius(float radius) { SetRadius1(radius); SetRadius2(radius); }

    protected:
        float length_ = 1.0f;
        float radius1_ = 0.5f;
        float radius2_ = 0.5f;

        virtual bool buildNewtonCollision() override;

    };

    class URHONEWTON_API NewtonCollisionShape_Cone : public NewtonCollisionShape {

        URHO3D_OBJECT(NewtonCollisionShape_Cone, NewtonCollisionShape);

    public:
        NewtonCollisionShape_Cone(Context* context);
        virtual ~NewtonCollisionShape_Cone();

        static void RegisterObject(Context* context);

        void SetRadius(float radius) { radius_ = radius; MarkDirty(true); }
        float GetRadius() const { return radius_; }

        void SetLength(float length) { length_ = length; MarkDirty(true); }
        float GetLength() const { return length_; }

    protected:
        float length_ = 1.0f;
        float radius_ = 0.5f;


        virtual bool buildNewtonCollision() override;

    };


    class URHONEWTON_API NewtonCollisionShape_Cylinder : public NewtonCollisionShape {

        URHO3D_OBJECT(NewtonCollisionShape_Cylinder, NewtonCollisionShape);

    public:
        NewtonCollisionShape_Cylinder(Context* context);
        virtual ~NewtonCollisionShape_Cylinder();

        static void RegisterObject(Context* context);

        /// Set Radius in 1st dimension
        void SetRadius1(float radius) { radius1_ = radius; MarkDirty(true); }

        float GetRadius1() const { return radius1_; }

        /// Set Radius in 2nd dimension
        void SetRadius2(float radius) { radius2_ = radius; MarkDirty(true); }

        float GetRadius2() const { return radius2_; }

        /// Set both radi
        void SetRadius(float radius) { SetRadius1(radius); SetRadius2(radius); }

        /// set the length
        void SetLength(float length) { length_ = length; MarkDirty(true); }

        float GetLength() const { return length_; }

    protected:
        float radius1_ = 0.5f;
        float radius2_ = 0.5f;
        float length_ = 1.0f;

        virtual bool buildNewtonCollision() override;

    };

    class URHONEWTON_API NewtonCollisionShape_ChamferCylinder : public NewtonCollisionShape {

        URHO3D_OBJECT(NewtonCollisionShape_ChamferCylinder, NewtonCollisionShape);

    public:
        NewtonCollisionShape_ChamferCylinder(Context* context);
        virtual ~NewtonCollisionShape_ChamferCylinder();

        static void RegisterObject(Context* context);

        /// Set Radius in 1st dimension
        void SetRadius(float radius) { radius_ = radius; MarkDirty(true); }
        float GetRadius() const { return radius_; }

        /// set the length
        void SetLength(float length) { length_ = length; MarkDirty(true); }

        float GetLength() const { return length_; }

    protected:
        float radius_ = 0.5f;
        float length_ = 1.0f;

        virtual bool buildNewtonCollision() override;

    };



    class URHONEWTON_API NewtonCollisionShape_Geometry : public NewtonCollisionShape {

        URHO3D_OBJECT(NewtonCollisionShape_Geometry, NewtonCollisionShape);

    public:
        NewtonCollisionShape_Geometry(Context* context);
        virtual ~NewtonCollisionShape_Geometry();

        static void RegisterObject(Context* context);

        /// Set model to create geometry from
        void SetModel(ea::weak_ptr<Model> model) { model_ = model; MarkDirty(); }

        Model* GetModel() const { return model_; }

        ResourceRef GetModelResourceRef() const;
        void SetModelByResourceRef(const ResourceRef& ref);

        void SetModelLodLevel(int lod) { modelLodLevel_ = lod; MarkDirty(); }

        int GetModelLodLevel() const { return modelLodLevel_; }

        /// Set the tolerancing for hull creation.
        void SetHullTolerance(float tolerance = 0.0f) { hullTolerance_ = 0.0f; MarkDirty(); }

        float GetHullTolerance() const { return hullTolerance_; }

    protected:
        /// optional Model reference
        ea::weak_ptr<Model> model_;
        /// lod level
        unsigned modelLodLevel_ = 0;
        /// model geometry index to use
        unsigned modelGeomIndx_ = 0;

        /// Hulling tolerance
        float hullTolerance_ = 0.0f;

        virtual bool buildNewtonCollision() override;

        void autoSetModel();

        ///forms newtonMesh_ from model geometry for later use.
        bool resolveOrCreateTriangleMeshFromModel();

        NewtonMeshObject* getCreateTriangleMesh(Geometry* geom, StringHash meshkey);


        virtual void OnNodeSet(Node* node) override;

    };

    class URHONEWTON_API NewtonCollisionShape_ConvexHullCompound : public NewtonCollisionShape_Geometry {

        URHO3D_OBJECT(NewtonCollisionShape_ConvexHullCompound, NewtonCollisionShape_Geometry);

    public:
        NewtonCollisionShape_ConvexHullCompound(Context* context);
        virtual ~NewtonCollisionShape_ConvexHullCompound();

        static void RegisterObject(Context* context);

    protected:

        virtual bool buildNewtonCollision() override;


    };

    
    class URHONEWTON_API NewtonCollisionShape_ConvexDecompositionCompound : public NewtonCollisionShape_Geometry {

        URHO3D_OBJECT(NewtonCollisionShape_ConvexDecompositionCompound, NewtonCollisionShape_Geometry);

    public:
        NewtonCollisionShape_ConvexDecompositionCompound(Context* context);
        virtual ~NewtonCollisionShape_ConvexDecompositionCompound();

        static void RegisterObject(Context* context);


    protected:
        NewtonMeshObject* meshDecomposition_ = nullptr;

        virtual bool buildNewtonCollision() override;

    };

    class URHONEWTON_API NewtonCollisionShape_ConvexHull : public NewtonCollisionShape_Geometry {

        URHO3D_OBJECT(NewtonCollisionShape_ConvexHull, NewtonCollisionShape_Geometry);

    public:
        NewtonCollisionShape_ConvexHull(Context* context);
        virtual ~NewtonCollisionShape_ConvexHull();

        static void RegisterObject(Context* context);


    protected:

        virtual bool buildNewtonCollision() override;

    };


    ///Collision that matches goemetry mesh data - rigid body using this shape must have zero mass.
    class URHONEWTON_API NewtonCollisionShape_TreeCollision : public NewtonCollisionShape_Geometry {

        URHO3D_OBJECT(NewtonCollisionShape_TreeCollision, NewtonCollisionShape_Geometry);

    public:
        NewtonCollisionShape_TreeCollision(Context* context);
        virtual ~NewtonCollisionShape_TreeCollision();

        static void RegisterObject(Context* context);


    protected:

        virtual bool buildNewtonCollision() override;


    };






    class URHONEWTON_API NewtonCollisionShape_HeightmapTerrain : public NewtonCollisionShape_Geometry {

        URHO3D_OBJECT(NewtonCollisionShape_HeightmapTerrain, NewtonCollisionShape_Geometry);

    public:
        NewtonCollisionShape_HeightmapTerrain(Context* context);
        virtual ~NewtonCollisionShape_HeightmapTerrain();

        static void RegisterObject(Context* context);



    protected:


        virtual bool buildNewtonCollision() override;

    };









}

