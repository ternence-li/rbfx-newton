#pragma once


#include "NewtonConstraint.h"


namespace Urho3D {
	class Context;



	class URHONEWTON_API NewtonGearConstraint : public NewtonConstraint
	{
		URHO3D_OBJECT(NewtonGearConstraint, NewtonConstraint);

	public:

		NewtonGearConstraint(Context* context);
		~NewtonGearConstraint();




		static void RegisterObject(Context* context);

		

		virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;


		void SetGearRatio(float ratio) {

			if (ratio != ratio_) {
				ratio_ = ratio;
				//have to remake constraint..
				MarkDirty();
			}
		}
		float GetGearRatio() const { return ratio_; }


	protected:

		virtual void buildConstraint() override;

		bool applyAllJointParams();


		float ratio_ = 1.0f;
	};


}
