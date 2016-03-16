#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f)};

	//pyramid vertices
	/*static PxVec3 pyramid_verts[] = {PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1)};
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = {1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1};

	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose=PxTransform(PxIdentity), PxReal density=1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose=PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs),end(pyramid_trigs)), pose)
		{
		}
	};*/

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0		= (1 << 0),
			ACTOR1		= (1 << 1),
			ACTOR2		= (1 << 2)
			//add more if you need
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box *bottom, *top;

	public:
		Trampoline(const PxVec3& dimensions=PxVec3(1.f,1.f,1.f), PxReal stiffness=1.f, PxReal damping=1.f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(PxVec3(0.f,thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			top = new Box(PxTransform(PxVec3(0.f,dimensions.y+thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,-dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,-dimensions.z)));

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
		}

		void AddToScene(Scene* scene)
		{
			scene->Add(bottom);
			scene->Add(top);
		}

		~Trampoline()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}
	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;

		MySimulationEventCallback() : trigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						trigger = true;
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//check eNOTIFY_TOUCH_FOUND
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
				}
				//check eNOTIFY_TOUCH_LOST
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
//		pairFlags |= PxPairFlag::eCCD_LINEAR;
		
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{
		Plane* plane;
		Box* paddle1;
		Box* paddle2;
		Sphere* ball;
		stubbornBox* box3;
		PxMaterial* rubber;
		MySimulationEventCallback* my_callback;

	private:
		int scoreGreen;
		int scoreRed;

	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene() 
		{
			scoreGreen = 0;
			scoreRed = 0;
		};

		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
		}

		void SetScore(int team)
		{
			if (team == 1)
				scoreGreen++;
			else
				scoreRed++;
		}

		int GetScore(int team)
		{
			if (team == 1)
				return scoreGreen;
			else
				return scoreRed;
		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			

			GetMaterial()->setDynamicFriction(.2f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			plane = new Plane();
			plane->Color(PxVec3(150.f/255.f,150.f/255.f,150.f/255.f));
			Add(plane);

			paddle1 = new Box(PxTransform(PxVec3(-15.0f,2.0f,-15.0f), PxQuat(PxPi / 8, PxVec3(0.f, 0.f, -1.f))), PxVec3(1.5f, 0.25f, 0.5f), PxReal(1000.0f));
			paddle1->Color(PxVec3(0.0f / 255.f, 75.0f / 255.f, 0.0f / 255.f));
			//set collision filter flags
			// box->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1);
			//use | operator to combine more actors e.g.
			// box->SetupFiltering(FilterGroup::ACTOR0, FilterGroup::ACTOR1 | FilterGroup::ACTOR2);
			//don't forget to set your flags for the matching actor as well, e.g.:
			// box2->SetupFiltering(FilterGroup::ACTOR1, FilterGroup::ACTOR0);
			paddle1->Name("Paddle1");
			Add(paddle1);			

			paddle2 = new Box(PxTransform(PxVec3(15.0f, 2.0f, -15.0f), PxQuat(PxPi / 8, PxVec3(0.f, 0.f, 1.f))), PxVec3(1.5f, 0.25f, 0.5f), PxReal(1000.0f));
			paddle2->Color(PxVec3(100.0f / 255.f, 0.0f / 255.f, 0.0f / 255.f));
			paddle2->Name("Paddle2");
			Add(paddle2);

			ball = new Sphere(PxTransform(PxVec3(-15.0f, 25.0f, -15.0f)), PxReal(1.0f), PxReal(0.01f)); 
			rubber = GetPhysics()->createMaterial(0.25f, 0.25f, 1.0f);
			ball->Material(rubber);
			ball->Color(PxVec3(255.0f / 255.f, 255.0f / 255.f, 255.0f / 255.f));
			ball->Name("Ball");
			Add(ball);

			box3 = new stubbornBox(PxTransform(PxVec3(0.0f, 5.75f, -15.0f)), PxVec3(0.25f, 5.0f, 1.0f), PxReal(1.0f));
			box3->Color(PxVec3(50.0f/255.f, 50.0f/255.f, 50.0f/255.f));
			box3->Name("Wall");
			Add(box3);

			box3 = new stubbornBox(PxTransform(PxVec3(0.0f, 0.5f, -15.0f)), PxVec3(25.0f, 0.25f, 1.0f), PxReal(1.0f));
			box3->Color(PxVec3(175.0f/255.f, 175.0f/255.f, 175.0f/255.f));
			box3->Name("Floor");
			Add(box3);

			box3 = new stubbornBox(PxTransform(PxVec3(25.0f - 0.125f, 35.5f, -15.0f)), PxVec3(0.25f, 35.f, 1.0f), PxReal(1.0f));
			box3->Color(PxVec3(175.0f / 255.f, 175.0f / 255.f, 175.0f / 255.f));
			box3->Name("LeftFloor");
			Add(box3);

			box3 = new stubbornBox(PxTransform(PxVec3(-25.0f - 0.125f, 35.5f, -15.0f)), PxVec3(0.25f, 35.f, 1.0f), PxReal(1.0f));
			box3->Color(PxVec3(175.0f / 255.f, 175.0f / 255.f, 175.0f / 255.f));
			box3->Name("RightFloor");
			Add(box3);

			box3 = new stubbornBox(PxTransform(PxVec3(0.0f, 70.5f, -15.0f)), PxVec3(25.0f, 0.25f, 1.0f), PxReal(1.0f));
			box3->Color(PxVec3(175.0f / 255.f, 175.0f / 255.f, 175.0f / 255.f));
			box3->Name("TopFloor");
			Add(box3);

			/*
			//joint two boxes together
			//the joint is fixed to the centre of the first box, oriented by 90 degrees around the Y axis
			//and has the second object attached 5 meters away along the Y axis from the first object.
			RevoluteJoint joint(box, PxTransform(PxVec3(0.f,0.f,0.f),PxQuat(PxPi/2,PxVec3(0.f,1.f,0.f))), box2, PxTransform(PxVec3(0.f,5.f,0.f)));
			*/
			D6 joint(NULL, PxTransform(PxVec3(-10.0f, 2.0f, 0.0f)), paddle1, PxTransform(PxVec3(0.f, 0.f, 15.f), PxQuat(PxPi / 8, PxVec3(0.f, 0.f, 1.f))));
			D6 joint2(NULL, PxTransform(PxVec3(10.0f, 2.0f, 0.0f)), paddle2, PxTransform(PxVec3(0.f, 0.f, 15.f), PxQuat(PxPi / 8, PxVec3(0.f, 0.f, -1.f))));
			D6 joint3(NULL, PxTransform(PxVec3(-10.0f, 2.0f, 0.0f)), ball, PxTransform(PxVec3(0.f, 0.f, 15.f)));
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
			/*PxRigidDynamic* px_actor = (PxRigidDynamic*)paddle1->Get();
			PxVec3 paddle1Pos = px_actor->getGlobalPose().p;
			if (paddle1Pos.x < -30)
			{
				paddle1Pos.x = -30;
			}
				px_actor->setGlobalPose(PxTransform(paddle1Pos));*/

			PxRigidDynamic* px_actor = (PxRigidDynamic*)ball->Get();
			px_actor->addForce(PxVec3(0, 0.5, 0));
			/*if(px_actor->getLinearVelocity().x > 0)
				px_actor->addForce(PxVec3(-0.0, 0, 0));
			else
				px_actor->addForce(PxVec3(0.0, 0, 0));*/

			if (px_actor->getGlobalPose().p.y < 1.8)
				if (px_actor->getGlobalPose().p.x > 0)
				{
					px_actor->setLinearVelocity(PxVec3(0, 0, 0));
					px_actor->setGlobalPose(PxTransform(PxVec3(15.0f, 25.0f, -15.0f)));
					SetScore(1);
				}
				else
				{
					px_actor->setLinearVelocity(PxVec3(0, 0, 0));
					px_actor->setGlobalPose(PxTransform(PxVec3(-15.0f, 25.0f, -15.0f)));
					SetScore(2);
				}
		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			cerr << "I am realeased!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}
	};
}
