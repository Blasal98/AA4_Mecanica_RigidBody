#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include <time.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <glm\gtc\quaternion.hpp>
#include <glm\gtx\quaternion.hpp>

//Exemple
extern void Exemple_GUI();
extern void Exemple_PhysicsInit();
extern void Exemple_PhysicsUpdate(float dt);
extern void Exemple_PhysicsCleanup();

bool show_test_window = false;
extern const float halfW = 0.5f;

extern bool renderCube;
bool detectionDone = false;


namespace myData {
	float indiceRebote = 2;

	//planos
	glm::vec3 XYn = glm::vec3(0, 0, 1);//normal de plano XY es Z
	glm::vec3 YZn = glm::vec3(1, 0, 0);
	glm::vec3 XZn = glm::vec3(0, 1, 0);
	glm::vec3 negYZn = glm::vec3(-1, 0, 0);
	glm::vec3 negXYn = glm::vec3(0, 0, -1);
	glm::vec3 negXZn = glm::vec3(0, -1, 0);

	glm::vec3 aux = glm::vec3(-5, 0, -5);
	glm::vec3 aux2 = glm::vec3(-5, 0, 5);
	glm::vec3 aux3 = glm::vec3(5, 0, -5);
	glm::vec3 aux4 = glm::vec3(-5, 10, -5);
	glm::vec3 vX1 = aux - aux2;
	glm::vec3 vX2 = aux - aux3;

	glm::vec3 cubeVerts[] = {
		glm::vec3(-halfW, -halfW, -halfW),
		glm::vec3(-halfW, -halfW,  halfW),
		glm::vec3(halfW, -halfW,  halfW) ,
		glm::vec3(halfW, -halfW, -halfW) ,
		glm::vec3(-halfW,  halfW, -halfW),
		glm::vec3(-halfW,  halfW,  halfW),
		glm::vec3(halfW,  halfW,  halfW) ,
		glm::vec3(halfW,  halfW, -halfW)
	};

	glm::vec3 PREcubeVerts[] = {
		glm::vec3(-halfW, -halfW, -halfW),
		glm::vec3(-halfW, -halfW,  halfW),
		glm::vec3(halfW, -halfW,  halfW) ,
		glm::vec3(halfW, -halfW, -halfW) ,
		glm::vec3(-halfW,  halfW, -halfW),
		glm::vec3(-halfW,  halfW,  halfW),
		glm::vec3(halfW,  halfW,  halfW) ,
		glm::vec3(halfW,  halfW, -halfW)
	};

	glm::vec3 initialCubeVerts[] = {
		glm::vec3(-halfW, -halfW, -halfW),
		glm::vec3(-halfW, -halfW,  halfW),
		glm::vec3(halfW, -halfW,  halfW) ,
		glm::vec3(halfW, -halfW, -halfW) ,
		glm::vec3(-halfW,  halfW, -halfW),
		glm::vec3(-halfW,  halfW,  halfW),
		glm::vec3(halfW,  halfW,  halfW) ,
		glm::vec3(halfW,  halfW, -halfW)
	};

	glm::vec3 auxCubeVerts[] = {
		glm::vec3(-halfW, -halfW, -halfW),
		glm::vec3(-halfW, -halfW,  halfW),
		glm::vec3(halfW, -halfW,  halfW) ,
		glm::vec3(halfW, -halfW, -halfW) ,
		glm::vec3(-halfW,  halfW, -halfW),
		glm::vec3(-halfW,  halfW,  halfW),
		glm::vec3(halfW,  halfW,  halfW) ,
		glm::vec3(halfW,  halfW, -halfW)
	};

	glm::vec3 auxPREcubeVerts[] = {
		glm::vec3(-halfW, -halfW, -halfW),
		glm::vec3(-halfW, -halfW,  halfW),
		glm::vec3(halfW, -halfW,  halfW) ,
		glm::vec3(halfW, -halfW, -halfW) ,
		glm::vec3(-halfW,  halfW, -halfW),
		glm::vec3(-halfW,  halfW,  halfW),
		glm::vec3(halfW,  halfW,  halfW) ,
		glm::vec3(halfW,  halfW, -halfW)
	};

	glm::vec3 linealReference;
	glm::vec3 positionReference;
	bool *vColision;
	int whichPlane = 0;

	/*
	//Els 2 serien el costat del cub/2
	float cubeSize = 0.5f;
	//Vertices del cubo
	// 1 1 1 es derecha abajo delante
	// x y z
	glm::vec3 aux111 = ourCube->position + glm::vec3(cubeSize, -cubeSize, -cubeSize); //derecha abajo delante
	glm::vec3 aux112 = ourCube->position + glm::vec3(cubeSize, -cubeSize, cubeSize); //derecha abajo detras
	glm::vec3 aux122 = ourCube->position + glm::vec3(cubeSize, cubeSize, cubeSize); //derecha arriba detras
	glm::vec3 aux121 = ourCube->position + glm::vec3(cubeSize, cubeSize, -cubeSize); //derecha arriba delante
	glm::vec3 aux211 = ourCube->position + glm::vec3(-cubeSize, -cubeSize, -cubeSize); //izquierda abajo delante
	glm::vec3 aux212 = ourCube->position + glm::vec3(-cubeSize, -cubeSize, cubeSize); //izquierda abajo detras
	glm::vec3 aux221 = ourCube->position + glm::vec3(-cubeSize, cubeSize, -cubeSize); //izquierda arriba delante
	glm::vec3 aux222 = ourCube->position + glm::vec3(-cubeSize, cubeSize, cubeSize); //izquierda arriba detras
	*/

	bool colisions = true;

	float planeD(glm::vec3 normal, glm::vec3 point) {
		return -(normal.x*point.x + normal.y*point.y + normal.z*point.z);
	}

	/*bool checkPointOnPlane(glm::vec3 normal, glm::vec3 pointPlane, glm::vec3 pointCheck) {
		return ((normal.x*pointCheck.x) + (normal.y*pointCheck.y) + (normal.z*pointCheck.z) + planeD(normal, pointPlane)) == 0;
	}*/
	/*bool checkPointOnPlane(glm::vec3 normal, glm::vec3 pointPlane, glm::vec3 pointCheck) {
		return ((normal.x*pointCheck.x) + (normal.y*pointCheck.y) + (normal.z*pointCheck.z) + planeD(normal, pointCheck)) == 0;
	}*/
	bool checkPointOnPlaneGround(glm::vec3 pointCheck) {
		//eq del plano suelo: x + 5 = 0
		return (pointCheck.x + 5) == 0;
	}
	bool checkPointOnPlaneGround(glm::vec3 normal, glm::vec3 pointPlane, glm::vec3 pointCheck) {
		//eq del plano suelo: x + 5 = 0
		return (pointCheck.x + 5) + planeD(normal, pointPlane) == 0;
	}
	float checkPointOnPlaneGroundFD(glm::vec3 normal, glm::vec3 pointPlane, glm::vec3 pointCheck) {
		//eq del plano suelo: x + 5 = 0
		return (pointCheck.x + 5) + planeD(normal, pointPlane);
	}
	float checkPointOnPlaneGroundF(glm::vec3 normal, glm::vec3 pointPlane, glm::vec3 pointCheck) {
		//eq del plano suelo: x + 5 = 0
		return (pointCheck.x + 5);
	}
	int checkPointOnPlaneGroundI(glm::vec3 normal, glm::vec3 pointPlane, glm::vec3 pointCheck) {
		//eq del plano suelo: x + 5 = 0
		return (pointCheck.x + 5) + planeD(normal, pointPlane);
	}

}

namespace Cube {

	extern void setupCube();
	extern void cleanupCube();
	extern void updateCube(const glm::mat4& transform);
	//extern void drawCube();

	glm::vec3 scale = glm::vec3(1, 1, 1);
	int mass = 1;
	glm::vec3 gravity = glm::vec3(0, -9.81f, 0) * (float)mass;

	//GUI
	bool showSpecs = false;
	bool gravityONOFF = false;
	bool moveCubeONOFF = true;
	bool randomSTART = true;
	glm::vec3 notRandom_EulerAngles;
	glm::vec3 notRandom_Position = glm::vec3(0,5,0);

	bool collisionONOFF = false;
	bool collisionHARDCODED = true;


	struct ForceOnPoint {
		glm::vec3 force;
		glm::vec3 point;

		ForceOnPoint(glm::vec3 _force, glm::vec3 _point) {
			force = _force;
			point = _point;
		}
	};

	struct CubeStruct {
		glm::vec3 position;
		glm::vec3 lastPosition;
		glm::vec3 initialAxis;
		glm::vec3 velocity;
		glm::vec3 lastVelocity;

		glm::vec3 linearMomentum;
		glm::vec3 lastLinearMomentum;
		glm::vec3 angularMomentum;
		glm::vec3 lastAngularMomentum;
		glm::vec3 torque;
		glm::vec3 totalForce;

		glm::vec3 angularVelocity;
		glm::vec3 lastAngularVelocity;

		glm::mat3 inertiaMatrix;
		glm::mat3 inertiaBody;
		//glm::mat3 rotationMatrix;

		glm::vec3 initialForce;
		glm::vec3 initialForcePoint;
		int maxInitialForce = 10;

		std::vector<ForceOnPoint> forces;

		glm::quat mainQuat;
		glm::quat lastQuat;
		
		std::vector<glm::vec3> vertexs;
		std::vector<glm::vec3> vertexsLast;
		std::vector<glm::vec3> vertexsLocal;

		float elasticity = 1;

		//bool colliding;

		//bool wasCollision;

		void updateVertexs() {
			for (int i = 0; i < 8; i++) {
				vertexs[i] = glm::toMat3(mainQuat) * vertexsLocal[i] + position;
			}
		}
		

		void addRandomForce() {
			initialForce = glm::vec3((rand() % maxInitialForce) - maxInitialForce / 2.f, (rand() % maxInitialForce) - maxInitialForce / 2.f, (rand() % maxInitialForce) - maxInitialForce / 2.f);
			//initialForce = glm::vec3(5,0,0);
			
			initialForcePoint = glm::toMat3(mainQuat) * glm::vec3((float) rand()/RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX) + position;
			//initialForcePoint = glm::toMat3(mainQuat) * glm::vec3(0.5f, 0.5f, 0.f) + position;
			forces.push_back(ForceOnPoint(initialForce, initialForcePoint));

			//std::cout << "Initial Force Point: " << initialForcePoint.x << " " << initialForcePoint.y << " " << initialForcePoint.z << std::endl;
		}
		void cubeReset() {
			
			if (Cube::randomSTART) {
				position = glm::vec3(rand() % 7 - 3, rand() % 7 + 2, rand() % 7 - 3);
				glm::vec3 auxVec3 = glm::normalize(glm::vec3((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX));
				float auxAngle = glm::radians((float)(rand() % 360));
				mainQuat = glm::quat(glm::cos(auxAngle*0.5f)
					, auxVec3.x * glm::sin(auxAngle*0.5f)
					, auxVec3.y * glm::sin(auxAngle*0.5f)
					, auxVec3.z * glm::sin(auxAngle*0.5f));
			}
			else {
				position = notRandom_Position;
				//mainQuat = glm::toQuat(auxMatrix);
				glm::vec3 EulerInR = glm::vec3(glm::radians(notRandom_EulerAngles.x), glm::radians(notRandom_EulerAngles.y), glm::radians(notRandom_EulerAngles.z));
				float c1,c2,c3,s1,s2,s3;
				c1 = glm::cos(EulerInR.y / 2.f);
				c2 = glm::cos(EulerInR.z / 2.f);
				c3 = glm::cos(EulerInR.x / 2.f);
				s1 = glm::sin(EulerInR.y / 2.f);
				s2 = glm::sin(EulerInR.z / 2.f);
				s3 = glm::sin(EulerInR.x / 2.f);
				mainQuat[3] = c1 * c2 * c3 - s1 * s2 * s3;
				mainQuat[0] = s1 * s2 * c3 + c1 * c2 * s3;
				mainQuat[1] = s1 * c2 * c3 + c1 * s2 * s3;
				mainQuat[2] = c1 * s2 * c3 - s1 * c2 * s3;
			}

			velocity = glm::vec3(0, 0, 0);
			linearMomentum = glm::vec3(0, 0, 0);
			angularMomentum = glm::vec3(0, 0, 0);
			torque = glm::vec3(0, 0, 0);
			totalForce = glm::vec3(0, 0, 0);
			inertiaBody = glm::mat3(1.f / 12.f * 2.f);
			forces.clear();
			initialForce = glm::vec3(0, 0, 0);
			//addRandomForce();
			
			vertexs = vertexsLocal;
			//wasCollision = false;
			//colliding = false;

			lastQuat = mainQuat;
			lastAngularVelocity = angularVelocity;
			lastVelocity = velocity;
			vertexsLast = vertexs;
			lastPosition = position;
			lastAngularMomentum = angularMomentum;
			lastLinearMomentum = linearMomentum;
		}

		CubeStruct() {
			for (int i = 0; i < 8; i++) {
				vertexsLocal.push_back(glm::vec3());
			}

			vertexsLocal[0] = glm::vec3(0.5f, 0.5f, 0.5f);
			vertexsLocal[1] = glm::vec3(-0.5f, 0.5f, 0.5f);
			vertexsLocal[2] = glm::vec3(-0.5f, -0.5f, 0.5f);
			vertexsLocal[3] = glm::vec3(-0.5f, -0.5f, -0.5f);
			vertexsLocal[4] = glm::vec3(0.5f, 0.5f, -0.5f);
			vertexsLocal[5] = glm::vec3(0.5f, -0.5f, -0.5f);
			vertexsLocal[6] = glm::vec3(0.5f, -0.5f, 0.5f);
			vertexsLocal[7] = glm::vec3(-0.5f, 0.5f, -0.5f);

			
			
			cubeReset();
		}
	};

}

Cube::CubeStruct *ourCube;

void detectColision() {


	if (ourCube->position != ourCube->lastPosition) {
		for (int i = 0; i < 8; i++) {
			myData::auxPREcubeVerts[i] = myData::auxCubeVerts[i];
			myData::auxCubeVerts[i] = glm::toMat3(ourCube->mainQuat) * myData::initialCubeVerts[i] + ourCube->position;
			//myData::PREcubeVerts[i] += ourCube->position;

		}
		detectionDone = false;

		for (int i = 0; i < 8; i++) {
			//suelo
			//                              inicial                                                                                    final
			if (((glm::dot(myData::XZn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::XZn, myData::aux))*(glm::dot(myData::XZn, myData::auxCubeVerts[i]) + myData::planeD(myData::XZn, myData::aux))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision GROUND-" << std::endl;
				myData::vColision[i] = true;
				detectionDone = true;
				myData::whichPlane = 1;
				return;
			}

			//plano derecha
			if (((glm::dot(myData::negYZn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::negYZn, myData::aux3))*(glm::dot(myData::negYZn, myData::auxCubeVerts[i]) + myData::planeD(myData::negYZn, myData::aux3))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision RIGHT-" << std::endl;
				myData::vColision[i] = true;
				detectionDone = true;
				myData::whichPlane = 2;
				return;
			}

			//plano delante
			if (((glm::dot(myData::negXYn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::negXYn, myData::aux3))*(glm::dot(myData::negXYn, myData::auxCubeVerts[i]) + myData::planeD(myData::negXYn, myData::aux3))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision FRONT-" << std::endl;
				myData::vColision[i] = true;
				detectionDone = true;
				myData::whichPlane = 3;
				return;
			}

			//plano detras
			if (((glm::dot(myData::XYn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::XYn, myData::aux2))*(glm::dot(myData::XYn, myData::auxCubeVerts[i]) + myData::planeD(myData::XYn, myData::aux2))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision BACK-" << std::endl;
				myData::vColision[i] = true;
				detectionDone = true;
				myData::whichPlane = 4;
				return;
			}

			//plano izquierda
			if (((glm::dot(myData::YZn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::YZn, myData::aux2))*(glm::dot(myData::YZn, myData::auxCubeVerts[i]) + myData::planeD(myData::YZn, myData::aux2))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision LEFT-" << std::endl;
				myData::vColision[i] = true;
				detectionDone = true;
				myData::whichPlane = 5;
				return;
			}

			//plano arriba
			if (((glm::dot(myData::negXZn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::negXZn, myData::aux4))*(glm::dot(myData::negXZn, myData::auxCubeVerts[i]) + myData::planeD(myData::negXZn, myData::aux4))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision ROOF-" << std::endl;
				myData::vColision[i] = true;
				detectionDone = true;
				myData::whichPlane = 6;
				return;
			}

		}
	}

}
void detectCollisions(float dt) {
	std::cout << "Frame | " << std::endl;
	detectionDone = false;
	int auxito = -1;
	int auxIndex;

	detectColision();
	/*for (int i = 0; i < 8; i++) {
		if (vertexs[i].y < 0) {
			if (!detectionDone) {
				detectionDone = true;
				auxIndex = i;
			}
		}
	}*/
	if (detectionDone && Cube::collisionONOFF) {
		if (Cube::collisionHARDCODED) {

			std::cout << "COL";
			glm::vec3 normal = glm::vec3(0, 1, 0);
			glm::vec3 impulse = glm::reflect(ourCube->lastVelocity, normal) * ourCube->elasticity;
			float wSpeed = glm::length(ourCube->lastAngularVelocity);
			float vSpeed = glm::length(ourCube->lastVelocity);

			ourCube->linearMomentum = glm::vec3(0, 0, 0);
			ourCube->angularMomentum = glm::vec3(0, 0, 0);
			ourCube->mainQuat = ourCube->lastQuat;
			ourCube->position = ourCube->lastPosition;
			//velocity = lastVelocity;
			//angularVelocity = lastAngularVelocity;
			ourCube->vertexs = ourCube->vertexsLast;

			ourCube->totalForce = glm::vec3(0, 0, 0);
			ourCube->torque = glm::vec3(0, 0, 0);
			//forces.push_back(ForceOnPoint(gravity, position));
			
			for (int i = 0; i < 8; i++) {
				if (myData::vColision[i]) {
					auxito = i;
				}
			}
			ourCube->forces.push_back(Cube::ForceOnPoint(impulse, ourCube->vertexs[auxito]));

			for (int i = 0; i < ourCube->forces.size(); i++) {
				ourCube->totalForce += ourCube->forces.at(i).force;
				ourCube->torque += glm::cross(ourCube->forces.at(i).point - ourCube->position, ourCube->forces.at(i).force);
			}
			ourCube->forces.clear();

			ourCube->linearMomentum += ourCube->totalForce;
			ourCube->angularMomentum += ourCube->torque * dt;

			ourCube->velocity = ourCube->linearMomentum / (float)Cube::mass;
			ourCube->velocity = glm::normalize(ourCube->velocity) * vSpeed;
			ourCube->position += dt * ourCube->velocity;

			ourCube->inertiaMatrix = glm::toMat3(ourCube->mainQuat) * glm::inverse(ourCube->inertiaBody) * glm::transpose(glm::toMat3(ourCube->mainQuat));
			ourCube->angularVelocity = ourCube->inertiaMatrix * ourCube->angularMomentum;
			//angularVelocity = glm::normalize(angularVelocity) /** wSpeed*/;

			//Rotacio
			glm::quat auxAngVel = glm::quat(0, ourCube->angularVelocity);
			glm::quat dQuat = (1.f / 2.f) * auxAngVel * ourCube->mainQuat;
			ourCube->mainQuat = glm::normalize(ourCube->mainQuat + dt * dQuat);
			//std::cout << glm::length(mainQuat) << std::endl;
			//std::cout << "dQuat: " << dQuat[0] << " " << dQuat[1] << " " << dQuat[2] << " " << dQuat[3] << std::endl;

			//colisions
			ourCube->updateVertexs();
		}
		else {
			ourCube->totalForce = glm::vec3(0, 0, 0);
			ourCube->torque = glm::vec3(0, 0, 0);
			ourCube->forces.push_back(Cube::ForceOnPoint(Cube::gravity, ourCube->position));
			ourCube->totalForce += ourCube->forces.at(0).force;
			//torque += glm::cross(forces.at(0).point - position, forces.at(0).force);
			//forces.clear();

			glm::vec3 linearMomentumStart = ourCube->lastLinearMomentum;
			glm::vec3 angularMomentumStart = ourCube->lastAngularMomentum;
			glm::quat mainQuatStart = ourCube->lastQuat;
			glm::vec3 positionStart = ourCube->lastPosition;
			glm::vec3 velocityStart = ourCube->lastVelocity;
			glm::vec3 angularVelocityStart = ourCube->lastAngularVelocity;
			std::vector<glm::vec3> vertexsStart = ourCube->vertexsLast;


			float interDt = dt / 200;
			float tFound = false;
			float z = 0;
			while (!tFound) {
				ourCube->linearMomentum = linearMomentumStart;
				ourCube->angularMomentum = angularMomentumStart;
				ourCube->mainQuat = mainQuatStart;
				ourCube->position = positionStart;
				ourCube->velocity = velocityStart;
				ourCube->angularVelocity = angularVelocityStart;
				ourCube->vertexs = vertexsStart;

				ourCube->linearMomentum += interDt * ourCube->totalForce;
				//angularMomentum += interDt * torque;

				ourCube->velocity = ourCube->linearMomentum / (float)Cube::mass;
				ourCube->position += interDt * ourCube->velocity;

				ourCube->inertiaMatrix = glm::toMat3(ourCube->mainQuat) * glm::inverse(ourCube->inertiaBody) * glm::transpose(glm::toMat3(ourCube->mainQuat));
				ourCube->angularVelocity = ourCube->inertiaMatrix * ourCube->angularMomentum;

				//Rotacio
				glm::quat auxAngVel = glm::quat(0, ourCube->angularVelocity);
				glm::quat dQuat = (1.f / 2.f) * auxAngVel * ourCube->mainQuat;
				ourCube->mainQuat = glm::normalize(ourCube->mainQuat + interDt * dQuat);

				//colisions
				ourCube->updateVertexs();

				if (ourCube->vertexs[auxito].y < 0) {
					tFound = true;
					std::cout << ourCube->vertexs[auxito].y << std::endl;
					std::cout << ourCube->vertexsLast[auxito].y << std::endl;
					std::cout << z << std::endl;
				}
				else {
					z++;
					ourCube->lastQuat = ourCube->mainQuat;
					ourCube->lastAngularVelocity = ourCube->angularVelocity;
					ourCube->lastVelocity = ourCube->velocity;
					ourCube->vertexsLast = ourCube->vertexs;
					ourCube->lastPosition = ourCube->position;
					ourCube->lastAngularMomentum = ourCube->angularMomentum;
					ourCube->lastLinearMomentum = ourCube->linearMomentum;
					interDt += dt / 200;


				}


			}



			if (myData::whichPlane == 1) {//GROUND
				glm::vec3 normal = glm::vec3(0, 1, 0);
				glm::vec3 vComPdc = ourCube->vertexs[auxito] - ourCube->position;
				glm::vec3 lastComPdc = ourCube->vertexsLast[auxito] - ourCube->lastPosition;
				float auxVel;

				auxVel = glm::dot(ourCube->lastVelocity + glm::cross(ourCube->lastAngularVelocity, lastComPdc), normal);

				float j = -(1 + 1) * auxVel / ((1 / Cube::mass) + glm::dot(normal, glm::cross(ourCube->inertiaMatrix * glm::cross(vComPdc, normal), vComPdc)));

				glm::vec3 J = normal * j;

				//std::cout << "                                                                                " << J.x << " " << J.y << " " << J.z << std::endl;
				//std::cout << "                                                                                " << glm::cross(vComPdc, J).x << " " << glm::cross(vComPdc, J).y << " " << glm::cross(vComPdc, J).z << std::endl;
				//std::cout << "                                                                                " << auxVel << std::endl;

				//std::cout << glm::length();
				ourCube->linearMomentum = ourCube->lastLinearMomentum + J;
				ourCube->angularMomentum = ourCube->lastAngularMomentum + glm::cross(vComPdc, J);
			}

			//Las fromulas de j a partir de aqui estan mal
			if (myData::whichPlane == 2) {//DERECHA
				glm::vec3 normal = glm::vec3(0, 1, 0);
				glm::vec3 vComPdc = ourCube->vertexs[auxito] - ourCube->position;
				glm::vec3 lastComPdc = ourCube->vertexsLast[auxito] - ourCube->lastPosition;
				float auxVel;

				auxVel = glm::dot(ourCube->lastVelocity + glm::cross(ourCube->lastAngularVelocity, lastComPdc), myData::negYZn);

				float j = -(1 + 1) * auxVel / ((1 / Cube::mass) + glm::dot(myData::negYZn, glm::cross(ourCube->inertiaMatrix * glm::cross(vComPdc, myData::negYZn), vComPdc)));

				glm::vec3 J = myData::negYZn * j;

				//std::cout << "                                                                                " << J.x << " " << J.y << " " << J.z << std::endl;
				//std::cout << "                                                                                " << glm::cross(vComPdc, J).x << " " << glm::cross(vComPdc, J).y << " " << glm::cross(vComPdc, J).z << std::endl;
				//std::cout << "                                                                                " << auxVel << std::endl;

				//std::cout << glm::length();
				ourCube->linearMomentum = ourCube->lastLinearMomentum + J;
				ourCube->angularMomentum = ourCube->lastAngularMomentum + glm::cross(vComPdc, J);
				
			}

			if (myData::whichPlane == 3) {//DELANTE
				glm::vec3 normal = glm::vec3(0, 1, 0);
				glm::vec3 vComPdc = ourCube->vertexs[auxito] - ourCube->position;
				glm::vec3 lastComPdc = ourCube->vertexsLast[auxito] - ourCube->lastPosition;
				float auxVel;

				auxVel = glm::dot(ourCube->lastVelocity + glm::cross(ourCube->lastAngularVelocity, lastComPdc), myData::negXYn);

				float j = -(1 + 1) * auxVel / ((1 / Cube::mass) + glm::dot(myData::negXYn, glm::cross(ourCube->inertiaMatrix * glm::cross(vComPdc, myData::negXYn), vComPdc)));

				glm::vec3 J = myData::negXYn * j;

				//std::cout << "                                                                                " << J.x << " " << J.y << " " << J.z << std::endl;
				//std::cout << "                                                                                " << glm::cross(vComPdc, J).x << " " << glm::cross(vComPdc, J).y << " " << glm::cross(vComPdc, J).z << std::endl;
				//std::cout << "                                                                                " << auxVel << std::endl;

				//std::cout << glm::length();
				ourCube->linearMomentum = ourCube->lastLinearMomentum + J;
				ourCube->angularMomentum = ourCube->lastAngularMomentum + glm::cross(vComPdc, J);
			}

			if (myData::whichPlane == 4) {//DETRAS
				
				glm::vec3 normal = glm::vec3(0, 1, 0);
				glm::vec3 vComPdc = ourCube->vertexs[auxito] - ourCube->position;
				glm::vec3 lastComPdc = ourCube->vertexsLast[auxito] - ourCube->lastPosition;
				float auxVel;

				auxVel = glm::dot(ourCube->lastVelocity + glm::cross(ourCube->lastAngularVelocity, lastComPdc), myData::XYn);

				float j = -(1 + 1) * auxVel / ((1 / Cube::mass) + glm::dot(myData::XYn, glm::cross(ourCube->inertiaMatrix * glm::cross(vComPdc, myData::XYn), vComPdc)));

				glm::vec3 J = myData::XYn * j;

				//std::cout << "                                                                                " << J.x << " " << J.y << " " << J.z << std::endl;
				//std::cout << "                                                                                " << glm::cross(vComPdc, J).x << " " << glm::cross(vComPdc, J).y << " " << glm::cross(vComPdc, J).z << std::endl;
				//std::cout << "                                                                                " << auxVel << std::endl;

				//std::cout << glm::length();
				ourCube->linearMomentum = ourCube->lastLinearMomentum + J;
				ourCube->angularMomentum = ourCube->lastAngularMomentum + glm::cross(vComPdc, J);
			}

			if (myData::whichPlane == 5) {//IZQUIERDA
				glm::vec3 normal = glm::vec3(0, 1, 0);
				glm::vec3 vComPdc = ourCube->vertexs[auxito] - ourCube->position;
				glm::vec3 lastComPdc = ourCube->vertexsLast[auxito] - ourCube->lastPosition;
				float auxVel;

				auxVel = glm::dot(ourCube->lastVelocity + glm::cross(ourCube->lastAngularVelocity, lastComPdc), myData::YZn);

				float j = -(1 + 1) * auxVel / ((1 / Cube::mass) + glm::dot(myData::YZn, glm::cross(ourCube->inertiaMatrix * glm::cross(vComPdc, myData::YZn), vComPdc)));

				glm::vec3 J = myData::YZn * j;

				//std::cout << "                                                                                " << J.x << " " << J.y << " " << J.z << std::endl;
				//std::cout << "                                                                                " << glm::cross(vComPdc, J).x << " " << glm::cross(vComPdc, J).y << " " << glm::cross(vComPdc, J).z << std::endl;
				//std::cout << "                                                                                " << auxVel << std::endl;

				//std::cout << glm::length();
				ourCube->linearMomentum = ourCube->lastLinearMomentum + J;
				ourCube->angularMomentum = ourCube->lastAngularMomentum + glm::cross(vComPdc, J);
			}

			if (myData::whichPlane == 6) {//ARRIBA
				glm::vec3 normal = glm::vec3(0, 1, 0);
				glm::vec3 vComPdc = ourCube->vertexs[auxito] - ourCube->position;
				glm::vec3 lastComPdc = ourCube->vertexsLast[auxito] - ourCube->lastPosition;
				float auxVel;

				auxVel = glm::dot(ourCube->lastVelocity + glm::cross(ourCube->lastAngularVelocity, lastComPdc), myData::negXZn);

				float j = -(1 + 1) * auxVel / ((1 / Cube::mass) + glm::dot(myData::negXZn, glm::cross(ourCube->inertiaMatrix * glm::cross(vComPdc, myData::negXZn), vComPdc)));

				glm::vec3 J = myData::negXZn * j;

				//std::cout << "                                                                                " << J.x << " " << J.y << " " << J.z << std::endl;
				//std::cout << "                                                                                " << glm::cross(vComPdc, J).x << " " << glm::cross(vComPdc, J).y << " " << glm::cross(vComPdc, J).z << std::endl;
				//std::cout << "                                                                                " << auxVel << std::endl;

				//std::cout << glm::length();
				ourCube->linearMomentum = ourCube->lastLinearMomentum + J;
				ourCube->angularMomentum = ourCube->lastAngularMomentum + glm::cross(vComPdc, J);
			}


			

			//velocity = linearMomentum / (float)Cube::mass;
			//position += interDt * velocity;


			//inertiaMatrix = glm::toMat3(mainQuat) * glm::inverse(inertiaBody) * glm::transpose(glm::toMat3(mainQuat));
			//angularVelocity = inertiaMatrix * angularMomentum;

			////Rotacio
			//glm::quat auxAngVel = glm::quat(0, angularVelocity);
			//glm::quat dQuat = (1.f / 2.f) * auxAngVel * mainQuat;
			//mainQuat = glm::normalize(mainQuat + interDt * dQuat);

			//updateVertexs();
		}
	}
}
void printSpecs() {
	if (Cube::showSpecs) {
		std::cout << "--------------------------------------------------------------------" << std::endl;
		std::cout << "Position: " << ourCube->position.x << " " << ourCube->position.y << " " << ourCube->position.z << std::endl;
		//std::cout << "Rotation: " << ourCube->rotation.x << " " << ourCube->rotation.y << " " << ourCube->rotation.z << std::endl;
		std::cout << "Velocity: " << ourCube->velocity.x << " " << ourCube->velocity.y << " " << ourCube->velocity.z << std::endl;
		std::cout << "Force: " << ourCube->totalForce.x << " " << ourCube->totalForce.y << " " << ourCube->totalForce.z << std::endl;
		std::cout << "Torque: " << ourCube->torque.x << " " << ourCube->torque.y << " " << ourCube->torque.z << std::endl;
		//std::cout << "Num Forces: " << ourCube->forces.size() << std::endl;

		std::cout << "Ang Vel: " << ourCube->angularVelocity.x << " " << ourCube->angularVelocity.y << " " << ourCube->angularVelocity.z << std::endl;
		std::cout << "Ang Mom: " << ourCube->angularMomentum.x << " " << ourCube->angularMomentum.y << " " << ourCube->angularMomentum.z << std::endl;
		std::cout << "Lin Mom: " << ourCube->linearMomentum.x << " " << ourCube->linearMomentum.y << " " << ourCube->linearMomentum.z << std::endl;
		std::cout << "Inertia BODY: " << std::endl;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				std::cout << ourCube->inertiaBody[i][j] << " ";
			}
			std::cout << std::endl;
		}
		std::cout << "Inertia MATRIX: " << std::endl;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				std::cout << ourCube->inertiaMatrix[i][j] << " ";
			}
			std::cout << std::endl;
		}
		std::cout << "Rotation MATRIX: " << std::endl;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				std::cout << glm::toMat3(ourCube->mainQuat)[i][j] << " ";
			}
			std::cout << std::endl;
		}
		std::cout << "MainQuat: " << ourCube->mainQuat[0] << " " << ourCube->mainQuat[1] << " " << ourCube->mainQuat[2] << " " << ourCube->mainQuat[3] << std::endl;
		//std::cout << "AuxQuat: " << ourCube->auxQuat[0] << " " << ourCube->auxQuat[1] << " " << ourCube->auxQuat[2] << " " << ourCube->auxQuat[3] << std::endl;

		std::cout << "vertexs:" << std::endl;
		for (int i = 0; i < 8; i++) {
			for (int j = 0; j < 3; j++) {
				std::cout << ourCube->vertexs[i][j] << " ";
			}
			std::cout << std::endl;
		}
		std::cout << "--------------------------------------------------------------------" << std::endl;
	}
}

void MyPhysicsInit() {
	srand(time(NULL));

	renderCube = true; //activem cub
	Cube::setupCube(); //el setejem
	ourCube = new Cube::CubeStruct(); //el creem
	
	glm::mat4 translation = glm::translate(glm::mat4(), ourCube->position);
	glm::mat4 scale = glm::scale(glm::mat4(), Cube::scale);
	myData::vColision = new bool[8];
	for (int i = 0; i < 8; i++) {
		myData::cubeVerts[i] += ourCube->position;
		myData::auxCubeVerts[i] += ourCube->position;
		myData::vColision[i] = false;
	}
	//Cube::updateCube(translation * glm::toMat4(ourCube->mainQuat)/* * scale*/);
	
	printSpecs();
}

void MyPhysicsUpdate(float dt) {
	float ourDt = dt;
	if (renderCube) {
		//if (!ourCube->colliding) {
			//std::cout << "entra" << std::endl;
			ourCube->totalForce = glm::vec3(0, 0, 0);
			ourCube->torque = glm::vec3(0, 0, 0);
			if (Cube::gravityONOFF)
				ourCube->forces.push_back(Cube::ForceOnPoint(Cube::gravity, ourCube->position));
			for (int i = 0; i < ourCube->forces.size(); i++) {
				ourCube->totalForce += ourCube->forces.at(i).force;
				ourCube->torque += glm::cross(ourCube->forces.at(i).point - ourCube->position, ourCube->forces[i].force);
			}
			ourCube->forces.clear();

			if (Cube::moveCubeONOFF) {
				ourCube->linearMomentum += ourDt * ourCube->totalForce;
			}
			else {
				ourCube->linearMomentum = glm::vec3(0, 0, 0);
			}
			ourCube->angularMomentum += ourDt * ourCube->torque;
			ourCube->velocity = ourCube->linearMomentum / (float)Cube::mass;
			if (Cube::moveCubeONOFF /*&& !ourCube->wasCollision*/) {
				ourCube->position += ourDt * ourCube->velocity;
			}
			//else if (ourCube->//wasCollision)
			//	ourCube->wasCollision = false;

			ourCube->inertiaMatrix = glm::toMat3(ourCube->mainQuat) * glm::inverse(ourCube->inertiaBody) * glm::transpose(glm::toMat3(ourCube->mainQuat));
			ourCube->angularVelocity = ourCube->inertiaMatrix * ourCube->angularMomentum;

			//Rotacio
			glm::quat auxAngVel = glm::quat(0, ourCube->angularVelocity);
			glm::quat dQuat = (1.f / 2.f) * auxAngVel * ourCube->mainQuat;
			ourCube->mainQuat = glm::normalize(ourCube->mainQuat + ourDt * dQuat);
			//std::cout << glm::length(ourCube->mainQuat) << std::endl;
			//std::cout << "dQuat: " << dQuat[0] << " " << dQuat[1] << " " << dQuat[2] << " " << dQuat[3] << std::endl;

			//colisions
			ourCube->updateVertexs();
			detectCollisions(ourDt);
		//}
		//else {
		//	ourCube->velocity = ourCube->linearMomentum / (float)Cube::mass;
		//	ourCube->position += ourDt * ourCube->velocity;

		//	ourCube->inertiaMatrix = glm::toMat3(ourCube->mainQuat) * glm::inverse(ourCube->inertiaBody) * glm::transpose(glm::toMat3(ourCube->mainQuat));
		//	ourCube->angularVelocity = ourCube->inertiaMatrix * ourCube->angularMomentum;

		//	//Rotacio
		//	glm::quat auxAngVel = glm::quat(0, ourCube->angularVelocity);
		//	glm::quat dQuat = (1.f / 2.f) * auxAngVel * ourCube->mainQuat;
		//	ourCube->mainQuat = glm::normalize(ourCube->mainQuat + ourDt * dQuat);

		//	ourCube->updateVertexs();
		//	ourCube->colliding = false;
		//	for (int i = 0; i < 8; i++) {
		//		if (ourCube->vertexs[i].y < 0) {
		//			ourCube->colliding = true;
		//		}
		//	}
		//}

		ourCube->lastQuat = ourCube->mainQuat;
		ourCube->lastAngularVelocity = ourCube->angularVelocity;
		ourCube->lastVelocity = ourCube->velocity;
		ourCube->vertexsLast = ourCube->vertexs;
		ourCube->lastPosition = ourCube->position;
		ourCube->lastAngularMomentum = ourCube->angularMomentum;
		ourCube->lastLinearMomentum = ourCube->linearMomentum;

		//Dibujo del Cubo Y Datos
		glm::mat4 translation = glm::translate(glm::mat4(), ourCube->position);
		Cube::updateCube(translation * glm::toMat4(ourCube->mainQuat));
		printSpecs();
	}
}
void MyPhysicsCleanup() {
	Cube::cleanupCube();
	delete ourCube;
}



void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		//Exemple_GUI();
	}
	ImGui::Checkbox("Show Specs", &Cube::showSpecs);
	ImGui::Checkbox("Gravity On/Off", &Cube::gravityONOFF);
	ImGui::Checkbox("Cube moves On/Off", &Cube::moveCubeONOFF);
	ImGui::Checkbox("Collision On/Off", &Cube::collisionONOFF);
	ImGui::Checkbox("Collision HardCoded On/Off", &Cube::collisionHARDCODED);
	if(ImGui::Button("Reset")) {
		ourCube->cubeReset();
	}
	if (ImGui::Button("Add Random Force")) {
		ourCube->addRandomForce();
	}
	ImGui::SliderInt("Force Intensity", &ourCube->maxInitialForce, 0, 200);

	ImGui::Checkbox("Random Star On/Off", &Cube::randomSTART);
	if (!Cube::randomSTART) {
		ImGui::SliderFloat("Starting X Angle", &Cube::notRandom_EulerAngles.x, 0, 360);
		ImGui::SliderFloat("Starting Y Angle", &Cube::notRandom_EulerAngles.y, 0, 360);
		ImGui::SliderFloat("Starting Z Angle", &Cube::notRandom_EulerAngles.z, 0, 360);

		ImGui::SliderFloat("Starting X Position", &Cube::notRandom_Position.x, -3, 3);
		ImGui::SliderFloat("Starting Y Position", &Cube::notRandom_Position.y, 2, 8);
		ImGui::SliderFloat("Starting Z Position", &Cube::notRandom_Position.z, -3, 3);
	}
	if (ImGui::Button("Stop Cube")) {
		ourCube->angularMomentum = glm::vec3(0, 0, 0);
		ourCube->linearMomentum = glm::vec3(0, 0, 0);
		Cube::gravityONOFF = false;
		Cube::moveCubeONOFF = false;
		
	}
	
	ImGui::End();
}

void PhysicsInit() {
	//Exemple_PhysicsInit();
	MyPhysicsInit();
}

void PhysicsUpdate(float dt) {
	//Exemple_PhysicsUpdate(dt);
	MyPhysicsUpdate(dt);
}

void PhysicsCleanup() {
	//Exemple_PhysicsCleanup();
	MyPhysicsCleanup();
}

