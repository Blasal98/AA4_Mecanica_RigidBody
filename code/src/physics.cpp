#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include <time.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <glm\gtc\quaternion.hpp>
#include <glm\gtx\quaternion.hpp>
#include <Windows.h>

//Exemple
extern void Exemple_GUI();
extern void Exemple_PhysicsInit();
extern void Exemple_PhysicsUpdate(float dt);
extern void Exemple_PhysicsCleanup();

bool show_test_window = false;
extern const float halfW = 0.5f;

extern bool renderCube;
namespace Cube {

	extern void setupCube();
	extern void cleanupCube();
	extern void updateCube(const glm::mat4& transform);
	//extern void drawCube();

	glm::vec3 scale = glm::vec3(1, 1, 1);
	int mass = 1;
	glm::vec3 gravity = glm::vec3(0, -9.81f, 0) * (float)mass;

	//GUI
	bool showSpecs = true;
	bool gravityONOFF = false;
	bool moveCubeONOFF = true;

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

		glm::vec3 linearMomentum;
		glm::vec3 angularMomentum;
		glm::vec3 torque;
		glm::vec3 totalForce;

		glm::vec3 angularVelocity;

		glm::mat3 inertiaMatrix;
		glm::mat3 inertiaBody;
		//glm::mat3 rotationMatrix;

		glm::vec3 initialForce;
		glm::vec3 initialForcePoint;
		int maxInitialForce = 10;

		std::vector<ForceOnPoint> forces;

		glm::quat mainQuat;
		//glm::quat auxQuat;

		void addRandomForce() {
			initialForce = glm::vec3((rand() % maxInitialForce) - maxInitialForce / 2.f, (rand() % maxInitialForce) - maxInitialForce / 2.f, (rand() % maxInitialForce) - maxInitialForce / 2.f);
			//initialForce = glm::vec3(0,0,5);
			
			initialForcePoint = glm::toMat3(mainQuat) * glm::vec3((float) rand()/RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX) + position;
			//initialForcePoint = glm::toMat3(mainQuat) * glm::vec3(0.5f, 0.5f, 0.5f) + position;
			forces.push_back(ForceOnPoint(initialForce, initialForcePoint));

			//std::cout << "Initial Force Point: " << initialForcePoint.x << " " << initialForcePoint.y << " " << initialForcePoint.z << std::endl;
		}
		void cubeReset() {
			
			//position = glm::vec3(rand() % 7 - 3, rand() % 7 + 2, rand() % 7 - 3);
			position = glm::vec3(0,5,0);

			//auxQuat = glm::quat(glm::cos(glm::radians(45.f)*0.5f), 0, 0, 1 * glm::sin(glm::radians(45.f)*0.5f));
			mainQuat = glm::quat(glm::cos(glm::radians(45.f)*0.5f), 0, 0, 1 * glm::sin(glm::radians(45.f)*0.5f)); //glm::normalize(auxQuat);

			velocity = glm::vec3(0, 0, 0);
			linearMomentum = glm::vec3(0, 0, 0);
			angularMomentum = glm::vec3(0, 0, 0);
			torque = glm::vec3(0, 0, 0);
			totalForce = glm::vec3(0, 0, 0);
			inertiaBody = glm::mat3(1.f / 12.f * 2.f);
			forces.clear();
			initialForce = glm::vec3(0, 0, 0);
			//addRandomForce();
		}

		CubeStruct() {
			cubeReset();
		}
	};

}
Cube::CubeStruct *ourCube;
Cube::CubeStruct *auxCube;

float ourDt;
float newDT;
float nextDT;

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
		return (pointCheck.x + 5) + planeD(normal,pointPlane) == 0;
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


bool detectColision() {

	
	if (ourCube->position != ourCube->lastPosition) {
		for (int i = 0; i < 8; i++) {
			myData::auxPREcubeVerts[i] = myData::auxCubeVerts[i];
			myData::auxCubeVerts[i] = glm::toMat3(ourCube->mainQuat) * myData::initialCubeVerts[i] + ourCube->position;
			//myData::PREcubeVerts[i] += ourCube->position;
			
		}


		for (int i = 0; i < 8; i++) {
			//                              inicial                                                                                    final
			if (((glm::dot(myData::XZn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::XZn, myData::aux))*(glm::dot(myData::XZn, myData::auxCubeVerts[i]) + myData::planeD(myData::XZn, myData::aux))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision GROUND-" << std::endl;
				myData::vColision[i] = true;
				myData::whichPlane = 1;
				return true;
			}
			
			//plano derecha
			if (((glm::dot(myData::negYZn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::negYZn, myData::aux3))*(glm::dot(myData::negYZn, myData::auxCubeVerts[i]) + myData::planeD(myData::negYZn, myData::aux3))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision RIGHT-" << std::endl;
				myData::vColision[i] = true;
				myData::whichPlane = 2;
				return true;
			}

			//plano delante
			if (((glm::dot(myData::negXYn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::negXYn, myData::aux3))*(glm::dot(myData::negXYn, myData::auxCubeVerts[i]) + myData::planeD(myData::negXYn, myData::aux3))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision FRONT-" << std::endl;
				myData::vColision[i] = true;
				myData::whichPlane = 3;
				return true;
			}

			//plano detras
			if (((glm::dot(myData::XYn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::XYn, myData::aux2))*(glm::dot(myData::XYn, myData::auxCubeVerts[i]) + myData::planeD(myData::XYn, myData::aux2))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision BACK-" << std::endl;
				myData::vColision[i] = true;
				myData::whichPlane = 4;
				return true;
			}

			//plano izquierda
			if (((glm::dot(myData::YZn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::YZn, myData::aux2))*(glm::dot(myData::YZn, myData::auxCubeVerts[i]) + myData::planeD(myData::YZn, myData::aux2))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision LEFT-" << std::endl;
				myData::vColision[i] = true;
				myData::whichPlane = 5;
				return true;	
			}
			
			//plano arriba
			if (((glm::dot(myData::negXZn, myData::auxPREcubeVerts[i]) + myData::planeD(myData::negXZn, myData::aux4))*(glm::dot(myData::negXZn, myData::auxCubeVerts[i]) + myData::planeD(myData::negXZn, myData::aux4))) <= 0) {
				std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++-->colision ROOF-" << std::endl;
				myData::vColision[i] = true;
				myData::whichPlane = 6;
				return true;
			}

		}
	}
	

	return false;

}

void printSpecs() {
	if (Cube::showSpecs) {
		std::cout << "--------------------------------------------------------------------" << std::endl;
		//std::cout << "Position: " << ourCube->position.x << " " << ourCube->position.y << " " << ourCube->position.z << std::endl;
		//std::cout << "Rotation: " << ourCube->rotation.x << " " << ourCube->rotation.y << " " << ourCube->rotation.z << std::endl;
		//std::cout << "Velocity: " << ourCube->velocity.x << " " << ourCube->velocity.y << " " << ourCube->velocity.z << std::endl;
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
		std::cout << std::endl;
		std::cout << "Linear Momentum(J): " << "(" << ourCube->linearMomentum.x << "/" << ourCube->linearMomentum.y << "/" << ourCube->linearMomentum.z << ")" << std::endl;
		std::cout << "Angular Momentum(auxTorque): " << "(" << ourCube->angularMomentum.x << "/" << ourCube->angularMomentum.y << "/" << ourCube->angularMomentum.z << ")" << std::endl;
		std::cout << "MassCenter POSITION: ";
		std::cout << "(" << ourCube->position.x << "/" << ourCube->position.y << "/" << ourCube->position.z << ")" << std::endl;
		std::cout << std::endl;
		std::cout << "VERTEX POSITIONS: " << std::endl;
		for (int i = 0; i < 8; i++) {
			std::cout << "(" << myData::auxCubeVerts[i].x << "/" << myData::auxCubeVerts[i].y << "/" << myData::auxCubeVerts[i].z << ")" << std::endl;
		}
		std::cout << "--------------------------------------------------------------------" << std::endl;
	}
}

void MyPhysicsInit() {
	srand(time(NULL));

	renderCube = true; //activem cub
	Cube::setupCube(); //el setejem
	ourCube = new Cube::CubeStruct(); //el creem
	auxCube = new Cube::CubeStruct(); 
	
	glm::mat4 translation = glm::translate(glm::mat4(), ourCube->position);
	glm::mat4 scale = glm::scale(glm::mat4(), Cube::scale);
	
	//Cube::updateCube(translation * glm::toMat4(ourCube->mainQuat)/* * scale*/);
	ourCube->lastPosition = ourCube->position;
	myData::vColision = new bool[8];
	
	for (int i = 0; i < 8; i++) {
		myData::cubeVerts[i] += ourCube->position;
		myData::auxCubeVerts[i] += ourCube->position;
		myData::vColision[i] = false;
	}
	printSpecs();
}

void updateAux(float dt) {
	auxCube->linearMomentum = myData::linealReference;

	auxCube->linearMomentum = ourCube->linearMomentum + dt * ourCube->totalForce;
	auxCube->velocity = auxCube->linearMomentum / (float)Cube::mass;

	auxCube->lastPosition = auxCube->position;//esto igual se quita
	auxCube->position = ourCube->position + dt * auxCube->velocity;

	if (detectColision()) {
		updateAux(dt / 2.f);
	}
	else {
		//setear los nuevos datos que hemos calculado en el cubo bueno para sacar lo demas
		//ourCube = auxCube;
		ourCube->position = auxCube->position;
		ourCube->linearMomentum = auxCube->linearMomentum;
		ourCube->velocity = auxCube->velocity;

		ourCube->inertiaMatrix = glm::toMat3(ourCube->mainQuat) * glm::inverse(ourCube->inertiaBody) * glm::transpose(glm::toMat3(ourCube->mainQuat));
		ourCube->angularVelocity = ourCube->inertiaMatrix * ourCube->angularMomentum;

		//Rotacio
		glm::quat auxAngVel = glm::quat(0, ourCube->angularVelocity);
		glm::quat dQuat = (1.f / 2.f) * auxAngVel * ourCube->mainQuat;
		ourCube->mainQuat = glm::normalize(ourCube->mainQuat + dt * dQuat);
		//std::cout << glm::length(ourCube->mainQuat) << std::endl;
		//std::cout << "dQuat: " << dQuat[0] << " " << dQuat[1] << " " << dQuat[2] << " " << dQuat[3] << std::endl;

		//updateCube?????


		//PASO 2 calcular nueva posicion despues de colisionar
		nextDT = ourDt - dt; //calculamos el tiempo que nos queda disponible

		int i = 0;
		bool out = false;
		while (i < 8 || out) {
			if (myData::vColision[i]) {
				out = false;
			}
			i++;
		}
		float e = 1.f;


		if (myData::whichPlane == 1) {//GROUND
			glm::vec3 primaPaquita = ourCube->velocity + glm::cross(ourCube->angularVelocity, (myData::auxCubeVerts[i] - ourCube->position));

			glm::vec3 vRel = myData::XZn*primaPaquita;

			//glm::vec3 j = (-(1 + e)*vRel / ((1 / (float)Cube::mass) + glm::cross(myData::XZn * (ourCube->inertiaMatrix * glm::cross(myData::auxCubeVerts[i], myData::XZn)), myData::auxCubeVerts[i])));
			glm::vec3 j = (-(1 + e)*vRel / ((1 / (float)Cube::mass) + glm::cross(myData::XZn * (ourCube->inertiaMatrix * glm::cross(glm::toMat3(ourCube->mainQuat)*myData::initialCubeVerts[i] * ourCube->position, myData::XZn)), myData::auxCubeVerts[i])));

			glm::vec3 J = j * myData::XZn;
			glm::vec3 auxTorque = glm::cross(J, myData::auxCubeVerts[i]);
			ourCube->angularMomentum += auxTorque;
			ourCube->linearMomentum += J;
		}

		if (myData::whichPlane == 2) {//DERECHA
			glm::vec3 primaPaquita = ourCube->velocity + glm::cross(ourCube->angularVelocity, (myData::auxCubeVerts[i] - ourCube->position));

			glm::vec3 vRel = myData::negYZn*primaPaquita;

			glm::vec3 j = (-(1 + e)*vRel / ((1 / (float)Cube::mass) + glm::cross(myData::negYZn * (ourCube->inertiaMatrix * glm::cross(glm::toMat3(ourCube->mainQuat)*myData::initialCubeVerts[i] * ourCube->position, myData::negYZn)), myData::auxCubeVerts[i])));

			glm::vec3 J = j * myData::negYZn;
			glm::vec3 auxTorque = glm::cross(J, myData::auxCubeVerts[i]);
			ourCube->angularMomentum += auxTorque;
			ourCube->linearMomentum += J;
		}

		if (myData::whichPlane == 3) {//DELANTE
			glm::vec3 primaPaquita = ourCube->velocity + glm::cross(ourCube->angularVelocity, (myData::auxCubeVerts[i] - ourCube->position));

			glm::vec3 vRel = myData::negXYn*primaPaquita;

			glm::vec3 j = (-(1 + e)*vRel / ((1 / (float)Cube::mass) + glm::cross(myData::negXYn * (ourCube->inertiaMatrix * glm::cross(glm::toMat3(ourCube->mainQuat)*myData::initialCubeVerts[i] * ourCube->position, myData::negXYn)), myData::auxCubeVerts[i])));

			glm::vec3 J = j * myData::negXYn;
			glm::vec3 auxTorque = glm::cross(J, myData::auxCubeVerts[i]);
			ourCube->angularMomentum += auxTorque;
			ourCube->linearMomentum += J;
		}

		if (myData::whichPlane == 4) {//DETRAS
			glm::vec3 primaPaquita = ourCube->velocity + glm::cross(ourCube->angularVelocity, (myData::auxCubeVerts[i] - ourCube->position));

			glm::vec3 vRel = myData::XYn*primaPaquita;

			glm::vec3 j = (-(1 + e)*vRel / ((1 / (float)Cube::mass) + glm::cross(myData::XYn * (ourCube->inertiaMatrix * glm::cross(glm::toMat3(ourCube->mainQuat)*myData::initialCubeVerts[i] * ourCube->position, myData::XYn)), myData::auxCubeVerts[i])));

			glm::vec3 J = j * myData::XYn;
			glm::vec3 auxTorque = glm::cross(J, myData::auxCubeVerts[i]);
			ourCube->angularMomentum += auxTorque;
			ourCube->linearMomentum += J;
		}

		if (myData::whichPlane == 5) {//IZQUIERDA
			glm::vec3 primaPaquita = ourCube->velocity + glm::cross(ourCube->angularVelocity, (myData::auxCubeVerts[i] - ourCube->position));

			glm::vec3 vRel = myData::YZn*primaPaquita;

			glm::vec3 j = (-(1 + e)*vRel / ((1 / (float)Cube::mass) + glm::cross(myData::YZn * (ourCube->inertiaMatrix * glm::cross(glm::toMat3(ourCube->mainQuat)*myData::initialCubeVerts[i] * ourCube->position, myData::YZn)), myData::auxCubeVerts[i])));

			glm::vec3 J = j * myData::YZn;
			glm::vec3 auxTorque = glm::cross(J, myData::auxCubeVerts[i]);
			ourCube->angularMomentum += auxTorque;
			ourCube->linearMomentum += J;
		}

		if (myData::whichPlane == 6) {//ARRIBA
			glm::vec3 primaPaquita = ourCube->velocity + glm::cross(ourCube->angularVelocity, (myData::auxCubeVerts[i] - ourCube->position));

			glm::vec3 vRel = myData::negXZn*primaPaquita;

			glm::vec3 j = (-(1 + e)*vRel / ((1 / (float)Cube::mass) + glm::cross(myData::negXZn * (ourCube->inertiaMatrix * glm::cross(glm::toMat3(ourCube->mainQuat)*myData::initialCubeVerts[i] * ourCube->position, myData::negXZn)), myData::auxCubeVerts[i])));

			glm::vec3 J = j * myData::negXZn;
			glm::vec3 auxTorque = glm::cross(J, myData::auxCubeVerts[i]);
			ourCube->angularMomentum += auxTorque;
			ourCube->linearMomentum += J;
		}
	


		//Dibujo del Cubo Y Datos
		glm::mat4 translation = glm::translate(glm::mat4(), ourCube->position);
		Cube::updateCube(translation * glm::toMat4(ourCube->mainQuat));
	}

}

void MyPhysicsUpdate(float dt) {
	ourDt = dt;
	newDT;
	nextDT;

	//Save data for the colision
	myData::linealReference = ourCube->linearMomentum;
	auxCube->velocity = ourCube->velocity;
	auxCube->position = ourCube->position;

	if (renderCube) {

		ourCube->totalForce = glm::vec3(0,0,0);
		ourCube->torque = glm::vec3(0,0,0);
		if (Cube::gravityONOFF)
			ourCube->forces.push_back(Cube::ForceOnPoint(Cube::gravity, ourCube->position));
		for (int i = 0; i < ourCube->forces.size(); i++) {
			ourCube->totalForce += ourCube->forces.at(i).force;
			ourCube->torque += glm::cross(ourCube->forces.at(i).point - ourCube->position, ourCube->forces[i].force);
		}
		ourCube->forces.clear();

		ourCube->linearMomentum += ourDt * ourCube->totalForce;
		ourCube->angularMomentum += ourDt * ourCube->torque;
		ourCube->velocity = ourCube->linearMomentum / (float)Cube::mass;
		if (Cube::moveCubeONOFF) {
			ourCube->lastPosition = ourCube->position;//guardamos la pos anterior para ver si ha cambiado mas adelante
			ourCube->position += ourDt * ourCube->velocity;
		}

		bool col = detectColision();
		if (col) {
			updateAux(dt / 2.f);
			//system("pause");
		}
		else {
			ourCube->inertiaMatrix = glm::toMat3(ourCube->mainQuat) * glm::inverse(ourCube->inertiaBody) * glm::transpose(glm::toMat3(ourCube->mainQuat));
			ourCube->angularVelocity = ourCube->inertiaMatrix * ourCube->angularMomentum;

			//Rotacio
			glm::quat auxAngVel = glm::quat(0, ourCube->angularVelocity);
			glm::quat dQuat = (1.f / 2.f) * auxAngVel * ourCube->mainQuat;
			ourCube->mainQuat = glm::normalize(ourCube->mainQuat + ourDt * dQuat);
			//std::cout << glm::length(ourCube->mainQuat) << std::endl;
			//std::cout << "dQuat: " << dQuat[0] << " " << dQuat[1] << " " << dQuat[2] << " " << dQuat[3] << std::endl;

			//Dibujo del Cubo Y Datos
			glm::mat4 translation = glm::translate(glm::mat4(), ourCube->position);
			Cube::updateCube(translation * glm::toMat4(ourCube->mainQuat));
		}
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
	if(ImGui::Button("Reset")) {
		ourCube->cubeReset();
	}
	if (ImGui::Button("Add Random Force")) {
		ourCube->addRandomForce();
	}
	ImGui::SliderInt("Force Intensity", &ourCube->maxInitialForce, 0, 200);
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

