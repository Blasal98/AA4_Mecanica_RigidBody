#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include <time.h>
#include <iostream>
#include <vector>
#include <math.h>

//Exemple
extern void Exemple_GUI();
extern void Exemple_PhysicsInit();
extern void Exemple_PhysicsUpdate(float dt);
extern void Exemple_PhysicsCleanup();

bool show_test_window = false;


extern bool renderCube;
namespace Cube {

	extern void setupCube();
	extern void cleanupCube();
	extern void updateCube(const glm::mat4& transform);
	//extern void drawCube();

	glm::vec3 scale = glm::vec3(1, 1, 1);
	int mass = 1;
	glm::vec3 gravity = glm::vec3(0, -9.81f, 0) * (float)mass;
	bool showSpecs = true;

	glm::mat3 setupRotationMatrix(glm::vec3 rotationAngles) { //si falla algo segurament sira aixo

		glm::mat3 rotationX; //= glm::mat3(1,0,0,  0,glm::cos(rotationAngles.x),-glm::sin(rotationAngles.x),  0, glm::sin(rotationAngles.x), glm::cos(rotationAngles.x));
		glm::mat3 rotationY; //= glm::mat3(glm::cos(rotationAngles.y),0, glm::sin(rotationAngles.y),  0,1,0,  -glm::sin(rotationAngles.y),0, glm::cos(rotationAngles.y));
		glm::mat3 rotationZ; //= glm::mat3(glm::cos(rotationAngles.z), -glm::sin(rotationAngles.z),0,  glm::sin(rotationAngles.z), glm::cos(rotationAngles.z),0,  0,0,1);

		rotationX[0][0] = 1; rotationX[0][1] = 0; rotationX[0][2] = 0;
		rotationX[1][0] = 0; rotationX[1][1] = glm::cos(rotationAngles.x); rotationX[1][2] = -glm::sin(rotationAngles.x);
		rotationX[2][0] = 0; rotationX[2][1] = glm::sin(rotationAngles.x); rotationX[2][2] = glm::cos(rotationAngles.x);

		rotationY[0][0] = glm::cos(rotationAngles.y); rotationY[0][1] = 0; rotationY[0][2] = glm::sin(rotationAngles.y);
		rotationY[1][0] = 0; rotationY[1][1] = 1; rotationY[1][2] = 0;
		rotationY[2][0] = -glm::sin(rotationAngles.y); rotationY[2][1] = 0; rotationY[2][2] = glm::cos(rotationAngles.y);

		rotationZ[0][0] = glm::cos(rotationAngles.z); rotationZ[0][1] = -glm::sin(rotationAngles.z); rotationZ[0][2] = 0;
		rotationZ[1][0] = glm::sin(rotationAngles.z); rotationZ[1][1] = glm::cos(rotationAngles.z); rotationZ[1][2] = 0;
		rotationZ[2][0] = 0; rotationZ[2][1] = 0; rotationZ[2][2] = 1;

		return rotationX * rotationY * rotationZ;
	}
	

	glm::mat3 generateWMatrix(glm::vec3 w) {
		glm::mat3 returnMatrix = glm::mat3();
		returnMatrix[0][0] = 0; returnMatrix[0][1] = -w.z; returnMatrix[0][2] = w.y;
		returnMatrix[1][0] = w.z; returnMatrix[1][1] = 0; returnMatrix[1][2] = -w.x;
		returnMatrix[2][0] = -w.y; returnMatrix[2][1] = w.x; returnMatrix[2][2] = 0;

		return returnMatrix;
	}
	glm::mat4 generateUpdateMatrix(glm::mat3 rot) {

		glm::mat4 returnMatrix = glm::mat4();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				returnMatrix[i][j] = rot[i][j];
			}
		}
		returnMatrix[0][1] *= -1;
		returnMatrix[1][0] *= -1;
		returnMatrix[1][2] *= -1;
		returnMatrix[2][1] *= -1;
		return returnMatrix;
	
	}
	/*void clearInertiaMatrix(glm::mat3 &I) {
		I[0][1] = 0; I[0][2] = 0;
		I[1][0] = 0; I[1][2] = 0;
		I[2][0] = 0; I[2][1] = 0;
	}*/

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
		glm::vec3 rotation;
		glm::vec3 velocity;

		glm::vec3 linearMomentum;
		glm::vec3 angularMomentum;
		glm::vec3 torque;
		glm::vec3 totalForce;

		glm::vec3 angularVelocity;

		glm::mat3 inertiaMatrix;
		glm::mat3 rotationMatrix;
		glm::mat3 inertiaBody;

		glm::vec3 initialForce;
		glm::vec3 initialForcePoint;

		std::vector<ForceOnPoint> forces;

		void cubeReset() {
			
			//position = glm::vec3(rand() % 7 - 3, rand() % 7 + 2, rand() % 7 - 3);
			position = glm::vec3(0,5,0);
			//rotation = glm::vec3(glm::radians((float)(rand() % 360)), glm::radians((float)(rand() % 360)), glm::radians((float)(rand() % 360)));
			rotation = glm::vec3(0, 0, 0);

			velocity = glm::vec3(0, 0, 0);
			linearMomentum = glm::vec3(0, 0, 0);
			angularMomentum = glm::vec3(0, 0, 0);
			torque = glm::vec3(0, 0, 0);
			totalForce = glm::vec3(0, 0, 0);

			inertiaBody = glm::mat3();
			inertiaBody = inertiaBody * (1.f / 12.f * 2.f);
			rotationMatrix = setupRotationMatrix(rotation);
			inertiaMatrix = rotationMatrix * inertiaBody * glm::transpose(rotationMatrix);
			
			int maxForce = 30;
			initialForce = glm::vec3(rand() % maxForce - maxForce / 2.f, rand() % maxForce - maxForce / 2.f, rand() % maxForce - maxForce / 2.f);
			//initialForce = glm::vec3(0,0,5);
			initialForcePoint = rotationMatrix * glm::vec3(0.5f, 0.5f, 0) + position;
			//initialForcePoint = glm::vec3(0.5f, 4.5f, 0);
			std::cout << "Initial Force Point: " << initialForcePoint.x << " " << initialForcePoint.y << " " << initialForcePoint.z << std::endl;
			forces.clear();
			//forces.push_back(ForceOnPoint(gravity,position)); 
			forces.push_back(ForceOnPoint(initialForce, initialForcePoint)); 
		}

		CubeStruct() {
			cubeReset();
		}
	};

}
Cube::CubeStruct *ourCube;

void printSpecs() {
	if (Cube::showSpecs) {
		std::cout << "--------------------------------------------------------------------" << std::endl;
		//std::cout << "Position: " << ourCube->position.x << " " << ourCube->position.y << " " << ourCube->position.z << std::endl;
		std::cout << "Rotation: " << ourCube->rotation.x << " " << ourCube->rotation.y << " " << ourCube->rotation.z << std::endl;
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
				std::cout << ourCube->rotationMatrix[i][j] << " ";
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
	
	Cube::updateCube(translation * Cube::generateUpdateMatrix(ourCube->rotationMatrix) * scale);
	
	printSpecs();
}

void MyPhysicsUpdate(float dt) {
	float ourDt = dt;
	if (renderCube) {

		ourCube->totalForce = glm::vec3(0,0,0);
		ourCube->torque = glm::vec3(0,0,0);
		for (int i = 0; i < ourCube->forces.size(); i++) {
			ourCube->totalForce += ourCube->forces.at(i).force;
			ourCube->torque += glm::cross(ourCube->forces.at(i).point - ourCube->position, ourCube->forces[i].force);
		}
		
		
		ourCube->linearMomentum += ourDt * ourCube->totalForce;
		ourCube->angularMomentum += ourDt * ourCube->torque;
		ourCube->velocity = ourCube->linearMomentum / (float)Cube::mass;
		ourCube->position += ourDt * ourCube->velocity;

		ourCube->inertiaMatrix = ourCube->rotationMatrix * ourCube->inertiaBody * glm::transpose(ourCube->rotationMatrix);
		ourCube->angularVelocity = glm::inverse(ourCube->inertiaMatrix) * ourCube->angularMomentum;
		//Cube::clearInertiaMatrix(ourCube->inertiaMatrix);

		glm::mat4 translation = glm::translate(glm::mat4(), ourCube->position);
		Cube::updateCube(translation * Cube::generateUpdateMatrix(ourCube->rotationMatrix));

		ourCube->rotationMatrix += ourDt * (Cube::generateWMatrix(ourCube->angularVelocity) * ourCube->rotationMatrix);
		//ourCube->rotation += dt * glm::vec3(ourCube->angularVelocity.x, ourCube->angularVelocity.y, ourCube->angularVelocity.z);
		//ourCube->rotation = Cube::setupRotation(ourCube->rotationMatrix);
		
		ourCube->forces.clear();
		//ourCube->forces.push_back(Cube::ForceOnPoint(Cube::gravity, ourCube->position));

		//printSpecs();
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
	ImGui::Checkbox("Show Specs: ", &Cube::showSpecs);
	if(ImGui::Button("Reset")) {
		ourCube->cubeReset();
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

