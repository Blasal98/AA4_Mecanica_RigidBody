#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\gtc\matrix_transform.hpp>
#include <time.h>
#include <iostream>

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

	glm::vec3 scale = glm::vec3(1,1,1);
	int mass = 1;
	glm::vec3 gravity = glm::vec3(0,-9.81f,0) * (float)mass;
	glm::mat3 inertia = glm::mat3(1/12,0,0,  0,1/12,0,  0,0,1/12);

	glm::mat3 setupRotationMatrix(glm::vec3 rotationAngles) {
		
		glm::mat3 rotationX = glm::mat3(1,0,0,  0,glm::cos(rotationAngles.x),-glm::sin(rotationAngles.x),  0, glm::sin(rotationAngles.x), glm::cos(rotationAngles.x));
		glm::mat3 rotationY = glm::mat3(glm::cos(rotationAngles.y),0, glm::sin(rotationAngles.y),  0,1,0,  -glm::sin(rotationAngles.y),0, glm::cos(rotationAngles.y));
		glm::mat3 rotationZ = glm::mat3(glm::cos(rotationAngles.z), -glm::sin(rotationAngles.z),0,  glm::sin(rotationAngles.z), glm::cos(rotationAngles.z),0,  0,0,1);

		return rotationX * rotationY * rotationZ;
	}

	struct CubeStruct {
		glm::vec3 position;
		glm::vec3 rotation;
		glm::vec3 velocity;

		glm::vec3 linearMomentum;
		glm::vec3 angularMomentum;
		glm::vec3 torque;

		glm::vec3 angularVelocity;

		glm::mat3 inertiaMatrix;
		glm::mat3 rotationMatrix;

		void cubeReset() {
			
			position = glm::vec3(rand() % 7 - 3, rand() % 7 + 2, rand() % 7 - 3);
			//rotation = glm::vec3(glm::radians((float)(rand() % 360)), glm::radians((float)(rand() % 360)), glm::radians((float)(rand() % 360)));
			rotation = glm::vec3(glm::radians(45.f),0,0);

			velocity = glm::vec3(0, 0, 0);
			linearMomentum = glm::vec3(0, 0, 0);
			angularMomentum = glm::vec3(0, 0, 0);
			torque = glm::vec3(0, 0, 0);

			rotationMatrix = setupRotationMatrix(rotation);
			inertiaMatrix = rotationMatrix * inertia * glm::transpose(rotationMatrix);
		}

		CubeStruct() {
			cubeReset();
		}
	};

}
Cube::CubeStruct *ourCube;


void MyPhysicsInit() {
	srand(time(NULL));

	renderCube = true; //activem cub
	Cube::setupCube(); //el setejem
	ourCube = new Cube::CubeStruct(); //el creem
	
	glm::mat4 translation = glm::translate(glm::mat4(), ourCube->position);
	glm::mat4 rotation = glm::mat4();
	rotation = glm::rotate(rotation, ourCube->rotation.x, glm::vec3(1, 0, 0));
	rotation = glm::rotate(rotation, ourCube->rotation.y, glm::vec3(0, 1, 0));
	rotation = glm::rotate(rotation, ourCube->rotation.z, glm::vec3(0, 0, 1));
	glm::mat4 scale = glm::scale(glm::mat4(), Cube::scale);
	
	Cube::updateCube(translation * rotation * scale);

	
}

void MyPhysicsUpdate(float dt) {
	if (renderCube) {

		
		ourCube->linearMomentum += dt * Cube::gravity;
		ourCube->angularMomentum += dt * ourCube->torque;
		ourCube->velocity = ourCube->linearMomentum / (float)Cube::mass;
		ourCube->position += dt * ourCube->velocity;

		ourCube->rotationMatrix = Cube::setupRotationMatrix(ourCube->rotation);
		ourCube->inertiaMatrix = ourCube->rotationMatrix * Cube::inertia * glm::transpose(ourCube->rotationMatrix);

		glm::mat4 translation = glm::translate(glm::mat4(), ourCube->position);
		glm::mat4 rotation = glm::mat4();
		rotation = glm::rotate(rotation, ourCube->rotation.x, glm::vec3(1, 0, 0));
		rotation = glm::rotate(rotation, ourCube->rotation.y, glm::vec3(0, 1, 0));
		rotation = glm::rotate(rotation, ourCube->rotation.z, glm::vec3(0, 0, 1));

		Cube::updateCube(translation * rotation);
		
	}
}
void MyPhysicsCleanup() {
	Cube::cleanupCube();
}



void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		//Exemple_GUI();
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

