//****************************************************
// Starter code for assignment #1.  It is provided to 
// help get you started, but you are not obligated to
// use this starter code.
//****************************************************

#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include"atlstr.h"
//include header file for glfw library so that we can use OpenGL
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#define PI 3.14159265 // Should be used from mathlib

using namespace std;

//****************************************************
// Global Variables
// Generally speaking, global variables should be 
// avoided, but for this small assignment we'll make
// and exception.
//****************************************************

GLfloat Translation[3] = { 0.0f, 0.0f, 0.0f };
bool Auto_strech = false;
bool otherLight = true;
int  Width_global = 800;
int  Height_global = 800;
int  SizeX_saved_global;
int  SizeY_saved_global;

int  PosX_saved_global;
int  PosY_saved_global;

const GLFWvidmode* VideoMode_global = NULL;

inline float sqr(float x) { return x * x; }

//****************************************************
// Simple init function
//****************************************************

void initializeRendering()
{
    glfwInit();
}


//****************************************************
// A routine to set a pixel by drawing a GL point.  This is not a
// general purpose routine as it assumes a lot of stuff specific to
// this example.
//****************************************************

void setPixel(float x, float y, GLfloat r, GLfloat g, GLfloat b) {
    glColor3f(r, g, b);
    glVertex2f(x + 0.5, y + 0.5);
    // The 0.5 is to target pixel centers
    // Note that some OpenGL implementations have created gaps in the past.
}

//****************************************************
// Keyboard inputs
//****************************************************

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    switch (key) {

        // Hint on making up/down left/right work: the variable Translation [0] and [1].

    case GLFW_KEY_ESCAPE:
    case GLFW_KEY_Q:
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        break;
    case GLFW_KEY_LEFT:
        break;
    case GLFW_KEY_RIGHT:
        break;
    case GLFW_KEY_UP:
        break;
    case GLFW_KEY_DOWN:
        break;
    case GLFW_KEY_F:
        if (action) {
            Auto_strech = !Auto_strech;
            if (Auto_strech) {
                glfwGetWindowPos(window, &PosX_saved_global, &PosY_saved_global);
                glfwGetWindowSize(window, &SizeX_saved_global, &SizeY_saved_global);
                glfwSetWindowSize(window, VideoMode_global->width, VideoMode_global->height);
                glfwSetWindowPos(window, 0, 0);
            }
            else {
                glfwSetWindowSize(window, SizeX_saved_global, SizeY_saved_global);
                glfwSetWindowPos(window, PosX_saved_global, PosY_saved_global);
            }
        }
        break;
    case GLFW_KEY_SPACE:
        break;
    default:
        break;
    }

}



//****************************************************
// Draw a filled circle.
//****************************************************

glm::vec3 lightPos(700.0f, 500.0f, 400.0f); //Position of light
glm::vec3 lightPos1(-700.0f, 500.0f, 400.0f); //Position of light1
glm::vec3 viewPos(0.0f, 0.0f, 5000.0f); //Position of eyes
glm::vec3 lightColor(1.0f, 1.0f, 1.0f);
glm::vec3 lightColor1(1.0f, 1.0f, 1.0f);
glm::vec3 objectColor(1.0f, 0.2f, 0.2f);
float ambientStrength = 0.3f;
float diffuseStrength = 10.0f;
float specularStrength = 1.0f;
float Rd = 0.7f;
float Rs = 0.2f;
float nu = 100.0;
float nv = 1000.0;

void drawCircle(float centerX, float centerY, float radius) {
    // Start drawing a list of points
    glBegin(GL_POINTS);

    // We could eliminate wasted work by only looping over the pixels
    // inside the sphere's radius.  But the example is more clear this
    // way.  In general drawing an object by loopig over the whole
    // screen is wasteful.

    int minI = max(0, (int)floor(centerX - radius));
    int maxI = min(Width_global - 1, (int)ceil(centerX + radius));

    int minJ = max(0, (int)floor(centerY - radius));
    int maxJ = min(Height_global - 1, (int)ceil(centerY + radius));


    for (int i = 0; i < Width_global; i++) {
        for (int j = 0; j < Height_global; j++) {

            // Location of the center of pixel relative to center of sphere
            float x = (i + 0.5 - centerX);
            float y = (j + 0.5 - centerY);

            float dist = sqrt(sqr(x) + sqr(y));

            if (dist <= radius) {


                // This is the front-facing Z coordinate
                float z = sqrt(radius * radius - dist * dist);

				glm::vec3 p(x, y, z); //point on the surface of ball
				glm::vec3 o(0.0f, 0.0f, 0.0f); //centre of sphere
				glm::vec3 norm = normalize(p - o);
				glm::vec3 lightDir = normalize(lightPos - p);
				glm::vec3 lightDir1 = normalize(lightPos1 - p);
                glm::vec3 viewDir = normalize(viewPos - p);
				glm::vec3 half = normalize(lightDir + viewDir);
				glm::vec3 half1 = normalize(lightDir1 + viewDir);
				float NdotL = max(dot(norm, lightDir), 0.0f);
				float NdotV = max(dot(norm, viewDir), 0.0f);
				float NdotH = max(dot(norm, half), 0.0f);
				float HdotV = max(dot(half, viewDir), 0.0f);
				float HdotL = max(dot(half, lightDir), 0.0f);
				float NdotL1 = max(dot(norm, lightDir1), 0.0f);
				float H1dotL1 = max(dot(half1, lightDir1), 0.0f);
				float NdotH1 = max(dot(norm, half1), 0.0f);
				float H1dotV = max(dot(half1, viewDir), 0.0f);

                //ambient
                glm::vec3 ambient = ambientStrength * (lightColor + lightColor1);
                
                //Diffuse
				float kd = (28.0f * Rd) * (1.0f - Rs) / (23.0f * PI);
				kd *= 1.0f - pow(1.0f - (NdotL / 2.0f), 5.0f);
				kd *= 1.0f - pow(1.0f - (NdotV / 2.0f), 5.0f);
				glm::vec3 diff = diffuseStrength * kd * lightColor;

				float kd1 = (28.0f * Rd) * (1.0f - Rs) / (23.0f * PI);
				kd1 *= 1.0f - pow(1.0f - (NdotL1 / 2.0f), 5.0f);
				kd1 *= 1.0f - pow(1.0f - (NdotV / 2.0f), 5.0f);
				glm::vec3 diff1 = diffuseStrength * kd1 * lightColor1;

                //Specular
                glm::vec3 y(0.0f, 1.0f, 0.0f);
                glm::vec3 v = normalize(y - norm * (norm.y));
                glm::vec3 u = normalize(cross(v, norm));
                float ks = Rs + (1.0f - Rs) * pow(1.0f - HdotL, 5.0f);
                ks *= sqrt((nu + 1.0f) * (nv + 1.0f)) / (8.0f * PI);
                ks *= pow(NdotH, (nu * pow(dot(half, u), 2.0f) + nv * pow(dot(half, v), 2.0f)) / (1.0f - pow(dot(half, norm), 2.0f)));
                ks *= 1 / HdotL * max(NdotL, NdotV);
                glm::vec3 spec = specularStrength * ks * lightColor;

				float ks1 = Rs + (1.0f - Rs) * pow(1.0f - H1dotL1, 5.0f);
				ks1 *= sqrt((nu + 1.0f) * (nv + 1.0f)) / (8.0f * PI);
				ks1 *= pow(NdotH1, (nu * pow(dot(half1, u), 2.0f) + nv * pow(dot(half1, v), 2.0f)) / (1.0f - pow(dot(half1, norm), 2.0f)));
				ks1 *= 1 / H1dotL1 * max(NdotL1, NdotV);
				glm::vec3 spec1 = specularStrength * ks1 * lightColor1;

                //ambient + diffuse + specular
				if (otherLight)
				{
					glm::vec3 result = (ambient + diff + diff1 + spec + spec1) * objectColor;
					setPixel(i, j, result.x, result.y, result.z);
				}
				else
				{
					glm::vec3 result = (ambient + diff + spec) * objectColor;
					setPixel(i, j, result.x, result.y, result.z);
				}
                // This is amusing, but it assumes negative color values are treated reasonably.
                // setPixel(i,j, x/radius, y/radius, z/radius );

            }
        }
    }

    glEnd();
}

//****************************************************
// function that does the actual drawing of stuff
//***************************************************

void display(GLFWwindow* window)
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);      //clear background screen to black

    glClear(GL_COLOR_BUFFER_BIT);                // clear the color buffer (sets everything to black)

    glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
    glLoadIdentity();                            // make sure transformation is "zero'd"

    //----------------------- code to draw objects --------------------------
    glPushMatrix();
    glTranslatef(Translation[0], Translation[1], Translation[2]);
    drawCircle(
        Width_global / 2.0,
        Height_global / 4.0,
        min(Width_global, Height_global) * 0.8 / 4.0);  // What do you think this is doing?
    glPopMatrix();

}

void displayImgui()
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(700, 420));
	ImGui::Begin("setting");
	ImGui::ColorEdit3("objColor", (float*)&objectColor);
    ImGui::ColorEdit3("LTColor", (float*)&lightColor);
    ImGui::Checkbox("otherLight", &otherLight);
    ImGui::ColorEdit3("LTColor1", (float*)&lightColor1);
	ImGui::SliderFloat("LTpos.x", &lightPos.x, -1000.0f, 1000.0f);
	ImGui::SliderFloat("LTpos.y", &lightPos.y, -1000.0f, 1000.0f);
	ImGui::SliderFloat("LTpos.z", &lightPos.z, 300.0f, 3000.0f);
	ImGui::SliderFloat("LTpos1.x", &lightPos1.x, -1000.0f, 1000.0f);
	ImGui::SliderFloat("LTpos1.y", &lightPos1.y, -1000.0f, 1000.0f);
	ImGui::SliderFloat("LTpos1.z", &lightPos1.z, 300.0f, 3000.0f);
	ImGui::SliderFloat("nu", &nu, 0.0f, 10000.0f);
	ImGui::SliderFloat("nv", &nv, 0.0f, 10000.0f);
	ImGui::SliderFloat("Rs", &Rs, 0.0f, 1.0f - Rd);
	ImGui::SliderFloat("Rd", &Rd, 0.0f, 1.0f - Rs);
    ImGui::SliderFloat("ambientStrength", &ambientStrength, 0.0f, 1.0f);   
    ImGui::SliderFloat("diffuseStrength", &diffuseStrength, 0.0f, 100.0f);
    ImGui::SliderFloat("specularStrength", &specularStrength, 0.0f, 30.0f);
	ImGui::End();
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
//****************************************************
// function that is called when window is resized
//***************************************************

void size_callback(GLFWwindow* window, int width, int height)
{
    // The width and height arguments are not used
    // because they are not the size of the window 
    // in pixels.

    // Get the pixel coordinate of the window
    // it returns the size, in pixels, of the 
    // framebuffer of the specified window
    glfwGetFramebufferSize(window, &Width_global, &Height_global);
   
    glViewport(0, 0, Width_global, Height_global);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, Width_global, 0, Height_global, 1, -1);

    display(window);
}


//****************************************************
// the usual stuff, nothing exciting here
//****************************************************


int main(int argc, char* argv[]) {

    //This initializes glfw
    initializeRendering();

    GLFWwindow* window = glfwCreateWindow(Width_global, Height_global, "Computer Graphics", NULL, NULL);
    if (!window)
    {
        cerr << "Error on window creating" << endl;
        glfwTerminate();
        return -1;
    }

    VideoMode_global = glfwGetVideoMode(glfwGetPrimaryMonitor());
    if (!VideoMode_global)
    {
        cerr << "Error on getting monitor" << endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    size_callback(window, 0, 0);

    glfwSetWindowSizeCallback(window, size_callback);
    glfwSetKeyCallback(window, key_callback);

	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 330");

    while (!glfwWindowShouldClose(window)) // main loop to draw object again and again
    {
        display(window);
        displayImgui();
        glfwPollEvents();
        glfwSwapBuffers(window);
    }

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
    return 0;
}








