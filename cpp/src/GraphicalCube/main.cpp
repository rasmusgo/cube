#include <chrono>
#include <cstdio>
#include <cstdlib>

#include <GLFW/glfw3.h>

#include "GraphicalCube.h"

static int window_width = 600;
static int window_height = 600;
static GraphicalCube gCube;

void SetupOpenGL ()
{
    glViewport(0, 0, window_width, window_height);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    // Setup camera and projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // left, right, bottom, top, near, far
    glFrustum(-1, 1, -1, 1, 2, 10);

    // Move the model in front of the camera
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0,0,-5);
    glRotatef(45, 1.0, 0.0, 0.0);

    // Enable backface culling and depth test
//  glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    // Enable lighting
    glShadeModel(GL_FLAT);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // Place the light
    GLfloat light_position[] = { -2.0, 10.0, 5.0, 0.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    // Setup light parameters
    GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat mat_shininess[] = { 20.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

    // Use glColor to control AMBIENT and DIFFUSE lighting
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
}

void Render()
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    gCube.drawCube();
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS)
    {
        switch (key)
        {
        case GLFW_KEY_Q:         gCube.move("Fc'");   break;
        case GLFW_KEY_W:         gCube.move("B");     break;
        case GLFW_KEY_E:         gCube.move("L'");    break;
        case GLFW_KEY_R:         gCube.move("l'");    break;
        case GLFW_KEY_T:         gCube.move("Lc'");   break;
        case GLFW_KEY_A:         gCube.move("Uc'");   break;
        case GLFW_KEY_S:         gCube.move("D");     break;
        case GLFW_KEY_D:         gCube.move("L");     break;
        case GLFW_KEY_F:         gCube.move("U'");    break;
        case GLFW_KEY_G:         gCube.move("F'");    break;
        case GLFW_KEY_V:         gCube.move("l");     break;
        case GLFW_KEY_B:         gCube.move("Lc");    break;

        case GLFW_KEY_P:         gCube.move("Fc");    break;
        case GLFW_KEY_O:         gCube.move("B'");    break;
        case GLFW_KEY_I:         gCube.move("R");     break;
        case GLFW_KEY_U:         gCube.move("r");     break;
        case GLFW_KEY_Y:         gCube.move("Rc");    break;
        case GLFW_KEY_SEMICOLON: gCube.move("Uc");    break;
        case GLFW_KEY_L:         gCube.move("D'");    break;
        case GLFW_KEY_K:         gCube.move("R'");    break;
        case GLFW_KEY_J:         gCube.move("U");     break;
        case GLFW_KEY_H:         gCube.move("F");     break;
        case GLFW_KEY_M:         gCube.move("r'");    break;
        case GLFW_KEY_N:         gCube.move("Rc'");   break;

        case GLFW_KEY_LEFT:      gCube.move("Uc");    break;
        case GLFW_KEY_RIGHT:     gCube.move("Uc'");   break;
        case GLFW_KEY_UP:        gCube.move("Rc");    break;
        case GLFW_KEY_DOWN:      gCube.move("Rc'");   break;

        case GLFW_KEY_SPACE:
            gCube.randomize();
            break;

        case GLFW_KEY_END:
            //gCube.setFaces("BLBBUDFBFRLDRRBDRRLDDLFFURFRULUDDLDBDUUBLFUFFLURRBLUFB");
            //gCube.setFaces("LUDRUDFBRULLRRBFBBUDBFFFLUUFFLDDURDRBURLLLFRDBLURBBDFD");
            gCube.setFaces("BDFBULRDFDDUFRBULDUFLLFRUFBRULUDURUBLDBBLFFBFLRDRBLRRD");
            break;

        case GLFW_KEY_HOME:
            //gCube.move("F L F U' R U F2 L2 U' L' B D' B' L2 U"); // Cube in cube
            gCube.move("F U F2 D B U R F L D R U L U B D2 R F U2 D2"); // 20 face turns
            break;

        case GLFW_KEY_ENTER:
            // Solve the cube
            gCube.solve();
            break;

        case GLFW_KEY_BACKSPACE:
            gCube.clean();
            break;
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GL_TRUE);
            break;
        default:
            printf("Unhandled key: %d\n", key);
            fflush(stdout);
        }
    }
}

void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}

int main(int argc, char **argv)
{
    if (!glfwInit())
        exit(EXIT_FAILURE);

    atexit(glfwTerminate);

    glfwSetErrorCallback(error_callback);

    GLFWwindow* window = glfwCreateWindow(640, 480, "CubeSolver", NULL, NULL);

    if (!window)
    {
        fputs("ERROR: glfwCreateWindow failed.", stderr);
        exit(EXIT_FAILURE);
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    glfwSetKeyCallback(window, key_callback);

    auto simulation_time = std::chrono::system_clock::now();
    while (!glfwWindowShouldClose(window))
    {
        glfwGetFramebufferSize(window, &window_width, &window_height);
        SetupOpenGL();
        Render();
        glfwSwapBuffers(window);
        glfwPollEvents();
        auto wall_time = std::chrono::system_clock::now();
        while (wall_time > simulation_time)
        {
            gCube.tick(0.005);
            simulation_time += std::chrono::milliseconds(5);
        }
    }

    glfwDestroyWindow(window);

    return EXIT_SUCCESS;
}
