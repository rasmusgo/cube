#include <stdio.h>
#include <stdlib.h>

#include <windows.h>
#include <gl/gl.h>

#include "GraphicalCube.h"

LRESULT CALLBACK WindowProc(HWND, UINT, WPARAM, LPARAM);
void EnableOpenGL(HWND hwnd, HDC*, HGLRC*);
void DisableOpenGL(HWND, HDC, HGLRC);
void SetupOpenGL();

static HDC hDC;
static int window_width = 600;
static int window_height = 600;
static GraphicalCube gCube;

void Render()
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	gCube.drawCube();

    SwapBuffers(hDC);
}

int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow)
{
    WNDCLASSEX wcex;
    HWND hwnd;
//    HDC hDC;
    HGLRC hRC;
    MSG msg;
    BOOL bQuit = FALSE;

    /* register window class */
    wcex.cbSize = sizeof(WNDCLASSEX);
    wcex.style = 0; // CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
    wcex.lpfnWndProc = WindowProc;
    wcex.cbClsExtra = 0;
    wcex.cbWndExtra = 0;
    wcex.hInstance = hInstance;
    wcex.hIcon = LoadIcon(NULL, IDI_APPLICATION);
    wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
    wcex.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wcex.lpszMenuName = NULL;
    wcex.lpszClassName = "CubeSolver";
    wcex.hIconSm = LoadIcon(NULL, IDI_APPLICATION);;


    if (!RegisterClassEx(&wcex))
        return 0;

    /* create main window */
    hwnd = CreateWindowEx(0,
                          "CubeSolver",
                          "CubeSolver",
                          WS_OVERLAPPEDWINDOW,
                          CW_USEDEFAULT,
                          CW_USEDEFAULT,
                          window_width,
                          window_height,
                          NULL,
                          NULL,
                          hInstance,
                          NULL);

    ShowWindow(hwnd, nCmdShow);

    /* enable OpenGL for the window */
    EnableOpenGL(hwnd, &hDC, &hRC);
    SetupOpenGL();

    /* program main loop */
    while (!bQuit)
    {
        /* check for messages */
        if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
        {
            /* handle or dispatch messages */
            if (msg.message == WM_QUIT)
            {
                bQuit = TRUE;
            }
            else
            {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
        }
        else
        {
            /* OpenGL animation code goes here */
        	gCube.tick(0.01);
        	Render();
            Sleep (1);
        }
    }

    /* shutdown OpenGL */
    DisableOpenGL(hwnd, hDC, hRC);

    /* destroy the window explicitly */
    DestroyWindow(hwnd);

    return msg.wParam;
}

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    switch (uMsg)
    {
    case WM_SYSCOMMAND:
    {
		switch (wParam)
		{
		case SC_SCREENSAVE:
		case SC_MONITORPOWER:
			//return 0; // Prevent From Happening
			;
		default:
			return DefWindowProc(hwnd, uMsg, wParam, lParam);
		}
		break;
	}
    case WM_SIZE:
    	window_width = LOWORD(lParam);
    	window_height = HIWORD(lParam);
    	SetupOpenGL();
    	break;
    case WM_PAINT:
    {
    	PAINTSTRUCT ps;
        BeginPaint(hwnd, &ps);
        EndPaint(hwnd, &ps);
        Render();
        break;
    }
	case WM_CLOSE:
		PostQuitMessage(0);
	break;

	case WM_DESTROY:
		return 0;

	case WM_KEYDOWN:
	{
		switch (wParam)
		{
		case 'Q':		gCube.move("Fc'");		break;
		case 'W':		gCube.move("B");		break;
		case 'E':		gCube.move("L'");		break;
		case 'R':		gCube.move("l'");		break;
		case 'T':		gCube.move("Lc'");		break;
		case 'A':		gCube.move("Uc'");		break;
		case 'S':		gCube.move("D");		break;
		case 'D':		gCube.move("L");		break;
		case 'F':		gCube.move("U'");		break;
		case 'G':		gCube.move("F'");		break;
		case 'V':		gCube.move("l");		break;
		case 'B':		gCube.move("Lc");		break;

		case 'P':		gCube.move("Fc");		break;
		case 'O':		gCube.move("B'");		break;
		case 'I':		gCube.move("R");		break;
		case 'U':		gCube.move("r");		break;
		case 'Y':		gCube.move("Rc");		break;
		case 192:		gCube.move("Uc");		break;
		case 'L':		gCube.move("D'");		break;
		case 'K':		gCube.move("R'");		break;
		case 'J':		gCube.move("U");		break;
		case 'H':		gCube.move("F");		break;
		case 'M':		gCube.move("r'");		break;
		case 'N':		gCube.move("Rc'");		break;

		case VK_LEFT:	gCube.move("Uc");		break;
		case VK_RIGHT:	gCube.move("Uc'");		break;
		case VK_UP:		gCube.move("Rc");		break;
		case VK_DOWN:	gCube.move("Rc'");		break;

		case VK_SPACE:
			gCube.randomize();
			break;

		case VK_END:
			//gCube.setFaces("BLBBUDFBFRLDRRBDRRLDDLFFURFRULUDDLDBDUUBLFUFFLURRBLUFB");
			//gCube.setFaces("LUDRUDFBRULLRRBFBBUDBFFFLUUFFLDDURDRBURLLLFRDBLURBBDFD");
			gCube.setFaces("BDFBULRDFDDUFRBULDUFLLFRUFBRULUDURUBLDBBLFFBFLRDRBLRRD");
			break;

		case VK_HOME:
			//gCube.move("F L F U' R U F2 L2 U' L' B D' B' L2 U"); // Cube in cube
			gCube.move("F U F2 D B U R F L D R U L U B D2 R F U2 D2"); // 20 face turns
			break;

		case VK_RETURN:
			// Solve the cube
			gCube.solve();
			break;

		case VK_BACK:
			gCube.clean();
			break;
		case VK_ESCAPE:
			PostQuitMessage(0);
			break;
		default:
			printf("Unknown vkey: %d\n", wParam);
			fflush(stdout);
		}
	}
	break;

	default:
		return DefWindowProc(hwnd, uMsg, wParam, lParam);
    }

    return 0;
}

void EnableOpenGL(HWND hwnd, HDC* hDC, HGLRC* hRC)
{
    PIXELFORMATDESCRIPTOR pfd;

    int iFormat;

    /* get the device context (DC) */
    *hDC = GetDC(hwnd);

    /* set the pixel format for the DC */
    ZeroMemory(&pfd, sizeof(pfd));

    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW |
                  PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 24;
    pfd.cDepthBits = 16;
    pfd.iLayerType = PFD_MAIN_PLANE;

    iFormat = ChoosePixelFormat(*hDC, &pfd);

    SetPixelFormat(*hDC, iFormat, &pfd);

    /* create and enable the render context (RC) */
    *hRC = wglCreateContext(*hDC);

    wglMakeCurrent(*hDC, *hRC);
}

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
//	glEnable(GL_CULL_FACE);
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

void DisableOpenGL (HWND hwnd, HDC hDC, HGLRC hRC)
{
    wglMakeCurrent(NULL, NULL);
    wglDeleteContext(hRC);
    ReleaseDC(hwnd, hDC);
}

