#include <cmath>
#include <cstdio>
#include <sstream>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <SolverLib/FaceCube.hpp>
#include <SolverLib/MoveTable.hpp>
#include <SolverLib/Tools.hpp>

#include "GraphicalCube.hpp"
#include "Sequence.hpp"

using namespace twophase;

GraphicalCube::GraphicalCube() :
    animationTime(0),
    search(NULL)
{
    for (int i = 0; i < 6; ++i)
        faceRotations[i] = 0;

    static const float colors[6][3] = {
        { 1, 1, 0 }, // yellow
        { 1, 0, 0 }, // red
        { 0, 0, 1 }, // blue
        { 1, 1, 1 }, // white
        { 1, 0.5, 0 }, // orange
        { 0, 1, 0 }, // green
    };

    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j)
            faceColors[i][j] = colors[i][j];
}

GraphicalCube::GraphicalCube(const GraphicalCube &gc) :
    faceCube(gc.faceCube),
    animationTime(0),
    search(NULL)
{
    for (int i = 0; i < 6; ++i)
    {
        faceRotations[i] = gc.faceRotations[i];
        for (int j = 0; j < 3; ++j)
            faceColors[i][j] = gc.faceColors[i][j];
    }
    for (int j = 0; j < 3; ++j)
        bottomColor[j] = bottomColor[j];
}

GraphicalCube::~GraphicalCube()
{
    delete search;
}

void GraphicalCube::move(std::string str)
{
    Sequence seq(str);

    if (seq.size() >= 2 && animations.size() > 0 && chainable(animations.back(), seq[1]))
    {
        animations.back().add(seq[0]);
        for (int i = 1; i < seq.size(); ++i)
            animate(seq[i]);
    }
    else
    {
        for (int i = 0; i < seq.size(); ++i)
            animate(seq[i]);
    }
}

void GraphicalCube::animate(const Sequence::Animation &a)
{
    animations.push_back(a);
}

static float bringTowardsZero(float &value, float d)
{
    if (value > 0)
    {
        if (value <= d)
        {
            float rest = d - value;
            value = 0;
            return rest;
        }
        else
        {
            value -= d;
            return 0;
        }
    }
    else
    {
        if (value >= -d)
        {
            float rest = -d - value;
            value = 0;
            return rest;
        }
        else
        {
            value += d;
            return 0;
        }
    }
}

static float interpolate(float a, float b, float t)
{
    return a + (b-a) * t;
}

// Execute queued moves
bool GraphicalCube::tick(float time)
{
    // Go forwards in time
    float rest = bringTowardsZero(animationTime, time);

    // Pop a new animation from the queue
    if (animationTime == 0 && animations.size() > 0)
    {
        activeAnimation = animations.front();
        animations.pop_front();
        animationTime = activeAnimation.time;
        bringTowardsZero(animationTime, rest);
        // Execute the associated move
        activeAnimation.move.call(faceCube);
    }

    // Update angles
    for (int i = 0; i < 3; ++i)
    {
        float t = animationTime / activeAnimation.time;
        float a = interpolate(activeAnimation.to[i][0], activeAnimation.from[i][0], t);
        float b = interpolate(activeAnimation.to[i][1], activeAnimation.from[i][1], t);
        float c = interpolate(activeAnimation.to[i][2], activeAnimation.from[i][2], t);
        faceRotations[i]   = (a - b) * M_PI_2;
        faceRotations[i+3] = (b - c) * M_PI_2;
        centerRotation[i]  =  b      * M_PI_2;
    }
    return animationTime != 0;
}

void GraphicalCube::randomize()
{
    try
    {
        faceCube = FaceCube( Tools::randomCube() );
    }
    catch ( const char str[] )
    {
        printf("exception caught: %s\n", str);
        fflush(stdout);
    }
    catch ( ... )
    {
        printf("exception caught\n");
        fflush(stdout);
    }
}

void GraphicalCube::solve()
{
    if (search == NULL)
        search = new Search();
    printf("Starting search for \"%s\"...\n", faceCube.to_String().c_str());
    fflush(stdout);

    std::string str = search->solution(faceCube.to_String(), 18, 15, false);

    int move_count = 0;
    bool new_word = true;
    for (auto c : str)
    {
        if (c == ' ')
        {
            new_word = true;
        }
        else
        {
            if (new_word)
            {
                move_count += 1;
            }
            new_word = false;
        }
    }
    printf("Solution in %d moves: %s\n", move_count, str.c_str());
    fflush(stdout);
    this->move(str);
}

void GraphicalCube::clean()
{
    faceCube = FaceCube();
}

void GraphicalCube::setFaces(std::string str)
{
    faceCube = FaceCube(str);
}

void GraphicalCube::drawCube()
{
    static GLfloat rotations[6][4] = {
        { -90, 1.0f, 0.0f, 0.0f }, // U
        {  90, 0.0f, 1.0f, 0.0f }, // R
        {   0, 0.0f, 1.0f, 0.0f }, // F
        {  90, 1.0f, 0.0f, 0.0f }, // D
        { -90, 0.0f, 1.0f, 0.0f }, // L
        { 180, 0.0f, 1.0f, 0.0f }, // B
    };

    // Rotate the whole cube
    glPushMatrix();
    glRotatef(centerRotation[0] * (-180.0 / M_PI), 0.0f, 1.0f, 0.0f);
    glRotatef(centerRotation[1] * (-180.0 / M_PI), 1.0f, 0.0f, 0.0f);
    glRotatef(centerRotation[2] * (-180.0 / M_PI), 0.0f, 0.0f, 1.0f);

    // Draw all centers
//    int k = 0;
    for (int i = 0; i < 6; ++i)
    {
        glPushMatrix();
        glRotatef(rotations[i][0], rotations[i][1], rotations[i][2], rotations[i][3]);
        glRotatef(faceRotations[i] * (-180.0 / M_PI), 0.0f, 0.0f, 1.0f);
//        drawCenterPiece(faceColors[i]);
//        for (int y = 1; y >= -1; --y) {
//            for (int x = -1; x <= 1; ++x) {
//                glColor3fv(faceColors[faceCube.f[k++]]);
//                drawFace(x,y);
//            }
//        }
        glColor3fv(faceColors[faceCube.f[i*9+4]]);
        drawFace(0,0);
        glPopMatrix();
    }


    // U, R, F, D, L, B
    float c[6]; // cos
    float s[6]; // sin
    for (int i = 0; i < 6; ++i)
    {
        c[i] = cos(faceRotations[i]);
        s[i] = sin(faceRotations[i]);
    }

    // Draw all edge pieces

    // These edge axes points in the direction of the normal to the facelet.
    // Together with their cross product, they build the coordinate system for each edge.
    // The first facelet is found in x-direction and the second in y-direction.
    // UR, UF, UL, UB, DR, DF, DL, DB, FR, FL, BL, BR
    float edge_axes[12][3][3] = {
            { { 0,     c[R], -s[R] }, {  c[U], 0,  s[U] }, { 0, 0, 0 } }, // UR
            { { s[F],  c[F], 0     }, { -s[U], 0,  c[U] }, { 0, 0, 0 } }, // UF
            { { 0,     c[L], s[L]  }, { -c[U], 0, -s[U] }, { 0, 0, 0 } }, // UL
            { { -s[B], c[B], 0     }, {  s[U], 0, -c[U] }, { 0, 0, 0 } }, // UB

            { { 0,    -c[R],  s[R] }, {  c[D], 0, -s[D] }, { 0, 0, 0 } }, // DR
            { { -s[F],-c[F], 0     }, {  s[D], 0,  c[D] }, { 0, 0, 0 } }, // DF
            { { 0,    -c[L], -s[L] }, { -c[D], 0,  s[D] }, { 0, 0, 0 } }, // DL
            { { s[B], -c[B], 0     }, { -s[D], 0, -c[D] }, { 0, 0, 0 } }, // DB

            { { 0,     s[R],  c[R] }, {  c[F], -s[F], 0 }, { 0, 0, 0 } }, // FR
            { { 0,    -s[L],  c[L] }, { -c[F],  s[F], 0 }, { 0, 0, 0 } }, // FL
            { { 0,     s[L], -c[L] }, { -c[B], -s[B], 0 }, { 0, 0, 0 } }, // BL
            { { 0,    -s[R], -c[R] }, {  c[B],  s[B], 0 }, { 0, 0, 0 } }, // BR
    };

    // Calculate the cross product which gives the third axis
    for (int i = 0; i < 12; ++i)
    {
        float x1 = edge_axes[i][0][0];
        float x2 = edge_axes[i][0][1];
        float x3 = edge_axes[i][0][2];
        float y1 = edge_axes[i][1][0];
        float y2 = edge_axes[i][1][1];
        float y3 = edge_axes[i][1][2];
        edge_axes[i][2][0] = x2*y3 - x3*y2;
        edge_axes[i][2][1] = x3*y1 - x1*y3;
        edge_axes[i][2][2] = x1*y2 - x2*y1;
    }

    // Transform and draw each edge piece
    GLfloat m[4*4];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            m[4*i+j] = 0;
    m[15] = 1;

    for (int i = 0; i < 12; ++i)
    {
        // Insert the vectors into the matrix
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 3; ++k)
                m[4*j+k] = edge_axes[i][j][k];
        }
        // Apply the transformation
        glPushMatrix();
        glMultMatrixf(m);
        // Find colors and draw the piece
        Color c1 = faceCube.f[ FaceCube::edgeFacelet[i][0] ];
        Color c2 = faceCube.f[ FaceCube::edgeFacelet[i][1] ];
//        Color c1 = FaceCube::edgeColor[i][0];
//        Color c2 = FaceCube::edgeColor[i][1];
        drawEdgePiece(faceColors[c1], faceColors[c2]);

        // Undo the transformation
        glPopMatrix();
    }

    // Draw all corner pieces

    // corner_axes is the coordinate system to draw corners in,
    // stored as references to edge_axes. ~ makes a value negative
    // and can be used to mirror the axis. Since opengl is right handed
    // and the corners are defined left handed, the last axis should point in
    // negative z-direction (opposite to the normal of the last face).
    // URF, UFL, ULB, UBR, DFR, DLF, DBL, DRB
    // Possible edges are:
    // UR, UF, UL, UB, DR, DF, DL, DB, FR, FL, BL, BR
    float corner_axes[8][3] = {
            { FR, UF, UR }, // URF
            {~FL, UL, UF }, // UFL
            { BL, UB, UL }, // ULB
            {~BR, UR, UB }, // UBR
            {~FR, DR, DF }, // DFR
            { FL, DF, DL }, // DLF
            {~BL, DL, DB }, // DBL
            { BR, DB, DR }, // DRB
    };

    for (int i = 0; i < 8; ++i)
    {
        // Insert the vectors into the matrix
        for (int j = 0; j < 3; ++j)
        {
            int e = corner_axes[i][j];
            if (e < 0) // Mirror the axis if negative
            {
                e = ~e;
                for (int k = 0; k < 3; ++k)
                    m[4*j+k] = -edge_axes[e][2][k];
            }
            else
            {
                for (int k = 0; k < 3; ++k)
                    m[4*j+k] = edge_axes[e][2][k];
            }
        }

        // Apply the transformation
        glPushMatrix();
        glMultMatrixf(m);
        // Find colors and draw the piece
        Color c1 = faceCube.f[ FaceCube::cornerFacelet[i][0] ];
        Color c2 = faceCube.f[ FaceCube::cornerFacelet[i][1] ];
        Color c3 = faceCube.f[ FaceCube::cornerFacelet[i][2] ];
        drawCornerPiece(faceColors[c1], faceColors[c2], faceColors[c3]);

        // Undo the transformation
        glPopMatrix();
    }

    // Restore rotation
    glPopMatrix();
}

void GraphicalCube::drawCenterPiece(float color[3])
{
    glColor3fv(color);
    drawFace(0,0);
}

void GraphicalCube::drawEdgePiece(float color1[3], float color2[3])
{
    const float size = 0.4;
    glBegin(GL_QUADS);
    glColor3fv(color1);
    glNormal3f(1.0, 0.0, 0.0);
    glVertex3f(1.5, 1 - size, -size);
    glVertex3f(1.5, 1 + size, -size);
    glVertex3f(1.5, 1 + size, +size);
    glVertex3f(1.5, 1 - size, +size);
    glColor3fv(color2);
    glNormal3f(0.0, 1.0, 0.0);
    glVertex3f(1 - size, 1.5, -size);
    glVertex3f(1 - size, 1.5, +size);
    glVertex3f(1 + size, 1.5, +size);
    glVertex3f(1 + size, 1.5, -size);
    glEnd();
}

void GraphicalCube::drawCornerPiece(float color1[3], float color2[3], float color3[3])
{
    const float size = 0.4;
    glBegin(GL_QUADS);
    glColor3fv(color1);
    glNormal3f(1.0, 0.0, 0.0);
    glVertex3f(1.5, 1 - size, -1 - size);
    glVertex3f(1.5, 1 + size, -1 - size);
    glVertex3f(1.5, 1 + size, -1 + size);
    glVertex3f(1.5, 1 - size, -1 + size);
    glColor3fv(color2);
    glNormal3f(0.0, 1.0, 0.0);
    glVertex3f(1 - size, 1.5, -1 - size);
    glVertex3f(1 - size, 1.5, -1 + size);
    glVertex3f(1 + size, 1.5, -1 + size);
    glVertex3f(1 + size, 1.5, -1 - size);
    glColor3fv(color3);
    glNormal3f(0.0, 0.0, -1.0);
    glVertex3f(1 - size, 1 - size, -1.5);
    glVertex3f(1 - size, 1 + size, -1.5);
    glVertex3f(1 + size, 1 + size, -1.5);
    glVertex3f(1 + size, 1 - size, -1.5);
    glEnd();
}

void GraphicalCube::drawFace(float x, float y)
{
    const float size = 0.4;
    glBegin(GL_QUADS);
    glNormal3f(0.0, 0.0, 1.0);
    glVertex3f(x - size, y - size, 1.5);
    glVertex3f(x + size, y - size, 1.5);
    glVertex3f(x + size, y + size, 1.5);
    glVertex3f(x - size, y + size, 1.5);
    glEnd();
}
