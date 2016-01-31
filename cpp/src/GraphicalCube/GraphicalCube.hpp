#pragma once

#include <deque>

#include <SolverLib/FaceCube.hpp>
#include <SolverLib/MoveTable.hpp>
#include <SolverLib/Search.hpp>

#include "Sequence.hpp"

class GraphicalCube {
    twophase::FaceCube faceCube;
    // U R F D L B order
    // x axis is pointing towards R
    // y axis is pointing towards U
    // z axis is pointing towards F
    float faceRotations[6];
    float centerRotation[3];
    float faceColors[6][3];
    float bottomColor[3];

    Sequence::Animation activeAnimation;
    float animationTime;
    std::deque<Sequence::Animation> animations;

    twophase::Search *search;

public:
    GraphicalCube();
    GraphicalCube(const GraphicalCube &gc);
    ~GraphicalCube();

private:
    void init();

public:
    void move(std::string str);
    bool tick(float time);
    void randomize();
    void solve();
    void clean();
    void setFaces(std::string str);
    void drawCube();

private:
    void animate(const Sequence::Animation &a);
    void drawCenterPiece(float color[3]);
    void drawEdgePiece(float color1[3], float color2[3]);
    void drawCornerPiece(float color1[3], float color2[3], float color3[3]);
    void drawFace(float x, float y);
};
