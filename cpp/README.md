# SimpleSolver
This program solves a Rubics cube using axial metric.
It is written by Rasmus Göransson based on the demonstrational
java package by Herbert Kociemba, see http://kociemba.org/cube.htm.

Usage: Enter cube to solve as the argument to the program, eg:

    SimpleSolver LFFUULFLLUFRDRBUBURUFDFLBDBLFLUDDRRRFBDRLRDFDURDLBUBBB

# GraphicalCube
Renders a Rubics cube and lets the user manipulate it by using the keyboard.
Spacebar scrambles the cube. Enter solves it (same solver as SimpleSolver).

| Key | Action |
|-----|--------|
| F   | U      |
| J   | U'     |
| G   | F'     |
| H   | F      |
| D   | L      |
| E   | L'     |
| I   | R      |
| K   | R'     |
| S   | D      |
| L   | D'     |
| A   | Z'     |
| Ö   | Z      |
| W   | B      |
| O   | B'     |
| Q   | Y'     |
| P   | Y      |
| T   | X      |
| Y   | X      |
| R   | wL'    |
| V   | wL     |
| U   | wR     |
| M   | wR'    |

# CubeRecognition

Find the state of a scrambled (or solved) cube from two images.

## Video

Track the cube over time from a video sequence to analyze the moves being made.

The cube state is tracked using a particle filter where each particle is a Kalman filter.
The state representation has a discrete and a continuous part.
The discrete part is the permutation of the cube.
The continuous part is the position and orientation of the cube relative to the camera and rotations of the sides (less than quarter turns).
