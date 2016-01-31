#include <stdio.h>
#include <string>

#include <SolverLib/FaceCube.hpp>
#include <SolverLib/CubieCube.hpp>
#include <SolverLib/CoordCube.hpp>
#include <SolverLib/Search.hpp>
#include <SolverLib/Tools.hpp>

int main( int argc, const char* argv[] ) {
    if (argc != 2) {
        printf(
            "This program solves a Rubics cube using axial metric.\n"
            "It is written by Rasmus Göransson based on the demonstrational\n"
            "java package by Herbert Kociemba, see http://kociemba.org/cube.htm.\n"
            "\n"
            "Usage: Enter cube to solve as the argument to the program, eg:\n"
            "\n"
            "    SimpleSolver LFFUULFLLUFRDRBUBURUFDFLBDBLFLUDDRRRFBDRLRDFDURDLBUBBB\n"
            "\n");
        return 0;
    }

    twophase::Search search;
    std::string str = search.solution(argv[1], 18, 15, false);
    printf("\n%s\n", str.c_str());
    fflush(stdout);
}
