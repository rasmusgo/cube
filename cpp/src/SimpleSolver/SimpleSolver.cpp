#include <cstdio>
#include <regex>
#include <sstream>
#include <string>

#include <SolverLib/CoordCube.hpp>
#include <SolverLib/CubieCube.hpp>
#include <SolverLib/FaceCube.hpp>
#include <SolverLib/MoveTable.hpp>
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
            "    SimpleSolver \"D2 F' D' U' R F D B R2 D' B2 L' D' F2 D2 L2 U2 R' U'\"\n"
            "\n");
        return 0;
    }

    std::string input = argv[1];
    if (!std::regex_match(input, std::regex("((U|R|F|D|L|B){54})"))) {
        // Try interpreting the input as a sequence of moves.
        twophase::FaceCube cube;
        std::string move;
        std::stringstream ss(input);
        while (ss)
        {
            ss >> move;
            MoveTable::move(cube, move);
        }
        input = cube.to_String();
    }
    twophase::Search search;
    std::string solution = search.solution(input, 18, 15, false);
    printf("%s\n", solution.c_str());
    fflush(stdout);
}
