#pragma once

#include "FaceCube.hpp"

#include <array>
#include <map>
#include <string>

const size_t SYMBOL_SIZE = 4;

using MoveSymbol = std::array<std::string, SYMBOL_SIZE>;

class MoveTable
{
public:
    struct TableEntry
    {
        int axis;
        int a;
        int b;
        int c;
        TableEntry() :
            axis(0), a(0), b(0), c(0)
        {
        }
        TableEntry(int axis, int a, int b, int c) :
            axis(axis), a(a), b(b), c(c)
        {
        }
        void call(twophase::FaceCube &fc)
        {
            fc.moveAxis(axis, a,b,c);
        }
    };

private:
    static std::map<std::string, TableEntry> table;

    // MoveTable cannot be instantiated
    MoveTable() {}
    static void init();
public:
    static const TableEntry& getMove(std::string mv);
    static const MoveSymbol getMoveSymbol(std::string mv);
    static void move(twophase::FaceCube &fc, std::string mv);
};
