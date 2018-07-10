#include "MoveTable.hpp"
#include "FaceCube.hpp"

#include <array>
#include <string>

std::map<std::string, MoveTable::TableEntry> MoveTable::table;

void MoveTable::init() {
    static bool initialized = false;
    if (initialized)
        return;

    // Build the translation table from strings to moves
    std::string names[3][8] = { // [axis][move] = name
            {"U", "Uw", "u", "Uc", "D", "Dw", "d", "Dc"},
            {"R", "Rw", "r", "Rc", "L", "Lw", "l", "Lc"},
            {"F", "Fw", "f", "Fc", "B", "Bw", "b", "Bc"}
    };
    int layers[8][3] = { // [move][layer] = direction
            {1, 0, 0},
            {1, 1, 0},
            {1, 1, 0},
            {1, 1, 1},
            {0, 0, -1},
            {0, -1, -1},
            {0, -1, -1},
            {-1, -1, -1}
    };
    std::string suffix[4] = {"", "2", "2'", "'"};
    int power[4] = {1, 2, -2, -1};
    for (int i = 0; i < 3; ++i) // each axis
        for (int j = 0; j < 8; ++j) // each move
            for (int k = 0; k < 4; ++k) // each power
                table[names[i][j] + suffix[k]] = TableEntry(i,
                        layers[j][0]*power[k],
                        layers[j][1]*power[k],
                        layers[j][2]*power[k]);

    // Axial moves
    const std::string axis_name[] = {"U", "R", "F", "D", "L", "B"};
    const std::string digits[] = {"0", "1", "2", "3"};

    for (int i = 0; i < 3; ++i) // each axis
        for (int j = 0; j < 4; ++j) // each first power
            for (int k = 0; k < 4; ++k) // each second power
                table[axis_name[i] + digits[j] + digits[k]] = TableEntry(i, ((j+2)&3)-2, 0, ((k+2)&3)-2);

    for (int i = 0; i < 3; ++i) // each axis
        for (int j = 0; j < 4; ++j) // each first power
            for (int k = 0; k < 4; ++k) // each second power
                for (int m = 0; m < 4; ++m) // each third power
                    table[axis_name[i] + digits[j] + digits[k] + digits[m]] = TableEntry(i, ((j+2)&3)-2, ((k+2)&3)-2, ((m+2)&3)-2);
}

const MoveTable::TableEntry& MoveTable::getMove(std::string mv) {
    init();
    const auto it = table.find(mv);
    if (it != table.end())
        return it->second;

    fprintf(stderr, "%s: %s(%d): Unknown move: %s\n", __FILE__, __FUNCTION__, __LINE__, mv.c_str());
    fflush(stderr);
    static const TableEntry e;
    return e;
}

const MoveSymbol MoveTable::getMoveSymbol(std::string mv) {
    init();
    std::string line_name = "       ";
    const size_t start = mv.size() >= line_name.size() ? 0 :
        line_name.size() / 2 - mv.size() / 2;
    for (size_t i = 0; i < mv.size() && i < line_name.size(); ++i) {
        line_name[start + i] = mv[i];
    }

    const auto it = table.find(mv);
    if (it != table.end()) {
        const TableEntry& move = it->second;
        if (move.axis == 1) {
            const std::array<std::string, 5> upper = {
                "╷ ", "╷ ", "╷ ", "▲ ", "▲ ",
            };
            const std::array<std::string, 5> middl = {
                "▼ ", "│ ", "│ ", "│ ", "▲ ",
            };
            const std::array<std::string, 5> lower = {
                "▼ ", "▼ ", "╵ ", "╵ ", "╵ ",
            };
            return MoveSymbol{
                " " + upper[move.c + 2] + upper[move.b + 2] + upper[move.a + 2],
                " " + middl[move.c + 2] + middl[move.b + 2] + middl[move.a + 2],
                " " + lower[move.c + 2] + lower[move.b + 2] + lower[move.a + 2],
                line_name,
            };
        } else if (move.axis == 0) {
            const std::array<std::string, 5> line_graphics{
                " ╶──▶▶ ",
                " ╶───▶ ",
                " ╶───╴ ",
                " ◀───╴ ",
                " ◀◀──╴ ",
            };
            return MoveSymbol{
                line_graphics[move.a + 2],
                line_graphics[move.b + 2],
                line_graphics[move.c + 2],
                line_name,
            };
        } else {
            const std::array<std::string, 5> line_graphics{
                " ◣◆──╮ ",
                " ◣───╮ ",
                " ╭───╮ ",
                " ╭───◢ ",
                " ╭──◆◢ ",
            };
            return MoveSymbol{
                line_graphics[move.c + 2],
                line_graphics[move.b + 2],
                line_graphics[move.a + 2],
                line_name,
            };
        }
    }
    fprintf(stderr, "%s: %s(%d): Unknown move: %s\n", __FILE__, __FUNCTION__, __LINE__, mv.c_str());
    fflush(stderr);
    return MoveSymbol{
        " ╭───╮ ",
        " │ ? │ ",
        " ╰───╯ ",
        line_name,
    };
}

void MoveTable::move(twophase::FaceCube &fc, std::string mv) {
    init();
    typeof(table.end()) it = table.find(mv);
    if (it != table.end())
        it->second.call(fc);
}
