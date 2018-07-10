#include <array>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

const size_t SYMBOL_SIZE = 4;

using MoveSymbol = std::array<std::string, SYMBOL_SIZE>;

std::map<std::string, MoveSymbol> generateMoves()
{
    std::map<std::string, MoveSymbol> moves;
    moves["R"] = {
        " ╷ ╷ ▲ ",
        " │ │ │ ",
        " ╵ ╵ ╵ ",
        "   R   ",
    };
    moves["R2"] = {
        " ╷ ╷ ▲ ",
        " │ │ ▲ ",
        " ╵ ╵ ╵ ",
        "   R2  ",
    };
    moves["R'"] = {
        " ╷ ╷ ╷ ",
        " │ │ │ ",
        " ╵ ╵ ▼ ",
        "   R'  ",
    };

    moves["L"] = {
        " ╷ ╷ ╷ ",
        " │ │ │ ",
        " ▼ ╵ ╵ ",
        "   L   ",
    };
    moves["L2"] = {
        " ╷ ╷ ╷ ",
        " ▼ │ │ ",
        " ▼ ╵ ╵ ",
        "   L2  ",
    };
    moves["L'"] = {
        " ▲ ╷ ╷ ",
        " │ │ │ ",
        " ╵ ╵ ╵ ",
        "   L'  ",
    };

    moves["U"] = {
        " ◀───╴ ",
        " ╶───╴ ",
        " ╶───╴ ",
        "   U   ",
    };
    moves["U2"] = {
        " ◀◀──╴ ",
        " ╶───╴ ",
        " ╶───╴ ",
        "   U2  ",
    };
    moves["U'"] = {
        " ╶───▶ ",
        " ╶───╴ ",
        " ╶───╴ ",
        "   U'  ",
    };

    moves["D"] = {
        " ╶───╴ ",
        " ╶───╴ ",
        " ╶───▶ ",
        "   D   ",
    };
    moves["D2"] = {
        " ╶───╴ ",
        " ╶───╴ ",
        " ╶──▶▶ ",
        "   D2  ",
    };
    moves["D'"] = {
        " ╶───╴ ",
        " ╶───╴ ",
        " ◀───╴ ",
        "   D'  ",
    };

    moves["F"] = {
        " ╭───╮ ",
        " ╭───╮ ",
        " ╭───◢ ",
        "   F   ",
    };
    moves["F2"] = {
        " ╭───╮ ",
        " ╭───╮ ",
        " ╭──◆◢ ",
        "   F2  ",
    };
    moves["F'"] = {
        " ╭───╮ ",
        " ╭───╮ ",
        " ◣───╮ ",
        "   F'  ",
    };

    moves["B"] = {
        " ◣───╮ ",
        " ╭───╮ ",
        " ╭───╮ ",
        "   B   ",
    };
    moves["B2"] = {
        " ◣◆──╮ ",
        " ╭───╮ ",
        " ╭───╮ ",
        "   B2  ",
    };
    moves["B'"] = {
        " ╭───◢ ",
        " ╭───╮ ",
        " ╭───╮ ",
        "   B'  ",
    };
    return moves;
}

int main()
{
    const auto known_moves = generateMoves();
    std::string line;
    while (std::getline(std::cin, line)) {
        std::array<std::ostringstream, SYMBOL_SIZE> sequence;
        std::istringstream line_stream(line);
        std::string word;
        while (line_stream >> word) {
            const auto it = known_moves.find(word);
            if (it != known_moves.end()) {
                const auto move = it->second;
                for (size_t row = 0; row < SYMBOL_SIZE; ++row) {
                    sequence[row] << move[row];
                }
            } else {
                std::cerr << "Unknown move: " << word << std::endl;
            }
        }
        for (size_t row = 0; row < SYMBOL_SIZE; ++row) {
            std::cout << sequence[row].str() << std::endl;
        }
    }
    return 0;
}
