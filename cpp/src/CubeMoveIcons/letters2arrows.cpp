#include <array>
#include <iostream>
#include <sstream>
#include <string>

#include "SolverLib/MoveTable.hpp"

int main()
{
    std::string line;
    while (std::getline(std::cin, line)) {
        std::array<std::ostringstream, SYMBOL_SIZE> sequence;
        std::istringstream line_stream(line);
        std::string word;
        while (line_stream >> word) {
            const auto move_symbol = MoveTable::getMoveSymbol(word);
            for (size_t row = 0; row < SYMBOL_SIZE; ++row) {
                sequence[row] << move_symbol[row];
            }
        }
        for (size_t row = 0; row < SYMBOL_SIZE; ++row) {
            std::cout << sequence[row].str() << std::endl;
        }
    }
    return 0;
}
