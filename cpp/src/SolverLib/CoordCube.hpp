#pragma once

#include "Common.hpp"

namespace twophase {

class CubieCube;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Representation of the cube on the coordinate level'
class CoordCube {
public:
    static const short N_TWIST = 2187;// 3^7 possible corner orientations
    static const short N_FLIP = 2048;// 2^11 possible edge flips
    static const short N_SLICE1 = 495;// 12 choose 4 possible positions of FR,FL,BL,BR edges
    static const short N_SLICE2 = 24;// 4! permutations of FR,FL,BL,BR edges in phase2
    static const short N_PARITY = 2; // 2 possible corner parities
    static const short N_URFtoDLF = 20160;// 8!/(8-6)! permutation of URF,UFL,ULB,UBR,DFR,DLF corners
    static const short N_FRtoBR = 11880; // 12!/(12-4)! permutation of FR,FL,BL,BR edges
    static const short N_URtoUL = 1320; // 12!/(12-3)! permutation of UR,UF,UL edges
    static const short N_UBtoDF = 1320; // 12!/(12-3)! permutation of UB,DR,DF edges
    static const short N_URtoDF = 20160; // 8!/(8-6)! permutation of UR,UF,UL,UB,DR,DF edges in phase2

    static const int N_URFtoDLB = 40320;// 8! permutations of the corners
    static const int N_URtoBR = 479001600;// 8! permutations of the corners

    static const short N_MOVE = 3*16; //18;

    // All coordinates are 0 for a solved cube except for UBtoDF, which is 114
    short twist;
    short flip;
    short parity;
    short FRtoBR;
    short URFtoDLF;
    short URtoUL;
    short UBtoDF;
    int URtoDF;

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Generate a CoordCube from a CubieCube
    explicit CoordCube(const CubieCube &c);

    // A move on the coordinate level
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    void move(int m) {
        twist = twistMove[twist][m];
        flip = flipMove[flip][m];
        parity = parityMove[parity][m];
        FRtoBR = FRtoBR_Move[FRtoBR][m];
        URFtoDLF = URFtoDLF_Move[URFtoDLF][m];
        URtoUL = URtoUL_Move[URtoUL][m];
        UBtoDF = UBtoDF_Move[UBtoDF][m];
        if (URtoUL < 336 && UBtoDF < 336)// updated only if UR,UF,UL,UB,DR,DF
            // are not in UD-slice
            URtoDF = MergeURtoULandUBtoDF[URtoUL][UBtoDF];
    }

    // Load or create all move tables
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    static void initMoveTables();

    // ******************************************Phase 1 move tables*****************************************************

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Move table for the twists of the corners
    // twist < 2187 in phase 2.
    // twist = 0 in phase 2.
    static short twistMove[N_TWIST][N_MOVE];
    static void initTwistMove();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Move table for the flips of the edges
    // flip < 2048 in phase 1
    // flip = 0 in phase 2.
    static short flipMove[N_FLIP][N_MOVE];
    static void initFlipMove();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Parity of the corner permutation. This is the same as the parity for the edge permutation of a valid cube.
    // parity has values 0 and 1
    // Parity changes every 90 degree turn
    static short parityMove[2][N_MOVE];

    // ***********************************Phase 1 and 2 movetable********************************************************

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Move table for the four UD-slice edges FR, FL, Bl and BR
    // FRtoBRMove < 11880 in phase 1
    // FRtoBRMove < 24 in phase 2
    // FRtoBRMove = 0 for solved cube
    static short FRtoBR_Move[N_FRtoBR][N_MOVE];
    static void initFRtoBR_Move();

    // *******************************************Phase 1 and 2 movetable************************************************

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Move table for permutation of six corners. The positions of the DBL and DRB corners are determined by the parity.
    // URFtoDLF < 20160 in phase 1
    // URFtoDLF < 20160 in phase 2
    // URFtoDLF = 0 for solved cube.
    static short URFtoDLF_Move[N_URFtoDLF][N_MOVE];
    static void initURFtoDLF_Move();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Move table for the permutation of six U-face and D-face edges in phase2. The positions of the DL and DB edges are
    // determined by the parity.
    // URtoDF < 665280 in phase 1
    // URtoDF < 20160 in phase 2
    // URtoDF = 0 for solved cube.
    static short URtoDF_Move[N_URtoDF][N_MOVE];
    static void initURtoDF_Move();

    // **************************helper move tables to compute URtoDF for the beginning of phase2************************

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Move table for the three edges UR,UF and UL in phase1.
    static short URtoUL_Move[N_URtoUL][N_MOVE];
    static void initURtoUL_Move();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Move table for the three edges UB,DR and DF in phase1.
    static short UBtoDF_Move[N_UBtoDF][N_MOVE];
    static void initUBtoDF_Move();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Table to merge the coordinates of the UR,UF,UL and UB,DR,DF edges at the beginning of phase2
    static short MergeURtoULandUBtoDF[336][336];
    static void initMergeURtoULandUBtoDF();

    // ****************************************Pruning tables for the search*********************************************

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Pruning table for the permutation of the corners and the UD-slice edges in phase2.
    // The pruning table entries give a lower estimation for the number of moves to reach the solved cube.
    static byte Slice_URFtoDLF_Parity_Prun[N_SLICE2 * N_URFtoDLF * N_PARITY / 2];
    static void initSlice_URFtoDLF_Parity_Prun();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Pruning table for the permutation of the edges in phase2.
    // The pruning table entries give a lower estimation for the number of moves to reach the solved cube.
    static byte Slice_URtoDF_Parity_Prun[N_SLICE2 * N_URtoDF * N_PARITY / 2];
    static void initSlice_URtoDF_Parity_Prun();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Pruning table for the twist of the corners and the position (not permutation) of the UD-slice edges in phase1
    // The pruning table entries give a lower estimation for the number of moves to reach the H-subgroup.
    static byte Slice_Twist_Prun[N_SLICE1 * N_TWIST / 2 + 1];
    static void initSlice_Twist_Prun();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Pruning table for the flip of the edges and the position (not permutation) of the UD-slice edges in phase1
    // The pruning table entries give a lower estimation for the number of moves to reach the H-subgroup.
    static byte Slice_Flip_Prun[N_SLICE1 * N_FLIP / 2];
    static void initSlice_Flip_Prun();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Set pruning value in table. Two values are stored in one byte.
    static void setPruning(byte table[], int index, byte value) {
        if ((index & 1) == 0)
            table[index >> 1] &= 0xf0 | value;
        else
            table[index >> 1] &= 0x0f | (value << 4);
    }

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Extract pruning value
    static byte getPruning(byte table[], int index) {
        if ((index & 1) == 0)
            return (byte) (table[index >> 1] & 0x0f);
        else
            return (byte) ((table[index >> 1] & 0xf0) >> 4);
    }
};

}
