#include <cstdio>

#include "CoordCube.hpp"
#include "CubieCube.hpp"

namespace twophase {

CoordCube::CoordCube(const CubieCube &c) {
    static bool initialized = false;
    if (!initialized)
    {
        initMoveTables();
        initialized = true;
    }
    twist = c.getTwist();
    flip = c.getFlip();
    parity = c.cornerParity();
    FRtoBR = c.getFRtoBR();
    URFtoDLF = c.getURFtoDLF();
    URtoUL = c.getURtoUL();
    UBtoDF = c.getUBtoDF();
    URtoDF = c.getURtoDF();// only needed in phase2
}

// Load or create all move tables
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void CoordCube::initMoveTables() {
    FILE * stream = 0;
    // Try to load move tables from disk
    stream = fopen("movetables", "rb");
    if (stream != NULL) {
        size_t bytes = 0;
#define LOAD(t) { bytes += fread( t, sizeof(char), sizeof(t), stream ); }
        LOAD(twistMove);
        LOAD(flipMove);
        LOAD(FRtoBR_Move);
        LOAD(URFtoDLF_Move);
        LOAD(URtoDF_Move);
        LOAD(URtoUL_Move);
        LOAD(UBtoDF_Move);
        LOAD(MergeURtoULandUBtoDF);
        LOAD(Slice_URFtoDLF_Parity_Prun);
        LOAD(Slice_URtoDF_Parity_Prun);
        LOAD(Slice_Twist_Prun);
        LOAD(Slice_Flip_Prun);
#undef LOAD
        fclose(stream);

        // Make sure the tables were loaded
        if ( bytes ==
                sizeof(twistMove) +
                sizeof(flipMove) +
                sizeof(FRtoBR_Move) +
                sizeof(URFtoDLF_Move) +
                sizeof(URtoDF_Move) +
                sizeof(URtoUL_Move) +
                sizeof(UBtoDF_Move) +
                sizeof(MergeURtoULandUBtoDF) +
                sizeof(Slice_URFtoDLF_Parity_Prun) +
                sizeof(Slice_URtoDF_Parity_Prun) +
                sizeof(Slice_Twist_Prun) +
                sizeof(Slice_Flip_Prun) ) {
            fprintf(stderr, "Loaded move tables from disk.\n"); fflush(stdout);
            return; // Load was successful
        } else {
            fprintf(stderr, "Failed to load move tables from disk.\n"); fflush(stdout);
        }
    }

    // Build move tables
    fprintf(stderr, "Building move tables\n"); fflush(stderr);
    initTwistMove();
    fprintf(stderr, "."); fflush(stderr);
    initFlipMove();
    fprintf(stderr, "."); fflush(stderr);
    initFRtoBR_Move();
    fprintf(stderr, "."); fflush(stderr);
    initURFtoDLF_Move();
    fprintf(stderr, "."); fflush(stderr);
    initURtoDF_Move();
    fprintf(stderr, "."); fflush(stdout);
    initURtoUL_Move();
    fprintf(stderr, "."); fflush(stdout);
    initUBtoDF_Move();
    fprintf(stderr, "."); fflush(stderr);
    initMergeURtoULandUBtoDF();
    fprintf(stderr, "."); fflush(stderr);
    initSlice_URFtoDLF_Parity_Prun();
    fprintf(stderr, "."); fflush(stderr);
    initSlice_URtoDF_Parity_Prun();
    fprintf(stderr, "."); fflush(stderr);
    initSlice_Twist_Prun();
    fprintf(stderr, "."); fflush(stderr);
    initSlice_Flip_Prun();
    fprintf(stderr, "."); fflush(stderr);
    fprintf(stderr, "Done\n"); fflush(stderr);

    // Store move tables to disk
    stream = fopen("movetables", "wb");
    if (stream != NULL) {
        size_t bytes = 0;
#define STORE(t) { bytes += fwrite( t, sizeof(char), sizeof(t), stream ); }
        STORE(twistMove);
        STORE(flipMove);
        STORE(FRtoBR_Move);
        STORE(URFtoDLF_Move);
        STORE(URtoDF_Move);
        STORE(URtoUL_Move);
        STORE(UBtoDF_Move);
        STORE(MergeURtoULandUBtoDF);
        STORE(Slice_URFtoDLF_Parity_Prun);
        STORE(Slice_URtoDF_Parity_Prun);
        STORE(Slice_Twist_Prun);
        STORE(Slice_Flip_Prun);
#undef STORE
        fclose(stream);
        // Make sure the tables were stored
        if ( bytes ==
                sizeof(twistMove) +
                sizeof(flipMove) +
                sizeof(FRtoBR_Move) +
                sizeof(URFtoDLF_Move) +
                sizeof(URtoDF_Move) +
                sizeof(URtoUL_Move) +
                sizeof(UBtoDF_Move) +
                sizeof(MergeURtoULandUBtoDF) +
                sizeof(Slice_URFtoDLF_Parity_Prun) +
                sizeof(Slice_URtoDF_Parity_Prun) +
                sizeof(Slice_Twist_Prun) +
                sizeof(Slice_Flip_Prun) ) {
            fprintf(stderr, "Stored move tables to disk.\n"); fflush(stderr);
        } else {
            fprintf(stderr, "Failed to store move tables to disk.\n"); fflush(stderr);
        }
    }
}

// ******************************************Phase 1 move tables*****************************************************

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Move table for the twists of the corners
// twist < 2187 in phase 2.
// twist = 0 in phase 2.
short CoordCube::twistMove[N_TWIST][N_MOVE];
void CoordCube::initTwistMove()
{
    CubieCube a;
    for (short i = 0; i < N_TWIST; i++) {
        a.setTwist(i);
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 4; ++k) {
                for (int m = 3; m >= 0; --m) {
                    a.cornerMultiply(CubieCube::moveCube[j+3]);
                    twistMove[i][16 * j + 4*k + m] = a.getTwist();
                }
                a.cornerMultiply(CubieCube::moveCube[j]);
            }
        }
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Move table for the flips of the edges
// flip < 2048 in phase 1
// flip = 0 in phase 2.
short CoordCube::flipMove[N_FLIP][N_MOVE];
void CoordCube::initFlipMove() {
    CubieCube a;
    for (short i = 0; i < N_FLIP; ++i) {
        a.setFlip(i);
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 4; ++k) {
                for (int m = 3; m >= 0; --m) {
                    a.edgeMultiply(CubieCube::moveCube[j+3]);
                    flipMove[i][16 * j + 4*k + m] = a.getFlip();
                }
                a.edgeMultiply(CubieCube::moveCube[j]);
            }
        }
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Parity of the corner permutation. This is the same as the parity for the edge permutation of a valid cube.
// parity has values 0 and 1
// Parity changes every 90 degree turn
short CoordCube::parityMove[2][N_MOVE] = {
        { 0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0, 0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0, 0,1,0,1,1,0,1,0,0,1,0,1,1,0,1,0 },
        { 1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1, 1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1, 1,0,1,0,0,1,0,1,1,0,1,0,0,1,0,1 } };

// ***********************************Phase 1 and 2 movetable********************************************************

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Move table for the four UD-slice edges FR, FL, Bl and BR
// FRtoBRMove < 11880 in phase 1
// FRtoBRMove < 24 in phase 2
// FRtoBRMove = 0 for solved cube
short CoordCube::FRtoBR_Move[N_FRtoBR][N_MOVE];
void CoordCube::initFRtoBR_Move() {
    CubieCube a;
    for (short i = 0; i < N_FRtoBR; ++i) {
        a.setFRtoBR(i);
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 4; ++k) {
                for (int m = 3; m >= 0; --m) {
                    a.edgeMultiply(CubieCube::moveCube[j+3]);
                    FRtoBR_Move[i][16 * j + 4*k + m] = a.getFRtoBR();
                }
                a.edgeMultiply(CubieCube::moveCube[j]);
            }
        }
    }
}

// *******************************************Phase 1 and 2 movetable************************************************

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Move table for permutation of six corners. The positions of the DBL and DRB corners are determined by the parity.
// URFtoDLF < 20160 in phase 1
// URFtoDLF < 20160 in phase 2
// URFtoDLF = 0 for solved cube.
short CoordCube::URFtoDLF_Move[N_URFtoDLF][N_MOVE];
void CoordCube::initURFtoDLF_Move() {
    CubieCube a;
    for (short i = 0; i < N_URFtoDLF; ++i) {
        a.setURFtoDLF(i);
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 4; ++k) {
                for (int m = 3; m >= 0; --m) {
                    a.cornerMultiply(CubieCube::moveCube[j+3]);
                    URFtoDLF_Move[i][16 * j + 4*k + m] = a.getURFtoDLF();
                }
                a.cornerMultiply(CubieCube::moveCube[j]);
            }
        }
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Move table for the permutation of six U-face and D-face edges in phase2. The positions of the DL and DB edges are
// determined by the parity.
// URtoDF < 665280 in phase 1
// URtoDF < 20160 in phase 2
// URtoDF = 0 for solved cube.
short CoordCube::URtoDF_Move[N_URtoDF][N_MOVE];
void CoordCube::initURtoDF_Move() {
    CubieCube a;
    for (short i = 0; i < N_URtoDF; ++i) {
        a.setURtoDF(i);
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 4; ++k) {
                for (int m = 3; m >= 0; --m) {
                    a.edgeMultiply(CubieCube::moveCube[j+3]);
                    URtoDF_Move[i][16 * j + 4*k + m] = (short) a.getURtoDF();
                    // Table values are only valid for phase 2 moves!
                    // For phase 1 moves, casting to short is not possible.
                }
                a.edgeMultiply(CubieCube::moveCube[j]);
            }
        }
    }
}

// **************************helper move tables to compute URtoDF for the beginning of phase2************************

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Move table for the three edges UR,UF and UL in phase1.
short CoordCube::URtoUL_Move[N_URtoUL][N_MOVE];
void CoordCube::initURtoUL_Move() {
    CubieCube a;
    for (short i = 0; i < N_URtoUL; ++i) {
        a.setURtoUL(i);
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 4; ++k) {
                for (int m = 3; m >= 0; --m) {
                    a.edgeMultiply(CubieCube::moveCube[j+3]);
                    URtoUL_Move[i][16 * j + 4*k + m] = a.getURtoUL();
                }
                a.edgeMultiply(CubieCube::moveCube[j]);
            }
        }
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Move table for the three edges UB,DR and DF in phase1.
short CoordCube::UBtoDF_Move[N_UBtoDF][N_MOVE];
void CoordCube::initUBtoDF_Move() {
    CubieCube a;
    for (short i = 0; i < N_UBtoDF; ++i) {
        a.setUBtoDF(i);
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 4; ++k) {
                for (int m = 3; m >= 0; --m) {
                    a.edgeMultiply(CubieCube::moveCube[j+3]);
                    UBtoDF_Move[i][16 * j + 4*k + m] = a.getUBtoDF();
                }
                a.edgeMultiply(CubieCube::moveCube[j]);
            }
        }
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Table to merge the coordinates of the UR,UF,UL and UB,DR,DF edges at the beginning of phase2
short CoordCube::MergeURtoULandUBtoDF[336][336];
void CoordCube::initMergeURtoULandUBtoDF() {
    // for i, j <336 the six edges UR,UF,UL,UB,DR,DF are not in the
    // UD-slice and the index is <20160
    for (short uRtoUL = 0; uRtoUL < 336; uRtoUL++) {
        for (short uBtoDF = 0; uBtoDF < 336; uBtoDF++) {
            MergeURtoULandUBtoDF[uRtoUL][uBtoDF] = (short) CubieCube::getURtoDF(uRtoUL, uBtoDF);
        }
    }
}

// ****************************************Pruning tables for the search*********************************************

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Pruning table for the permutation of the corners and the UD-slice edges in phase2.
// The pruning table entries give a lower estimation for the number of moves to reach the solved cube.
byte CoordCube::Slice_URFtoDLF_Parity_Prun[N_SLICE2 * N_URFtoDLF * N_PARITY / 2];
void CoordCube::initSlice_URFtoDLF_Parity_Prun() {
    for (int i = 0; i < N_SLICE2 * N_URFtoDLF * N_PARITY / 2; i++)
        Slice_URFtoDLF_Parity_Prun[i] = (byte)-1;
    int depth = 0;
    setPruning(Slice_URFtoDLF_Parity_Prun, 0, (byte) 0);
    int done = 1;
    while (done != N_SLICE2 * N_URFtoDLF * N_PARITY) {
        for (int i = 0; i < N_SLICE2 * N_URFtoDLF * N_PARITY; i++) {
            int parity = i % 2;
            int URFtoDLF = (i / 2) / N_SLICE2;
            int slice = (i / 2) % N_SLICE2;
            if (getPruning(Slice_URFtoDLF_Parity_Prun, i) == depth) {
                for (int j = 1; j < N_MOVE; j++) {
                    if ((j&15) == 0) // Skip null moves
                        continue;
                    if (j>15 && (j & 5) != 0) // Disallow 90 degree turns on axis two and three
                        continue;
                    int newSlice = FRtoBR_Move[slice][j];
                    int newURFtoDLF = URFtoDLF_Move[URFtoDLF][j];
                    int newParity = parityMove[parity][j];
                    if (getPruning(Slice_URFtoDLF_Parity_Prun, (N_SLICE2 * newURFtoDLF + newSlice) * 2 + newParity) == 0x0f) {
                        setPruning(Slice_URFtoDLF_Parity_Prun, (N_SLICE2 * newURFtoDLF + newSlice) * 2 + newParity,
                                (byte) (depth + 1));
                        done++;
                    }
                }
            }
        }
        depth++;
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Pruning table for the permutation of the edges in phase2.
// The pruning table entries give a lower estimation for the number of moves to reach the solved cube.
byte CoordCube::Slice_URtoDF_Parity_Prun[N_SLICE2 * N_URtoDF * N_PARITY / 2];
void CoordCube::initSlice_URtoDF_Parity_Prun() {
    for (int i = 0; i < N_SLICE2 * N_URtoDF * N_PARITY / 2; i++)
        Slice_URtoDF_Parity_Prun[i] = (byte)-1;
    int depth = 0;
    setPruning(Slice_URtoDF_Parity_Prun, 0, (byte) 0);
    int done = 1;
    while (done != N_SLICE2 * N_URtoDF * N_PARITY) {
        for (int i = 0; i < N_SLICE2 * N_URtoDF * N_PARITY; i++) {
            int parity = i % 2;
            int URtoDF = (i / 2) / N_SLICE2;
            int slice = (i / 2) % N_SLICE2;
            if (getPruning(Slice_URtoDF_Parity_Prun, i) == depth) {
                for (int j = 1; j < N_MOVE; j++) {
                    if ((j&15) == 0) // Skip null moves
                        continue;
                    if (j>15 && (j & 5) != 0) // Disallow 90 degree turns on axis two and three
                        continue;
                    int newSlice = FRtoBR_Move[slice][j];
                    int newURtoDF = URtoDF_Move[URtoDF][j];
                    int newParity = parityMove[parity][j];
                    if (getPruning(Slice_URtoDF_Parity_Prun, (N_SLICE2 * newURtoDF + newSlice) * 2 + newParity) == 0x0f) {
                        setPruning(Slice_URtoDF_Parity_Prun, (N_SLICE2 * newURtoDF + newSlice) * 2 + newParity,
                                (byte) (depth + 1));
                        done++;
                    }
                }
            }
        }
        depth++;
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Pruning table for the twist of the corners and the position (not permutation) of the UD-slice edges in phase1
// The pruning table entries give a lower estimation for the number of moves to reach the H-subgroup.
byte CoordCube::Slice_Twist_Prun[N_SLICE1 * N_TWIST / 2 + 1];
void CoordCube::initSlice_Twist_Prun() {
    for (int i = 0; i < N_SLICE1 * N_TWIST / 2 + 1; i++)
        Slice_Twist_Prun[i] = (byte)-1;
    int depth = 0;
    setPruning(Slice_Twist_Prun, 0, (byte) 0);
    int done = 1;
    while (done != N_SLICE1 * N_TWIST) {
        for (int i = 0; i < N_SLICE1 * N_TWIST; i++) {
            int twist = i / N_SLICE1, slice = i % N_SLICE1;
            if (getPruning(Slice_Twist_Prun, i) == depth) {
                for (int j = 1; j < N_MOVE; j++) {
                    if ((j&15) == 0) // Skip null moves
                        continue;
                    int newSlice = FRtoBR_Move[slice * 24][j] / 24;
                    int newTwist = twistMove[twist][j];
                    if (getPruning(Slice_Twist_Prun, N_SLICE1 * newTwist + newSlice) == 0x0f) {
                        setPruning(Slice_Twist_Prun, N_SLICE1 * newTwist + newSlice, (byte) (depth + 1));
                        done++;
                    }
                }
            }
        }
        depth++;
    }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Pruning table for the flip of the edges and the position (not permutation) of the UD-slice edges in phase1
// The pruning table entries give a lower estimation for the number of moves to reach the H-subgroup.
byte CoordCube::Slice_Flip_Prun[N_SLICE1 * N_FLIP / 2];
void CoordCube::initSlice_Flip_Prun() {
    for (int i = 0; i < N_SLICE1 * N_FLIP / 2; i++)
        Slice_Flip_Prun[i] = -1;
    int depth = 0;
    setPruning(Slice_Flip_Prun, 0, (byte) 0);
    int done = 1;
    while (done != N_SLICE1 * N_FLIP) {
        for (int i = 0; i < N_SLICE1 * N_FLIP; i++) {
            int flip = i / N_SLICE1, slice = i % N_SLICE1;
            if (getPruning(Slice_Flip_Prun, i) == depth) {
                for (int j = 1; j < N_MOVE; j++) {
                    if ((j&15) == 0) // Skip null moves
                        continue;
                    int newSlice = FRtoBR_Move[slice * 24][j] / 24;
                    int newFlip = flipMove[flip][j];
                    if (getPruning(Slice_Flip_Prun, N_SLICE1 * newFlip + newSlice) == 0x0f) {
                        setPruning(Slice_Flip_Prun, N_SLICE1 * newFlip + newSlice, (byte) (depth + 1));
                        done++;
                    }
                }
            }
        }
        depth++;
    }
}

}
