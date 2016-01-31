#include "FaceCube.hpp"
#include "CubieCube.hpp"

namespace twophase {

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Map the corner positions to facelet positions. cornerFacelet[URF][0] e.g. gives the position of the
// facelet in the URF corner position, which defines the orientation.<br>
// cornerFacelet[URF][1] and cornerFacelet[URF][2] give the position of the other two facelets
// of the URF corner (clockwise).
const Facelet FaceCube::cornerFacelet[8][3] = { { U9, R1, F3 }, { U7, F1, L3 }, { U1, L1, B3 }, { U3, B1, R3 },
        { D3, F9, R7 }, { D1, L9, F7 }, { D7, B9, L7 }, { D9, R9, B7 } };

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Map the edge positions to facelet positions. edgeFacelet[UR][0] e.g. gives the position of the facelet in
// the UR edge position, which defines the orientation.<br>
// edgeFacelet[UR][1] gives the position of the other facelet
const Facelet FaceCube::edgeFacelet[12][2] = { { U6, R2 }, { U8, F2 }, { U4, L2 }, { U2, B2 }, { D6, R8 }, { D2, F8 },
        { D4, L8 }, { D8, B8 }, { F6, R4 }, { F4, L6 }, { B6, L4 }, { B4, R6 } };

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Map the corner positions to facelet colors.
const Color FaceCube::cornerColor[8][3] = { { U, R, F }, { U, F, L }, { U, L, B }, { U, B, R }, { D, F, R }, { D, L, F },
        { D, B, L }, { D, R, B } };

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Map the edge positions to facelet colors.
const Color FaceCube::edgeColor[12][2] = { { U, R }, { U, F }, { U, L }, { U, B }, { D, R }, { D, F }, { D, L }, { D, B },
        { F, R }, { F, L }, { B, L }, { B, R } };

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Gives CubieCube representation of a faceletcube
CubieCube FaceCube::toCubieCube() {
    // Compensate for rotation of the whole cube
    Color t[6] = {U, R, F, D, L, B};
    for (int i=0; i < 6; ++i) {
        t[f[4+i*9]] = (Color)i;
    }

    short ori;
    CubieCube ccRet;
    for (int i = 0; i < 8; i++)
        ccRet.cp[i] = URF;// invalidate corners
    for (int i = 0; i < 12; i++)
        ccRet.ep[i] = UR;// and edges
    Color col1, col2;
    for (int i = 0; i < 8; ++i) {
        // get the colors of the cubie at corner i, starting with U/D
        for (ori = 0; ori < 3; ori++)
            if (t[f[cornerFacelet[i][ori]]] == U || t[f[cornerFacelet[i][ori]]] == D)
                break;
        col1 = t[f[cornerFacelet[i][(ori + 1) % 3]]];
        col2 = t[f[cornerFacelet[i][(ori + 2) % 3]]];

        for (int j = 0; j < 8; ++j) {
            if (col1 == cornerColor[j][1] && col2 == cornerColor[j][2]) {
                // in cornerposition i we have cornercubie j
                ccRet.cp[i] = (Corner)j;
                ccRet.co[i] = (byte) (ori % 3);
                break;
            }
        }
    }
    for (int i = 0; i < 12; ++i)
        for (int j = 0; j < 12; ++j) {
            if (t[f[edgeFacelet[i][0]]] == edgeColor[j][0]
                    && t[f[edgeFacelet[i][1]]] == edgeColor[j][1]) {
                ccRet.ep[i] = (Edge)j;
                ccRet.eo[i] = 0;
                break;
            }
            if (t[f[edgeFacelet[i][0]]] == edgeColor[j][1]
                    && t[f[edgeFacelet[i][1]]] == edgeColor[j][0]) {
                ccRet.ep[i] = (Edge)j;
                ccRet.eo[i] = 1;
                break;
            }
        }
    return ccRet;
};

}
