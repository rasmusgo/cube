/*
 * FaceCube.h
 *
 *  Created on: 2011-jun-23
 *      Author: morotspaj
 */
// TODO: fix appropriate credit line to kociemba

#ifndef FACECUBE_H_
#define FACECUBE_H_

#include "common.h"
#include "CubieCube.h"

#include <cstdio>
#include <string>

namespace twophase {

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Cube on the facelet level
class FaceCube {
public:

	Color f[54];

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Map the corner positions to facelet positions. cornerFacelet[URF][0] e.g. gives the position of the
	// facelet in the URF corner position, which defines the orientation.<br>
	// cornerFacelet[URF][1] and cornerFacelet[URF][2] give the position of the other two facelets
	// of the URF corner (clockwise).
	const static Facelet cornerFacelet[8][3];

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Map the edge positions to facelet positions. edgeFacelet[UR][0] e.g. gives the position of the facelet in
	// the UR edge position, which defines the orientation.<br>
	// edgeFacelet[UR][1] gives the position of the other facelet
	const static Facelet edgeFacelet[12][2];

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Map the corner positions to facelet colors.
	const static Color cornerColor[8][3];

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Map the edge positions to facelet colors.
	const static Color edgeColor[12][2];

	static char colorToChar(Color c) {
		static const char colors[] = {'U','R','F','D','L','B'};
		if (c < 0 || c >=6) {
			static char e[64];
			sprintf(e, "Invalid color %d", (int)c);
			throw e;
		}
		return colors[c];
	}

	static Color charToColor(char c) {
		switch (c)
		{
		case 'U':
			return U;
		case 'R':
			return R;
		case 'F':
			return F;
		case 'D':
			return D;
		case 'L':
			return L;
		case 'B':
			return B;
		}
		static char e[] = "Invalid character:    ";
		e[19] = c;
		throw e;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	FaceCube() {
		int k = 0;
		for (int i = 0; i < 6; ++i)
			for (int j = 0; j < 9; ++j)
				f[k++] = (Color)i;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Construct a facelet cube from a string
	FaceCube(std::string cubeString) {
		for (unsigned int i = 0; i < cubeString.length(); i++)
			f[i] = charToColor(cubeString[i]);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Construct a facelet cube from a CubieCube
	FaceCube(const CubieCube &cc) {
		fromCubieCube(cc);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Copy constructor
	FaceCube(const FaceCube &fc) {
		for (unsigned int i = 0; i < 54; ++i)
			f[i] = fc.f[i];
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void fromCubieCube(const CubieCube &cc) {
		for (int i = 0; i < 6; ++i)
			f[i*9+4] = (Color)i;

		for (int i = 0; i < 8; ++i) {
			int j = cc.cp[i];// cornercubie with index j is at
			if (j<0 || j>=8)
				throw "Invalid cornercubie";
			// cornerposition with index i
			byte ori = cc.co[i];// Orientation of this cubie
			for (int n = 0; n < 3; n++)
				f[FaceCube::cornerFacelet[i][(n + ori) % 3]] = FaceCube::cornerColor[j][n];
		}
		for (int i = 0; i < 12; ++i) {
			int j = cc.ep[i];// edgecubie with index j is at edgeposition
			if (j<0 || j>=12)
				throw "Invalid edgecubie";
			// with index i
			byte ori = cc.eo[i];// Orientation of this cubie
			for (int n = 0; n < 2; n++)
				f[FaceCube::edgeFacelet[i][(n + ori) % 2]] = FaceCube::edgeColor[j][n];
		}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Gives string representation of a facelet cube
	std::string to_String() {
		std::string s = "";
		for (int i = 0; i < 54; i++)
			s += colorToChar(f[i]);
		return s;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Gives CubieCube representation of a faceletcube
	CubieCube toCubieCube();

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Moves the cube in U axis (Y axis)
	void moveU(int a, int b, int c) {
		moveAxis(0, a, b, c);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Moves the cube in R axis (X axis)
	void moveR(int a, int b, int c) {
		moveAxis(1, a, b, c);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Moves the cube in F axis (Z axis)
	void moveF(int a, int b, int c) {
		moveAxis(2, a, b, c);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Moves the cube in an axis (U, R or F axis)
	void moveAxis(int axis, int a, int b, int c) {
		// TODO: Handle axis out of bounds
		// Starting index on closest and most distant face for each axis
		static const int sf[3][2] = {
				{ U1, D1 },
				{ R1, L1 },
				{ F1, B1 }
		};
		// Inner face cycle, clockwise
		static const int c1[8] = { 0, 1, 2, 5, 8, 7, 6, 3 };

		// Outer faces cycles, clockwise
		// axis, layer, cycle element
		static const int c2[3][3][12] = {
				{ // U
						{ F3, F2, F1, L3, L2, L1, B3, B2, B1, R3, R2, R1 },
						{ F6, F5, F4, L6, L5, L4, B6, B5, B4, R6, R5, R4 },
						{ F9, F8, F7, L9, L8, L7, B9, B8, B7, R9, R8, R7 }
				},
				{ // R
						{ F9, F6, F3, U9, U6, U3, B1, B4, B7, D9, D6, D3 },
						{ F8, F5, F2, U8, U5, U2, B2, B5, B8, D8, D5, D2 },
						{ F7, F4, F1, U7, U4, U1, B3, B6, B9, D7, D4, D1 }
				},
				{ // F
						{ U7, U8, U9, R1, R4, R7, D3, D2, D1, L9, L6, L3 },
						{ U4, U5, U6, R2, R5, R8, D6, D5, D4, L8, L5, L2 },
						{ U1, U2, U3, R3, R6, R9, D9, D8, D7, L7, L4, L1 }
				}
		};

		// Put the rotations into 0,1,2,3
		a = -a & 3;
		b = -b & 3;
		c = -c & 3;

		// Create temporary copy to read from
		FaceCube fc(*this);

		// Rotate closest and most distant face
		int o1[2] = {a*2, (-c & 3)*2 }; // Offset
		for (int layer = 0; layer < 2; ++layer) {
			for (int i = 0; i < 8; ++i)
				f[sf[axis][layer] + c1[i]] = fc.f[ sf[axis][layer] + c1[(i+o1[layer]) % 8] ];
		}

		// Rotate faces on perpendicular sides
		int o2[3] = {a*3, b*3, c*3}; // Offset
		for (int layer = 0; layer < 3; ++layer) {
			for (int i = 0; i < 12; ++i)
				f[ c2[axis][layer][i] ] = fc.f[ c2[axis][layer][(i+o2[layer]) % 12] ];
		}
	}
};

}

#endif /* FACECUBE_H_ */
