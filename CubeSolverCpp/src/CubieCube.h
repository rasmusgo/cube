/*
 * CubieCube.h
 *
 *  Created on: 2011-jun-22
 *      Author: morotspaj
 */

#ifndef CUBIECUBE_H_
#define CUBIECUBE_H_

#include "common.h"

namespace twophase {

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Cube on the cubie level
class CubieCube {

public:

	// corner permutation
	typedef Corner dCorner;
	typedef Edge dEdge;

	dCorner cp[8];

	// corner orientation
	byte co[8];

	// edge permutation
	dEdge ep[12];

	// edge orientation
	byte eo[12];

	// this CubieCube array represents the 6 basic cube moves
	static CubieCube moveCube[6];

	// ************************************** Moves on the cubie level ***************************************************
private:

	void init() {
		bool initialized = false;
		if (!initialized) {
			const Corner cpU[] = { UBR, URF, UFL, ULB, DFR, DLF, DBL, DRB };
			const byte coU[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			const Edge epU[] = { UB, UR, UF, UL, DR, DF, DL, DB, FR, FL, BL, BR };
			const byte eoU[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

			const Corner cpR[] = { DFR, UFL, ULB, URF, DRB, DLF, DBL, UBR };
			const byte coR[] = { 2, 0, 0, 1, 1, 0, 0, 2 };
			const Edge epR[] = { FR, UF, UL, UB, BR, DF, DL, DB, DR, FL, BL, UR };
			const byte eoR[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

			const Corner cpF[] = { UFL, DLF, ULB, UBR, URF, DFR, DBL, DRB };
			const byte coF[] = { 1, 2, 0, 0, 2, 1, 0, 0 };
			const Edge epF[] = { UR, FL, UL, UB, DR, FR, DL, DB, UF, DF, BL, BR };
			const byte eoF[] = { 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0 };

			const Corner cpD[] = { URF, UFL, ULB, UBR, DLF, DBL, DRB, DFR };
			const byte coD[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			const Edge epD[] = { UR, UF, UL, UB, DF, DL, DB, DR, FR, FL, BL, BR };
			const byte eoD[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

			const Corner cpL[] = { URF, ULB, DBL, UBR, DFR, UFL, DLF, DRB };
			const byte coL[] = { 0, 1, 2, 0, 0, 2, 1, 0 };
			const Edge epL[] = { UR, UF, BL, UB, DR, DF, FL, DB, FR, UL, DL, BR };
			const byte eoL[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

			const Corner cpB[] = { URF, UFL, UBR, DRB, DFR, DLF, ULB, DBL };
			const byte coB[] = { 0, 0, 1, 2, 0, 0, 2, 1 };
			const Edge epB[] = { UR, UF, UL, BR, DR, DF, DL, BL, FR, FL, UB, DB };
			const byte eoB[] = { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1 };

			for (int i = 0; i < 8; i++) {
				moveCube[0].cp[i] = cpU[i];
				moveCube[0].co[i] = coU[i];
				moveCube[1].cp[i] = cpR[i];
				moveCube[1].co[i] = coR[i];
				moveCube[2].cp[i] = cpF[i];
				moveCube[2].co[i] = coF[i];
				moveCube[3].cp[i] = cpD[i];
				moveCube[3].co[i] = coD[i];
				moveCube[4].cp[i] = cpL[i];
				moveCube[4].co[i] = coL[i];
				moveCube[5].cp[i] = cpB[i];
				moveCube[5].co[i] = coB[i];
			}
			for (int i = 0; i < 12; i++) {
				moveCube[0].ep[i] = epU[i];
				moveCube[0].eo[i] = eoU[i];
				moveCube[1].ep[i] = epR[i];
				moveCube[1].eo[i] = eoR[i];
				moveCube[2].ep[i] = epF[i];
				moveCube[2].eo[i] = eoF[i];
				moveCube[3].ep[i] = epD[i];
				moveCube[3].eo[i] = eoD[i];
				moveCube[4].ep[i] = epL[i];
				moveCube[4].eo[i] = eoL[i];
				moveCube[5].ep[i] = epB[i];
				moveCube[5].eo[i] = eoB[i];
			}
			initialized = true;
		}
	}

public:

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// initialize to Id-Cube
	CubieCube() {
		init();
		for (int i = 0; i < 8; i++) {
			this->cp[i] = (Corner)i;
			this->co[i] = 0;
		}
		for (int i = 0; i < 12; i++) {
			this->ep[i] = (Edge)i;
			this->eo[i] = 0;
		}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	CubieCube(Corner cp[], byte co[], Edge ep[], byte eo[]) {
		init();
		for (int i = 0; i < 8; i++) {
			this->cp[i] = cp[i];
			this->co[i] = co[i];
		}
		for (int i = 0; i < 12; i++) {
			this->ep[i] = ep[i];
			this->eo[i] = eo[i];
		}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Copy constructor
	CubieCube(const CubieCube &cc) {
		init();
		for (int i = 0; i < 8; ++i) {
			this->cp[i] = cc.cp[i];
			this->co[i] = cc.co[i];
		}
		for (int i = 0; i < 12; ++i) {
			this->ep[i] = cc.ep[i];
			this->eo[i] = cc.eo[i];
		}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// n choose k
	static int Cnk(int n, int k) {
		int i, j, s;
		if (n < k)
			return 0;
		if (k > n / 2)
			k = n - k;
		for (s = 1, i = n, j = 1; i != n - k; i--, j++) {
			s *= i;
			s /= j;
		}
		return s;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	static void rotateLeft(dCorner arr[], int l, int r)
	// Left rotation of all array elements between l and r
	{
		dCorner temp = arr[l];
		for (int i = l; i < r; i++)
			arr[i] = arr[i + 1];
		arr[r] = temp;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	static void rotateRight(dCorner arr[], int l, int r)
	// Right rotation of all array elements between l and r
	{
		dCorner temp = arr[r];
		for (int i = r; i > l; i--)
			arr[i] = arr[i - 1];
		arr[l] = temp;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	static void rotateLeft(Edge arr[], int l, int r)
	// Left rotation of all array elements between l and r
	{
		Edge temp = arr[l];
		for (int i = l; i < r; i++)
			arr[i] = arr[i + 1];
		arr[r] = temp;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	static void rotateRight(Edge arr[], int l, int r)
	// Right rotation of all array elements between l and r
	{
		Edge temp = arr[r];
		for (int i = r; i > l; i--)
			arr[i] = arr[i - 1];
		arr[l] = temp;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// return cube in facelet representation
	/*
	FaceCube toFaceCube() {
		FaceCube fcRet;
		for (int i = 0; i < 8; ++i) {
			int j = cp[i];// cornercubie with index j is at
			// cornerposition with index i
			byte ori = co[i];// Orientation of this cubie
			for (int n = 0; n < 3; n++)
				fcRet.f[FaceCube::cornerFacelet[i][(n + ori) % 3]] = FaceCube::cornerColor[j][n];
		}
		for (int i = 0; i < 12; ++i) {
			int j = ep[i];// edgecubie with index j is at edgeposition
			// with index i
			byte ori = eo[i];// Orientation of this cubie
			for (int n = 0; n < 2; n++)
				fcRet.f[FaceCube::edgeFacelet[i][(n + ori) % 2]] = FaceCube::edgeColor[j][n];
		}
		return fcRet;
	}
	*/

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Multiply this CubieCube with another cubiecube b, restricted to the corners.<br>
	// Because we also describe reflections of the whole cube by permutations, we get a complication with the corners. The
	// orientations of mirrored corners are described by the numbers 3, 4 and 5. The composition of the orientations
	// cannot
	// be computed by addition modulo three in the cyclic group C3 any more. Instead the rules below give an addition in
	// the dihedral group D3 with 6 elements.<br>
	//
	// NOTE: Because we do not use symmetry reductions and hence no mirrored cubes in this simple implementation of the
	// Two-Phase-Algorithm, some code is not necessary here.
	//
	void cornerMultiply(const CubieCube &b) {
		dCorner cPerm[8];
		byte cOri[8];
		for (int i = 0; i < 8; ++i) {
			cPerm[i] = cp[b.cp[i]];

			byte oriA = co[b.cp[i]];
			byte oriB = b.co[i];
			byte ori = 0;

			if (oriA < 3 && oriB < 3) // if both cubes are regular cubes...
			{
				ori = (byte) (oriA + oriB); // just do an addition modulo 3 here
				if (ori >= 3)
					ori -= 3; // the composition is a regular cube

				// +++++++++++++++++++++not used in this implementation +++++++++++++++++++++++++++++++++++
			} else if (oriA < 3 && oriB >= 3) // if cube b is in a mirrored
			// state...
			{
				ori = (byte) (oriA + oriB);
				if (ori >= 6)
					ori -= 3; // the composition is a mirrored cube
			} else if (oriA >= 3 && oriB < 3) // if cube a is an a mirrored
			// state...
			{
				ori = (byte) (oriA - oriB);
				if (ori < 3)
					ori += 3; // the composition is a mirrored cube
			} else if (oriA >= 3 && oriB >= 3) // if both cubes are in mirrored
			// states...
			{
				ori = (byte) (oriA - oriB);
				if (ori < 0)
					ori += 3; // the composition is a regular cube
				// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			}
			cOri[i] = ori;
		}
		for (int i = 0; i < 8; ++i) {
			cp[i] = cPerm[i];
			co[i] = cOri[i];
		}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Multiply this CubieCube with another cubiecube b, restricted to the edges.
	void edgeMultiply(const CubieCube &b) {
		dEdge ePerm[12];
		byte eOri[12];
		for (int i = 0; i < 12; ++i) {
			ePerm[i] = ep[b.ep[i]];
			eOri[i] = (byte) ((b.eo[i] + eo[b.ep[i]]) % 2);
		}
		for (int i = 0; i < 12; ++i) {
			ep[i] = ePerm[i];
			eo[i] = eOri[i];
		}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Multiply this CubieCube with another CubieCube b.
	void multiply(CubieCube b) {
		cornerMultiply(b);
		// edgeMultiply(b);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Compute the inverse CubieCube
	void invCubieCube(CubieCube c) {
		for (int i = 0; i < 12; ++i)
			c.ep[ep[i]] = (Edge)i;
		for (int i = 0; i < 12; ++i)
			c.eo[i] = eo[c.ep[i]];
		for (int i = 0; i < 8; ++i)
			c.cp[cp[i]] = (Corner)i;
		for (int i = 0; i < 8; ++i) {
			byte ori = co[c.cp[i]];
			if (ori >= 3)// Just for completeness. We do not invert mirrored
				// cubes in the program.
				c.co[i] = ori;
			else {// the standard case
				c.co[i] = (byte) -ori;
				if (c.co[i] < 0)
					c.co[i] += 3;
			}
		}
	}

	// ********************************************* Get and set coordinates *********************************************

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// return the twist of the 8 corners. 0 <= twist < 3^7
	short getTwist() const {
		short ret = 0;
		for (int i = URF; i < DRB; ++i)
			ret = (short) (3 * ret + co[i]);
		return ret;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void setTwist(short twist) {
		int twistParity = 0;
		for (int i = DRB - 1; i >= URF; --i) {
			twistParity += co[i] = (byte) (twist % 3);
			twist /= 3;
		}
		co[DRB] = (byte) ((3 - twistParity % 3) % 3);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// return the flip of the 12 edges. 0<= flip < 2^11
	short getFlip() const {
		short ret = 0;
		for (int i = UR; i < BR; ++i)
			ret = (short) (2 * ret + eo[i]);
		return ret;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void setFlip(short flip) {
		int flipParity = 0;
		for (int i = BR - 1; i >= UR; --i) {
			flipParity += eo[i] = (byte) (flip % 2);
			flip /= 2;
		}
		eo[BR] = (byte) ((2 - flipParity % 2) % 2);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Parity of the corner permutation
	short cornerParity() const {
		int s = 0;
		for (int i = DRB; i >= URF + 1; --i)
			for (int j = i - 1; j >= URF; --j)
				if (cp[j] > cp[i])
					s++;
		return (short) (s % 2);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Parity of the edges permutation. Parity of corners and edges are the same if the cube is solvable.
	short edgeParity() const {
		int s = 0;
		for (int i = BR; i >= UR + 1; --i)
			for (int j = i - 1; j >= UR; --j)
				if (ep[j] > ep[i])
					s++;
		return (short) (s % 2);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// permutation of the UD-slice edges FR,FL,BL and BR
	short getFRtoBR() const {
		int a = 0, x = 0;
		dEdge edge4[4];
		// compute the index a < (12 choose 4) and the permutation array perm.
		for (int j = BR; j >= UR; --j)
			if (FR <= ep[j] && ep[j] <= BR) {
				a += Cnk(11 - j, x + 1);
				edge4[3 - x++] = ep[j];
			}

		int b = 0;
		for (int j = 3; j > 0; j--)// compute the index b < 4! for the
		// permutation in perm
		{
			int k = 0;
			while (edge4[j] != j + 8) {
				rotateLeft(edge4, 0, j);
				k++;
			}
			b = (j + 1) * b + k;
		}
		return (short) (24 * a + b);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void setFRtoBR(short idx) {
		int x;
		dEdge sliceEdge[] = { FR, FL, BL, BR };
		dEdge otherEdge[] = { UR, UF, UL, UB, DR, DF, DL, DB };
		int b = idx % 24; // Permutation
		int a = idx / 24; // Combination
		for (int e = 0; e < 12; ++e)
			ep[e] = DB;// Use UR to invalidate all edges

		for (int j = 1, k; j < 4; j++)// generate permutation from index b
		{
			k = b % (j + 1);
			b /= j + 1;
			while (k-- > 0)
				rotateRight(sliceEdge, 0, j);
		}

		x = 3;// generate combination and set slice edges
		for (int j = UR; j <= BR; j++)
			if (a - Cnk(11 - j, x + 1) >= 0) {
				ep[j] = sliceEdge[3 - x];
				a -= Cnk(11 - j, x-- + 1);
			}
		x = 0; // set the remaining edges UR..DB
		for (int j = UR; j <= BR; j++)
			if (ep[j] == DB)
				ep[j] = otherEdge[x++];

	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Permutation of all corners except DBL and DRB
	short getURFtoDLF() const {
		int a = 0, x = 0;
		dCorner corner6[6];
		// compute the index a < (8 choose 6) and the corner permutation.
		for (int j = URF; j <= DRB; j++)
			if (cp[j] <= DLF) {
				a += Cnk(j, x + 1);
				corner6[x++] = cp[j];
			}

		int b = 0;
		for (int j = 5; j > 0; j--)// compute the index b < 6! for the
		// permutation in corner6
		{
			int k = 0;
			while (corner6[j] != j) {
				rotateLeft(corner6, 0, j);
				k++;
			}
			b = (j + 1) * b + k;
		}
		return (short) (720 * a + b);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void setURFtoDLF(short idx) {
		int x;
		dCorner corner6[] = { URF, UFL, ULB, UBR, DFR, DLF };
		dCorner otherCorner[] = { DBL, DRB };
		int b = idx % 720; // Permutation
		int a = idx / 720; // Combination
		for (int c = 0; c < 8; ++c)
			cp[c] = DRB;// Use DRB to invalidate all corners

		for (int j = 1, k; j < 6; j++)// generate permutation from index b
		{
			k = b % (j + 1);
			b /= j + 1;
			while (k-- > 0)
				rotateRight(corner6, 0, j);
		}
		x = 5;// generate combination and set corners
		for (int j = DRB; j >= 0; j--)
			if (a - Cnk(j, x + 1) >= 0) {
				cp[j] = corner6[x];
				a -= Cnk(j, x-- + 1);
			}
		x = 0;
		for (int j = URF; j <= DRB; j++)
			if (cp[j] == DRB)
				cp[j] = otherCorner[x++];
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Permutation of the six edges UR,UF,UL,UB,DR,DF.
	int getURtoDF() const {
		int a = 0, x = 0;
		dEdge edge6[6];
		// compute the index a < (12 choose 6) and the edge permutation.
		for (int j = UR; j <= BR; j++)
			if (ep[j] <= DF) {
				a += Cnk(j, x + 1);
				edge6[x++] = ep[j];
			}

		int b = 0;
		for (int j = 5; j > 0; j--)// compute the index b < 6! for the
		// permutation in edge6
		{
			int k = 0;
			while (edge6[j] != j) {
				rotateLeft(edge6, 0, j);
				k++;
			}
			b = (j + 1) * b + k;
		}
		return 720 * a + b;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void setURtoDF(int idx) {
		int x;
		dEdge edge6[] = { UR, UF, UL, UB, DR, DF };
		dEdge otherEdge[] = { DL, DB, FR, FL, BL, BR };
		int b = idx % 720; // Permutation
		int a = idx / 720; // Combination
		for (int e = 0; e < 12; ++e)
			ep[e] = BR;// Use BR to invalidate all edges

		for (int j = 1, k; j < 6; j++)// generate permutation from index b
		{
			k = b % (j + 1);
			b /= j + 1;
			while (k-- > 0)
				rotateRight(edge6, 0, j);
		}
		x = 5;// generate combination and set edges
		for (int j = BR; j >= 0; j--)
			if (a - Cnk(j, x + 1) >= 0) {
				ep[j] = edge6[x];
				a -= Cnk(j, x-- + 1);
			}
		x = 0; // set the remaining edges DL..BR
		for (int j = UR; j <= BR; j++)
			if (ep[j] == BR)
				ep[j] = otherEdge[x++];
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Permutation of the six edges UR,UF,UL,UB,DR,DF
	static int getURtoDF(short idx1, short idx2) {
		CubieCube a;
		CubieCube b;
		a.setURtoUL(idx1);
		b.setUBtoDF(idx2);
		for (int i = 0; i < 8; i++) {
			if (a.ep[i] != BR) {
				if (b.ep[i] != BR)// collision
					return -1;
				else
					b.ep[i] = a.ep[i];
			}
		}
		return b.getURtoDF();
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Permutation of the three edges UR,UF,UL
	short getURtoUL() const {
		int a = 0, x = 0;
		dEdge edge3[3];
		// compute the index a < (12 choose 3) and the edge permutation.
		for (int j = UR; j <= BR; j++)
			if (ep[j] <= UL) {
				a += Cnk(j, x + 1);
				edge3[x++] = ep[j];
			}

		int b = 0;
		for (int j = 2; j > 0; j--)// compute the index b < 3! for the
		// permutation in edge3
		{
			int k = 0;
			while (edge3[j] != j) {
				rotateLeft(edge3, 0, j);
				k++;
			}
			b = (j + 1) * b + k;
		}
		return (short) (6 * a + b);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void setURtoUL(short idx) {
		int x;
		dEdge edge3[] = { UR, UF, UL };
		int b = idx % 6; // Permutation
		int a = idx / 6; // Combination
		for (int e = 0; e < 12; ++e)
			ep[e] = BR;// Use BR to invalidate all edges

		for (int j = 1, k; j < 3; j++)// generate permutation from index b
		{
			k = b % (j + 1);
			b /= j + 1;
			while (k-- > 0)
				rotateRight(edge3, 0, j);
		}
		x = 2;// generate combination and set edges
		for (int j = BR; j >= 0; j--)
			if (a - Cnk(j, x + 1) >= 0) {
				ep[j] = edge3[x];
				a -= Cnk(j, x-- + 1);
			}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Permutation of the three edges UB,DR,DF
	short getUBtoDF() const {
		int a = 0, x = 0;
		dEdge edge3[3];
		// compute the index a < (12 choose 3) and the edge permutation.
		for (int j = UR; j <= BR; j++)
			if (UB <= ep[j] && ep[j] <= DF) {
				a += Cnk(j, x + 1);
				edge3[x++] = ep[j];
			}

		int b = 0;
		for (int j = 2; j > 0; j--)// compute the index b < 3! for the
		// permutation in edge3
		{
			int k = 0;
			while (edge3[j] != UB + j) {
				rotateLeft(edge3, 0, j);
				k++;
			}
			b = (j + 1) * b + k;
		}
		return (short) (6 * a + b);
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void setUBtoDF(short idx) {
		int x;
		dEdge edge3[] = { UB, DR, DF };
		int b = idx % 6; // Permutation
		int a = idx / 6; // Combination
		for (int e = 0; e < 12; ++e)
			ep[e] = BR;// Use BR to invalidate all edges

		for (int j = 1, k; j < 3; j++)// generate permutation from index b
		{
			k = b % (j + 1);
			b /= j + 1;
			while (k-- > 0)
				rotateRight(edge3, 0, j);
		}
		x = 2;// generate combination and set edges
		for (int j = BR; j >= 0; j--)
			if (a - Cnk(j, x + 1) >= 0) {
				ep[j] = edge3[x];
				a -= Cnk(j, x-- + 1);
			}
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	int getURFtoDLB() const {
		dCorner perm[8];
		int b = 0;
		for (int i = 0; i < 8; i++)
			perm[i] = cp[i];
		for (int j = 7; j > 0; j--)// compute the index b < 8! for the permutation in perm
		{
			int k = 0;
			while (perm[j] != j) {
				rotateLeft(perm, 0, j);
				k++;
			}
			b = (j + 1) * b + k;
		}
		return b;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void setURFtoDLB(int idx) {
		dCorner perm[] = { URF, UFL, ULB, UBR, DFR, DLF, DBL, DRB };
		int k;
		for (int j = 1; j < 8; j++) {
			k = idx % (j + 1);
			idx /= j + 1;
			while (k-- > 0)
				rotateRight(perm, 0, j);
		}
		int x = 7;// set corners
		for (int j = 7; j >= 0; j--)
			cp[j] = perm[x--];
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	int getURtoBR() const {
		dEdge perm[12];
		int b = 0;
		for (int i = 0; i < 12; i++)
			perm[i] = ep[i];
		for (int j = 11; j > 0; j--)// compute the index b < 12! for the permutation in perm
		{
			int k = 0;
			while (perm[j] != j) {
				rotateLeft(perm, 0, j);
				k++;
			}
			b = (j + 1) * b + k;
		}
		return b;
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void setURtoBR(int idx) {
		dEdge perm[] = { UR, UF, UL, UB, DR, DF, DL, DB, FR, FL, BL, BR };
		int k;
		for (int j = 1; j < 12; j++) {
			k = idx % (j + 1);
			idx /= j + 1;
			while (k-- > 0)
				rotateRight(perm, 0, j);
		}
		int x = 11;// set edges
		for (int j = 11; j >= 0; j--)
			ep[j] = perm[x--];
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Check a cubiecube for solvability. Return the error code.
	// 0: Cube is solvable
	// -2: Not all 12 edges exist exactly once
	// -3: Flip error: One edge has to be flipped
	// -4: Not all corners exist exactly once
	// -5: Twist error: One corner has to be twisted
	// -6: Parity error: Two corners ore two edges have to be exchanged
	int verify() const {
		int sum = 0;
		int edgeCount[12];
		for (int i = 0; i < 12; ++i)
			edgeCount[i] = 0;
		for (int e = 0; e < 12; ++e)
			edgeCount[ep[e]]++;
		for (int i = 0; i < 12; i++)
			if (edgeCount[i] != 1)
				return -2;

		for (int i = 0; i < 12; i++)
			sum += eo[i];
		if (sum % 2 != 0)
			return -3;

		int cornerCount[8];
		for (int i = 0; i < 8; ++i)
			cornerCount[i] = 0;
		for (int c = 0; c < 8; ++c)
			cornerCount[cp[c]]++;
		for (int i = 0; i < 8; i++)
			if (cornerCount[i] != 1)
				return -4;// missing corners

		sum = 0;
		for (int i = 0; i < 8; i++)
			sum += co[i];
		if (sum % 3 != 0)
			return -5;// twisted corner

		if ((edgeParity() ^ cornerParity()) != 0)
			return -6;// parity error

		return 0;// cube ok
	}
};

}

#endif /* CUBIECUBE_H_ */
