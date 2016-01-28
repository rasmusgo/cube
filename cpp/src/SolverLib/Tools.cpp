/*
 * Tools.cpp
 *
 *  Created on: 2011-jun-28
 *      Author: morotspaj
 */

#include "Tools.h"
#include "CoordCube.h"
#include "CubieCube.h"
#include "FaceCube.h"

#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <string>

namespace twophase {

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Check if the cube string s represents a solvable cube.
// 0: Cube is solvable
// -1: There is not exactly one facelet of each colour
// -2: Not all 12 edges exist exactly once
// -3: Flip error: One edge has to be flipped
// -4: Not all corners exist exactly once
// -5: Twist error: One corner has to be twisted
// -6: Parity error: Two corners or two edges have to be exchanged
//
/**
 * Check if the cube definition string s represents a solvable cube.
 *
 * @param s is the cube definition string , see {@link Facelet}
 * @return 0: Cube is solvable<br>
 *         -1: There is not exactly one facelet of each colour<br>
 *         -2: Not all 12 edges exist exactly once<br>
 *         -3: Flip error: One edge has to be flipped<br>
 *         -4: Not all 8 corners exist exactly once<br>
 *         -5: Twist error: One corner has to be twisted<br>
 *         -6: Parity error: Two corners or two edges have to be exchanged
 */
int Tools::verify(std::string s) {
	int count[6];
	try {
		for (int i = 0; i < 54; i++)
			++count[FaceCube::charToColor(s[i])];
	} catch (...) {
		return -1;
	}

	for (int i = 0; i < 6; i++)
		if (count[i] != 9)
			return -1;

	FaceCube fc(s);
	CubieCube cc = fc.toCubieCube();

	return cc.verify();
}

/**
 * Generates a random cube.
 * @return A random cube in the string representation. Each cube of the cube space has the same probability.
 */
std::string Tools::randomCube() {
	static bool initialized = false;
	if (!initialized) {
		srand(time(NULL));
		initialized = true;
	}

	CubieCube cc;
	cc.setFlip((short) (rand() % CoordCube::N_FLIP));
	cc.setTwist((short) (rand() % CoordCube::N_TWIST));
	do {
		cc.setURFtoDLB((rand() % CoordCube::N_URFtoDLB));
		cc.setURtoBR((rand() % CoordCube::N_URtoBR));
	} while ((cc.edgeParity() ^ cc.cornerParity()) != 0);

	FaceCube fc(cc);
	return fc.to_String();
}

}
