/*
 * Search.cpp
 *
 *  Created on: 2011-jun-28
 *      Author: morotspaj
 */

#include "Search.h"
#include "FaceCube.h"
#include "CubieCube.h"
#include "CoordCube.h"

#include "cstdlib"
#include "cstdio"
#include "time.h"

namespace twophase {

int Search::ax[31]; // The axis of the move
int Search::po[31]; // The power of the move

int Search::flip[31]; // phase1 coordinates
int Search::twist[31];
int Search::slice[31];

int Search::parity[31]; // phase2 coordinates
int Search::URFtoDLF[31];
int Search::FRtoBR[31];
int Search::URtoUL[31];
int Search::UBtoDF[31];
int Search::URtoDF[31];

int Search::minDistPhase1[31]; // IDA* distance do goal estimations
int Search::minDistPhase2[31];

const char* Search::axis_name[] = {"U", "R", "F", "D", "L", "B"};
const char* Search::power_name[] = {"00", "01", "02", "03", "10", "11", "12", "13", "20", "21", "22", "23", "30", "31", "32", "33"};

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// generate the solution string from the array data
std::string Search::solutionToString(int length) {
	std::string s;
	for (int i = 0; i < length; i++) {
		s += axis_name[ax[i]];
		s += power_name[po[i]];
		s += " ";
	}
	return s;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// generate the solution string from the array data including a separator between phase1 and phase2 moves
std::string Search::solutionToString(int length, int depthPhase1) {
	std::string s;
	for (int i = 0; i < length; i++) {
		s += axis_name[ax[i]];
		s += power_name[po[i]];
		s += " ";
		if (i == depthPhase1 - 1)
			s += (". ");
	}
	return s;
};

/**
 * Computes the solver string for a given cube.
 *
 * @param facelets
 *          is the cube definition string, see {@link Facelet} for the format.
 *
 * @param maxDepth
 *          defines the maximal allowed maneuver length. For random cubes, a maxDepth of 21 usually will return a
 *          solution in less than 0.5 seconds. With a maxDepth of 20 it takes a few seconds on average to find a
 *          solution, but it may take much longer for specific cubes.
 *
 *@param timeOut
 *          defines the maximum computing time of the method in seconds. If it does not return with a solution, it returns with
 *          an error code.
 *
 * @param useSeparator
 *          determines if a " . " separates the phase1 and phase2 parts of the solver string like in F' R B R L2 F .
 *          U2 U D for example.<br>
 * @return The solution string or an error code:<br>
 *         Error 1: There is not exactly one facelet of each colour<br>
 *         Error 2: Not all 12 edges exist exactly once<br>
 *         Error 3: Flip error: One edge has to be flipped<br>
 *         Error 4: Not all corners exist exactly once<br>
 *         Error 5: Twist error: One corner has to be twisted<br>
 *         Error 6: Parity error: Two corners or two edges have to be exchanged<br>
 *         Error 7: No solution exists for the given maxDepth<br>
 *         Error 8: Timeout, no solution within given time
 */
std::string Search::solution(std::string facelets, int maxDepth, long timeOut, bool useSeparator) {
	int s;

	// +++++++++++++++++++++check for wrong input +++++++++++++++++++++++++++++
	int count[6];
	for (int i = 0; i < 6; ++i)
		count[i] = 0;
	try {
		for (int i = 0; i < 54; i++)
			++count[FaceCube::charToColor(facelets[i])];
	} catch (...) {
		return "Error 1";
	}
	for (int i = 0; i < 6; i++)
		if (count[i] != 9)
			return "Error 1";

	FaceCube fc(facelets);
	CubieCube cc(fc.toCubieCube());
	if ((s = cc.verify()) != 0) {
		char e[40];
		sprintf(e, "Error %d", abs(s));
		return e;
	}

	// +++++++++++++++++++++++ initialization +++++++++++++++++++++++++++++++++
	CoordCube c(cc);

	po[0] = 0;
	ax[0] = 0;
	flip[0] = c.flip;
	twist[0] = c.twist;
	parity[0] = c.parity;
	slice[0] = c.FRtoBR / 24;
	URFtoDLF[0] = c.URFtoDLF;
	FRtoBR[0] = c.FRtoBR;
	URtoUL[0] = c.URtoUL;
	UBtoDF[0] = c.UBtoDF;

	minDistPhase1[1] = 1;// else failure for depth=1, n=0
	int mv = 1, n = 0;
	bool busy = false; // gets set when a move is popped from the stack
	int depthPhase1 = 1;

	clock_t tStart = clock();
	timeOut *= CLOCKS_PER_SEC;

	// +++++++++++++++++++ Main loop ++++++++++++++++++++++++++++++++++++++++++
	do {
		/* Find the next node to test
		 * This is an Iterative Deepening A* search (IDA*)
		 * which means we are doing a depth first search over and over again
		 * with increasing depth limit while pruning branches which would
		 * give too many total moves.
		 *
		 * n is current depth
		 * depthPhase1 is current max depth
		 * minDistPhase1[n + 1] is the pruning value which gives a lower bound
		 * on the moves necessary after the current move
		 *
		 * the sequence of moves are stored as ax[] and po[] which together represents each move.
		 * ax[n] is the axis, 0 to 2
		 * po[n] is the power, 1 to 15 (which combination of moves on the axis)
		 */
		do { // loop for choice of node while busy
			if ((n + minDistPhase1[n + 1] < depthPhase1) && !busy) {
				// Dig deeper
				// Initialize next move, avoid previous axis
				if (ax[n] != 0)
					ax[++n] = 0;
				else
					ax[++n] = 1;
				po[n] = 1;
			} else if (++po[n] > 15) { // increment power
				do {// increment axis
					if (++ax[n] > 2) {

						if (clock() - tStart > timeOut)
							return "Error 8";

						if (n == 0) {
							if (depthPhase1 >= maxDepth)
								return "Error 7";
							else {
								// Start over with greater depth
								depthPhase1++;
								ax[n] = 0;
								po[n] = 1;
								busy = false;
								break;
							}
						} else {
							// Decrease current depth
							n--;
							busy = true;
							break;
						}

					} else {
						po[n] = 1;
						busy = false;
					}
				} while (n != 0 && (ax[n - 1] == ax[n]));
			} else
				busy = false;
		} while (busy);

		// +++++++++++++ compute new coordinates and new minDistPhase1 ++++++++++
		// if minDistPhase1 =0, the H subgroup is reached
		mv = 16 * ax[n] + po[n];
		if (mv > CoordCube::N_MOVE)
			printf("n: %d ax[n]: %d po[n]: %d mv: %d ", n, ax[n], po[n], mv);
		flip[n + 1] = CoordCube::flipMove[flip[n]][mv];
		twist[n + 1] = CoordCube::twistMove[twist[n]][mv];
		slice[n + 1] = CoordCube::FRtoBR_Move[slice[n] * 24][mv] / 24;
		minDistPhase1[n + 1] = std::max(CoordCube::getPruning(CoordCube::Slice_Flip_Prun, CoordCube::N_SLICE1 * flip[n + 1]
				+ slice[n + 1]), CoordCube::getPruning(CoordCube::Slice_Twist_Prun, CoordCube::N_SLICE1 * twist[n + 1]
				+ slice[n + 1]));
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		if (minDistPhase1[n + 1] == 0 && n >= depthPhase1 - 5) {
			minDistPhase1[n + 1] = 10;// instead of 10 any value >5 is possible
			if (n == depthPhase1 - 1 && (s = totalDepth(depthPhase1, maxDepth)) >= 0) {
				if (s == depthPhase1
						|| (ax[depthPhase1 - 1] != ax[depthPhase1] && ax[depthPhase1 - 1] != ax[depthPhase1] + 3))
					return useSeparator ? solutionToString(s, depthPhase1) : solutionToString(s);
			}

		}
	} while (true);
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Apply phase2 of algorithm and return the combined phase1 and phase2 depth. In phase2, only the moves
// U,D,R2,F2,L2 and B2 are allowed.
int Search::totalDepth(int depthPhase1, int maxDepth) {
	int mv = 0, d1 = 0, d2 = 0;
	int maxDepthPhase2 = std::min(10, maxDepth - depthPhase1);// Allow only max 10 moves in phase2
	for (int i = 0; i < depthPhase1; i++) {
		mv = 16 * ax[i] + po[i];
		URFtoDLF[i + 1] = CoordCube::URFtoDLF_Move[URFtoDLF[i]][mv];
		FRtoBR[i + 1] = CoordCube::FRtoBR_Move[FRtoBR[i]][mv];
		parity[i + 1] = CoordCube::parityMove[parity[i]][mv];
	}

	if ((d1 = CoordCube::getPruning(CoordCube::Slice_URFtoDLF_Parity_Prun,
			(CoordCube::N_SLICE2 * URFtoDLF[depthPhase1] + FRtoBR[depthPhase1]) * 2 + parity[depthPhase1])) > maxDepthPhase2)
		return -1;

	for (int i = 0; i < depthPhase1; i++) {
		mv = 16 * ax[i] + po[i];
		URtoUL[i + 1] = CoordCube::URtoUL_Move[URtoUL[i]][mv];
		UBtoDF[i + 1] = CoordCube::UBtoDF_Move[UBtoDF[i]][mv];
	}
	URtoDF[depthPhase1] = CoordCube::MergeURtoULandUBtoDF[URtoUL[depthPhase1]][UBtoDF[depthPhase1]];

	if ((d2 = CoordCube::getPruning(CoordCube::Slice_URtoDF_Parity_Prun,
			(CoordCube::N_SLICE2 * URtoDF[depthPhase1] + FRtoBR[depthPhase1]) * 2 + parity[depthPhase1])) > maxDepthPhase2)
		return -1;

	if ((minDistPhase2[depthPhase1] = std::max(d1, d2)) == 0)// already solved
		return depthPhase1;

	// now set up search

	int depthPhase2 = 1;
	int n = depthPhase1;
	bool busy = false;
	po[depthPhase1] = 0;
	ax[depthPhase1] = 0;
	minDistPhase2[n + 1] = 1;// else failure for depthPhase2=1, n=0
	// +++++++++++++++++++ end initialization +++++++++++++++++++++++++++++++++
	do {
		do {
			if ((depthPhase1 + depthPhase2 - n > minDistPhase2[n + 1]) && !busy) {
				// Dig deeper
				// Initialize next move, avoid previous axis
				if (ax[n] != 0) {
					ax[++n] = 0;
					po[n] = 1;
				}
				else {
					ax[++n] = 1;
					po[n] = 2;
				}
			} else
			{
				// increment move
				if (ax[n] == 0)
					++po[n];
				else if (po[n] == 8)
					po[n] = 10;
				else
					po[n] += 6;
				if (po[n] > 15) {
					do {// increment axis
						if (++ax[n] > 2) {
							if (n == depthPhase1) {
								if (depthPhase2 >= maxDepthPhase2)
									return -1;
								else {
									depthPhase2++;
									ax[n] = 0;
									po[n] = 2;
									busy = false;
									break;
								}
							} else {
								n--;
								busy = true;
								break;
							}

						} else {
							if (ax[n] == 0)
								po[n] = 1;
							else
								po[n] = 2;
							busy = false;
						}
					} while (n != depthPhase1 && (ax[n - 1] == ax[n]));
				} else
					busy = false;
			}
		} while (busy);
		// +++++++++++++ compute new coordinates and new minDist ++++++++++
		mv = 16 * ax[n] + po[n];

		URFtoDLF[n + 1] = CoordCube::URFtoDLF_Move[URFtoDLF[n]][mv];
		FRtoBR[n + 1] = CoordCube::FRtoBR_Move[FRtoBR[n]][mv];
		parity[n + 1] = CoordCube::parityMove[parity[n]][mv];
		URtoDF[n + 1] = CoordCube::URtoDF_Move[URtoDF[n]][mv];

		minDistPhase2[n + 1] = std::max(
				CoordCube::getPruning(CoordCube::Slice_URtoDF_Parity_Prun,
						(CoordCube::N_SLICE2 * URtoDF[n + 1] + FRtoBR[n + 1]) * 2 + parity[n + 1]),
				CoordCube::getPruning(CoordCube::Slice_URFtoDLF_Parity_Prun,
						(CoordCube::N_SLICE2 * URFtoDLF[n + 1] + FRtoBR[n + 1]) * 2 + parity[n + 1]));
		// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	} while (minDistPhase2[n + 1] != 0);
	return depthPhase1 + depthPhase2;
}

}
