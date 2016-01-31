#pragma once

#include <vector>
#include <string>
#include <sstream>

#include <SolverLib/Common.hpp>
#include <SolverLib/MoveTable.hpp>

class Sequence {
public:
	struct Animation {
		Animation() :
			time(0.025f)
		{
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j) {
					from[i][j] = 0;
					to[i][j] = 0;
				}
		}
		void add(const Animation &a) {
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					this->from[i][j] += a.from[i][j];
					this->to[i][j] += a.to[i][j];
				}
			}
		}

		friend bool chainable(const Animation &n1, const Animation n2) {
			if (n1.move.axis == n2.move.axis)
				return true;
			// Moves are chainable if the turns of the sides are in opposite directions
			int u = n1.move.a - n1.move.b;
			int d = n1.move.b - n1.move.c;
			int r = n2.move.a - n2.move.b;
			int l = n2.move.b - n2.move.c;

			if (u*d < 0 || r*l < 0) // middle slice turns
				return false;

			if (u == 2 || d == 2 || r == 2 || l == 2) // 180 degree turns
				return false;

			if ((u > 0 || d > 0) && (r > 0 || l > 0)) // both clockwise
				return false;

			if ((u < 0 || d < 0) && (r < 0 || l < 0)) // both counter-clockwise
				return false;

			return true;
		}

		// Rotations [axis][slice]
		float from[3][3];
		float to[3][3];
		float time;
		// Move to execute after animation
		MoveTable::TableEntry move;
	};

private:
	std::string original;
	std::vector<MoveTable::TableEntry> axialSeq;
	std::vector<Animation> animations;

	// Queue moves as axial moves
	void queue(const MoveTable::TableEntry &e) {
		// Merge moves on same axis
		if (axialSeq.size() > 0 && axialSeq.back().axis == e.axis) {
			// Merge with previous
			MoveTable::TableEntry &back = axialSeq.back();
			back.a = (back.a + e.a) & 3;
			back.b = (back.b + e.b) & 3;
			back.c = (back.c + e.c) & 3;
		} else {
			// Add to queue as new move
			axialSeq.push_back(MoveTable::TableEntry(e.axis, e.a & 3, e.b & 3, e.c & 3));
		}
	}

	// Take the axial moves and convert them to moves with only 90 degrees
	void to90() {
		using namespace twophase;
		// Moves with 180 degree turns can be converted into moves with only
		// 90 degree turns by allowing rotations of the center.
		// Eg. F200 becomes F133 or F311. R102 becomes R031.
		// The next move in the sequence must be adjusted to compensate for
		// the rotation of the center slice. Eg. U201 R100 becomes U130 F100

		// Transformation of the cube
		int t[6] = {U,R,F,D,L,B};
		// Rotation around axis i will make side x become y
		// rot[i][x] = y
		const int rot[3][6] = {
				{U, F, L, D, B, R}, // Rotate cube in U direction = Uc
				{B, R, U, F, L, D}, // Rotate cube in R direction = Rc
				{R, D, F, L, U, B}, // Rotate cube in F direction = Fc
		};

		for (typeof(axialSeq.begin()) it = axialSeq.begin(); it != axialSeq.end(); ++it) {
			// Apply current rotation
			it->axis = t[it->axis];

			// Convert D,L,B into U,R,F
			if (it->axis >= 3) {
				*it = MoveTable::TableEntry(
						it->axis - 3,
						(-it->c) & 3,
						(-it->b) & 3,
						(-it->a) & 3);
			}

			// Rotate around axis to remove 180 degree turns
			while (it->a == 2 || it->b == 2 || it->c == 2) {
				it->a = (it->a + 1) & 3;
				it->b = (it->b + 1) & 3;
				it->c = (it->c + 1) & 3;
				for (int i = 0; i < 6; ++i)
					t[i] = rot[it->axis][t[i]];
			}
		}
	}

	void toSigned() {
		for (typeof(axialSeq.begin()) it = axialSeq.begin(); it != axialSeq.end(); ++it) {
			it->a = ((it->a + 1) & 3) - 1;
			it->b = ((it->b + 1) & 3) - 1;
			it->c = ((it->c + 1) & 3) - 1;
		}
	}

	void animate() {
		animations.clear();
		for (typeof(axialSeq.begin()) it = axialSeq.begin(); it != axialSeq.end(); ++it)
			push_animation(*it);
	}

	void push_animation(const MoveTable::TableEntry &e) {
		// Split 180 degree turns in half
		if (e.a == 2 || e.b == 2 || e.c == 2 || e.a == -2 || e.b == -2 || e.c == -2) {
			//                -2, -1, 0, 1, 2, 3
			int to_first[] = {-1, -1, 0, 1, 1, -1};
			int to_second[] = {-1, 0, 0, 0, 1, 0};
			push_animation(MoveTable::TableEntry(e.axis, to_first[e.a+2], to_first[e.b+2], to_first[e.c+2]));
			push_animation(MoveTable::TableEntry(e.axis, to_second[e.a+2], to_second[e.b+2], to_second[e.c+2]));
			return;
		}

		// Create two keyframes
		// 1) a: From 0 to 45 degrees (tension)
		// 2) b: move
		// 3) b: From -45 to 0 degrees (relaxation)
		Animation a, b;
		a.to[e.axis][0] = e.a * 0.5;
		a.to[e.axis][1] = e.b * 0.5;
		a.to[e.axis][2] = e.c * 0.5;
		b.move = e;
		for (int i = 0; i < 3; ++i)
			b.from[e.axis][i] =	-a.to[e.axis][i];

		// Chain animations if possible
		Animation &c = animations.back();
		if (animations.size() > 0  && chainable(c, b)) {
			// Merge the second part of the last move with the first part of this move
			c.add(a);
			animations.push_back(b);
		} else {
			animations.push_back(a);
			animations.push_back(b);
		}
	}

	void compress() {
		axialSeq.clear();
		std::istringstream iss(original, std::istringstream::in);
		std::string m;
		while( iss >> m )
		{
			// Skip empty strings
			if (m == "")
				continue;
			// Queue move (will merge moves on same axis)
			queue(MoveTable::getMove(m));
		}

		// Convert into 90 degree turns
		to90();

		// Convert 3's to -1's eg. U003 to U00-1
		toSigned();

		// Create animation
		animate();

		// Chain moves by merging animations where possible
		//chain();
	}

public:
	Sequence(std::string str) :
		original(str)
	{
		compress();
	}

	int size() {
		return animations.size();
	}

	const Animation & operator [] (int i) const {
		return animations[i];
	}
};
