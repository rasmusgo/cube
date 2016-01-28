/*
 * MoveTable.h
 *
 *  Created on: 2011-jun-28
 *      Author: morotspaj
 */

#ifndef MOVETABLE_H_
#define MOVETABLE_H_

#include "FaceCube.h"

#include <map>
#include <string>

class MoveTable {
public:
	struct TableEntry {
		int axis;
		int a;
		int b;
		int c;
		TableEntry() :
			axis(0), a(0), b(0), c(0)
		{
		}
		TableEntry(int axis, int a, int b, int c) :
			axis(axis), a(a), b(b), c(c)
		{
		}
		void call(twophase::FaceCube &fc) {
//			printf("%s: %s(%d)\n", __FILE__, __FUNCTION__, __LINE__);
//			fflush(stdout);

			fc.moveAxis(axis, a,b,c);
		}
	};

private:
	static std::map<std::string, TableEntry> table;

	// MoveTable cannot be instantiated
	MoveTable() {};
	static void init();
public:
	static const TableEntry& getMove(std::string mv);
	static void move(twophase::FaceCube &fc, std::string mv);
};

#endif /* MOVETABLE_H_ */
