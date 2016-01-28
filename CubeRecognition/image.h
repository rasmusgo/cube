/*
 * image.h
 *
 *  Created on: 2011-aug-27
 *      Author: morotspaj
 */

#ifndef IMAGE_H_
#define IMAGE_H_

template <typename T>
T diff(T a, T b) {
	return (a < b) ? (b - a) : (a - b);
}

template <typename T>
T min(T a, T b, T c) {
	if (a < b)
		return (a < c) ? a : c;
	return (b < c) ? b : c;
}

template <typename T>
T max(T a, T b, T c) {
	if (a < b)
		return (b < c) ? c : b;
	return (a < c) ? c : a;
}

// Calculates the difference between this pixel and its neighbors
template <typename T>
T pixeldelta(T r, T g, T b, T rx, T gx, T bx, T ry, T gy, T by) {
	return  diff(r,rx) + diff(g,gx) + diff(b,bx) +
			diff(r,ry) + diff(g,gy) + diff(b,by) +
			diff(max(r,g,b)-min(r,g,b), max(rx,gx,bx)-min(rx,gx,bx)) +
			diff(max(r,g,b)-min(r,g,b), max(ry,gy,by)-min(ry,gy,by));
	// $d = max($r,$g,$b)-min($r,$g,$b);
	// $dx = max($rx,$gx,$bx)-min($rx,$gx,$bx);
	// $dy = max($ry,$gy,$by)-min($ry,$gy,$by);
	//$delta = abs($rx-$r) + abs($gx-$g) + abs($bx-$b) + abs($dx-$d) +
	//         abs($ry-$r) + abs($gy-$g) + abs($by-$b) + abs($dy-$d);
}

#endif /* IMAGE_H_ */
