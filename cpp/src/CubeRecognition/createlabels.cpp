/*
 * createlabels.cpp
 *
 *  Created on: 2011-aug-31
 *      Author: morotspaj
 */

#include <stdio.h>
#include <stdlib.h>

#include "createlabels.h"

#define foreach(c, it) \
	for (typeof(c.begin()) it = c.begin(); it != c.end(); ++it)

// Find all distinct areas, paint them and and measure them
std::vector<Label> createlabels(CImg<DT> &im_out, const CImg<float> &im, float threshold) {
	/*
	global time_debug;

	stopwatch = new Stopwatch();
	stopwatch->start();
	*/

	int sx = im._width;
	int sy = im._height;
	float imagearea = sx*sy;

	std::vector<Label> labels;

	const int numcolors = 6*6;
	unsigned int colorarea[numcolors] = {
		0x7f0000, 0x7f007f, 0x00007f, 0x007f7f, 0x007f00, 0x7f7f00,
		0x643232, 0x643264, 0x323264, 0x326464, 0x326432, 0x646432,
		0xc80000, 0xc800c8, 0x0000c8, 0x00c8c8, 0x00c800, 0xc8c800,
		0x7f1e1e, 0x7f1e7f, 0x1e1e7f, 0x1e7f7f, 0x1e7f1e, 0x7f7f1e,
		0x1e1e1e, 0x3c3c3c, 0x5a5a5a, 0x787878, 0x969696, 0xb4b4b4};

	int i = 0;

	//Start with some generic guesses and continue with guesses from good quality labels
	std::vector<Vec2r> queue;
	for (int y = floor(sy*0.25); y < ceil(sy*0.75); y += 20) {
		for (int x = floor(sx*0.25); x < ceil(sx*0.75); x += 20) {
			queue.push_back(Vec2r(x,y));
		}
	}

	//time_debug .= '* Create labels setup time: '. stopwatch->lap(). '<br>';

	//findborder_watch = new Stopwatch();

	float minarea = 100;

	while (queue.size() > 0) {
		Vec2r p = queue.back();
		queue.pop_back();
		p.x = floor(p.x);
		p.y = floor(p.y);

		//findborder_watch->start();

		Label label = findborder(im_out, im, p.x, p.y, threshold, colorarea[i]);
		i = (i + 1) % numcolors;
		if (label.area != 0 && label.area < minarea) { // if label is small, trace another direction
			label = findborder(im_out, im, p.x, p.y, threshold, colorarea[i], true);
			i = (i + 1) % numcolors;
		}
		//findborder_watch->stop();
		if (label.area > 0) {
			if (label.area > minarea &&  label.qmax > 0.5 && label.area / label.qmax < imagearea/10) // && label.qmax > label.quality[3]
			{
				labels.push_back(label);
				std::vector<Vec2r> neighbors = label.guessneighbors();
				foreach (neighbors, n) {
					queue.push_back(*n);
				}
			}
		}
		//imagesetpixel(im_out, x, y, white);
	}
/*
	global findarea_setup_watch;
	global findarea_work_watch;

	//time_debug .= '*** Findarea setup time: '. findarea_setup_watch->getTime(). '<br>';
	//time_debug .= '*** Findarea work time: '. findarea_work_watch->getTime(). '<br>';
	time_debug .= '** Findborder time: '. findborder_watch->getTime(). '<br>';

	time_debug .= '* Create labels work time: '. stopwatch->lap(). '<br>';

	//findarea_setup_watch->reset();
	//findarea_work_watch->reset();
*/
	return labels;
}

/* Findborder finds an edge and follows it around until the startposition is reached again.
 *
 */
Label findborder(CImg<DT> &im_out, const CImg<float> &im, int x, int y, float threshold, DT markupcolor, bool secondarytrace) {
	//global debugstring;
	int imagesx = im._width;
	int imagesy = im._height;
	int startx = x;

	float black = 0;

	int dx = 0;
	int dy = 0;

	// Walk until we find an edge or exit the image
	// Walk towards the closest border left or right
	if (startx < imagesx/2)
		dx = -1;
	else
		dx = 1;

	if (secondarytrace)
		dx = -dx;

	Real zerosize[8] = {0,0,0,0,0,0,0,0,};
	Label null(0,0,0,zerosize);

	while (true) {
		// Check bounds
		if (x < 0 || x+2 >= imagesx || y < 0 || y+2 >= imagesy)
			return Label(0,0,0,zerosize);

		// Abort if this label is found already
		if (im_out(x,y) != black)
		{
			//debugstring .= '<br>aborting search because of not black';
			return null;
		}

		// Continue until we find an edge
		if (im(x,y) > threshold)
			break;

		// Step forward
		x += dx;
	}

	// Abort if we are on an edge at the start
	if (x == startx)
		return null;

	// Turn right
	if (dx < 0) {
		dx = 0;
		dy = -1;
	} else {
		dx = 0;
		dy = 1;
		x -= 1;
		y -= 1;
	}

	Real xmax = -99999999;
	Real xmin = 99999999;
	Real ymax = -99999999;
	Real ymin = 99999999;
	Real ypmax = -99999999;
	Real ypmin = 99999999;
	Real ymmax = -99999999;
	Real ymmin = 99999999;

	int area = 0;
	Real cx = 0;
	Real cy = 0;

	int x0 = x;
	int y0 = y;

	// Mark as discovered
	im_out(x,y) = markupcolor;

	int i = 0;
	// We are now on the first edge pixel to the left
	// Follow the edge until we find our footprints
	while (true) {
		i += 1;
		if (i >= 1000) {
			printf("warning, got stuck in findborder (%d, %d)\n", x, y);
			fflush(stdout);
			//debugstring .= '<br>warning, got stuck in findborder';
			/*
			white = imagecolorallocate(im_out, 255, 255, 255);
			for (j = y-10; j < y+10; j++)
				for (i = x-10; i < x+10; i++)
					if (pixeldelta(im, j, i) > threshold)
						imagesetpixel(im_out, j, i, white);
			*/
			return null;
		}

		// Update bounds
		if (x > xmax) xmax = x;
		if (x < xmin) xmin = x;
		if (y > ymax) ymax = y;
		if (y < ymin) ymin = y;
		Real yp = xy2yp(x,y);
		Real ym = xy2ym(x,y);
		if (yp > ypmax) ypmax = yp;
		if (yp < ypmin) ypmin = yp;
		if (ym > ymmax) ymmax = ym;
		if (ym < ymmin) ymmin = ym;

		// Step forward
		x += dx;
		y += dy;

		// Calculate area and centroid (center of mass)
		//         x1*y2     -   y1*x2
		int darea = (x-dx)*y - (y-dy)*x;
		area += darea;
		cx += (x+x-dx)*darea;
		cy += (y+y-dy)*darea;

		// We are done when we arrive at start again.
		if ( x == x0 && y == y0 ) {
			Real size[8] = {xmin, xmax, ymin, ymax, ypmin, ypmax, ymmin, ymmax};
			return Label(cx/(3*area), cy/(3*area), area/2, size);
		}

		// Check bounds
		if (x < 0 || x+2 >= imagesx || y < 0 || y+2 >= imagesy)
			return null;

		// Abort if this label is found already
		/*
		DT & color = im_out(x,y);
		if (color != black && color != markupcolor)
		{
			//debugstring .= '<br>aborting search because of not black inside loop';
			//return null;
		}
		*/

		// Mark as discovered
		im_out(x,y) = markupcolor;

		// Find out where to go next
		// Check pixels front right and front left
		if (im(x+0.5+(dx-dy)*0.5, y+0.5+(dy+dx)*0.5) > threshold) {
			// Turn right
			int tmp = dx;
			dx = -dy;
			dy = tmp;
		} else if (im(x+0.5+(dx+dy)*0.5, y+0.5+(dy-dx)*0.5) <= threshold) {
			// Turn left
			int tmp = dx;
			dx = dy;
			dy = -tmp;
		} // Otherwise continue forward
	}
}
