/*
 * createlabels.h
 *
 *  Created on: 2011-aug-31
 *      Author: morotspaj
 */

#ifndef CREATELABELS_H_
#define CREATELABELS_H_

#include "CImg.h"
using namespace cimg_library;

#include "Label.hpp"

typedef float DT;
std::vector<Label> createlabels(CImg<DT> &im_out, const CImg<float> &im, float threshold);
Label findborder(CImg<DT> &im_out, const CImg<float> &im, int x, int y, float threshold, DT markupcolor, bool secondarytrace = false);

#endif /* CREATELABELS_H_ */
