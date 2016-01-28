/*
 * MoveIcons.cpp
 *
 *  Created on: 2011-aug-29
 *      Author: morotspaj
 */

#include <stdio.h>
#include <stdlib.h>

#include "CImg.h"
using namespace cimg_library;

void bend(float &x, float &y, int size) {
	// Transform the object by bending it as if it was bent in a elliptical way
	//      __
	//     / /
	//    / /
	//   | |
	//   | |
	//   | |
	//    \ \
	//     \_\
	//
	// The x-coordinate acts as the angle
	// The y-coordinate acts as the radius

	float a = ( y / size - 0.5 ) * M_PI_2;
	float d = ( x / size + 0.5 );
	x = -cos(a)*d*size + size*0.925;
	y = sin(a)*d*size + size*0.5;
}

// Generate move icons and store them on disk
int main() {
	// Arrows are triangles and rectangles
	//       _           _           _
	//      / \     ^   | |         / \
	//     /   \    |   | |        /   \
	//    /     \   | arrowsize   / / \ \
	//   /       \  |   | |      / /| |\ \
	//  /_________\ v   | |     /_/ | | \_\
	//      | |         | |         | |
	//      | |         | |         | |
	//<---->| |         | |         | |
	//margin| |         | |         | |
	//      | |<------->| |         | |
	//      | | spacing | |         | |
	//      | |         | |         | |
	//      | |         | |         | |
	//      | |         | |         | |
	//    ->|_|<-       |_|         |_|
	//   linewidth

	int arrowsize = 100;
	int arrowspacing = 150;
	int margin_x = 120;
	int margin_y = 50;
	int length = 500;
	int linewidth = 30;
	int height = margin_y*2 + length;
	int width = margin_x*2 + linewidth;

	// The arrows are black on a white background
	unsigned char color[3] = {0, 0, 0};
	unsigned char color2[3] = {64, 64, 64};

	const int arrow_point_count = 5;
	CImg<float> arrow1(arrow_point_count,2);
	{
		int i = 0;
		arrow1(i,0) = margin_x + linewidth/2;            arrow1(i++,1) = margin_y - linewidth/2;
		arrow1(i,0) = margin_x;                          arrow1(i++,1) = margin_y;
		//arrow1(i,0) = margin_x - arrowsize/2;            arrow1(i++,1) = margin_y + arrowsize/2;
		arrow1(i,0) = margin_x - arrowsize;              arrow1(i++,1) = margin_y + arrowsize;
		//arrow1(i,0) = margin - arrowsize + linewidth;  arrow1(i++,1) = margin + arrowsize + linewidth;
		//arrow1(i,0) = margin;                          arrow1(i++,1) = margin + linewidth*2;
		//arrow1(i,0) = margin + linewidth;              arrow1(i++,1) = margin + linewidth*2;
		//arrow1(i,0) = margin + arrowsize;              arrow1(i++,1) = margin + arrowsize + linewidth;
		arrow1(i,0) = margin_x + linewidth + arrowsize;  arrow1(i++,1) = margin_y + arrowsize;
		//arrow1(i,0) = margin_x + linewidth + arrowsize/2;arrow1(i++,1) = margin_y + arrowsize/2;
		arrow1(i,0) = margin_x + linewidth;              arrow1(i++,1) = margin_y;
	}

	CImg<float> arrow2(arrow1);
	for (int i = 0; i < arrow_point_count; ++i)
		arrow2(i,1) += arrowspacing;

	CImg<float> line(22,2);
	{
		int i = 0;
		int from = margin_y;
		int to = height - margin_y - 1;
		for (int j = 0; j <= 10; ++j) {
			line(i,0) = margin_x;               line(i++,1) = from + (to-from)*(j*0.1);
		}
		for (int j = 10; j >= 0; --j) {
			line(i,0) = margin_x + linewidth;   line(i++,1) = from + (to-from)*(j*0.1);
		}
	}

	// The arrows are first drawn straight with zero, one and two arrowheads
	// The arrow coordinates are then bent and different arrowheads are drawn
	// The images are then mirrored and rotated to create arrows in all desired directions

	CImg<unsigned char> bg(width,height, 1, 3);
	bg.fill(255);

	CImg<unsigned char> img_R0(bg);
	img_R0.draw_polygon(line, color2);

	CImg<unsigned char> img_R1(bg);
	img_R1.draw_polygon(line, color);
	img_R1.draw_polygon(arrow1, color);

	CImg<unsigned char> img_R2(img_R1);
	img_R2.draw_polygon(arrow2, color);

	CImg<unsigned char> img_R1m(img_R1.get_mirror('y'));
	CImg<unsigned char> img_R2m(img_R2.get_mirror('y'));

	CImg<unsigned char> img_U0(img_R0.get_rotate(-90));
	CImg<unsigned char> img_U1(img_R1.get_rotate(-90));
	CImg<unsigned char> img_U2(img_R2.get_rotate(-90));
	CImg<unsigned char> img_U1m(img_R1m.get_rotate(-90));
	CImg<unsigned char> img_U2m(img_R2m.get_rotate(-90));

	// Bend the polygon arrows
	for (int i=0; i < arrow_point_count; ++i)
		bend(arrow1(i,0), arrow1(i,1), height);
	for (int i=0; i < arrow_point_count; ++i)
		bend(arrow2(i,0), arrow2(i,1), height);
	for (int i=0; i < 22; ++i)
		bend(line(i,0), line(i,1), height);

	CImg<unsigned char> img_F0(bg);
	img_F0.draw_polygon(line, color2);

	CImg<unsigned char> img_F1(bg);
	img_F1.draw_polygon(line, color);
	img_F1.draw_polygon(arrow1, color);

	CImg<unsigned char> img_F2(img_F1);
	img_F2.draw_polygon(arrow2, color);

	img_F0.rotate(90);
	img_F1.rotate(90);
	img_F2.rotate(90);

	CImg<unsigned char> img_F1m(img_F1.get_mirror('x'));
	CImg<unsigned char> img_F2m(img_F2.get_mirror('x'));

	CImgList<unsigned char> list(15);
	{
		int i = 0;
		list[i++] = img_U2m;
		list[i++] = img_U1m;
		list[i++] = img_U0;
		list[i++] = img_U1;
		list[i++] = img_U2;
		list[i++] = img_R2m;
		list[i++] = img_R1m;
		list[i++] = img_R0;
		list[i++] = img_R1;
		list[i++] = img_R2;
		list[i++] = img_F2m;
		list[i++] = img_F1m;
		list[i++] = img_F0;
		list[i++] = img_F1;
		list[i++] = img_F2;
	}

	const char name[][8] = {
			"U-2",
			"U-1",
			"U0",
			"U1",
			"U2",
			"R-2",
			"R-1",
			"R0",
			"R1",
			"R2",
			"F-2",
			"F-1",
			"F0",
			"F1",
			"F2",
	};

	for (int i = 0; i < 15; ++i) {
		// Generate filename
		char str[120];
		sprintf(str, "icons/move_%s.bmp", name[i]);
		//printf("Saving %s\n", str);

		// Save
		list[i].resize(-10, -10, -100, -100, 2).save(str);

		// Convert to png
		sprintf(str, "convert icons/move_%s.bmp icons/move_%s.png", name[i], name[i]);
		printf("%s\n", str);
		fflush(stdout);
		//system(str);
	}

	CImgDisplay main_disp(list, "Arrow");
	while (!main_disp.is_closed() )
		main_disp.wait();
	return 0;
/*
	// Line positions
	int lpos[] = {
			margin + linewidth*2 + spacing*2,
			margin + linewidth + spacing,
			margin,
	};

	// Draw the lines of the arrows as rectangles
	for (int i = 0; i < 3; ++i)
		img.draw_rectangle(
				lpos[i], margin,
				lpos[i] + linewidth - 1, height - margin - 1,
				color);

	// Arrow positions
	int apos[] = {
			lpos[0] + linewidth * 0.5,
			lpos[1] + linewidth * 0.5,
			lpos[2] + linewidth * 0.5,
	};


	// There are three lines and six possible positions for arrowheads
	int i[3];
	for (i[0] = -2; i[0] <= 2; ++i[0])
		for (i[1] = -2; i[1] <= 2; ++i[1])
			for (i[2] = -2; i[2] <= 2; ++i[2]) {
				// Start from an image with background lines
				CImg<> icon(img);

				// Draw arrowheads
				for (int j = 0; j < 3; ++j) {

					if (i[j] <= -1) {
						// Erase the line under the tip of the arrowhead
						icon.draw_rectangle(
								apos[j] - arrowsize, height - margin - 1 - arrowsize,
								apos[j] + arrowsize, height - margin - 1,
								bgcolor);
						// Draw one arrowhead
						icon.draw_triangle(
								apos[j], height - margin - 1,
								apos[j] - arrowsize, height - margin - 1 - arrowsize,
								apos[j] + arrowsize, height - margin - 1 - arrowsize,
								color);
						// Draw a second arrowhead
						if (i[j] <= -2)
							icon.draw_triangle(
									apos[j], height - margin - 1 - arrowsize,
									apos[j] - arrowsize, height - margin - 1 - 2*arrowsize,
									apos[j] + arrowsize, height - margin - 1 - 2*arrowsize,
									color);
					} else if (i[j] >= 1) {
						// Erase the line under the tip of the arrowhead
						icon.draw_rectangle(
								apos[j] - arrowsize, margin,
								apos[j] + arrowsize, margin + arrowsize,
								bgcolor);
						// Draw one arrowhead
						icon.draw_triangle(
								apos[j], margin,
								apos[j] - arrowsize, margin + arrowsize,
								apos[j] + arrowsize, margin + arrowsize,
								color);
						// Draw a second arrowhead
						if (i[j] >= 2)
							icon.draw_triangle(
									apos[j], margin + arrowsize,
									apos[j] - arrowsize, margin + 2*arrowsize,
									apos[j] + arrowsize, margin + 2*arrowsize,
									color);
					}
				}

				// Store the image on disk
				// First as U, then rotate and store as R,
				// then bend the arrows and store as F

				// Generate filename
				char str[120];
				sprintf(str, "icons/move_%s%d%d%d.bmp", "R", i[0], i[1], i[2]);
				printf("Saving %s\n", str);
				icon.save(str);
				printf("Converting %s\n", str);
				sprintf(str, "convert icons/move_%s%d%d%d.bmp icons/move_%s%d%d%d.png", "R", i[0], i[1], i[2], "R", i[0], i[1], i[2]);
				system(str);

				// Rotate and store as U
				icon.rotate(-90, 0, 0, 1, 2, 0);
				sprintf(str, "icons/move_%s%d%d%d.bmp", "U", i[0], i[1], i[2]);
				printf("Saving %s\n", str);
				icon.save(str);
				printf("Converting %s\n", str);
				sprintf(str, "convert icons/move_%s%d%d%d.bmp icons/move_%s%d%d%d.png", "U", i[0], i[1], i[2], "U", i[0], i[1], i[2]);
				system(str);

				// Bend the lines
				icon = img.get_fill(255);
			}
	return 0;
	*/
}
