//============================================================================
// Name        : CubeRecognition.cpp
// Author      : Rasmus Göransson
// Version     :
// Copyright   :
// Description : Hello World in C, Ansi-style
//============================================================================

#include <stdio.h>
#include <stdlib.h>

#include "CImg.h"
using namespace cimg_library;

#include "image.h"

#include "Label.h"
#include "createlabels.h"

#define cimg_for3x3rgb(img,x,y,z,R,G,B,T) \
  cimg_for3((img)._height,y) for (int x = 0, \
   _p1##x = 0, \
   _n1##x = (int)( \
   (R[0] = R[1] = (T)(img)(0,_p1##y,z,0)), \
   (R[3] = R[4] = (T)(img)(0,y,z,0)), \
   (R[6] = R[7] = (T)(img)(0,_n1##y,z,0)),      \
   (G[0] = G[1] = (T)(img)(0,_p1##y,z,1)), \
   (G[3] = G[4] = (T)(img)(0,y,z,1)), \
   (G[6] = G[7] = (T)(img)(0,_n1##y,z,1)),      \
   (B[0] = B[1] = (T)(img)(0,_p1##y,z,2)), \
   (B[3] = B[4] = (T)(img)(0,y,z,2)), \
   (B[6] = B[7] = (T)(img)(0,_n1##y,z,2)),      \
   1>=(img)._width?(img).width()-1:1); \
   (_n1##x<(img).width() && ( \
   (R[2] = (T)(img)(_n1##x,_p1##y,z,0)), \
   (R[5] = (T)(img)(_n1##x,y,z,0)), \
   (R[8] = (T)(img)(_n1##x,_n1##y,z,0)), \
   (G[2] = (T)(img)(_n1##x,_p1##y,z,1)), \
   (G[5] = (T)(img)(_n1##x,y,z,1)), \
   (G[8] = (T)(img)(_n1##x,_n1##y,z,1)), \
   (B[2] = (T)(img)(_n1##x,_p1##y,z,2)), \
   (B[5] = (T)(img)(_n1##x,y,z,2)), \
   (B[8] = (T)(img)(_n1##x,_n1##y,z,2)),1)) || \
   x==--_n1##x; \
   R[0] = R[1], R[1] = R[2], \
   R[3] = R[4], R[4] = R[5], \
   R[6] = R[7], R[7] = R[8], \
   G[0] = G[1], G[1] = G[2], \
   G[3] = G[4], G[4] = G[5], \
   G[6] = G[7], G[7] = G[8], \
   B[0] = B[1], B[1] = B[2], \
   B[3] = B[4], B[4] = B[5], \
   B[6] = B[7], B[7] = B[8], \
   _p1##x = x++, ++_n1##x)

int main() {
	CImg<float> img("cube_0b.png");
	CImg<float> gradient(img.get_channel(0));
	CImg<float> delta(img.get_channel(0));
	CImg<float> k33(img.get_channel(0));
	CImg<float> intensity(img.get_channel(0));

	cimg_forC(img, c)
		printf("c: %d\n", c);

	CImg_3x3(R, float);
	CImg_3x3(G, float);
	CImg_3x3(B, float);
	cimg_for3x3rgb(img, x, y, 0, R, G, B, float) {
		const float Dcc = max(Rcc, Gcc, Bcc) - min(Rcc, Gcc, Bcc);
		const float Dnc = max(Rnc, Gnc, Bnc) - min(Rnc, Gnc, Bnc);
		const float Dpc = max(Rpc, Gpc, Bpc) - min(Rpc, Gpc, Bpc);
		const float Dcn = max(Rcn, Gcn, Bcn) - min(Rcn, Gcn, Bcn);
		const float Dcp = max(Rcp, Gcp, Bcp) - min(Rcp, Gcp, Bcp);
		const float Dnn = max(Rnn, Gnn, Bnn) - min(Rnn, Gnn, Bnn);

		/*
		const float rx = (Rnc-Rpc), ry = (Rcn-Rcp);
		const float gx = (Gnc-Gpc), gy = (Gcn-Gcp);
		const float bx = (Bnc-Bpc), by = (Bcn-Bcp);
		const float dx = (Dnc-Dpc), dy = (Dcn-Dcp);
		*/

		const float rx = (Rnc-Rpc);
		const float gx = (Gnc-Gpc);
		const float bx = (Bnc-Bpc);
		const float dx = (Dnc-Dpc);
		const float ix = rx+gx+bx;

		const float ry = (Rcn-Rcp);
		const float by = (Bcn-Bcp);
		const float gy = (Gcn-Gcp);
		const float dy = (Dcn-Dcp);
		const float iy = ry+gy+by;

		const float rgx = (Rnc-Gnc) - (Rpc-Gpc);
		const float rbx = (Rnc-Bnc) - (Rpc-Bpc);
		const float gbx = (Gnc-Bnc) - (Gpc-Bpc);

		const float rgy = (Rcn-Gcn) - (Rcp-Gcp);
		const float rby = (Rcn-Bcn) - (Rcp-Bcp);
		const float gby = (Gcn-Bcn) - (Gcp-Bcp);

		gradient(x,y) = rx*rx + gx*gx + bx*bx +
						ry*ry + gy*gy + by*by;
		delta(x,y)    = dx*dx + dy*dy;
		k33(x,y)      = rgx*rgx + rbx*rbx + gbx*gbx +
						rgy*rgy + rby*rby + gby*gby;
		intensity(x,y)= ix*ix + iy*iy;
	}
	CImg<> composite = gradient + 9*delta + 4*k33;
	composite.sqrt();
	gradient.sqrt();
	delta.sqrt();
	k33.sqrt();
	intensity.sqrt();

	CImg<DT> im_out(composite._width, composite._height);
	im_out.fill(0);
	std::vector<Label> labels = createlabels(im_out, composite, 80);
	composite.threshold(80);

	printf("labels.size(): %d\n", labels.size());
	fflush(stdout);

	int image_id = 0;
	const int image_count = 7;
	CImg<> * images[image_count] = {&img, &gradient, &intensity, &k33, &delta, &composite, &im_out};
	const char * titles[image_count] = {"Original", "RGB Gradient", "Intensity Gradient", "k33 Gradient", "Saturation Gradient", "Composite", "Debug output"};

	CImgDisplay main_disp(img, "Click to change image");

	while (!main_disp.is_closed() ) {
		main_disp.wait();
		if ((main_disp.button() & 1) && main_disp.mouse_y() >= 0) {
			image_id = (image_id + 1) % image_count;
			main_disp.display(*images[image_id] );
			main_disp.set_title( titles[image_id] );

			main_disp.set_button();
		}
		if ((main_disp.button() & 2) && main_disp.mouse_y() >= 0) {
			image_id = (image_id + image_count-1) % image_count;
			main_disp.display(*images[image_id] );
			main_disp.set_title( titles[image_id] );

			main_disp.set_button();
		}
	}
	return 0;
}
