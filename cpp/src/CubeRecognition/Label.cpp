#include "Label.hpp"

Label::Label(Real x, Real y, Real area, Real size[8]) :
    x(x), y(y), area(area), used_in_grid(false)
{
    for (int i = 0; i < 8; ++i)
    {
        this->size[i] = size[i];
    }
    Real sx = size[1] - size[0] + 1;
    Real sy = size[3] - size[2] + 1;
    Real syp = size[5] - size[4] + 1;
    Real sym = size[7] - size[6] + 1;
    Real typ_x = area / (sx * sym);
    Real typ_y = area / (sx * syp);
    Real typ_z = area / ypym2area(syp, sym);
    Real typ_s = area / (sx * sy);

    Real quality[4] = {typ_x, typ_y, typ_z, typ_s};

    this->type = 0;
    this->qmax = quality[0];
    for (int i = 1; i < 4; ++i) {
        if (quality[i] > this->qmax) {
            this->type = i;
            this->qmax = quality[i];
        }
    }

    native = this->xy2native(x,y);
}

// Give proposals on where neighbors can be found
std::vector<Vec2r> Label::guessneighbors() {
    Vec2r np = this->xy2native(this->x, this->y);

    // Find native size
    Real nsize[4][4] = {
        // Type 0: 0167 x,ym
        {this->size[0], this->size[1], this->size[6], this->size[7]},
        // Type 1: 0145 x,yp
        {this->size[0], this->size[1], this->size[4], this->size[5]},
        // Type 2: 4567 yp,ym
        {this->size[4], this->size[5], this->size[6], this->size[7]},
        // Type 3 (Square): 0123 x,y
        {this->size[0], this->size[1],this->size[2], this->size[3]},
    };

    Real snx = nsize[type][1] - nsize[type][0];
    Real sny = nsize[type][3] - nsize[type][2];
    Real snx2 = snx*1.5;
    Real sny2 = sny*1.5;

    std::vector<Vec2r> guesses;
    guesses.push_back(this->native2xy(np.x + snx, np.y));
    guesses.push_back(this->native2xy(np.x - snx, np.y));
    guesses.push_back(this->native2xy(np.x, np.y + sny));
    guesses.push_back(this->native2xy(np.x, np.y - sny));
    guesses.push_back(this->native2xy(np.x + snx2, np.y));
    guesses.push_back(this->native2xy(np.x - snx2, np.y));
    guesses.push_back(this->native2xy(np.x, np.y + sny2));
    guesses.push_back(this->native2xy(np.x, np.y - sny2));

    guesses.push_back(this->native2xy(np.x + snx, np.y + sny));
    guesses.push_back(this->native2xy(np.x - snx, np.y - sny));
    guesses.push_back(this->native2xy(np.x - snx, np.y + sny));
    guesses.push_back(this->native2xy(np.x + snx, np.y - sny));
    guesses.push_back(this->native2xy(np.x + snx2, np.y + sny2));
    guesses.push_back(this->native2xy(np.x - snx2, np.y - sny2));
    guesses.push_back(this->native2xy(np.x - snx2, np.y + sny2));
    guesses.push_back(this->native2xy(np.x + snx2, np.y - sny2));
    return guesses;
}

Vec2r Label::native2xy(Real nx, Real ny) {
    if (this->type == 0) {
        // x,ym
        return Vec2r(nx, xym2y(nx, ny));
    } else if (this->type == 1) {
        // x,yp
        return Vec2r(nx, xyp2y(nx, ny));
    } else if (this->type == 2) {
        // yp,ym
        return Vec2r(ypym2x(nx, ny), ypym2y(nx, ny));
    } else {
        // Square x,y
        return Vec2r(nx, ny);
    }
}

Vec2r Label::xy2native(Real x, Real y) {
    if (this->type == 0) {
        // x,ym
        return Vec2r(x, xy2ym(x, y));
    } else if (this->type == 1) {
        // x,yp
        return Vec2r(x, xy2yp(x, y));
    } else if (this->type == 2) {
        // yp,ym
        return Vec2r(xy2yp(x, y), xy2ym(x, y));
    } else {
        // Square x,y
        return Vec2r(x, y);
    }
}
/*
    function is_neighbor(other) {
        if (this->type != other->type)
            return false;

        // Find native coordinates
        asize = this->nativesize();
        bsize = other->nativesize();

        // See if limits are close to each other
        sx = min(asize[1] - asize[0], bsize[1] - bsize[0]);
        sy = min(asize[3] - asize[2], bsize[3] - bsize[2]);

        tolx = sx*0.5;
        toly = sy*0.5;

        // neighbors in x
        // |a||b|
        // and neighbors in y
        // |a|
        // |b|
        if ((abs(asize[0] - bsize[0]) < tolx || abs(asize[1] - bsize[1]) < tolx || abs(asize[0] - bsize[1]) < tolx || abs(asize[1] - bsize[0]) < tolx) &&
            (abs(asize[2] - bsize[2]) < toly || abs(asize[3] - bsize[3]) < toly || abs(asize[2] - bsize[3]) < toly || abs(asize[3] - bsize[2]) < toly) )
            return true;
        return false;
    }

// Draw a little box around the center of the label
// im - image resource to be drawn on
public function mark_centre(im) {
    fill = imagecolorallocate(im,
        this->quality[0]*255,
        this->quality[1]*255,
        this->quality[2]*255);
    red = imagecolorallocate(im, 255, 0, 0);
    green = imagecolorallocate(im, 0, 255, 0);
    blue = imagecolorallocate(im, 0, 0, 255);
    black = imagecolorallocate(im, 0, 0, 0);
    gray = imagecolorallocate(im, 195, 195, 195);
    white = imagecolorallocate(im, 255, 255, 255);

    colors = array(red,green,blue,gray);
    bestcolor = colors[this->type];
    size = this->nativesize();
    polygon = array_merge(
            this->native2xy(size[0], size[3]),
            this->native2xy(size[0], size[2]),
            this->native2xy(size[1], size[2]),
            this->native2xy(size[1], size[3]));
    imagepolygon(im, polygon, 4, bestcolor);
    if (this->used_in_grid)
    {
        imagefilledrectangle(im, floor(this->x-1), floor(this->y-1), ceil(this->x+1), ceil(this->y+1), fill);
        imagerectangle(im, floor(this->x-2), floor(this->y-2), ceil(this->x+2), ceil(this->y+2), white);
    }
    neighbors = this->guessneighbors();
    foreach (neighbors as n) {
        list(x, y) = n;
        polygon = array(
            x,y+2,
            x-2,y,
            x,y-2,
            x+2,y);
        imagepolygon(im, polygon, 4, bestcolor);
    }
}
*/
