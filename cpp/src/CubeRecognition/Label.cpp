#include "Label.hpp"

Label::Label(const cv::Point2f& center, float area, const cv::Vec<float, 8>& size)
    : center(center), size(size), area(area), used_in_grid(false)
{
    float sx = size[1] - size[0] + 1;
    float sy = size[3] - size[2] + 1;
    float syp = size[5] - size[4] + 1;
    float sym = size[7] - size[6] + 1;
    float type_f = area / (sx * sym);
    float type_r = area / (sx * syp);
    float type_u = area / ypym2area(syp, sym);
    float type_s = area / (sx * sy);

    float quality[4] = {type_f, type_r, type_u, type_s};

    this->type = 0;
    this->qmax = quality[0];
    for (int i = 1; i < 4; ++i)
    {
        if (quality[i] > this->qmax)
        {
            this->type = i;
            this->qmax = quality[i];
        }
    }

    cv::Rect2f rects[] = {
        cv::Rect2f(cv::Point2f(size[0], size[6]), cv::Point(size[1], size[7])),
        cv::Rect2f(cv::Point2f(size[0], size[4]), cv::Point(size[1], size[5])),
        cv::Rect2f(cv::Point2f(size[4], size[6]), cv::Point(size[5], size[7])),
        cv::Rect2f(cv::Point2f(size[0], size[2]), cv::Point(size[1], size[3])),
    };
    native = this->xy2native(center.x, center.y);
    native_rect = rects[type];
}

// Give proposals on where neighbors can be found
std::vector<cv::Point> Label::guessneighbors()
{
    cv::Point2f np = this->native;

    // Find native size
    float nsize[4][4] = {
        // Type 0: 0167 x,ym
        {this->size[0], this->size[1], this->size[6], this->size[7]},
        // Type 1: 0145 x,yp
        {this->size[0], this->size[1], this->size[4], this->size[5]},
        // Type 2: 4567 yp,ym
        {this->size[4], this->size[5], this->size[6], this->size[7]},
        // Type 3 (Square): 0123 x,y
        {this->size[0], this->size[1],this->size[2], this->size[3]},
    };

    float snx = nsize[type][1] - nsize[type][0];
    float sny = nsize[type][3] - nsize[type][2];
    float snx2 = snx*1.5;
    float sny2 = sny*1.5;

    std::vector<cv::Point> guesses;
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

cv::Point2f Label::native2xy(float nx, float ny) const
{
    if (this->type == 0) {
        // x,ym
        return cv::Point2f(nx, xym2y(nx, ny));
    } else if (this->type == 1) {
        // x,yp
        return cv::Point2f(nx, xyp2y(nx, ny));
    } else if (this->type == 2) {
        // yp,ym
        return cv::Point2f(ypym2x(nx, ny), ypym2y(nx, ny));
    } else {
        // Square x,y
        return cv::Point2f(nx, ny);
    }
}

cv::Point2f Label::xy2native(float x, float y) const
{
    if (this->type == 0) {
        // x,ym
        return cv::Point2f(x, xy2ym(x, y));
    } else if (this->type == 1) {
        // x,yp
        return cv::Point2f(x, xy2yp(x, y));
    } else if (this->type == 2) {
        // yp,ym
        return cv::Point2f(xy2yp(x, y), xy2ym(x, y));
    } else {
        // Square x,y
        return cv::Point2f(x, y);
    }
}

bool are_neighbors(const Label& a, const Label& b)
{
    if (a.type != b.type)
        return false;

    // See if limits are close to each other
    float sx = std::min(a.native_rect.width, b.native_rect.width);
    float sy = std::min(a.native_rect.height, b.native_rect.height);

    float tolx = sx*0.5;
    float toly = sy*0.5;

    float a_left   = a.native_rect.x;
    float a_right  = a.native_rect.x + a.native_rect.width;
    float a_top    = a.native_rect.y;
    float a_bottom = a.native_rect.y + a.native_rect.height;

    float b_left   = b.native_rect.x;
    float b_right  = b.native_rect.x + b.native_rect.width;
    float b_top    = b.native_rect.y;
    float b_bottom = b.native_rect.y + b.native_rect.height;
    // neighbors in x
    // |a||b|
    // and neighbors in y
    // |a|
    // |b|
    if ((std::abs(a_left - b_left) < tolx || std::abs(a_right - b_right) < tolx || std::abs(a_left - b_right) < tolx || std::abs(a_right - b_left) < tolx) &&
        (std::abs(a_top - b_top) < toly || std::abs(a_bottom - b_bottom) < toly || std::abs(a_top - b_bottom) < toly || std::abs(a_bottom - b_top) < toly) )
        return true;
    return false;
}

/*
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
