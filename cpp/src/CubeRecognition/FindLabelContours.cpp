#include <cstdio>
#include <cstdlib>
#include <functional>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "FindLabelContours.hpp"

/**
 * Finds an edge and follows it around until the startposition is reached again.
 *
 * Edges are defined as the crossing \a between two pixels.
 *
 *     ┌───┬───┬───┬───┬───┐
 *     │0,0│   │   │   │   │
 *     ├───┼───┼─E─┼─E─┼───┤
 *     │   │   E   │   E   │
 *     ├───┼─E─┼───┼───┼───┤
 *     │   E   │   │   E   │
 *     ├───┼─E─┼─E─┼─E─┼───┤
 *     │   │   │   │   │   │
 *     └───┴───┴───┴───┴───┘
 */
LabelContour findLabelContour(const cv::Size& size, const cv::Point& p0, EdgeFunctionType& edge_function,
    cv::Mat1i& canvas, int markupcolor, bool secondarytrace)
{
    cv::Point p = p0;

    const int black = 0;

    // Walk until we find an edge or exit the image.
    // Walk towards the closest border left or right.
    cv::Point d((p0.x < size.width / 2) ? -1 : 1, 0);

    if (secondarytrace)
        d.x = -d.x;

    cv::Vec<float, 8> zerosize{0,0,0,0,0,0,0,0};
    LabelContour bad_label(cv::Point2f(0,0), 0.f, zerosize, {});

    auto is_inside_image = [&](const cv::Point& p)
    {
        return p.x > 0 && p.y > 0 && p.x + 1 < size.width && p.y + 1 < size.height;
    };

    if (!is_inside_image(p))
        return bad_label;

    while (true)
    {
        // Abort if this label is found already
        if (canvas(p) != black)
        {
            return bad_label;
        }

        // Continue until we find an edge
        if (edge_function(p, p + d))
            break;

        // Step forward
        p += d;

        if (!is_inside_image(p))
            return bad_label;
    }

    // Abort if we are on an edge at the start
    if (p.x == p0.x)
        return bad_label;

    // Turn right
    d = cv::Point(-d.y, d.x);

    float xmax = -99999999;
    float xmin = 99999999;
    float ymax = -99999999;
    float ymin = 99999999;
    float ypmax = -99999999;
    float ypmin = 99999999;
    float ymmax = -99999999;
    float ymmin = 99999999;

    auto update_bounds = [&](const cv::Point& p)
    {
        if (p.x > xmax) xmax = p.x;
        if (p.x < xmin) xmin = p.x;
        if (p.y > ymax) ymax = p.y;
        if (p.y < ymin) ymin = p.y;
        float yp = xy2yp(p.x, p.y);
        float ym = xy2ym(p.x, p.y);
        if (yp > ypmax) ypmax = yp;
        if (yp < ypmin) ypmin = yp;
        if (ym > ymmax) ymmax = ym;
        if (ym < ymmin) ymmin = ym;
    };

    int area = 0;
    float cx = 0;
    float cy = 0;

    std::vector<cv::Point> contour_points;

    auto step_forward = [&]()
    {
        // Take a step forward.
        p += d;

        // Check bounds
        if (!is_inside_image(p))
            return false;

        // Calculate area and centroid (center of mass)
        //         x1*y2     -   y1*x2
        int darea = (p.x-d.x)*p.y - (p.y-d.y)*p.x;
        area += darea;
        cx += (p.x + p.x - d.x) * darea;
        cy += (p.y + p.y - d.y) * darea;

        // Mark as discovered
        canvas(p) = markupcolor;

        // Update bounds
        update_bounds(p);

        // Store contour
        contour_points.push_back(p);
        return true;
    };

    // Mark as discovered
    canvas(p) = markupcolor;

    // This is the first point of the contour.
    cv::Point p1 = p;

    // We now have the first edge pixel to our left.
    // Follow the edge until we come back.
    //    ┌───────┬───────┬───────┬───────┬───────┬───────┐
    //    │       │       │       │       │       │       │
    //    │  0,0  │       │       │       │       │       │
    //    │       │       │       │       │       │       │
    //    ├───────┼───E───┼───E───┼───E───┼───E───┼───────┤
    //    │       │       │       │       │       │       │
    //    │       E       │       │       │       E       │
    //    │       │       │       │       │       │       │
    //    ├───────┼───────┼───────┼───────┼───────┼───────┤
    //    │       │   ↑   │       │       │       │       │
    //    │       E   p   │       │       │       E       │
    //    │       │       │       │       │       │       │
    //    ├───────┼───E───┼───E───┼───────┼───E───┼───────┤
    //    │       │       │       │       │       │       │
    //    │       │       │       E       │       E       │
    //    │       │       │       │       │       │       │
    //    ├───────┼───────┼───────┼───E───┼───E───┼───────┤
    //    │       │       │       │       │       │       │
    //    │       │       │       │       │       │       │
    //    │       │       │       │       │       │       │
    //    └───────┴───────┴───────┴───────┴───────┴───────┘
    while (true)
    {
        // Find out where to go next
        // Check pixels front right and front left.
        cv::Point left(p.x + d.y, p.y - d.x);
        cv::Point front = p + d;
        if (!edge_function(p, left))
        {
            // Turn left
            d = cv::Point(d.y, -d.x);
            if (!step_forward())
                return bad_label;
        }
        else if (!edge_function(p, front))
        {
            if (!step_forward())
                return bad_label;
        }
        else
        {
            // Turn right
            d = cv::Point(-d.y, d.x);
            continue;
        }

        // We are done when we arrive at start again.
        if ( p == p1 )
        {
            cv::Vec<float, 8> label_size = {xmin, xmax, ymin, ymax, ypmin, ypmax, ymmin, ymmax};
            return LabelContour(cv::Point2f(cx/(3*area), cy/(3*area)), area/2, label_size,
                std::move(contour_points));
        }
    }
}

/**
 * Find all distinct areas, paint them and and measure them.
 */
std::vector<LabelContour> findLabelContours(const cv::Mat3b& img, double threshold, bool visualize)
{
    EdgeFunctionType edge_function = [&](const cv::Point& a, const cv::Point& b)
    {
        return cv::norm(img(a), img(b)) > threshold;
    };

    return findLabelContours(img.size(), edge_function, visualize);
}

std::vector<LabelContour> findLabelContours(cv::Size size, EdgeFunctionType& edge_function, bool visualize)
{
    float imagearea = size.area();

    std::vector<LabelContour> labels;

    const int numcolors = 6*5;
    const int colorarea[numcolors] = {
        0x7f0000, 0x7f007f, 0x00007f, 0x007f7f, 0x007f00, 0x7f7f00,
        0x643232, 0x643264, 0x323264, 0x326464, 0x326432, 0x646432,
        0xc80000, 0xc800c8, 0x0000c8, 0x00c8c8, 0x00c800, 0xc8c800,
        0x7f1e1e, 0x7f1e7f, 0x1e1e7f, 0x1e7f7f, 0x1e7f1e, 0x7f7f1e,
        0x1e1e1e, 0x3c3c3c, 0x5a5a5a, 0x787878, 0x969696, 0xb4b4b4};

    cv::Mat1i canvas(size, 0);
    int i = 0;

    //Start with some generic guesses and continue with guesses from good quality labels
    std::vector<cv::Point> queue;
    for (int y = floor(size.height*0.25); y < ceil(size.height*0.75); y += 20)
    {
        for (int x = floor(size.width*0.25); x < ceil(size.width*0.75); x += 20)
        {
            queue.push_back(cv::Point(x,y));
        }
    }

    float minarea = 100;

    while (queue.size() > 0)
    {
        cv::Point p = queue.back();
        queue.pop_back();
        p.x = floor(p.x);
        p.y = floor(p.y);

        LabelContour label = findLabelContour(size, p, edge_function, canvas, colorarea[i], false);
        i = (i + 1) % numcolors;
        if (label.area != 0 && label.area < minarea)
        {
            // if label is small, trace another direction
            label = findLabelContour(size, p, edge_function, canvas, colorarea[i], true);
            i = (i + 1) % numcolors;
        }
        if (label.area > 0)
        {
            if (label.area > minarea &&  label.qmax > 0.5 && label.area / label.qmax < imagearea/10)
                // && label.qmax > label.quality[3]
            {
                labels.push_back(label);
                std::vector<cv::Point> neighbors = label.guessneighbors();
                for (const auto& vec : neighbors)
                {
                    queue.push_back(cv::Point(vec.x, vec.y));
                }
            }
        }
    }

    if (visualize)
    {
        cv::Mat4b colored_canvas = cv::Mat(canvas.size(), CV_8UC4, canvas.data);
        cv::imshow("debug labels", colored_canvas);
    }
    return labels;
}

std::vector<std::vector<cv::Point2f>> findLabelCorners(const std::vector<LabelContour>& labels)
{
    std::vector<std::vector<cv::Point2f>> all_corners;
    for (const auto& label : labels)
    {
        std::vector<cv::Point2f> hull_native;
        cv::convexHull(label.xy2native(cast<cv::Point2f>(label.contour_points)), hull_native, true);

        std::vector<cv::Point2f> smoothed_hull_native;
        cv::approxPolyDP(hull_native, smoothed_hull_native, 3, true);

//        cv::polylines(canvas, cast<cv::Point>(label.native2xy(smoothed_hull_native)),
//            true, cv::Scalar(255, 0, 0));

        using Edge = std::pair<cv::Point2f, cv::Point2f>;
        std::vector<Edge> edges;
        size_t zeroth_edge = 0;
        cv::Point2f center_native = label.xy2native(label.center);
        for (int i = 0; i < smoothed_hull_native.size(); ++i)
        {
            int j = (i + 1) % smoothed_hull_native.size();
            cv::Point2f a = smoothed_hull_native[i];
            cv::Point2f b = smoothed_hull_native[j];
            cv::Point2f ac = a - center_native;
            cv::Point2f bc = b - center_native;
            if ((ac.x > 0) != (bc.x > 0) || (ac.y > 0) != (bc.y > 0))
            {
                if ((ac.x > 0) && (bc.x > 0) && (ac.y > 0) != (bc.y > 0))
                {
                    zeroth_edge = edges.size();
                }
                // The points ac and bc are in different quadrants.
                edges.emplace_back(label.native2xy(a), label.native2xy(b));
            }
        }

        if (edges.size() != 4)
        {
            printf("%lu edges\n", edges.size());
            continue;
        }

        std::vector<cv::Point2f> corners;
        for (int k = 0; k < edges.size(); ++k)
        {
            int i = (k + zeroth_edge) % edges.size();
            int j = (k + zeroth_edge + 1) % edges.size();
            // Find the intersection of the two lines.
            cv::Point2f a = edges[i].first;
            cv::Point2f b = edges[i].second;
            cv::Point2f c = edges[j].first;
            cv::Point2f d = edges[j].second;
            // p = a + (b-a) * t1 = c + (d-c) * t2
            // (b-a) * t1 - (d-c) * t2 = c - a
            // (b-a) * t1 + (c-d) * t2 = c - a
            // Ax = B
            // A = [ (b-a), (c-d) ]
            // x = [ t1, t2 ]'
            // B = c - a
            // At*Ax = At*B
            // x = At*A \ At*B
            // p = a + (b - a) * t1
            cv::Matx22d A = {
                b.x - a.x, c.x - d.x,
                b.y - a.y, c.y - d.y };
            cv::Matx21d B = { c.x - a.x, c.y - a.y };
            cv::Matx21d x = (A.t() * A).inv() * (A.t() * B);
            cv::Point2f p = a + (b - a) * x(0);
            corners.push_back(p);
        }
        all_corners.push_back(corners);
    }
    return all_corners;
}

void drawLabel(cv::Mat& canvas, const LabelContour& label, const cv::Scalar& color)
{
    cv::Point2f np = label.native;

    // Find native size
    float nsize[4][4] = {
        // Type 0: 0167 x,ym
        {label.size[0], label.size[1], label.size[6], label.size[7]},
        // Type 1: 0145 x,yp
        {label.size[0], label.size[1], label.size[4], label.size[5]},
        // Type 2: 4567 yp,ym
        {label.size[4], label.size[5], label.size[6], label.size[7]},
        // Type 3 (Square): 0123 x,y
        {label.size[0], label.size[1],label.size[2], label.size[3]},
    };

    float snx = nsize[label.type][1] - nsize[label.type][0];
    float sny = nsize[label.type][3] - nsize[label.type][2];
    float snx2 = snx*0.5;
    float sny2 = sny*0.5;

    std::vector<cv::Point2f> corners = {
        label.native2xy(np.x - snx2, np.y - sny2),
        label.native2xy(np.x - snx2, np.y + sny2),
        label.native2xy(np.x + snx2, np.y + sny2),
        label.native2xy(np.x + snx2, np.y - sny2),
    };

    std::vector<cv::Point> cv_corners;
    for (auto corner : corners)
    {
        cv_corners.emplace_back(corner.x * 255, corner.y * 255);
    }
    cv::polylines(canvas, cv_corners, true, color, 1, cv::LINE_AA, 8);
    cv::polylines(canvas, label.contour_points, true, color, 1, cv::LINE_AA);
}

void drawLabels(cv::Mat& canvas, const std::vector<LabelContour>& labels, const cv::Scalar& color)
{
    for (auto label : labels)
    {
        drawLabel(canvas, label, cv::Scalar(255, 255, 255));
    }
}

void showDetectedLabels(
    const cv::Mat3b& img,
    const std::vector<LabelContour>& labels,
    const std::vector<std::vector<cv::Point2f>>& detected_corners)
{
    cv::Mat3b canvas = img * 0.25f;
    drawLabels(canvas, labels, cv::Scalar(255, 255, 255));

    for (const auto& corners : detected_corners)
    {
        cv::polylines(canvas, cast<cv::Point>(corners), true, cv::Scalar(0, 0, 255));
        for (size_t i = 0; i < corners.size(); ++i)
        {
            char text[12];
            sprintf(text, "%lu", i);
            cv::putText(canvas, text, corners[i],
                cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255));
        }
    }

    cv::imshow("detected labels", canvas);
}
