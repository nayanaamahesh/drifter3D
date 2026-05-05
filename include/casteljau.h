#include <list>
#include <vector>
#include <algorithm>

#include "point.h"

// Helper function: evaluate the Bezier curve at parameter t using De Casteljau
point evaluate(float t, std::list<point> P) {
    // Copy P to Q
    std::list<point> Q = P;

    // Repeat until Q has only 1 point
    while (Q.size() > 1) {
        std::list<point> R; // new empty list

        // iterate through adjacent pairs in Q
        auto p1 = Q.begin();
        auto p2 = std::next(Q.begin());
        while (p2 != Q.end()) {
            // linear interpolation between p1 and p2
            point p = ((1 - t) * (*p1)) + (t * (*p2));
            R.push_back(p);

            ++p1;
            ++p2;
        }

        Q.clear();   // empty Q before copying
        Q = R;       // assign R to Q
    }

    return Q.front(); // return the single point left
}

// Main function: evaluate Bezier curve at num_evaluations points
std::vector<point> EvaluateBezierCurve(std::vector<point> ctrl_points, int num_evaluations) {
    std::vector<point> curve;
    float offset = 1.0f / num_evaluations;

    // push the first control point
    curve.push_back(ctrl_points.front());

    for (int e = 0; e < num_evaluations; ++e) {
        float t = offset * (e + 1);
        std::list<point> ps(ctrl_points.begin(), ctrl_points.end());
        point p = evaluate(t, ps);
        curve.push_back(p);
    }

    return curve;
}

float* MakeFloatsFromVector(const std::vector<point>& curve, int& num_verts, int& num_floats, float r, float g, float b)
{
    num_verts = static_cast<int>(curve.size()); // number of points in the curve

    if (num_verts == 0) {
        num_floats = 0;
        return nullptr; // nothing to allocate
    }

    num_floats = num_verts * 6; // 3 floats for position + 3 floats for color per vertex

    float* vertices = new float[num_floats]; // allocate memory

    for (int i = 0; i < num_verts; ++i) {
        vertices[i * 6 + 0] = curve[i].x; // position x
        vertices[i * 6 + 1] = curve[i].y; // position y
        vertices[i * 6 + 2] = 0.0f;       // position z (flat curve in XY plane)
        vertices[i * 6 + 3] = r;          // color r
        vertices[i * 6 + 4] = g;          // color g
        vertices[i * 6 + 5] = b;          // color b
    }

    return vertices;
}

