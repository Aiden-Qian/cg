#include "attributes.h"
#include <iostream>
#include <math.h>

const float PI = 3.14;

Triangle::Triangle(const VertexAttributes &v1, const VertexAttributes &v2, const VertexAttributes &v3) {
    vs[0] = v1;
    vs[1] = v2;
    vs[2] = v3;
}

bool Triangle::isInside(Eigen::Vector4f &pt) {
    float d1, d2, d3;
    bool neg, pos;
    d1=(pt.x() -vs[1].position.x()) * (vs[0].position.y() -vs[1].position.y()) - (vs[0].position.x() - vs[1].position.x()) * (pt.y() - vs[1].position.y());
    d2=(pt.x() -vs[2].position.x()) * (vs[1].position.y() -vs[2].position.y()) - (vs[1].position.x() - vs[2].position.x()) * (pt.y() - vs[2].position.y());
    d3=(pt.x() -vs[0].position.x()) * (vs[2].position.y() -vs[0].position.y()) - (vs[2].position.x() - vs[0].position.x()) * (pt.y() - vs[0].position.y());

    neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(neg && pos);
}


void Triangle::shift(Eigen::Vector4f &shift) {
    for (auto &x: vs) {
        x.position.x() += shift.x();
        x.position.y() += shift.y();
    }
}

void Triangle::shift(float x, float y) {
    for (auto &v: vs) {
        v.position.x() += x;
        v.position.y() += y;
    }
}

Eigen::Vector4f Triangle::centroid() {
    float mx,my;
    mx = my = 0;
    for (auto &v: vs) {
        mx += v.position.x();
        my += v.position.y();
    }
    return {mx/3.0, my/3.0, 0, 1};
}

void Triangle::rotate(float degree) {
    auto center = centroid();
    float cos = std::cos(PI*degree/180);
    float sin = std::sin(PI*degree/180);
    for (auto &v: vs) {
        auto ov = v.position-center;
        float nx = ov.x()*cos - ov.y()*sin;
        float ny = ov.x()*sin + ov.y()*cos;
        v.position.x() = nx + center.x();
        v.position.y() = ny + center.y();
    }
}

void Triangle::scale(float factor) {
    auto center = centroid();
    for (auto &v: vs) {
        auto ov = v.position-center;
        float nx = ov.x()*factor;
        float ny = ov.y()*factor;
        v.position.x() = nx + center.x();
        v.position.y() = ny + center.y();
    }
}
