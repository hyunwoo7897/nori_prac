/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    auto tent = [](float u) -> float {
        if (u < 0.5f)
            return -1.0f + sqrt(2.0f * u);
        else
            return 1.0f - sqrt(2.0f - 2.0f * u);
    };
    return Point2f(tent(sample.x()), tent(sample.y()));
}

float Warp::squareToTentPdf(const Point2f &p) {
    float P_t1, P_t2;
    if (p.x() <= 0.5f)
        P_t1 = 1.0f - abs(-1.0f + sqrt(2.0f * p.x()));
    else
        P_t1 = 1.0f - abs(1.0f - sqrt(2.0f * (1.0f - p.x())));
    if (p.y() <= 0.5f)
        P_t2 = 1.0f - abs(-1.0f + sqrt(2.0f * p.y()));
    else
        P_t2 = 1.0f - abs(1.0f - sqrt(2.0f * (1.0f - p.y())));
    return P_t1 * P_t2;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float P_x, P_y;
    P_x = std::sqrt(sample.x()) * std::cos(2.0f * M_PI * sample.y());
    P_y = std::sqrt(sample.x()) * std::sin(2.0f * M_PI * sample.y());
    return Point2f(P_x, P_y);
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return 2.0f * std::sqrt(p.x());
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float P_x, P_y, P_z;
    float phi = 2.0f * M_PI * sample.x();
    float theta = std::acos(-2.0f * (sample.y() - 1));
    P_x = std::sin(theta) * std::cos(phi);
    P_y = std::sin(theta) * std::sin(phi);
    P_z = std::cos(theta);
    return Vector3f(P_x, P_y, P_z);
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return std::sin(std::acos(-2.0f * (v.y() - 1))) / (4.0f * M_PI);
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float P_x, P_y, P_z;
    float phi = 2.0f * M_PI * sample.x();
    float theta = std::acos(-sample.y());
    P_x = std::sin(theta) * std::cos(phi);
    P_y = std::sin(theta) * std::sin(phi);
    P_z = std::cos(theta);
    return Vector3f(P_x, P_y, P_z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return std::sin(std::acos(-v.y())) / (2.0f * M_PI);
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float r = std::sqrt(sample.x());
    float theta = 2.0f * M_PI * sample.y();
    float x = r * std::cos(theta);
    float y = r * std::sin(theta);
    float z = std::sqrt(1.0f - r * r);
    return Vector3f(x, y, z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    return std::sin(2.0f * std::acos(2.0f * std::sqrt(1 - v.y()))) / (2.0f * M_PI);
}


Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float P_x, P_y, P_z;
    //float theta = std::atan(std::sqrt(-alpha * alpha * std::log(M_PI * sample.y() + 1.0f)));
    float theta = std::atan(std::sqrt(-alpha * alpha * std::log(1.0f - sample.y())));
    float phi = 2.0f * M_PI * sample.x();
    P_x = std::sin(theta) * std::cos(phi);
    P_y = std::sin(theta) * std::sin(phi); 
    P_z = std::cos(theta);
    return Vector3f(P_x, P_y, P_z);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    float cosTheta = m.z();
    if(cosTheta <= 0)
        return 0.0f;
    float tanTheta2 = (m.x() * m.x() + m.y() * m.y()) / (m.z() * m.z());
    float result = (std::exp(-tanTheta2 / (alpha * alpha))) / (M_PI * alpha * alpha * cosTheta * cosTheta * cosTheta);
    if (result < 1e-20f)
        return 0.0f; // Avoid numerical issues with very small values
    return result;
}

NORI_NAMESPACE_END
