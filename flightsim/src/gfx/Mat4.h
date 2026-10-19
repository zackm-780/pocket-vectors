#pragma once
// Minimal column-major 4x4 float matrix (OpenGL layout) for the renderer. Large-scale positions
// never go through this type: ECEF stays in double and is reduced to camera-relative floats first.

#include <cmath>

#include "core/Geo.h"

namespace fsim::gfx {

struct Mat4 {
    float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};  // m[col * 4 + row]

    float& at(int row, int col) { return m[col * 4 + row]; }
    float at(int row, int col) const { return m[col * 4 + row]; }

    static Mat4 perspective(float vFovRad, float aspect, float nearM, float farM) {
        Mat4 r;
        const float f = 1.0f / std::tan(vFovRad * 0.5f);
        for (float& v : r.m) v = 0;
        r.at(0, 0) = f / aspect;
        r.at(1, 1) = f;
        r.at(2, 2) = (farM + nearM) / (nearM - farM);
        r.at(2, 3) = 2.0f * farM * nearM / (nearM - farM);
        r.at(3, 2) = -1.0f;
        return r;
    }

    // Rotation (3x3) followed by... i.e. M = [R | R*t]: maps p -> R * (p + t).
    static Mat4 rotationOfTranslated(const Mat3d& rot, const float t[3]) {
        Mat4 r;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) r.at(row, col) = static_cast<float>(rot.m[row][col]);
            r.at(row, 3) = static_cast<float>(rot.m[row][0]) * t[0] +
                           static_cast<float>(rot.m[row][1]) * t[1] +
                           static_cast<float>(rot.m[row][2]) * t[2];
        }
        return r;
    }
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            float sum = 0;
            for (int k = 0; k < 4; ++k) sum += a.at(row, k) * b.at(k, col);
            r.at(row, col) = sum;
        }
    }
    return r;
}

}  // namespace fsim::gfx
