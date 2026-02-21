#include "quaternion.hpp"

Quaternion quatConjugate(const Quaternion& q) {
    Quaternion out;
    out.w =  q.w;
    out.x = -q.x;
    out.y = -q.y;
    out.z = -q.z;
    return out;
}

Quaternion quatMultiply(const Quaternion& a, const Quaternion& b) {
    Quaternion out;
    out.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    out.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    out.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    out.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return out;
}

void quatNormalize(Quaternion& q) {
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n > 0.0f) {
        q.w /= n;
        q.x /= n;
        q.y /= n;
        q.z /= n;
    }
}