#ifndef PROJECTION_H
#define PROJECTION_H

#include "tools/rotation.h"

// camera : 9 dims array with
// [0-2] : angle-axis rotation
// [3-5] : translateion
// [6-8] : camera parameter, [6] focal length, [7-8] second and forth order radial distortion
// point : 3D location.
// predictions : 2D predictions with center of the image plane.

template<typename T>
inline bool CamProjectionWithDistortion(const T* camera, const T* point, T* predictions){
    // Rodrigues' formula
    T p[3];
    AngleAxisRotatePoint(camera, point, p); // p = R * point
    // camera[3,4,5] are the translation
    p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5]; // p = p + t，所以这两步是把世界坐标系的点转到相机坐标系

    // Compute the center fo distortion
    T xp = -p[0]/p[2];
    T yp = -p[1]/p[2];  // 归一化

    // Apply second and fourth order radial distortion
    const T& l1 = camera[7];
    const T& l2 = camera[8];

    T r2 = xp*xp + yp*yp;
    T distortion = T(1.0) + r2 * (l1 + l2 * r2);

    // x_distorted = x(1 + k1 * r^2 + k2 * r^4)
    // y_distorted = y(1 + k1 * r^2 + k2 * r^4)
    // cx = cy = 0;
    // u = f * x_distorted
    // v = f * y_distorted
    const T& focal = camera[6];
    predictions[0] = focal * distortion * xp;
    predictions[1] = focal * distortion * yp;

    return true;
}



#endif // projection.h
