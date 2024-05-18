#pragma once

#include "vec.h"

// Forward declaration of BSDF class
class BSDF;

struct Interaction
{
    Vector3f colour_reflect;
    // Position of interaction
    Vector3f p;
    // Normal of the surface at interaction
    Vector3f n;
    // The uv co-ordinates at the intersection point
    Vector2f uv;
    // The viewing direction in local shading frame
    Vector3f wi;
    // Distance of intersection point from origin of the ray
    float t = 1e30f;
    // Used for light intersection, holds the radiance emitted by the emitter.
    Vector3f emissiveColor = Vector3f(0.f, 0.f, 0.f);
    // BSDF at the shading point
    BSDF *bsdf;
    // Vectors defining the orthonormal basis
    Vector3f a, b, c;

    bool didIntersect = false;

    Vector3f toWorld(Vector3f w) {
        // TODO: Implement this
        return Vector3f(0, 0, 0);
    }

    Vector3f toWorld2(Vector3f w, Vector3f Normal, Vector3f omega_o)
    {
        Vector3f z_dash = Normalize(Normal);
        Vector3f y_dash = Normalize(Cross(z_dash, omega_o));
        Vector3f x_dash = Normalize(Cross(y_dash, z_dash));

        Vector3f a1 = Vector3f(x_dash.x, y_dash.x, z_dash.x);
        Vector3f a2 = Vector3f(x_dash.y, y_dash.y, z_dash.y);
        Vector3f a3 = Vector3f(x_dash.z, y_dash.z, z_dash.z);

        return Vector3f(Dot(a1, w), Dot(a2, w), Dot(a3, w));
    }

    Vector3f toLocal(Vector3f w)
    {
        // Ensure the basis is orthonormal
        // a = Normalize(a);
        // b = Normalize(Cross(n, a));
        // c = Cross(a, b);

        // Transform vector w from world to local coordinates
        return Vector3f(0, 0, 0);
    }
};