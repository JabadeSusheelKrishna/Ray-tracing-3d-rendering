#pragma once

#include "common.h"

enum LightType {
    POINT_LIGHT=0,
    DIRECTIONAL_LIGHT=1,
    AREA_LIGHT=2,
    NUM_LIGHT_TYPES
};

struct LightSample {
    // Position of the light sample
    Vector3f p;

    // Outgoing direction in world coordinates
    Vector3f wo;

    // Distance to the sampled point
    float d;
};

class Light {
    public:
        Light(LightType type, nlohmann::json config);

        /**
         * Samples the light to return the outgoing direction along with
         * the appropriately scaled radiance.
         *
         * \param si
         * The interaction struct of the shading point
         *
         * \return scaled_radiance
         * The incoming radiance from the light source scaled by the PDF of sampling
         * \return wo
         * The sampled direction in world space
         */
        std::pair<Vector3f, LightSample> sample(Interaction *si);

        /**
         * Checks whether a ray intersects with the light.
         * The method returns immediately for POINT/DIRECTIONAL, with si.didIntersect = false
         *
         * \param ray
         * The ray to test for intersections
         *
         * \return si
         * The interaction struct describing intersection with the light if any
         */
        Interaction intersectLight(Ray *ray);

    // private:
        LightType type;

        // Applicable only for point lights.
        Vector3f position;

        // Applicable only for directional lights
        Vector3f direction;

        // Applicable only for area lights
        Vector3f center, vx, vy, normal;

        // Radiance of the emitter
        Vector3f radiance;

        int type_sample = -1;
};