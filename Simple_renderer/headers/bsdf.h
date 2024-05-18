#pragma once

#include "common.h"
#include "texture.h"

class BSDF {
    public:
        // Constructor when textures are provided
        BSDF(std::string diffuseTexname, std::string alphaTexname, Vector3f diffuse, float alpha);

        // No parameter constructor
        BSDF() : BSDF("", "", Vector3f(1, 1, 1), 1) {}

        /**
         * Evaluate the amount of light reflected.
         * 
         * \param si
         * The interaction struct of the shading point
         * \param wo
         * The outgoing direction at the shading point
         * 
         * \return fac
         * The ratio of light reflected
         */
       Vector3f eval(Interaction *si, Vector3f wo);

    private:
        Vector3f diffuse;
        float alpha;
        Texture diffuseTexture, alphaTexture;

        bool hasDiffuseTexture();
        bool hasAlphaTexture();
    };