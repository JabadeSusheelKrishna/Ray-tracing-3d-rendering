#include "bsdf.h"

BSDF::BSDF(std::string diffuseTexname, std::string alphaTexname, Vector3f diffuse, float alpha) {
    this->diffuse = diffuse;
    if (diffuseTexname != "") {
        this->diffuseTexture = Texture(diffuseTexname);
    }

    this->alpha = alpha;
    if (alphaTexname != "") {
        this->alphaTexture = Texture(alphaTexname);
    }
}

Vector3f BSDF::eval(Interaction *si, Vector3f wo) {
    Vector3f diffuseColor = diffuse;
    if (this->hasDiffuseTexture())
        diffuseColor = this->diffuseTexture.nearestNeighbourFetch(si->uv);
    return diffuseColor / M_PI;
}

bool BSDF::hasDiffuseTexture() {
    return diffuseTexture.data != 0;
}
bool BSDF::hasAlphaTexture() {
    return alphaTexture.data != 0;
}