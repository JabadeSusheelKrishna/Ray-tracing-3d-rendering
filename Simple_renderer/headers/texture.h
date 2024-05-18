#pragma once

#include "common.h"

enum TextureType {
    UNSIGNED_INTEGER_ALPHA = 0, // RGBA uint32
    FLOAT_ALPHA, // RGBA float
    NUM_TEXTURE_TYPES
};

struct Texture {
    unsigned long long data = 0;
    TextureType type;

    Vector2i resolution;

    Texture() {};
    Texture(std::string pathToImage);

    void allocate(TextureType type, Vector2i resolution);
    void writePixelColor(Vector3f color, int x, int y);

    Vector3f nearestNeighbourFetch(Vector2f uv);
    
    void loadJpg(std::string pathToJpg);
    void loadPng(std::string pathToPng);
    void loadExr(std::string pathToExr);
        
    void save(std::string path);
    void saveExr(std::string path);
    void savePng(std::string path);
};