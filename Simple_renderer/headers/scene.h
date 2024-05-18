#pragma once

#include "camera.h"
#include "surface.h"
#include "light.h"

struct Scene {
    std::vector<Surface> surfaces;
    std::vector<uint32_t> surfaceIdxs;
    std::vector<Light> lights;
    Camera camera;
    Vector2i imageResolution;

    AABB bbox;
    BVHNode* nodes;
    int numBVHNodes = 0;

    Scene() {};
    Scene(std::string sceneDirectory, std::string sceneJson);
    Scene(std::string pathToJson);
    
    void parse(std::string sceneDirectory, nlohmann::json sceneConfig);

    void buildBVH();
    uint32_t getIdx(uint32_t idx);
    void updateNodeBounds(uint32_t nodeIdx);
    void subdivideNode(uint32_t nodeIdx);
    void intersectBVH(uint32_t nodeIdx, Ray& ray, Interaction& si);

    Interaction rayIntersect(Ray& ray);
    Interaction rayEmitterIntersect(Ray& ray);

    int Lights_count = 0;
};