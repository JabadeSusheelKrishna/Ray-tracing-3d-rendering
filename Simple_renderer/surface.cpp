#include "bsdf.h"
#include "surface.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"

std::vector<Surface> createSurfaces(std::string pathToObj, bool isLight, uint32_t shapeIdx)
{
    std::string objDirectory;
    const size_t last_slash_idx = pathToObj.rfind('/');
    if (std::string::npos != last_slash_idx) {
        objDirectory = pathToObj.substr(0, last_slash_idx);
    }

    std::vector<Surface> surfaces;

    tinyobj::ObjReader reader;
    tinyobj::ObjReaderConfig reader_config;
    if (!reader.ParseFromFile(pathToObj, reader_config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }

    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        Surface surf;
        surf.isLight = isLight;
        surf.shapeIdx = shapeIdx;
        std::set<int> materialIds;

        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            if (fv > 3) {
                std::cerr << "Not a triangle mesh" << std::endl;
                exit(1);
            }

            // Loop over vertices in the face. Assume 3 vertices per-face
            Vector3f vertices[3], normals[3];
            Vector2f uvs[3];
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

                // Check if `normal_index` is zero or positive. negative = no normal data
                if (idx.normal_index >= 0) {
                    tinyobj::real_t nx = attrib.normals[3 * size_t(idx.normal_index) + 0];
                    tinyobj::real_t ny = attrib.normals[3 * size_t(idx.normal_index) + 1];
                    tinyobj::real_t nz = attrib.normals[3 * size_t(idx.normal_index) + 2];

                    normals[v] = Vector3f(nx, ny, nz);
                }

                // Check if `texcoord_index` is zero or positive. negative = no texcoord data
                if (idx.texcoord_index >= 0) {
                    tinyobj::real_t tx = attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
                    tinyobj::real_t ty = attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];

                    uvs[v] = Vector2f(tx, ty);
                }

                vertices[v] = Vector3f(vx, vy, vz);
            }

            int vSize = surf.vertices.size();
            Vector3i findex(vSize, vSize + 1, vSize + 2);

            surf.vertices.push_back(vertices[0]);
            surf.vertices.push_back(vertices[1]);
            surf.vertices.push_back(vertices[2]);

            surf.normals.push_back(normals[0]);
            surf.normals.push_back(normals[1]);
            surf.normals.push_back(normals[2]);

            surf.uvs.push_back(uvs[0]);
            surf.uvs.push_back(uvs[1]);
            surf.uvs.push_back(uvs[2]);

            surf.indices.push_back(findex);

            // Create Triangle
            Tri triangle;
            triangle.v1 = vertices[0];
            triangle.v2 = vertices[1];
            triangle.v3 = vertices[2];

            triangle.uv1 = uvs[0];
            triangle.uv2 = uvs[1];
            triangle.uv3 = uvs[2];

            triangle.normal = Normalize(normals[0] + normals[1] + normals[2]);
            triangle.centroid = (triangle.v1 + triangle.v2 + triangle.v3) / 3.f;

            for (int i = 0; i < 3; i++) {
                triangle.bbox.min = Vector3f(
                    std::min(triangle.bbox.min.x, vertices[i].x),
                    std::min(triangle.bbox.min.y, vertices[i].y),
                    std::min(triangle.bbox.min.z, vertices[i].z)
                );

                triangle.bbox.max = Vector3f(
                    std::max(triangle.bbox.max.x, vertices[i].x),
                    std::max(triangle.bbox.max.y, vertices[i].y),
                    std::max(triangle.bbox.max.z, vertices[i].z)
                );

                triangle.bbox.centroid = (triangle.bbox.min + triangle.bbox.max) / 2.f;
            }

            surf.tris.push_back(triangle);

            // BVH indirection indices
            surf.triIdxs.push_back(f);

            // Update surface AABB
            surf.bbox.min = Vector3f(
                std::min(surf.bbox.min.x, triangle.bbox.min.x),
                std::min(surf.bbox.min.y, triangle.bbox.min.y),
                std::min(surf.bbox.min.z, triangle.bbox.min.z)
            );

            surf.bbox.max = Vector3f(
                std::max(surf.bbox.max.x, triangle.bbox.max.x),
                std::max(surf.bbox.max.y, triangle.bbox.max.y),
                std::max(surf.bbox.max.z, triangle.bbox.max.z)
            );

            surf.bbox.centroid = (surf.bbox.min + surf.bbox.max) / 2.f;

            // per-face material
            materialIds.insert(shapes[s].mesh.material_ids[f]);

            index_offset += fv;
        }

        if (materialIds.size() > 1) {
            std::cerr << "One of the meshes has more than one material. This is not allowed." << std::endl;
            exit(1);
        }


        if (materialIds.size() == 0) {
            std::cerr << "One of the meshes has no material definition, may cause unexpected behaviour." << std::endl;
        } else {
            // Load textures from Materials
            auto matId = *materialIds.begin();
            if (matId != -1) {
                auto mat = materials[matId];

                Vector3f diffuse(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
                float alpha = mat.specular[0];
                std::string diffuseTexname("");
                if (mat.diffuse_texname != "") {
                    diffuseTexname = objDirectory + "/" + mat.diffuse_texname;
                }
                std::string alphaTexname("");
                if (mat.specular_texname != "") {
                    alphaTexname = objDirectory + "/" + mat.alpha_texname;
                }
                surf.bsdf = BSDF(
                    diffuseTexname,
                    alphaTexname,
                    diffuse,
                    alpha
                );
            } else {
                surf.bsdf = BSDF("", "", Vector3f(1, 1, 1), 1);
            }
        }

        // Allocate memory for BVH & build the BVH
        surf.nodes = (BVHNode*)malloc((2 * surf.triIdxs.size() - 1) * sizeof(BVHNode));
        for (int i = 0; i < 2 * surf.triIdxs.size() - 1; i++) {
            surf.nodes[i] = BVHNode();
        }

        surf.buildBVH();

        surfaces.push_back(surf);
        shapeIdx++;
    }

    return surfaces;
}

Interaction Surface::rayPlaneIntersect(Ray ray, Vector3f p, Vector3f n)
{
    Interaction si;

    float dDotN = Dot(ray.d, n);
    if (dDotN != 0.f) {
        float t = -Dot((ray.o - p), n) / dDotN;

        if (t >= 0.f) {
            si.didIntersect = true;
            si.t = t;
            si.n = n;
            si.p = ray.o + ray.d * si.t;
        }
    }

    return si;
}

Interaction Surface::rayTriangleIntersect(Ray ray, Vector3f v1, Vector3f v2, Vector3f v3, Vector3f n)
{
    Interaction si = this->rayPlaneIntersect(ray, v1, n);

    if (si.didIntersect) {
        bool edge1 = false, edge2 = false, edge3 = false;

        // Check edge 1
        {
            Vector3f nIp = Cross((si.p - v1), (v3 - v1));
            Vector3f nTri = Cross((v2 - v1), (v3 - v1));
            edge1 = Dot(nIp, nTri) > 0;
        }

        // Check edge 2
        {
            Vector3f nIp = Cross((si.p - v1), (v2 - v1));
            Vector3f nTri = Cross((v3 - v1), (v2 - v1));
            edge2 = Dot(nIp, nTri) > 0;
        }

        // Check edge 3
        {
            Vector3f nIp = Cross((si.p - v2), (v3 - v2));
            Vector3f nTri = Cross((v1 - v2), (v3 - v2));
            edge3 = Dot(nIp, nTri) > 0;
        }

        if (edge1 && edge2 && edge3 && si.t >= 0.f) {
            // Intersected triangle!
            si.didIntersect = true;
        }
        else {
            si.didIntersect = false;
        }
    }

    return si;
}

void Surface::buildBVH()
{
    // Root node
    this->numBVHNodes += 1;

    BVHNode& rootNode = this->nodes[0];
    rootNode.firstPrim = 0;
    rootNode.primCount = this->triIdxs.size();

    this->updateNodeBounds(0);
    this->subdivideNode(0);
}

uint32_t Surface::getIdx(uint32_t idx)
{
    return this->triIdxs[idx];
}

void Surface::updateNodeBounds(uint32_t nodeIdx)
{
    BVHNode& node = this->nodes[nodeIdx];

    for (int i = 0; i < node.primCount; i++) {
        auto triangle = this->tris[this->getIdx(i + node.firstPrim)];
        node.bbox.min = Vector3f(
            std::min(node.bbox.min.x, triangle.bbox.min.x),
            std::min(node.bbox.min.y, triangle.bbox.min.y),
            std::min(node.bbox.min.z, triangle.bbox.min.z)
        );

        node.bbox.max = Vector3f(
            std::max(node.bbox.max.x, triangle.bbox.max.x),
            std::max(node.bbox.max.y, triangle.bbox.max.y),
            std::max(node.bbox.max.z, triangle.bbox.max.z)
        );

        node.bbox.centroid = (node.bbox.min + node.bbox.max) / 2.f;
    }
}

void Surface::subdivideNode(uint32_t nodeIdx)
{
    BVHNode& node = this->nodes[nodeIdx];

    if (node.primCount <= 1) return;

    Vector3f extent = node.bbox.max - node.bbox.min;

    int ax = 0;
    if (extent.y > extent.x) ax = 1;
    if (extent.z > extent[ax]) ax = 2;
    float split = node.bbox.min[ax] + extent[ax] * 0.5f;

    int i = node.firstPrim;
    int j = i + node.primCount - 1;

    while (i <= j) {
        if (this->tris[this->getIdx(i)].centroid[ax] < split)
            i++;
        else {
            auto temp = this->triIdxs[i];
            this->triIdxs[i] = this->triIdxs[j];
            this->triIdxs[j--] = temp;
        }
    }

    int leftCount = i - node.firstPrim;
    if (leftCount == 0 || leftCount == node.primCount) return;

    uint32_t lidx = this->numBVHNodes++;
    BVHNode& left = this->nodes[lidx];
    left.firstPrim = node.firstPrim;
    left.primCount = leftCount;
    this->updateNodeBounds(lidx);

    uint32_t ridx = this->numBVHNodes++;
    BVHNode& right = this->nodes[ridx];
    right.firstPrim = i;
    right.primCount = node.primCount - leftCount;
    this->updateNodeBounds(ridx);

    node.left = lidx;
    node.right = ridx;
    node.primCount = 0;

    this->subdivideNode(lidx);
    this->subdivideNode(ridx);
}

void Surface::intersectBVH(uint32_t nodeIdx, Ray& ray, Interaction& si)
{
    BVHNode& node = this->nodes[nodeIdx];

    if (!node.bbox.intersects(ray)) return;

    if (node.primCount != 0) {
        // Leaf
        for (uint32_t i = 0; i < node.primCount; i++) {
            Vector3f v1 = this->tris[this->getIdx(i + node.firstPrim)].v1;
            Vector3f v2 = this->tris[this->getIdx(i + node.firstPrim)].v2;
            Vector3f v3 = this->tris[this->getIdx(i + node.firstPrim)].v3;
            Vector3f normal = this->tris[this->getIdx(i + node.firstPrim)].normal;

            Vector2f uv1 = this->tris[this->getIdx(i + node.firstPrim)].uv1;
            Vector2f uv2 = this->tris[this->getIdx(i + node.firstPrim)].uv2;
            Vector2f uv3 = this->tris[this->getIdx(i + node.firstPrim)].uv3;

            Interaction siIntermediate = this->rayTriangleIntersect(
                ray, v1, v2, v3, normal);

            if (siIntermediate.t <= ray.t && siIntermediate.didIntersect) {
                si = siIntermediate;
                ray.t = si.t;

                // Barycentric interpolation of UV coordinates
                float triArea = 0.5f * Cross(v2 - v1, v3 - v1).Length();
                float alpha = 0.5f * Cross(si.p - v2, v3 - v2).Length() / triArea;
                float gamma = 0.5f * Cross(si.p - v2, v1 - v2).Length() / triArea;
                float beta = 0.5f * Cross(si.p - v1, v3 - v1).Length() / triArea;
                Vector2f uv = alpha * uv1 +
                    beta * uv2 +
                    gamma * uv3;
                uv.x = std::min(std::max(uv.x, 0.f), 1.f);
                uv.y = std::min(std::max(uv.y, 0.f), 1.f);
                si.uv = uv;

                si.bsdf = &this->bsdf;

                // TODO: Obtain the ONB and store in interaction here

                // Set the view direction in local coordinates
                // TODO: Uncomment this after implementing ONB
                // si.wi = si.toLocal(-ray.d);
            }
        }
    }
    else {
        this->intersectBVH(node.left, ray, si);
        this->intersectBVH(node.right, ray, si);
    }
}

Interaction Surface::rayIntersect(Ray& ray)
{
    Interaction si;
    si.didIntersect = false;

    this->intersectBVH(0, ray, si);

    return si;
}