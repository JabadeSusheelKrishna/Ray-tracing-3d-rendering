#include "light.h"

Light::Light(LightType type, nlohmann::json config)
{
    switch (type)
    {
    case LightType::POINT_LIGHT:
        this->position = Vector3f(config["location"][0], config["location"][1], config["location"][2]);
        break;
    case LightType::DIRECTIONAL_LIGHT:
        this->direction = Vector3f(config["direction"][0], config["direction"][1], config["direction"][2]);
        break;
    case LightType::AREA_LIGHT:
        this->center = Vector3f(config["center"][0], config["center"][1], config["center"][2]);
        this->vx = Vector3f(config["vx"][0], config["vx"][1], config["vx"][2]);
        this->vy = Vector3f(config["vy"][0], config["vy"][1], config["vy"][2]);
        this->normal = Vector3f(config["normal"][0], config["normal"][1], config["normal"][2]);
        this->radiance = Vector3f(config["radiance"][0], config["radiance"][1], config["radiance"][2]);
        break;
    default:
        std::cout << "WARNING: Invalid light type detected";
        break;
    }

    this->radiance = Vector3f(config["radiance"][0], config["radiance"][1], config["radiance"][2]);
    this->type = type;
}

float angle_rec(float range)
{
    float angle = next_float();
    angle -= 0.5;
    angle *= range * 2;
    return angle;
}

std::pair<Vector3f, LightSample> Light::sample(Interaction *si)
{
    LightSample ls;
    memset(&ls, 0, sizeof(ls));

    Vector3f radiance;
    switch (type)
    {
    case LightType::POINT_LIGHT:
        ls.wo = (position - si->p);
        ls.d = ls.wo.Length();
        ls.wo = Normalize(ls.wo);
        radiance = (1.f / (ls.d * ls.d)) * this->radiance;
        break;
    case LightType::DIRECTIONAL_LIGHT:
        ls.wo = Normalize(direction);
        ls.d = 1e10;
        radiance = this->radiance;
        break;
    case LightType::AREA_LIGHT:
        if (this->type_sample == 2)
        {
            Vector3f new_x = (next_float() - 0.5) * this->vx * 2;
            Vector3f new_y = (next_float() - 0.5) * this->vy * 2;
            ls.p = this->center + new_x + new_y;
            ls.d = (ls.p - si->p).Length();
            ls.wo = Normalize(ls.p - si->p);
            float cos_l = Dot(this->normal, -1 * ls.wo);
            // printf("cosl : %f\n", cos_l);
            radiance = this->radiance * (cos_l / (ls.d * ls.d)) * (this->vx.Length() * this->vy.Length());
        }

        if (this->type_sample == 0)
        {
            ls.p = this->normal;
            float alpha = (next_float()) * 2 * M_PI;
            float beeta = (next_float());
            beeta = acos(beeta);
            float x = sin(beeta) * cos(alpha);
            float y = sin(beeta) * sin(alpha);
            float z = cos(beeta);
            ls.wo = Vector3f(x, y, z);
            ls.d = 1000;
            radiance = this->radiance * M_PI * 2; // Update the radiance term
        }

        if (this->type_sample == 1)
        {
            ls.p = this->normal;
            float n1 = next_float();
            float n2 = next_float();
            float phi = n1 * 2 * M_PI;
            float zi = sqrt(n2);
            float theta = acos(zi);
            float x = sin(theta) * cos(phi);
            float y = sin(theta) * sin(phi);
            float z = cos(theta);
            ls.wo = Vector3f(x, y, z);
            radiance = this->radiance * M_PI;
        }
        break;
    }
    return {radiance, ls};
}

Interaction ray_plane_intersect(Ray ray, Vector3f p, Vector3f n)
{
    Interaction si;

    float dDotN = Dot(ray.d, n);
    if (dDotN != 0.f)
    {
        float t = -Dot((ray.o - p), n) / dDotN;

        if (t >= 0.f)
        {
            si.didIntersect = true;
            si.t = t;
            si.n = n;
            si.p = ray.o + ray.d * si.t;
        }
    }

    return si;
}

Interaction Light::intersectLight(Ray *ray)
{
    Interaction si;
    memset(&si, 0, sizeof(si));

    int Var = 0;

    if (type == LightType::AREA_LIGHT)
    {
        Vector3f Av1, Av2, Av3;
        Vector3f Bv1, Bv2, Bv3;
        Av1 = this->center - this->vx - this->vy;
        Bv1 = this->center + this->vx + this->vy;
        Av2 = this->center + this->vx - this->vy;
        Bv3 = this->center - this->vx - this->vy;
        Av3 = this->center + this->vx + this->vy;
        Bv2 = this->center - this->vx + this->vy;

        Interaction si = ray_plane_intersect(*ray, Av1, this->normal);
        Interaction si2 = ray_plane_intersect(*ray, Bv1, this->normal);
        if (si.didIntersect)
        {
            bool edge1 = false, edge2 = false, edge3 = false;

            // Check edge 1
            {
                Vector3f nIp = Cross((si.p - Av1), (Av3 - Av1));
                Vector3f nTri = Cross((Av2 - Av1), (Av3 - Av1));
                edge1 = Dot(nIp, nTri) > 0;
            }

            // Check edge 2
            {
                Vector3f nIp = Cross((si.p - Av1), (Av2 - Av1));
                Vector3f nTri = Cross((Av3 - Av1), (Av2 - Av1));
                edge2 = Dot(nIp, nTri) > 0;
            }

            // Check edge 3
            {
                Vector3f nIp = Cross((si.p - Av2), (Av3 - Av2));
                Vector3f nTri = Cross((Av1 - Av2), (Av3 - Av2));
                edge3 = Dot(nIp, nTri) > 0;
            }

            if (edge1 && edge2 && edge3 && si.t >= 0.f && Var == 0)
            {
                // Intersected triangle!
                si.didIntersect = true;
                si.colour_reflect = this->radiance;
                // si.colour_reflect = Vector3f(1, 1, 1);
            }
            else
            {
                si.didIntersect = false;
            }
        }
        if (si2.didIntersect)
        {
            bool edge1 = false, edge2 = false, edge3 = false;

            // Check edge 1
            {
                Vector3f nIp = Cross((si.p - Bv1), (Bv3 - Bv1));
                Vector3f nTri = Cross((Bv2 - Bv1), (Bv3 - Bv1));
                edge1 = Dot(nIp, nTri) > 0;
            }

            // Check edge 2
            {
                Vector3f nIp = Cross((si.p - Bv1), (Bv2 - Bv1));
                Vector3f nTri = Cross((Bv3 - Bv1), (Bv2 - Bv1));
                edge2 = Dot(nIp, nTri) > 0;
            }

            // Check edge 3
            {
                Vector3f nIp = Cross((si.p - Bv2), (Bv3 - Bv2));
                Vector3f nTri = Cross((Bv1 - Bv2), (Bv3 - Bv2));
                edge3 = Dot(nIp, nTri) > 0;
            }

            if (edge1 && edge2 && edge3 && si.t >= 0.f && Var == 0)
            {
                // Intersected triangle!
                si.didIntersect = true;
                si.colour_reflect = this->radiance;
                // si.colour_reflect = Vector3f(1, 1, 1);
            }
        }

        return si;
    }

    return si;
}