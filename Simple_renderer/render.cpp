#include "render.h"
int Area_sample = 0;
int Offset_samp = 0;
int sam_type = 0;

Integrator::Integrator(Scene &scene)
{
    this->scene = scene;
    this->outputImage.allocate(TextureType::UNSIGNED_INTEGER_ALPHA, this->scene.imageResolution);
}

long long Integrator::render()
{
    auto startTime = std::chrono::high_resolution_clock::now();
    for (int x = 0; x < this->scene.imageResolution.x; x++)
    {
        for (int y = 0; y < this->scene.imageResolution.y; y++)
        {
            bool nope = false;
            Vector3f Outcolor = Vector3f(0, 0, 0);
            Vector3f avg_col = Vector3f(0, 0, 0);
            for (int off = 0; off < Offset_samp && nope == false; off++)
            {
                Ray cameraRay = this->scene.camera.generateRay(x, y, 0.5, 0.5);
                // Ray cameraRay = this->scene.camera.generateRay(x, y, next_float(), next_float());
                Interaction si = this->scene.rayIntersect(cameraRay);
                Interaction si2 = this->scene.rayEmitterIntersect(cameraRay);
                Vector3f result(0, 0, 0);

                if (si.didIntersect)
                {
                    Vector3f radiance;
                    LightSample ls;
                    if (sam_type == 0 || sam_type == 1)
                    {
                        for (Light &light : this->scene.lights)
                        {
                            if (light.type != 2)
                            {
                                std::tie(radiance, ls) = light.sample(&si);

                                Ray shadowRay(si.p + 1e-3f * si.n, ls.wo);
                                Interaction siShadow = this->scene.rayIntersect(shadowRay);

                                if (!siShadow.didIntersect || siShadow.t > ls.d)
                                {
                                    result += si.bsdf->eval(&si, si.toLocal(ls.wo)) * radiance * std::abs(Dot(si.n, ls.wo));
                                }
                            }

                            if (light.type == 2)
                            {
                                for (int i = 0; i < Area_sample; i++)
                                {
                                    light.type_sample = sam_type;
                                    std::tie(radiance, ls) = light.sample(&si);
                                    ls.wo = si.toWorld2(ls.wo, si.n, si.p - this->scene.camera.from);

                                    Ray shadowRay(si.p + 1e-3f * si.n, (ls.wo));
                                    Interaction siShadow = this->scene.rayIntersect(shadowRay);

                                    Ray shadowRay2(si.p + 1e-3f * si.n, (ls.wo));
                                    Interaction area_test = this->scene.rayEmitterIntersect(shadowRay2);

                                    if (area_test.didIntersect)
                                    {
                                        if (!siShadow.didIntersect || siShadow.t > area_test.t)
                                        {
                                            if (Dot(ls.p, si.n) <= 0)
                                            {
                                                if (sam_type == 0)
                                                    result += si.bsdf->eval(&si, si.toLocal(ls.wo)) * radiance * (AbsDot(si.n, (ls.wo)));
                                                if (sam_type == 1)
                                                    result += si.bsdf->eval(&si, si.toLocal(ls.wo)) * radiance;
                                            }
                                        }
                                    }
                                }
                                result /= Area_sample;
                            }
                        }
                    }

                    if (sam_type == 2)
                    {
                        int num = -1;
                        for (int sampling = 0; sampling < this->scene.Lights_count; sampling++)
                        {
                            int ccount = 0;
                            num++;
                            for (Light &light : this->scene.lights)
                            {
                                if (light.type != 2)
                                {
                                    continue;
                                }
                                if (ccount != num)
                                {
                                    ccount++;
                                    continue;
                                }
                                Vector3f radiance;
                                LightSample ls;
                                Vector3f temp_res = Vector3f(0, 0, 0);
                                for (int i = 0; i < Area_sample; i++)
                                {
                                    // printf("-\n");
                                    light.type_sample = sam_type;
                                    std::tie(radiance, ls) = light.sample(&si);

                                    Ray shadowRay(si.p + 1e-3f * si.n, ls.wo);
                                    Interaction siShadow = this->scene.rayIntersect(shadowRay);
                                    if (!siShadow.didIntersect || siShadow.t > ls.d)
                                    {
                                        // printf("%f %f %f ", radiance.x, radiance.y, radiance.z);
                                        temp_res += si.bsdf->eval(&si, si.toLocal(ls.wo)) * radiance * std::abs(Dot(si.n, ls.wo));
                                    }
                                }
                                temp_res /= Area_sample;
                                temp_res *= 4;
                                result += temp_res;
                                break;
                            }
                        }
                    }

                    // Question - 3
                }
                avg_col += result;
                if (si2.didIntersect)
                {
                    avg_col += si2.colour_reflect;
                    Outcolor = si2.colour_reflect;
                    nope = true;
                }
            }
            if (nope == true)
            {
                this->outputImage.writePixelColor(Outcolor, x, y);
            }
            else
            {
                this->outputImage.writePixelColor(avg_col / Offset_samp, x, y);
            }
        }
        // printf("Layer : %d\n", x);
    }
    auto finishTime = std::chrono::high_resolution_clock::now();

    return std::chrono::duration_cast<std::chrono::microseconds>(finishTime - startTime).count();
}

int main(int argc, char **argv)
{
    if (argc != 6)
    {
        std::cerr << "Usage: ./render <scene_config> <out_path> <num_samples> <sampling_strategy> <area_samples>";
        return 1;
    }
    Scene scene(argv[1]);

    Integrator rayTracer(scene);
    int spp = atoi(argv[3]);
    Offset_samp = spp;
    sam_type = atoi(argv[4]);
    Area_sample = atoi(argv[5]);
    auto renderTime = rayTracer.render();

    std::cout << "Render Time: " << std::to_string(renderTime / 1000.f) << " ms" << std::endl;
    rayTracer.outputImage.save(argv[2]);

    return 0;
}
