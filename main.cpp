#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <cmath>
#include <cfloat>
#include <unordered_map>
#include <ctime>

//#define WIDTH 3840
//#define HEIGHT 2160
#define WIDTH 500
#define HEIGHT 500

#define ARRAYSIZE(x) (sizeof(x)/sizeof((x)[0]))

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "vec3.h"
#include "vec2.h"
#include "math.h"

static constexpr Vector3f GLOBAL_UP_VECTOR = Vector3f(0.0f, 0.0f, 1.0f);

struct Camera3f {
    Vector3f position;
    Vector3f up, right, forward;
    float horizontalFOV; // in radians
    float aspectRatioY;

    Camera3f(Vector3f const& eye, Vector3f const& center, float const& aspect, float const& fov)
    : aspectRatioY(aspect), horizontalFOV(fov), position(eye)
    {
        forward = Normalize(center - eye);
        right = Normalize(Cross(forward, GLOBAL_UP_VECTOR));
        up = Cross(right, forward);
    }
};

struct Plane {
    float x, y, z, w;

    Plane() = default;

    Plane(float nx, float ny, float nz, float d)
    : x(nx), y(ny), z(nz), w(d)
    {}

    Plane(const Vector3f& n, float d) : x(n.x), y(n.y), z(n.z), w(d) {}

    operator const Vector3f&() const
    {
        return (reinterpret_cast<const Vector3f&>(x));
    }
};

static float IntersectRayWithPlane(const Plane &f, const Point3f& p, const Vector3f& v)
{
    float fv = Dot(f,v);

    if (fabsf(fv) > FLT_MIN) {
        return Dot(f,p)/fv;
    }

    return -1.0f;
}

static float IntersectSphere(Vector3f const& ro, Vector3f const& rd, Vector3f const& center, float radius)
{
    Vector3f v = ro - center;
    float b = 2.0f*Dot(rd,v), c = Dot(v,v) - radius*radius, delta = b*b - 4.0f*c;

    if(delta < 0.f) return -1.0f;
    else if(delta == 0.0f && -b >= 0.0) return -b*0.5f;
    else if(delta > 0.0f) {
        float h = sqrt(delta);
        float t0 = -b+h, t1 = -b-h;
        float tmin = Min(t0, t1), tmax = Max(t0, t1);
        return (tmin > 0.0f ? tmin : tmax > 0.0f ? tmax : -1.0f)*0.5f;
    }
    return -1.0f;
}

static Vector3f ComputeSphereNormal(Vector3f const& center, float radius, Vector3f const& pos)
{
    return (pos - center) / radius;
}

#define MSAA_SAMPLES 4
#define RAY_T_MAX 1000.0f
#define RAY_T_MIN 0.03

struct Sphere {
    Vector3f c;
    float r;
    int matid;

    Sphere() = default;

    Sphere(float sx, float sy, float sz, float sr, int materialId)
    : c(sx, sy, sz), r(sr), matid(materialId)
    {}

    Sphere(const Vector3f &sc, float sr, int materialId)
    : c(sc), r(sr), matid(materialId)
    {}
};


struct PointLight {
    Point3f origin;
    float power;
    Vector3f color;
};

struct SphereLight {
    Point3f origin;
    float radius;
    float power;
    Vector3f color;
};

enum GeometryType : int {
    GEOMETRY_TYPE_INVALID = -1,
    GEOMETRY_TYPE_SPHERE = 0,
    GEOMETRY_TYPE_PLANE = 1,
    GEOMETRY_TYPE_AXIS_ALIGNED_BOUNDING_BOX = 2,
    GEOMETRY_TYPE_ORIENTED_BOUNDING_BOX = 3,
    GEOMETRY_TYPE_TRIANGLE = 4
};

struct Material {
    int matid;
    Vector3f albedo;

    Material() = default;
};

typedef int MaterialId;

struct Scene {
    uint32_t sphereCount;
    Sphere *spheres;

    uint32_t planeCount;
    Plane *planes;

    uint32_t cameraCount;
    Camera3f *cameras;

    uint32_t pointLightCount;
    PointLight *pointLights;

    uint32_t sphereLightCount;
    SphereLight *sphereLights;

    int lastMaterialId;
    std::unordered_map<int, Material> materials;
};

static Scene CreateScene()
{
    Scene scene = {};
    scene.sphereCount = 0;
    scene.spheres = NULL;
    scene.planeCount = 0;
    scene.planes = NULL;
    scene.cameraCount = 0;
    scene.cameras = NULL;
    scene.pointLightCount = 0;
    scene.pointLights = NULL;
    scene.sphereLightCount = 0;
    scene.sphereLights = NULL;
    scene.lastMaterialId = 0;
    return scene;
}

static void AddSphereToScene(Scene *scene, Sphere sphere)
{
    size_t n = scene->sphereCount+1;
    if (scene->sphereCount == 0)
        scene->spheres = (Sphere *) malloc(sizeof(Sphere) * n);
    else
        scene->spheres = (Sphere *) realloc(scene->spheres, sizeof(Sphere) * n);

    scene->spheres[scene->sphereCount] = sphere;
    scene->sphereCount = n;
}

static int AddMaterialToScene(Scene *scene, Material material)
{
    int matid = scene->lastMaterialId++;
    scene->materials[matid] = material;
    return matid;
}

struct Ray {
    float t1, t2;
    Point3f o;
    Vector3f d;

    Ray() = default;

    Ray(const Point3f &ro, const Vector3f& rd, float t1 = RAY_T_MIN, float t2 = RAY_T_MAX)
    : o(ro), d(rd), t1(t1), t2(t2)
    {}
};

struct HitRecord {
    float t;
    int objectId, materialId;
    Point3f collisionPoint;
    Vector3f n;
    GeometryType geometryType;

    HitRecord() = default;
};

static bool RayTraceScene(const Scene *scene, Ray ray, HitRecord *hit)
{
    float closestT = ray.t2; // Start at maximum ray distance
    int objectId = -1;
    GeometryType geometryType = GEOMETRY_TYPE_INVALID;

    // Test intersection with every kind of geometry
    for (int i = 0; i < scene->sphereCount; ++i) {
        const Sphere *s = &scene->spheres[i];
        float t = IntersectSphere(ray.o,ray.d,s->c,s->r);

        if (t >= ray.t1 && t < closestT) {
            if (hit == nullptr) {
                // Early exit, we do not care about testing more objects to find closest hit points
                // and surface attributes
                return true;
            }

            closestT = t, objectId = i, geometryType = GEOMETRY_TYPE_SPHERE;
        }
    }

    if (objectId != -1) {
        // Exit if hit record structure is not provided
        if (hit == nullptr)
            return true;

        Point3f p = ray.o + ray.d * closestT;
        

        switch (geometryType) {
            case GEOMETRY_TYPE_SPHERE: {
                const Sphere *s = &scene->spheres[objectId];
                hit->n = ComputeSphereNormal(s->c, s->r, p);
                hit->materialId = s->matid;
                break;
            }
        }

        hit->t = closestT;
        hit->geometryType = geometryType;
        hit->objectId = objectId;
        hit->collisionPoint = p;
        return true;
    }

    return false;
}

static inline float RandFloatInRange(float x1, float x2)
{
    float len = fabsf(x2-x1);
    return ((float)(rand()%INT16_MAX) / (float)INT16_MAX)*len + x1;
}

int main(int argc, char *argv[])
{
    uint8_t *buffer = (uint8_t *)malloc(WIDTH * HEIGHT * 4);

    float ASPECT = (float)HEIGHT/(float)WIDTH;

    srand(time(NULL));

    Scene scene = CreateScene();

    int sphereMaterialIds[32];

    for (int & sphereMaterialId : sphereMaterialIds) {
        Material material;
        material.albedo = Vector3f((float)(rand()%1000)/1000.0f,
                                   (float)(rand()%1000)/1000.0f,
                                   (float)(rand()%1000)/1000.0f);

        sphereMaterialId = AddMaterialToScene(&scene, material);
    }

    float E = 1.5f, S = 0.32f, R = 0.314f;
//    for (float x = -E; x <= +E; x += S) {
//        for (float y = -E; y <= +E; y += S) {
//            for (float z = -E; z <= +E; z += S) {
//                AddSphereToScene(&scene, Sphere(Vector3f(x,y,z),R,rand()%ARRAYSIZE(sphereMaterialIds)));
//            }
//        }
//    }

    for (int i = 0; i < 50; ++i) {
        float x = RandFloatInRange(-E,+E);
        float y = RandFloatInRange(-E,+E);
        float z = RandFloatInRange(-E,+E);
        float radius = RandFloatInRange(0.121f,0.274f);
        AddSphereToScene(&scene, Sphere(Vector3f(x,y,z),radius,rand()%ARRAYSIZE(sphereMaterialIds)));
    }

    Camera3f camera(Vector3f(0.0f, -2.6f, 1.9f), Vector3f(0.0f, 0.0f, 0.0f), ASPECT, M_PI/2.0f);

    for (int i = 0; i < WIDTH; ++i) {
        for (int j = 0; j < HEIGHT; ++j) {
            unsigned char *pixel = buffer + (j * WIDTH + i) * 4;

            Vector3f color = {0};

            for (int msaaX = 0; msaaX < MSAA_SAMPLES; ++msaaX) {
                for (int msaaY = 0; msaaY < MSAA_SAMPLES; ++msaaY) {
                    float jitterX = ((float)msaaX + 0.5f) / (float)MSAA_SAMPLES;
                    float jitterY = ((float)msaaY + 0.5f) / (float)MSAA_SAMPLES;

                    Vector2f uv = Vector2f(((float)i+jitterX) / (float)WIDTH,
                                           ((float)j+jitterY) / (float)HEIGHT);
                    Vector2f p = uv * 2.0f - 1.0f;

                    float halfWidth = tanf(camera.horizontalFOV/2.0f);
                    float halfHeight = halfWidth * camera.aspectRatioY;

                    Ray ray(camera.position, Normalize(
                            camera.forward + camera.right * halfWidth * p.x + camera.up * halfHeight * p.y));

                    HitRecord hit;
                    if (RayTraceScene(&scene, ray, &hit)) {
                        Material &mat = scene.materials[hit.materialId];

                        Vector3f lightPos = Vector3f(-0.0f, -1.5f, 2.5f);
                        Vector3f surfaceToL = lightPos - hit.collisionPoint;
                        float distToLight = Length(surfaceToL);
                        Vector3f lightDir = Normalize(surfaceToL);

                        Ray shadowRay(hit.collisionPoint, lightDir, 0.001, distToLight);
                        if (!RayTraceScene(&scene, shadowRay, nullptr)) {
                            float intensity = 30.0f / (4.0f * M_PI * powf(distToLight, 2.0f));
                            float lambert = intensity * Max(Dot(hit.n, lightDir), 0.0f);
                            color += Saturate(mat.albedo * lambert);
                        }

                        color += mat.albedo * 0.12f;
                    }
                }
            }

            color /= MSAA_SAMPLES * MSAA_SAMPLES;

            pixel[0] = (unsigned char)floorf(Clamp(color.x, 0.0f, 1.0f) * 255.0f);
            pixel[1] = (unsigned char)floorf(Clamp(color.y, 0.0f, 1.0f) * 255.0f);
            pixel[2] = (unsigned char)floorf(Clamp(color.z, 0.0f, 1.0f) * 255.0f);
            pixel[3] = 255;//(unsigned char)floorf(clamp(color.a, 0.0f, 1.0f) * 255.0f);
        }
    }

    // Vertically flip the image on write
    stbi_flip_vertically_on_write(1);

    if (!stbi_write_png("image.png", WIDTH, HEIGHT, 4, buffer, WIDTH * 4)) {
        fprintf(stderr, "%s\n", "Failed to write image!");
    }

    free(buffer);

    return 0;
}