#include <iostream>
#include <vector>
#include <algorithm>
#include <DirectXMath.h>
#include "thirteen.h"

#include "utils.h"

using namespace DirectX;

const XMVECTOR white = XMVectorSet(1, 1, 1, 0);
const XMVECTOR blue = XMVectorSet(0.5f, 0.7f, 1.0f, 0);

enum MaterialType
{
    Metal,
    Lambert,
    Dielectric
};

struct Ray
{
    XMVECTOR origin;
    XMVECTOR direction;
};

struct Material
{
    XMVECTOR albedo;
    MaterialType type;
    union
    {
        float refractiveIndex;
        float fuzz;
    };
};

struct HitRecord
{
    XMVECTOR point;
    XMVECTOR normal;
    Material mat;

    float t;
    bool front_face;

    void SetFaceNormal(const Ray &r, const XMVECTOR outward_normal)
    {
        front_face = XMVectorGetX(XMVector3Dot(r.direction, outward_normal)) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

struct SphereSoA
{
    std::vector<XMVECTOR> centers;
    std::vector<float> radii;
    std::vector<Material> materials;

    size_t Size() const { return centers.size(); }
    void Add(const XMVECTOR &center, float radius, const Material &mat)
    {
        centers.push_back(center);
        radii.push_back(radius);
        materials.push_back(mat);
    }
};

// bounding boxes
struct AABB
{
    XMVECTOR min;
    XMVECTOR max;

    bool Hit(const Ray &r, float tMin, float tMax) const
    {

        for (int axis = 0; axis < 3; axis++)
        {
            // P(t) = origin + t * direction -> t = (plane-origin)/direction
            float invD = 1.0f / XMVectorGetByIndex(r.direction, axis);

            // calculate distance where ray intersects 2 planes
            float t0 = (XMVectorGetByIndex(min, axis) - XMVectorGetByIndex(r.origin, axis)) * invD;
            float t1 = (XMVectorGetByIndex(max, axis) - XMVectorGetByIndex(r.origin, axis)) * invD;

            if (invD < 0.0f)
                std::swap(t0, t1);

            tMin = t0 > tMin ? t0 : tMin;
            tMax = t1 < tMax ? t1 : tMax;

            if (tMax <= tMin)
                return false;
        }
        // overlaps all 3 slabs
        return true;
    }
};

struct BVHNode
{
    AABB box;
    int left = -1;
    int right = -1;
    std::vector<size_t> objectIndices; // only for leaf nodes

    bool IsLeaf() const { return objectIndices.size() > 0; }
};

AABB SphereBoundingBox(const XMVECTOR &center, float radius)
{
    XMVECTOR radVec = XMVectorReplicate(radius);
    return {center - radVec, center + radVec};
}

int BuildBVHNode(std::vector<BVHNode> &nodes, const SphereSoA &spheres,
                 std::vector<size_t> &indices, int leafCount = 2)
{
    BVHNode node;

    // calculate bounding box
    XMVECTOR boxMin = XMVectorSet(FLT_MAX, FLT_MAX, FLT_MAX, 0);
    XMVECTOR boxMax = XMVectorSet(-FLT_MAX, -FLT_MAX, -FLT_MAX, 0);

    // expand node to fit all spheres
    for (size_t idx : indices)
    {
        AABB bbox = SphereBoundingBox(spheres.centers[idx], spheres.radii[idx]);
        boxMin = XMVectorMin(boxMin, bbox.min);
        boxMax = XMVectorMax(boxMax, bbox.max);
    }
    node.box = {boxMin, boxMax};

    int currentIdx = (int)nodes.size();
    nodes.push_back(node); // reserve parent node

    // register leafNode
    if (indices.size() <= leafCount) // leaf
    {
        nodes[currentIdx].objectIndices = indices;
        return currentIdx;
    }

    // split axis
    XMVECTOR extents = boxMax - boxMin;

    // pick largest axis
    float ex = XMVectorGetX(extents), ey = XMVectorGetY(extents), ez = XMVectorGetZ(extents);
    int axis = ex > ey ? (ex > ez ? 0 : 2) : (ey > ez ? 1 : 2);

    // sort speres in order on axis, and assign 1st/2nd halfs accordingly
    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b)
              { return XMVectorGetByIndex(spheres.centers[a], axis) < XMVectorGetByIndex(spheres.centers[b], axis); });
    size_t mid = indices.size() / 2;
    std::vector<size_t> leftIndices(indices.begin(), indices.begin() + mid);
    std::vector<size_t> rightIndices(indices.begin() + mid, indices.end());

    // recursive split
    nodes[currentIdx].left = BuildBVHNode(nodes, spheres, leftIndices);
    nodes[currentIdx].right = BuildBVHNode(nodes, spheres, rightIndices);

    return currentIdx;
}

bool HitSphereSoA(const SphereSoA &spheres, size_t i,
                  const Ray &ray, float tMin, float tMax, HitRecord &rec)
{
    XMVECTOR oc = ray.origin - spheres.centers[i];
    float half_b = XMVectorGetX(XMVector3Dot(oc, ray.direction));
    float c = XMVectorGetX(XMVector3Dot(oc, oc)) - spheres.radii[i] * spheres.radii[i];

    float discriminant = half_b * half_b - c;
    if (discriminant < 0)
        return false;

    float sqrtd = std::sqrtf(discriminant);
    float root = (-half_b - sqrtd);
    if (root < tMin || root > tMax)
    {
        root = -half_b + sqrtd;
        if (root < tMin || root > tMax)
            return false;
    }

    rec.t = root;
    rec.point = ray.origin + root * ray.direction;
    XMVECTOR outwardNormal = (rec.point - spheres.centers[i]) / spheres.radii[i];
    rec.SetFaceNormal(ray, outwardNormal);
    rec.mat = spheres.materials[i];
    return true;
}

bool HitSphereSoA(
    const SphereSoA &spheres, const Ray &ray,
    float tMin, float tMax, HitRecord &rec)
{
    bool anyHit = false;
    float closest = tMax;

    for (size_t i = 0; i < spheres.Size(); i++)
    {
        // a = 1 -> ray direction is normalized
        XMVECTOR oc = ray.origin - spheres.centers[i];
        float half_b = XMVectorGetX(XMVector3Dot(oc, ray.direction));
        float c = XMVectorGetX(XMVector3Dot(oc, oc)) - spheres.radii[i] * spheres.radii[i];

        float discriminant = half_b * half_b - c;
        if (discriminant < 0)
            continue;

        float sqrtd = std::sqrtf(discriminant);
        float root = (-half_b - sqrtd);

        if (root < tMin || root > closest)
        {
            root = -half_b + sqrtd;
            if (root < tMin || root > closest)
                continue;
        }

        closest = root;
        rec.t = root;
        rec.point = ray.origin + root * ray.direction;

        XMVECTOR outwardNormal =
            (rec.point - spheres.centers[i]) / spheres.radii[i];

        rec.SetFaceNormal(ray, outwardNormal);
        rec.mat = spheres.materials[i];
        anyHit = true;
    }
    return anyHit;
}

bool HitBVH(const SphereSoA &spheres, const std::vector<BVHNode> &nodes,
            int nodeIdx, const Ray &ray, float tMin, float tMax, HitRecord &rec)
{
    const BVHNode &node = nodes[nodeIdx];

    if (!node.box.Hit(ray, tMin, tMax))
        return false;

    bool hitAnything = false;
    float closest = tMax;

    if (node.IsLeaf()) // ~2 spheres per leaf
    {
        for (size_t i : node.objectIndices)
        {
            HitRecord tempRec;
            if (HitSphereSoA(spheres, i, ray, tMin, closest, tempRec))
            {
                hitAnything = true;
                closest = tempRec.t;
                rec = tempRec;
            }
        }
    }
    else
    {
        HitRecord leftRec, rightRec;

        // recursive tree traversal
        bool hitLeft = HitBVH(spheres, nodes, node.left, ray, tMin, closest, leftRec);
        if (hitLeft)
            closest = leftRec.t;

        bool hitRight = HitBVH(spheres, nodes, node.right, ray, tMin, closest, rightRec);

        if (hitLeft && hitRight)
            rec = (leftRec.t < rightRec.t) ? leftRec : rightRec;
        else if (hitLeft)
            rec = leftRec;
        else if (hitRight)
            rec = rightRec;

        hitAnything = hitLeft || hitRight;
    }

    return hitAnything;
}

bool Scatter(const Ray &rayIn, const Material &mat,
             const HitRecord &rec, XMVECTOR &attenuation, Ray &scattered)
{
    switch (mat.type)
    {
    case MaterialType::Metal:
        XMVECTOR reflected = XMVector3Reflect(rayIn.direction, rec.normal);
        reflected = XMVector3Normalize(reflected) + XMVectorScale(Utils::RandUnitVector(), mat.fuzz);
        scattered = Ray{rec.point, reflected};
        attenuation = mat.albedo;
        return XMVectorGetX(XMVector3Dot(scattered.direction, rec.normal)) > 0;
    case MaterialType::Lambert:
        XMVECTOR scatterDirection = rec.normal + Utils::RandUnitVector();

        // catch degenerate direction
        if (Utils::NearZero(scatterDirection))
            scatterDirection = rec.normal;

        scattered = Ray{rec.point, scatterDirection};
        attenuation = mat.albedo;
        return true;
    case MaterialType::Dielectric:
        attenuation = XMVectorSet(1, 1, 1, 0);
        float ri = rec.front_face ? 1.0f / mat.refractiveIndex : mat.refractiveIndex;
        XMVECTOR unitDirection = rayIn.direction;

        float cosTheta = XMVectorGetX(XMVector3Dot(-unitDirection, rec.normal));
        cosTheta = cosTheta > 1 ? 1 : cosTheta;

        float sinTheta = std::sqrtf(1.0f - cosTheta * cosTheta);
        bool cannotReflect = ri * sinTheta > 1;
        XMVECTOR direction;

        if (cannotReflect)
            direction = XMVector3Reflect(unitDirection, rec.normal);
        else
            direction = XMVector3Refract(unitDirection, rec.normal, ri);

        scattered = Ray{rec.point, direction};
        return true;
    }
    return false;
}

XMVECTOR RayColor(Ray ray, const SphereSoA &world, int depth)
{
    XMVECTOR accumulated = white;

    for (int bounce = 0; bounce < depth; bounce++)
    {
        HitRecord rec;
        if (!HitSphereSoA(world, ray, 0.00001f, FLT_MAX, rec))
        {
            float a = 0.5f * (XMVectorGetY(ray.direction) + 1.0f);
            accumulated *= (white * (1 - a) + blue * a);
            break;
        }

        Ray scattered;
        XMVECTOR attenuation;
        if (!Scatter(ray, rec.mat, rec, attenuation, scattered))
            return XMVectorZero();

        accumulated *= attenuation;
        ray = scattered;
    }

    return accumulated;
}

XMVECTOR RayColorBVH(Ray ray, const SphereSoA &world,
                     const std::vector<BVHNode> &bvhNodes,
                     int rootNode, int depth)
{
    XMVECTOR accumulated = white;

    HitRecord rec;
    for (int bounce = 0; bounce < depth; bounce++)
    {
        if (!HitBVH(world, bvhNodes, rootNode, ray, 0.001f, FLT_MAX, rec))
        {
            // basic sky gradient on no hit
            XMVECTOR dir = XMVector3Normalize(ray.direction);
            float a = 0.5f * (XMVectorGetY(dir) + 1.0f);
            accumulated *= (white * (1 - a) + blue * a);
            break;
        }

        // scatter according to material
        Ray scattered;
        XMVECTOR attenuation;
        if (!Scatter(ray, rec.mat, rec, attenuation, scattered))
            return XMVectorZero(); // absorb if scattering fails

        accumulated *= attenuation;
        ray = scattered;
    }

    return accumulated;
}

struct Camera
{
    // render vars
    float aspectRatio = 16.0f / 9.0f;
    int imageWidth = 400;
    int samplesPerPixel = 100;
    int maxBounces = 50;
    int imageHeight;
    float pixelSamples_scale;

    // render preview vars
    int previewScale = 16;
    int previewWidth, previewHeight;
    int previewBounces = 1;
    int previewSamplesPerPixel = 1;
    float previewSamples_scale;

    // image vars
    XMVECTOR center;
    XMVECTOR pixel00Loc;
    XMVECTOR pixelDeltaU;
    XMVECTOR pixelDeltaV;

    // view vars
    float vfov = 20;
    XMVECTOR lookFrom = XMVectorZero();
    XMVECTOR lookAt = XMVectorSet(0, 0, -1, 0);
    XMVECTOR vup = XMVectorSet(0, 1, 0, 0);

    // focus
    float defocusAngle = 0;
    float focusDist = 10;

    // fly cam
    float yaw = -90.0f; // looking toward -Z initially
    float pitch = 0.0f;
    float moveSpeed = 5.0f;
    float mouseSensitivity = 0.1f;

    XMVECTOR defocusDiskU;
    XMVECTOR defocusDiskV;

    XMVECTOR u, v, w;

    void Initialize()
    {
        // setup render texture
        imageHeight = int(imageWidth / aspectRatio);
        imageHeight = max(1, imageHeight);
        pixelSamples_scale = 1.0f / samplesPerPixel;

        previewWidth = imageWidth / previewScale;
        previewHeight = imageHeight / previewScale;
        previewSamples_scale = 1.0f / previewSamplesPerPixel;

        // viewport dimensions
        center = lookFrom;

        // float focalLength = XMVectorGetX(XMVector3Length(lookFrom - lookAt));
        float theta = XMConvertToRadians(vfov);
        float h = std::tanf(theta / 2.0f);
        float viewportHeight = 2.0f * h * focusDist;
        float viewportWidth = viewportHeight * float(imageWidth) / float(imageHeight);

        w = XMVector3Normalize(lookFrom - lookAt);
        u = XMVector3Normalize(XMVector3Cross(vup, w));
        v = XMVector3Cross(w, u);

        // calculate vectors across viewport horizontal & edges
        XMVECTOR viewportU = viewportWidth * u;
        XMVECTOR viewportV = viewportHeight * -v;

        // calculate per pixel delta vectors
        pixelDeltaU = viewportU / float(imageWidth);
        pixelDeltaV = viewportV / float(imageHeight);

        // calculate upperleft corner
        XMVECTOR viewport_upper_left =
            center - (focusDist * w) - viewportU / 2.0f - viewportV / 2.0f;
        pixel00Loc =
            viewport_upper_left + 0.5f * (pixelDeltaU + pixelDeltaV);

        float defocusRadius = focusDist * std::tanf(XMConvertToRadians(defocusAngle / 2.0f));
        defocusDiskU = u * defocusRadius;
        defocusDiskV = v * defocusRadius;
    }

    XMVECTOR DefocusDiskSample()
    {
        XMVECTOR p = Utils::RandUnitDisc();
        return center + (defocusDiskU * XMVectorGetX(p)) + (defocusDiskV * XMVectorGetY(p));
    }

    Ray GetRay(int i, int j)
    {
        float rx = Utils::Randf() - 0.5f;
        float ry = Utils::Randf() - 0.5f;

        XMVECTOR pixelSample =
            pixel00Loc + ((i + rx) * pixelDeltaU) + ((j + ry) * pixelDeltaV);

        auto rayOrigin = (defocusAngle <= 0) ? center : DefocusDiskSample();
        auto rayDirection = pixelSample - rayOrigin;

        return Ray{rayOrigin, XMVector3Normalize(rayDirection)};
    }

    void UpdateLookDirection()
    {
        float yawRad = XMConvertToRadians(yaw);
        float pitchRad = XMConvertToRadians(pitch);

        XMVECTOR dir = XMVectorSet(
            cosf(yawRad) * cosf(pitchRad),
            sinf(pitchRad),
            sinf(yawRad) * cosf(pitchRad),
            0.0f);

        lookAt = lookFrom + XMVector3Normalize(dir);
    }

    void RenderRegion(
        int xStart, int xEnd,
        int yStart, int yEnd,
        unsigned char *buffer,
        const SphereSoA &world,
        const std::vector<BVHNode> &bvhNodes,
        int rootNode,
        bool previewMode)
    {
        int bufWidth = previewMode ? previewWidth : imageWidth;
        int bufHeight = previewMode ? previewHeight : imageHeight;

        int spp = previewMode ? previewSamplesPerPixel : samplesPerPixel;
        int maxDepth = previewMode ? previewBounces : maxBounces;
        float scale = previewMode ? previewSamples_scale : pixelSamples_scale;

        for (int j = yStart; j < yEnd; j++)
        {
            for (int i = xStart; i < xEnd; i++)
            {
                XMVECTOR pixelColor = XMVectorZero();

                for (int s = 0; s < spp; s++)
                {
                    // Map preview pixels to full camera pixel space
                    float fx = previewMode
                                   ? (float)i / bufWidth * imageWidth
                                   : (float)i;

                    float fy = previewMode
                                   ? (float)j / bufHeight * imageHeight
                                   : (float)j;

                    Ray r = GetRay(int(fx), int(fy));

                    pixelColor += RayColorBVH(
                        r, world, bvhNodes, rootNode, maxDepth);
                }

                pixelColor = XMVectorScale(pixelColor, scale);

                XMVECTOR clamped = XMVectorClamp(
                    pixelColor,
                    XMVectorZero(),
                    XMVectorReplicate(1.0f));

                float r = Utils::LinearToGamma(XMVectorGetX(clamped));
                float g = Utils::LinearToGamma(XMVectorGetY(clamped));
                float b = Utils::LinearToGamma(XMVectorGetZ(clamped));

                int idx = (j * bufWidth + i) * 4;

                // scale to 255 & write to buffer
                buffer[idx + 0] = static_cast<unsigned char>(255.999f * r);
                buffer[idx + 1] = static_cast<unsigned char>(255.999f * g);
                buffer[idx + 2] = static_cast<unsigned char>(255.999f * b);
                buffer[idx + 3] = 255;
            }
        }
    }
};

SphereSoA GenWorld()
{
    // world
    SphereSoA world;

    // ground
    Material groundMat{XMVectorSet(0.5f, 0.5f, 0.5f, 0), MaterialType::Lambert};
    world.Add(XMVectorSet(0.0f, -1000.0f, 0.0f, 0), 1000.0f, groundMat);

    // random spheres
    for (int a = -11; a < 11; a++)
    {
        for (int b = -11; b < 11; b++)
        {
            float chooseMat = Utils::Randf();
            XMVECTOR center = XMVectorSet(
                a + 0.9f * Utils::Randf(),
                0.2f,
                b + 0.9f * Utils::Randf(),
                0);

            XMVECTOR diff = center - XMVectorSet(4, 0.2f, 0, 0);
            if (XMVectorGetX(XMVector3Length(diff)) > 0.9f)
            {
                Material sphereMat{};
                if (chooseMat < 0.8f)
                {
                    // diffuse
                    XMVECTOR albedo = Utils::RandVec3() * Utils::RandVec3();
                    sphereMat = {albedo, MaterialType::Lambert};
                }
                else if (chooseMat < 0.95f)
                {
                    // metal
                    XMVECTOR albedo = Utils::RandVec3(0.5f, 1.0f);
                    float fuzz = Utils::Randf(0.0f, 0.5f);
                    sphereMat = {albedo, MaterialType::Metal, fuzz};
                }
                else
                {
                    // glass
                    sphereMat = {white, MaterialType::Dielectric, 1.5f};
                }

                world.Add(center, 0.2f, sphereMat);
            }
        }
    }

    Material material1{white, MaterialType::Dielectric, 1.5f};
    Material material2{XMVectorSet(0.4f, 0.2f, 0.1f, 0), MaterialType::Lambert};
    Material material3{XMVectorSet(0.7f, 0.6f, 0.5f, 0), MaterialType::Metal, 0.0f};

    world.Add(XMVectorSet(0, 1, 0, 0), 1.0f, material1);
    world.Add(XMVectorSet(-4, 1, 0, 0), 1.0f, material2);
    world.Add(XMVectorSet(4, 1, 0, 0), 1.0f, material3);

    return world;
}

bool UpdateCamera(Camera &cam, bool &fullRender, int &line, float dt = 0.016f)
{
    static POINT lastMouse = {};
    static bool firstMouse = true;

    bool moved = false;

    // mouse delta
    POINT currentMouse;
    GetCursorPos(&currentMouse);
    ScreenToClient(GetActiveWindow(), &currentMouse);

    if (firstMouse)
    {
        lastMouse = currentMouse;
        firstMouse = false;
    }

    float deltaX = float(currentMouse.x - lastMouse.x);
    float deltaY = float(currentMouse.y - lastMouse.y);
    lastMouse = currentMouse;

    // mouse look
    if (Thirteen::GetMouseButton(1) || Thirteen::GetMouseButton(0))
    {
        cam.yaw += deltaX * cam.mouseSensitivity;
        cam.pitch -= deltaY * cam.mouseSensitivity;

        cam.pitch = std::clamp(cam.pitch, -89.0f, 89.0f);

        cam.UpdateLookDirection();
        moved = true;
    }

    // movement
    float velocity = cam.moveSpeed * dt;

    XMVECTOR forward = XMVector3Normalize(cam.lookAt - cam.lookFrom);
    XMVECTOR right = XMVector3Normalize(XMVector3Cross(forward, cam.vup));

    if (Thirteen::GetKey('W'))
    {
        cam.lookFrom += forward * velocity;
        moved = true;
    }
    if (Thirteen::GetKey('S'))
    {
        cam.lookFrom -= forward * velocity;
        moved = true;
    }
    if (Thirteen::GetKey('A'))
    {
        cam.lookFrom -= right * velocity;
        moved = true;
    }
    if (Thirteen::GetKey('D'))
    {
        cam.lookFrom += right * velocity;
        moved = true;
    }
    if (Thirteen::GetKey('E'))
    {
        cam.lookFrom += cam.vup * velocity;
        moved = true;
    }
    if (Thirteen::GetKey('Q'))
    {
        cam.lookFrom -= cam.vup * velocity;
        moved = true;
    }

    // rebuild camera if changed
    if (moved)
    {
        cam.UpdateLookDirection();
        cam.Initialize();

        fullRender = false;
        line = 0;
    }

    return moved;
}

int main()
{
    // Image
    float aspectRatio = 16.0f / 9.0f;

    SphereSoA world = GenWorld();

    Camera cam{};
    cam.aspectRatio = 16.0f / 9.0f;
    cam.imageWidth = 640;
    cam.samplesPerPixel = 20;
    cam.maxBounces = 6;

    cam.previewScale = 16;
    cam.previewSamplesPerPixel = 1;
    cam.previewBounces = 1;

    cam.vfov = 20;
    cam.lookFrom = XMVectorSet(13, 2, 3, 0);
    cam.lookAt = XMVectorZero();
    cam.vup = XMVectorSet(0, 1, 0, 0);

    cam.defocusAngle = 0.6f;
    cam.focusDist = 10.0f;

    cam.Initialize();

    // Initialize BVH
    std::vector<size_t> indices(world.Size());
    for (size_t i = 0; i < indices.size(); i++)
        indices[i] = i;

    std::vector<BVHNode> bvhNodes;
    int rootNode = BuildBVHNode(bvhNodes, world, indices);

    // Thirteen window
    unsigned char *pixels = Thirteen::Init(cam.imageWidth, cam.imageHeight);
    if (!pixels)
        return 1;

    unsigned int frameIndex = 0;

    // downscale preview bufer
    std::vector<unsigned char> previewBuffer(cam.previewWidth * cam.previewHeight * 4, 0);

    bool fullRender = false;
    bool completedOutput = false;
    bool outputPPM = true;
    int line = 0;
    float previewHeightRatio = (float)cam.previewHeight / (float)cam.imageHeight;
    float previewWidthRatio = (float)cam.previewWidth / (float)cam.imageWidth;

    // Go until window is closed or escape is pressed
    while (Thirteen::Render() && !Thirteen::GetKey(VK_ESCAPE))
    {
        UpdateCamera(cam, fullRender, line, (float)Thirteen::GetDeltaTime());

        if (!fullRender)
        {
            cam.RenderRegion(
                0, cam.previewWidth, 0, cam.previewHeight,
                previewBuffer.data(), world, bvhNodes, rootNode,
                true);

            // nearest neighbor upscaling
            for (int j = 0; j < cam.imageHeight; j++)
            {
                int srcY = int(j * previewHeightRatio);
                for (int i = 0; i < cam.imageWidth; i++)
                {
                    int srcX = int(i * previewWidthRatio);
                    int srcIdx = (srcY * cam.previewWidth + srcX) * 4;
                    int dstIdx = (j * cam.imageWidth + i) * 4;
                    pixels[dstIdx + 0] = previewBuffer[srcIdx + 0];
                    pixels[dstIdx + 1] = previewBuffer[srcIdx + 1];
                    pixels[dstIdx + 2] = previewBuffer[srcIdx + 2];
                    pixels[dstIdx + 3] = 255; // alpha always 255
                }
            }
            if (Thirteen::GetKey(VK_SPACE))
                fullRender = true;
        }
        else
        {
            if (Thirteen::GetKey(VK_TAB))
            {
                fullRender = false;
                line = 0;
            }

            if (line == cam.imageHeight)
            {
                if (!completedOutput && outputPPM)
                {
                    printf("P3\n%d %d\n255\n", cam.imageWidth, cam.imageHeight);
                    // output buffer
                    for (size_t i = 0; i < cam.imageWidth * cam.imageHeight * 4; i += 4)
                    {
                        std::cout << unsigned int(pixels[i]) << ' '
                                  << unsigned int(pixels[i + 1]) << ' '
                                  << unsigned int(pixels[i + 2]) << ' ';
                    }
                    std::cout << ' ';
                    std::clog << "\rDone!                                       " << std::flush;
                    completedOutput = true;
                }
            }
            else
            {
                // draw indicator scan line
                for (int offset = 1; offset <= 2; offset++)
                {
                    int scanY = line + offset;

                    if (scanY >= cam.imageHeight)
                        break;

                    for (int x = 0; x < cam.imageWidth; x++)
                    {
                        int idx = (scanY * cam.imageWidth + x) * 4;
                        pixels[idx + 0] = 0;
                        pixels[idx + 1] = 0;
                        pixels[idx + 2] = 0;
                        pixels[idx + 3] = 255;
                    }
                }

                cam.RenderRegion(0, cam.imageWidth,
                                 line, line + 1, pixels,
                                 world, bvhNodes, rootNode, false);
                line++;
            }
        }
        frameIndex++;
    }

    Thirteen::Shutdown();
    return 0;
}