#pragma once
#include <limits>
#include <random>
#include <DirectXMath.h>

namespace Utils
{
    constexpr float epsilon = 1e-60f;

    inline float Randf()
    {
        static std::uniform_real_distribution<float> distribution(0, 1);
        static std::mt19937 generator;
        return distribution(generator);
    }

    inline float Randf(float min, float max)
    {
        return min + (max - min) * Randf();
    }

    inline DirectX::XMVECTOR RandVec3()
    {
        return DirectX::XMVectorSet(Randf(-1, 1), Randf(-1, 1), Randf(-1, 1), 0);
    }

    inline DirectX::XMVECTOR RandVec3(float min, float max)
    {
        return DirectX::XMVectorSet(Randf(min, max), Randf(min, max), Randf(min, max), 0.0f);
    }

    inline DirectX::XMVECTOR RandUnitVector()
    {
        while (true)
        {
            auto v = RandVec3();
            if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(v)) > epsilon)
                return DirectX::XMVector3Normalize(v);
        }
    }

    inline DirectX::XMVECTOR RandOnHemisphere(const DirectX::XMVECTOR &normal)
    {
        auto v = RandUnitVector();
        if (DirectX::XMVectorGetX(DirectX::XMVector3Dot(v, normal)) > 0)
            return v;
        else
            return DirectX::XMVectorScale(v, -1);
    }

    inline DirectX::XMVECTOR RandUnitDisc()
    {
        while (true)
        {
            auto v = DirectX::XMVectorSet(Randf(-1, 1), Randf(-1, 1), 0, 0);
            if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(v)) < 1)
                return v;
        }
    }

    inline float LinearToGamma(float linearComponent)
    {
        if (linearComponent > 0)
            return std::sqrtf(linearComponent);
        return 0;
    }

    inline bool NearZero(const DirectX::XMVECTOR &vec)
    {
        return DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(vec)) < epsilon;
    }

    inline float Reflectance(float cosine, float refractiveIndex)
    {
        float r0 = (1.0f - refractiveIndex) / (1.0f + refractiveIndex);
        r0 = r0 * r0;
        return r0 + (1 - r0) * std::powf((1.0f - cosine), 5);
    }

}