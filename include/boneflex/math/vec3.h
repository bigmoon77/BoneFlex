#pragma once

#include <cmath>

namespace boneflex
{
    struct vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        constexpr vec3() = default;
        constexpr vec3(float x_value, float y_value, float z_value)
            : x(x_value), y(y_value), z(z_value)
        {
        }

        constexpr vec3 operator+(const vec3& other) const
        {
            return vec3(x + other.x, y + other.y, z + other.z);
        }

        constexpr vec3 operator-(const vec3& other) const
        {
            return vec3(x - other.x, y - other.y, z - other.z);
        }

        constexpr vec3 operator*(float scalar) const
        {
            return vec3(x * scalar, y * scalar, z * scalar);
        }

        constexpr vec3 operator/(float scalar) const
        {
            return vec3(x / scalar, y / scalar, z / scalar);
        }

        constexpr vec3& operator+=(const vec3& other)
        {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        constexpr vec3& operator-=(const vec3& other)
        {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        constexpr vec3& operator*=(float scalar)
        {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        constexpr float length_squared() const
        {
            return x * x + y * y + z * z;
        }

        float length() const
        {
            return std::sqrt(length_squared());
        }

        constexpr float dot(const vec3& other) const
        {
            return x * other.x + y * other.y + z * other.z;
        }

        vec3 normalized() const
        {
            const float magnitude = length();
            if (magnitude <= 1.0e-8f)
            {
                return vec3();
            }
            return *this / magnitude;
        }
    };

    constexpr vec3 operator*(float scalar, const vec3& value)
    {
        return value * scalar;
    }
}
