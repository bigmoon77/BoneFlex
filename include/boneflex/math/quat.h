#pragma once

#include <cmath>

#include <boneflex/math/vec3.h>

namespace boneflex
{
    struct quat
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 1.0f;

        constexpr quat() = default;

        constexpr quat(float x_value, float y_value, float z_value, float w_value)
            : x(x_value), y(y_value), z(z_value), w(w_value)
        {
        }

        constexpr quat operator*(const quat& other) const
        {
            return quat(
                w * other.x + x * other.w + y * other.z - z * other.y,
                w * other.y - x * other.z + y * other.w + z * other.x,
                w * other.z + x * other.y - y * other.x + z * other.w,
                w * other.w - x * other.x - y * other.y - z * other.z
            );
        }

        constexpr quat conjugated() const
        {
            return quat(-x, -y, -z, w);
        }

        constexpr float length_squared() const
        {
            return x * x + y * y + z * z + w * w;
        }

        quat normalized() const
        {
            const float magnitude = std::sqrt(length_squared());
            if (magnitude <= 1.0e-8f)
            {
                return quat();
            }

            return quat(x / magnitude, y / magnitude, z / magnitude, w / magnitude);
        }

        vec3 rotate(const vec3& value) const
        {
            const quat rotation = normalized();
            const quat vector(value.x, value.y, value.z, 0.0f);
            const quat result = rotation * vector * rotation.conjugated();
            return vec3(result.x, result.y, result.z);
        }

        static quat from_axis_angle(const vec3& axis, float radians)
        {
            const vec3 normalized_axis = axis.normalized();
            if (normalized_axis.length_squared() <= 1.0e-8f)
            {
                return quat();
            }

            const float half_angle = radians * 0.5f;
            const float sine = std::sin(half_angle);
            return quat(
                normalized_axis.x * sine,
                normalized_axis.y * sine,
                normalized_axis.z * sine,
                std::cos(half_angle)
            ).normalized();
        }
    };
}
