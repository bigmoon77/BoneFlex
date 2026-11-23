#pragma once

#include <boneflex/math/quat.h>
#include <boneflex/math/vec3.h>

namespace boneflex
{
    struct transform
    {
        vec3 position;
        quat rotation;

        vec3 transform_point(const vec3& point) const
        {
            return rotation.rotate(point) + position;
        }

        vec3 inverse_transform_point(const vec3& point) const
        {
            return rotation.normalized().conjugated().rotate(point - position);
        }

        transform operator*(const transform& child) const
        {
            transform result;
            result.position = transform_point(child.position);
            result.rotation = (rotation * child.rotation).normalized();
            return result;
        }
    };
}
