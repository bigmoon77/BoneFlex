#include <boneflex/animation/skeleton.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    bool nearly_equal(float left, float right, float epsilon = 1.0e-4f)
    {
        return std::abs(left - right) <= epsilon;
    }

    bool test_parent_child_global_pose()
    {
        boneflex::skeleton rig;

        std::vector<boneflex::bone> bones(2);
        bones[0].name = "root";
        bones[0].parent_index = boneflex::bone::root_parent;
        bones[1].name = "child";
        bones[1].parent_index = 0;
        bones[1].bind_local_pose.position = boneflex::vec3(0.0f, 1.0f, 0.0f);

        if (!rig.set_bones(std::move(bones)))
        {
            return false;
        }

        boneflex::transform root_pose;
        root_pose.position = boneflex::vec3(2.0f, 0.0f, 0.0f);
        root_pose.rotation = boneflex::quat::from_axis_angle(
            boneflex::vec3(0.0f, 0.0f, 1.0f),
            1.57079632679f
        );

        if (!rig.set_local_pose(0, root_pose))
        {
            return false;
        }

        rig.update_global_pose();

        const boneflex::vec3 child_position = rig.global_poses()[1].position;
        return nearly_equal(child_position.x, 1.0f)
            && nearly_equal(child_position.y, 0.0f)
            && nearly_equal(child_position.z, 0.0f);
    }

    bool test_rejects_invalid_parent_order()
    {
        boneflex::skeleton rig;
        std::vector<boneflex::bone> bones(2);
        bones[0].name = "child";
        bones[0].parent_index = 1;
        bones[1].name = "root";
        bones[1].parent_index = boneflex::bone::root_parent;

        std::string error_message;
        return !rig.set_bones(std::move(bones), &error_message)
            && !error_message.empty()
            && rig.bone_count() == 0;
    }

    bool test_pose_array_size_validation()
    {
        boneflex::skeleton rig;
        std::vector<boneflex::bone> bones(1);
        bones[0].name = "root";

        if (!rig.set_bones(std::move(bones)))
        {
            return false;
        }

        return !rig.set_local_poses(std::vector<boneflex::transform>(2))
            && !rig.set_local_pose(1, boneflex::transform());
    }
}

int main()
{
    if (!test_parent_child_global_pose())
    {
        std::cerr << "Parent-child global pose test failed.\n";
        return 1;
    }

    if (!test_rejects_invalid_parent_order())
    {
        std::cerr << "Invalid parent order test failed.\n";
        return 1;
    }

    if (!test_pose_array_size_validation())
    {
        std::cerr << "Pose array size validation test failed.\n";
        return 1;
    }

    std::cout << "All skeleton tests passed.\n";
    return 0;
}
