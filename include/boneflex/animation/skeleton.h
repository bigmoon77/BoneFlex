#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <boneflex/animation/bone.h>

namespace boneflex
{
    class skeleton
    {
    public:
        bool set_bones(std::vector<bone> bones, std::string* error_message = nullptr);

        bool set_local_pose(std::size_t bone_index, const transform& local_pose);
        bool set_local_poses(const std::vector<transform>& local_poses);

        void reset_pose();
        void update_global_pose();
        std::vector<transform> build_skinning_palette() const;

        std::size_t bone_count() const;
        const std::vector<bone>& bones() const;
        const std::vector<transform>& local_poses() const;
        const std::vector<transform>& global_poses() const;

    private:
        std::vector<bone> _bones;
        std::vector<transform> _local_poses;
        std::vector<transform> _global_poses;
    };
}
