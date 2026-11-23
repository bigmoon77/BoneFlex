#include <boneflex/animation/skeleton.h>

#include <utility>

namespace boneflex
{
    bool skeleton::set_bones(std::vector<bone> bones, std::string* error_message)
    {
        for (std::size_t index = 0; index < bones.size(); ++index)
        {
            const std::int32_t parent_index = bones[index].parent_index;
            if (parent_index < bone::root_parent)
            {
                if (error_message != nullptr)
                {
                    *error_message = "Bone parent index is smaller than root_parent.";
                }
                return false;
            }

            if (parent_index >= 0 && static_cast<std::size_t>(parent_index) >= index)
            {
                if (error_message != nullptr)
                {
                    *error_message = "Bones must be ordered before their children.";
                }
                return false;
            }
        }

        _bones = std::move(bones);
        reset_pose();

        if (error_message != nullptr)
        {
            error_message->clear();
        }
        return true;
    }

    bool skeleton::set_local_pose(std::size_t bone_index, const transform& local_pose)
    {
        if (bone_index >= _local_poses.size())
        {
            return false;
        }

        _local_poses[bone_index] = local_pose;
        return true;
    }

    bool skeleton::set_local_poses(const std::vector<transform>& local_poses)
    {
        if (local_poses.size() != _bones.size())
        {
            return false;
        }

        _local_poses = local_poses;
        return true;
    }

    void skeleton::reset_pose()
    {
        _local_poses.resize(_bones.size());
        _global_poses.resize(_bones.size());

        for (std::size_t index = 0; index < _bones.size(); ++index)
        {
            _local_poses[index] = _bones[index].bind_local_pose;
        }

        update_global_pose();
    }

    void skeleton::update_global_pose()
    {
        if (_global_poses.size() != _bones.size())
        {
            _global_poses.resize(_bones.size());
        }

        for (std::size_t index = 0; index < _bones.size(); ++index)
        {
            const std::int32_t parent_index = _bones[index].parent_index;
            if (parent_index == bone::root_parent)
            {
                _global_poses[index] = _local_poses[index];
            }
            else
            {
                _global_poses[index] =
                    _global_poses[static_cast<std::size_t>(parent_index)] * _local_poses[index];
            }

            _global_poses[index].rotation = _global_poses[index].rotation.normalized();
        }
    }

    std::vector<transform> skeleton::build_skinning_palette() const
    {
        std::vector<transform> bind_global_poses(_bones.size());
        std::vector<transform> palette(_bones.size());

        for (std::size_t index = 0; index < _bones.size(); ++index)
        {
            const std::int32_t parent_index = _bones[index].parent_index;
            if (parent_index == bone::root_parent)
            {
                bind_global_poses[index] = _bones[index].bind_local_pose;
            }
            else
            {
                bind_global_poses[index] =
                    bind_global_poses[static_cast<std::size_t>(parent_index)]
                    * _bones[index].bind_local_pose;
            }

            bind_global_poses[index].rotation = bind_global_poses[index].rotation.normalized();
            palette[index] = _global_poses[index] * bind_global_poses[index].inverse();
            palette[index].rotation = palette[index].rotation.normalized();
        }

        return palette;
    }

    std::size_t skeleton::bone_count() const
    {
        return _bones.size();
    }

    const std::vector<bone>& skeleton::bones() const
    {
        return _bones;
    }

    const std::vector<transform>& skeleton::local_poses() const
    {
        return _local_poses;
    }

    const std::vector<transform>& skeleton::global_poses() const
    {
        return _global_poses;
    }
}
