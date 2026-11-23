#include <boneflex/physics/shape_matching.h>

#include <algorithm>
#include <cmath>
#include <utility>

#include <boneflex/math/quat.h>

namespace boneflex
{
    bool shape_matching_clusters::set_data(
        std::size_t control_point_count,
        std::vector<std::uint32_t> offsets,
        std::vector<std::uint32_t> control_point_indices,
        std::string* error_message
    )
    {
        const auto fail = [this, error_message](const char* message) -> bool
        {
            _control_point_count = 0;
            _offsets.clear();
            _control_point_indices.clear();
            if (error_message != nullptr) *error_message = message;
            return false;
        };
        if (control_point_count == 0 || offsets.size() < 2 || offsets.front() != 0
            || offsets.back() != control_point_indices.size())
            return fail("Invalid shape matching CSR dimensions.");
        for (std::size_t cluster = 0; cluster + 1 < offsets.size(); ++cluster)
        {
            if (offsets[cluster] >= offsets[cluster + 1] || offsets[cluster + 1] > control_point_indices.size())
                return fail("Every shape matching cluster must contain control points.");
        }
        for (std::uint32_t index : control_point_indices)
            if (index >= control_point_count) return fail("Shape matching control point index is out of range.");

        _control_point_count = control_point_count;
        _offsets = std::move(offsets);
        _control_point_indices = std::move(control_point_indices);
        if (error_message != nullptr) error_message->clear();
        return true;
    }

    std::size_t shape_matching_clusters::cluster_count() const { return _offsets.empty() ? 0 : _offsets.size() - 1; }
    std::size_t shape_matching_clusters::control_point_count() const { return _control_point_count; }
    const std::vector<std::uint32_t>& shape_matching_clusters::offsets() const { return _offsets; }
    const std::vector<std::uint32_t>& shape_matching_clusters::control_point_indices() const { return _control_point_indices; }

    namespace
    {
        quat fit_rotation(const std::vector<vec3>& rest, const std::vector<vec3>& current,
            const std::vector<std::uint32_t>& indices, std::uint32_t begin, std::uint32_t end,
            const vec3& rest_center, const vec3& current_center)
        {
            float s[3][3]{};
            for (std::uint32_t i = begin; i < end; ++i)
            {
                const vec3 a = rest[indices[i]] - rest_center;
                const vec3 b = current[indices[i]] - current_center;
                s[0][0] += a.x * b.x; s[0][1] += a.x * b.y; s[0][2] += a.x * b.z;
                s[1][0] += a.y * b.x; s[1][1] += a.y * b.y; s[1][2] += a.y * b.z;
                s[2][0] += a.z * b.x; s[2][1] += a.z * b.y; s[2][2] += a.z * b.z;
            }
            const float xx=s[0][0], xy=s[0][1], xz=s[0][2];
            const float yx=s[1][0], yy=s[1][1], yz=s[1][2];
            const float zx=s[2][0], zy=s[2][1], zz=s[2][2];
            const float n[4][4] = {
                {xx+yy+zz, yz-zy, zx-xz, xy-yx},
                {yz-zy, xx-yy-zz, xy+yx, zx+xz},
                {zx-xz, xy+yx, -xx+yy-zz, yz+zy},
                {xy-yx, zx+xz, yz+zy, -xx-yy+zz}
            };
            float a[4][4]{};
            float v[4][4]{};
            for (int r=0;r<4;++r) for (int c=0;c<4;++c) { a[r][c]=n[r][c]; v[r][c]=(r==c)?1.0f:0.0f; }
            for (int sweep=0;sweep<32;++sweep)
            {
                int p=0,q=1;
                float largest=std::abs(a[0][1]);
                for (int r=0;r<4;++r) for (int c=r+1;c<4;++c)
                    if (std::abs(a[r][c])>largest) { largest=std::abs(a[r][c]); p=r; q=c; }
                if (largest<1.0e-7f) break;
                const float angle=0.5f*std::atan2(2.0f*a[p][q],a[q][q]-a[p][p]);
                const float cs=std::cos(angle), sn=std::sin(angle);
                for (int k=0;k<4;++k) if (k!=p && k!=q)
                {
                    const float ap=a[k][p], aq=a[k][q];
                    a[k][p]=a[p][k]=cs*ap-sn*aq;
                    a[k][q]=a[q][k]=sn*ap+cs*aq;
                }
                const float app=a[p][p], aqq=a[q][q], apq=a[p][q];
                a[p][p]=cs*cs*app-2.0f*sn*cs*apq+sn*sn*aqq;
                a[q][q]=sn*sn*app+2.0f*sn*cs*apq+cs*cs*aqq;
                a[p][q]=a[q][p]=0.0f;
                for (int k=0;k<4;++k)
                {
                    const float vp=v[k][p], vq=v[k][q];
                    v[k][p]=cs*vp-sn*vq; v[k][q]=sn*vp+cs*vq;
                }
            }
            int largest_index=0;
            for (int i=1;i<4;++i) if (a[i][i]>a[largest_index][largest_index]) largest_index=i;
            return quat(v[1][largest_index],v[2][largest_index],v[3][largest_index],v[0][largest_index]).normalized();
        }
    }

    bool solve_shape_matching(const std::vector<vec3>& rest_positions,
        const std::vector<vec3>& current_positions, const shape_matching_clusters& clusters,
        float stiffness, std::vector<vec3>& output_positions)
    {
        return solve_shape_matching(
            rest_positions, current_positions, clusters, std::vector<float>(), stiffness, output_positions);
    }

    bool solve_shape_matching(const std::vector<vec3>& rest_positions,
        const std::vector<vec3>& current_positions, const shape_matching_clusters& clusters,
        const std::vector<float>& fixed_point_weights,
        float stiffness, std::vector<vec3>& output_positions)
    {
        if (rest_positions.empty() || rest_positions.size()!=current_positions.size()
            || rest_positions.size()!=clusters.control_point_count()
            || clusters.cluster_count()==0 || !std::isfinite(stiffness) || stiffness<0.0f || stiffness>1.0f
            || (!fixed_point_weights.empty() && fixed_point_weights.size()!=rest_positions.size()))
            return false;
        for (float weight : fixed_point_weights)
            if (!std::isfinite(weight) || weight < 0.0f) return false;
        for (std::size_t i=0;i<rest_positions.size();++i)
            if (!std::isfinite(rest_positions[i].x)||!std::isfinite(rest_positions[i].y)||!std::isfinite(rest_positions[i].z)
                ||!std::isfinite(current_positions[i].x)||!std::isfinite(current_positions[i].y)||!std::isfinite(current_positions[i].z))
                return false;

        std::vector<vec3> accumulated(rest_positions.size());
        std::vector<float> counts(rest_positions.size(),0.0f);
        const std::vector<std::uint32_t>& offsets=clusters.offsets();
        const std::vector<std::uint32_t>& indices=clusters.control_point_indices();
        for (std::size_t c=0;c<clusters.cluster_count();++c)
        {
            const std::uint32_t begin=offsets[c], end=offsets[c+1];
            vec3 rest_center, current_center;
            for (std::uint32_t i=begin;i<end;++i) { rest_center+=rest_positions[indices[i]]; current_center+=current_positions[indices[i]]; }
            const float inverse_count=1.0f/static_cast<float>(end-begin);
            rest_center*=inverse_count; current_center*=inverse_count;
            const quat rotation=fit_rotation(rest_positions,current_positions,indices,begin,end,rest_center,current_center);

            vec3 translation_center = current_center;
            if (!fixed_point_weights.empty())
            {
                vec3 fixed_rest_center;
                vec3 fixed_current_center;
                float fixed_weight_sum = 0.0f;
                for (std::uint32_t i=begin;i<end;++i)
                {
                    const std::uint32_t index=indices[i];
                    const float weight=fixed_point_weights[index];
                    if (weight <= 0.0f) continue;
                    fixed_rest_center+=rest_positions[index]*weight;
                    fixed_current_center+=current_positions[index]*weight;
                    fixed_weight_sum+=weight;
                }
                if (fixed_weight_sum > 0.0f)
                {
                    fixed_rest_center*=1.0f/fixed_weight_sum;
                    fixed_current_center*=1.0f/fixed_weight_sum;
                    translation_center=fixed_current_center
                        - rotation.rotate(fixed_rest_center-rest_center)
                        + rotation.rotate(rest_center);
                }
            }
            for (std::uint32_t i=begin;i<end;++i)
            {
                const std::uint32_t index=indices[i];
                accumulated[index]+=translation_center+rotation.rotate(rest_positions[index]-rest_center);
                counts[index]+=1.0f;
            }
        }
        std::vector<vec3> result=current_positions;
        for (std::size_t i=0;i<result.size();++i)
            if (counts[i]>0.0f) result[i]=current_positions[i]*(1.0f-stiffness)
                +(accumulated[i]/counts[i])*stiffness;
        output_positions=std::move(result);
        return true;
    }
}
