#pragma once

#include <cstddef>
#include <string>

#include <boneflex/physics/reduced_mesh.h>
#include <boneflex/physics/shape_matching.h>

namespace boneflex
{
    struct shape_matching_cluster_options
    {
        // Number of control points gathered by graph traversal for each cluster.
        std::size_t cluster_size = 32;

        // Fraction of each cluster expected to overlap neighboring clusters.
        // Higher values produce more, smaller-spaced clusters.
        float overlap_ratio = 0.5f;
    };

    // Generates deterministic, overlapping geodesic neighborhoods from the
    // reduced surface topology. Every control point is included in at least
    // one cluster. Spatial farthest-point sampling selects cluster seeds.
    bool generate_shape_matching_clusters(
        const reduced_mesh& mesh,
        const shape_matching_cluster_options& options,
        shape_matching_clusters& output_clusters,
        std::string* error_message = nullptr
    );
}
