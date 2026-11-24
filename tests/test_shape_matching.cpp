#include <boneflex/physics/shape_matching.h>

#include <cmath>
#include <iostream>
#include <vector>

int main()
{
    const std::vector<boneflex::vec3> rest{
        {-1.0f,0.0f,0.0f},{1.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,1.0f}
    };
    // A rigid 90-degree rotation around Z followed by translation.
    const std::vector<boneflex::vec3> current{
        {3.0f,-1.0f,2.0f},{3.0f,1.0f,2.0f},{2.0f,0.0f,2.0f},{3.0f,0.0f,3.0f}
    };
    boneflex::shape_matching_clusters clusters;
    if (!clusters.set_data(4,{0,4},{0,1,2,3})) return 1;
    std::vector<boneflex::vec3> output;
    if (!boneflex::solve_shape_matching(rest,current,clusters,1.0f,output)) return 2;
    for (std::size_t i=0;i<output.size();++i)
        if ((output[i]-current[i]).length()>1.0e-3f) return 3;

    // A cluster without a fixed point must preserve its own translation.
    // It must not be pulled toward an unrelated anchor in a neighboring cluster.
    const std::vector<boneflex::vec3> anchor_rest{
        {0,0,0},{1,0,0},{0,1,0},{1,1,0},{0,0,1},{1,0,1}
    };
    std::vector<boneflex::vec3> anchor_current = anchor_rest;
    for (std::size_t i = 1; i < anchor_current.size(); ++i)
        anchor_current[i].y -= 1.0f;
    boneflex::shape_matching_clusters anchored_clusters;
    if (!anchored_clusters.set_data(6, {0,1,6}, {0,1,2,3,4,5})) return 6;
    const std::vector<float> fixed_weights{1,0,0,0,0,0};
    std::vector<boneflex::vec3> anchored_output;
    if (!boneflex::solve_shape_matching(
        anchor_rest, anchor_current, anchored_clusters, fixed_weights, 1.0f, anchored_output)) return 7;
    for (std::size_t i = 1; i < anchor_rest.size(); ++i)
        if ((anchored_output[i] - anchor_current[i]).length() > 1.0e-3f) return 8;

    std::vector<boneflex::vec3> invalid_output{{9.0f,9.0f,9.0f}};
    if (boneflex::solve_shape_matching(rest,current,clusters,2.0f,invalid_output)) return 4;
    if (invalid_output.size()!=1 || std::abs(invalid_output[0].x-9.0f)>1.0e-5f) return 5;
    std::cout << "All shape matching tests passed.\n";
    return 0;
}
