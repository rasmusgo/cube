#include <algorithm>
#include <cstdio>
#include <iostream>
#include <set>
#include <sstream>
#include <sstream>
#include <tuple>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "AssignColors.hpp"

int idToCubie(int id)
{
    auto coord_to_cubie = [](int x, int y, int z)
    {
        return 1 + (x+1) + ((y+1) + (z+1) * 3) * 3;
    };

    int side = id / 9;
    int id_x = id % 3 - 1;
    int id_y = (id % 9) / 3 - 1;

    if (id_x == 0 && id_y == 0)
    {
        // The center pieces form one semantic cubie.
        return 0;
    }

    if (side == 0) // F
    {
        return coord_to_cubie(id_x, -id_y, 1);
    }
    if (side == 1) // R
    {
        return coord_to_cubie(1, -id_y, -id_x);
    }
    if (side == 2) // U
    {
        return coord_to_cubie(id_x, 1, id_y);
    }
    if (side == 3) // L
    {
        return coord_to_cubie(-1, id_y, -id_x);
    }
    if (side == 4) // B
    {
        return coord_to_cubie(id_x, id_y, -1);
    }
    if (side == 5) // D
    {
        return coord_to_cubie(-id_y, -1, -id_x);
    }
    printf("unsupported side in idToCubie: %d", side);
    assert(false);
    exit(-1);
}

struct PairCost
{
    PairCost(size_t a, size_t b, double cost)
        : a(a), b(b), cost(cost)
    {
    }

    bool operator < (const PairCost& other) const
    {
        return cost < other.cost;
    }

    size_t a;
    size_t b;
    double cost;
};

class GroupMerger
{
public:
    explicit GroupMerger(size_t capacity)
        : parents(capacity), num_groups(capacity)
    {
        for (size_t i = 0; i < capacity; ++i)
        {
            parents[i] = i;
        }
    }

    size_t findRoot(size_t node)
    {
        if (parents[node] == node)
        {
            // Is root node.
            return node;
        }
        else
        {
            // Perform lookup and update parent to root.
            size_t root = findRoot(parents[node]);
            parents[node] = root;
            return root;
        }
    }

    size_t mergeNodes(size_t node_a, size_t node_b)
    {
        size_t root_a = findRoot(node_a);
        size_t root_b = findRoot(node_b);

        if (root_a == root_b)
        {
            // Already same group.
            return root_a;
        }
        else if (root_a < root_b)
        {
            parents[root_b] = root_a;
            num_groups -= 1;
            return root_a;
        }
        else
        {
            parents[root_a] = root_b;
            num_groups -= 1;
            return root_b;
        }
    }

    size_t getNumGroups()
    {
        return num_groups;
    }

private:
    // Indices refers to other nodes belonging to the same group.
    // Referrals are always to a lower index or self.
    std::vector<size_t> parents;
    size_t num_groups;
};


/**
 * Assign sides to colors by grouping similar colors while respecting cubie constraints.
 *
 * @return vector of side indices.
 */
std::vector<size_t> assignColorsToSides(const std::vector<cv::Scalar>& colors)
{
    assert(colors.size() == 6*9);

    auto pair_cost = [](const cv::Scalar& color_a, const cv::Scalar& color_b)
    {
        return cv::norm(color_a, color_b);
    };

    std::vector<std::vector<size_t>> adjacency(colors.size());
    std::vector<PairCost> pair_costs;
    for (size_t a = 0; a < colors.size(); ++a)
    {
        for (size_t b = a+1; b < colors.size(); ++b)
        {
            if (idToCubie(a) == idToCubie(b))
            {
                adjacency[a].push_back(b);
                adjacency[b].push_back(a);
            }
            else
            {
                pair_costs.emplace_back(a, b, pair_cost(colors[a], colors[b]));
            }
        }
    }

    std::sort(pair_costs.begin(), pair_costs.end());

    // There will be 8 merges per color times 6 colors. 8*6*25 = 1200
    cv::Mat3b canvas(cv::Size(1200, 50), cv::Vec3b(0,0,0));
    size_t merge_count = 0;

    GroupMerger groups(colors.size());

    for (const auto& pair : pair_costs)
    {
        // This pair wants to merge the color groups of pair.a and pair.b
        size_t root_a = groups.findRoot(pair.a);
        size_t root_b = groups.findRoot(pair.b);
        if (root_a == root_b)
        {
            // Already merged.
            continue;
        }

        // We are not allowed to merge the groups if it would create an invalid pair somewhere.
        // The two groups must therefore not both be present on the same cubie anywhere.
        size_t count_a = 0;
        size_t count_b = 0;
        for (size_t i = 0; i < adjacency.size(); ++i)
        {
            size_t root_i = groups.findRoot(i);
            if (root_i == root_a)
            {
                ++count_a;
                for (size_t j : adjacency[i])
                {
                    size_t root_j = groups.findRoot(j);
                    if (root_j == root_b)
                    {
                        // Merging these nodes would cause a problem.
                        goto ILLEGAL_MERGE;
                    }
                }
            }
            if (root_i == root_b)
            {
                ++count_b;
            }
        }

        if (count_a + count_b > 9)
        {
            // Merging these nodes would cause a group with more than 9 labels.
            goto ILLEGAL_MERGE;
        }


        {
            cv::rectangle(canvas, cv::Rect(merge_count * 25, 0,  25, 25), colors[pair.a], cv::FILLED);
            cv::rectangle(canvas, cv::Rect(merge_count * 25, 25, 25, 25), colors[pair.b], cv::FILLED);
            std::stringstream ss1;
            std::stringstream ss2;
            ss1 << idToCubie(pair.a);
            ss2 << idToCubie(pair.b);
            cv::putText(canvas, ss1.str(), cv::Point(merge_count * 25, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
            cv::putText(canvas, ss2.str(), cv::Point(merge_count * 25, 43), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
            ++merge_count;
        }

        // Merge groups.
        groups.mergeNodes(root_a, root_b);

        if (groups.getNumGroups() == 6)
        {
            cv::imshow("merges", canvas);

            // We can now assign sides because we have exactly 6 groups.
            std::vector<size_t> center_groups(6);
            for (int center = 0; center < 6; ++center)
            {
                center_groups[center] = groups.findRoot(4 + center * 9);
            }
            std::vector<size_t> label_sides(colors.size());
            for (size_t i = 0; i < colors.size(); ++i)
            {
                size_t root_i = groups.findRoot(i);
                auto center_it = std::find(center_groups.begin(), center_groups.end(), root_i);
                label_sides[i] = std::distance(center_groups.begin(), center_it);
            }
            return label_sides;
        }

    ILLEGAL_MERGE:
        continue;
    }

    // Failed to assign colors.
    printf("Failed to assign colors to sides. Got stuck at %lu groups.", groups.getNumGroups());
    fflush(stdout);
    exit(-1);
}
