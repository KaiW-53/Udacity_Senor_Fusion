#ifndef __EUCLIDEAN_CLUSTER_H__
#define __EUCLIDEAN_CLUSTER_H__
#include "kdtree.h"

class EU_CLUSTER
{
public:

    static std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol)
    {
        std::vector<std::vector<int>> clusters;
        // TODO: Fill out this function to return list of indices for each cluster
        std::unordered_set<int> visited;
        for (int i = 0; i < points.size(); i++)
        {
            if (visited.count(i) > 0)
                continue;
            std::vector<int> cluster;
            std::queue<int> q;
            q.push(i);
            visited.insert(i);
            cluster.push_back(i);
            while (!q.empty())
            {
                int idx = q.front();
                q.pop();
                std::vector<int> neighbors = tree->search(points[idx], distanceTol);
                for (int n : neighbors)
                {
                    if (visited.count(n) > 0)
                        continue;
                    visited.insert(n);
                    cluster.push_back(n);
                    q.push(n);
                }
            }
            clusters.push_back(cluster);
        }

        return clusters;
    };
};
#endif
