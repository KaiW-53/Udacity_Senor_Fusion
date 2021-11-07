#ifndef _RANSAC_3D_H_
#define _RANSAC_3D_H_

#include <pcl/common/common.h>
#include <unordered_set>
#include <algorithm>

// Practice writing a templated class
template <typename PointT>
class RANSAC3D
{

public:
    static std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
    {
        std::unordered_set<int> inliersResult;
        srand(time(NULL));

        // TODO: Fill in this function

        // For max iterations
        
        while (maxIterations--)
        {
            std::unordered_set<int> inliers;
            while (inliers.size() < 3)
            {
                inliers.insert(std::rand() % cloud->points.size());
            }
            auto itr = inliers.begin();
            float x1, x2, x3, y1, y2, y3, z1, z2, z3;
            x1 = cloud->points[*itr].x;
            y1 = cloud->points[*itr].y;
            z1 = cloud->points[*itr].z;
            itr++;
            x2 = cloud->points[*itr].x;
            y2 = cloud->points[*itr].y;
            z2 = cloud->points[*itr].z;
            itr++;
            x3 = cloud->points[*itr].x;
            y3 = cloud->points[*itr].y;
            z3 = cloud->points[*itr].z;

            /*std::vector<float> l;
            l.push_back(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2)));
            l.push_back(sqrt((x1-x3)*(x1-x3)+(y1-y3)*(y1-y3)+(z1-z3)*(z1-z3)));
            l.push_back(sqrt((x3-x2)*(x3-x2)+(y3-y2)*(y3-y2)+(z3-z2)*(z3-z2)));

            std::sort(l.begin(), l.end());
            if (abs(l[2] - l[1] - l[0]) < 0.1) {
                maxIterations++;
                continue;
            }*/

            float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
            float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
            float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
            float d = -(a * x1 + b * y1 + c * z1);

            float denorm = std::sqrt(a * a + b * b + c * c);

            for (int idx = 0; idx < cloud->points.size(); idx++)
            {
                if (inliers.count(idx) > 0)
                    continue;
                float x = cloud->points[idx].x;
                float y = cloud->points[idx].y;
                float z = cloud->points[idx].z;
                float dist = std::abs(a * x + b * y + c * z + d) / denorm;
                if (dist <= distanceTol) {
                    inliers.insert(idx);
                }
            }

            if (inliers.size() > inliersResult.size())
            {
                inliersResult = inliers;
            }
        }

        // Randomly sample subset and fit line

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier

        // Return indicies of inliers from fitted line with most inliers

        return inliersResult;
    };
};
#endif