#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/search/kdtree.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

typedef PointXYZ PointType;
typedef PointCloud<PointXYZ> Cloud;

void compute(Cloud &cloud_a, Cloud &cloud_b)
{
    // Estimate
    TicToc tt;
    tt.tic();
    // compare A to B
    pcl::search::KdTree<PointType> tree_b;
    tree_b.setInputCloud(cloud_b.makeShared());
    float max_dist_a = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud_a.points.size(); ++i)
    {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);
        tree_b.nearestKSearch(cloud_a.points[i], 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_a)
            max_dist_a = sqr_distances[0];
    }
    // compare B to A
    pcl::search::KdTree<PointType> tree_a;
    tree_a.setInputCloud(cloud_a.makeShared());
    float max_dist_b = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < cloud_b.points.size(); ++i)
    {
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);

        tree_a.nearestKSearch(cloud_b.points[i], 1, indices, sqr_distances);
        if (sqr_distances[0] > max_dist_b)
            max_dist_b = sqr_distances[0];
    }

    max_dist_a = std::sqrt(max_dist_a);
    max_dist_b = std::sqrt(max_dist_b);

    float dist = std::max(max_dist_a, max_dist_b);

    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms : ");
    print_info("A->B: ");
    print_value("%f", max_dist_a);
    print_info(", B->A: ");
    print_value("%f", max_dist_b);
    print_info(", Hausdorff Distance: ");
    print_value("%f", dist);
    print_info(" ]\n");
}

/* ---[ */
//豪斯多夫距离（Hausdorff Distance，HD）
int main(int argc, char **argv)
{
    Cloud::Ptr cloud_a(new Cloud);
    if (!loadPLYFile(argv[1], *cloud_a))
        assert(0);
    Cloud::Ptr cloud_b(new Cloud);
    if (!loadPLYFile(argv[2], *cloud_b))
        assert(0);
    // Compute the Hausdorff distance
    compute(*cloud_a, *cloud_b);
}
