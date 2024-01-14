#include <iostream>
#include <pcl/filters/crop_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  //加载点云数据
    pcl::PCDReader reader;
    string input_path = argv[1];
    string output_path = argv[2];
    reader.read(input_path, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr bound(
        new pcl::PointCloud<pcl::PointXYZ>);       //创建多边形区域指针
    bound->push_back(pcl::PointXYZ(0.1, 0.1, 0));  //根据顶点坐标创建区域
    bound->push_back(pcl::PointXYZ(0.1, -0.1, 0));
    bound->push_back(pcl::PointXYZ(-0.1, 0.1, 0));
    bound->push_back(pcl::PointXYZ(-0.1, -0.1, 0));
    bound->push_back(pcl::PointXYZ(0.15, 0.1, 0));

    pcl::ConvexHull<pcl::PointXYZ> hull;  //建立一个凸包对象
    hull.setInputCloud(bound);            //输入设置好的凸包范围
    hull.setDimension(2);                 //设置凸包的维度
    std::vector<pcl::Vertices> polygons;  //设置动态数组用于保存凸包顶点
    //设置点云用于描述凸包的形状
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
    hull.reconstruct(*surface_hull, polygons);  //计算凸包结果

    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropHull<pcl::PointXYZ> crop;  //创建CropHull对象
    crop.setDim(2);                     //设置维度
    crop.setInputCloud(cloud);          //设置输入点云
    crop.setHullIndices(polygons);      //输入封闭多边形的顶点
    crop.setHullCloud(surface_hull);    //输入封闭多边形的形状
    crop.filter(*objects);              //执行CropHull滤波并存储相应的结果
    std::cout << "Cloud after crop has:" << objects->size() << " data points." << std::endl;
    //初始化共享指针用于进行可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(
        new pcl::visualization::PCLVisualizer("crophull display"));
    view->setBackgroundColor(255, 255, 255);

    //利用多视口输出可视化结果
    int v1(0);
    view->createViewPort(0.0, 0.0, 0.33, 1, v1);
    view->setBackgroundColor(255, 255, 255, v1);
    view->addPointCloud(cloud, "cloud", v1);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0,
                                           "cloud");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                           "cloud");
    view->addPolygon<pcl::PointXYZ>(surface_hull, 0, .069 * 255, 0.2 * 255,
                                    "backview_hull_polyLine1", v1);

    int v2(0);
    view->createViewPort(0.33, 0.0, 0.66, 1, v2);
    view->setBackgroundColor(255, 255, 255, v2);
    view->addPointCloud(surface_hull, "surface_hull", v2);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1,
                                           "surface_hull");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8,
                                           "surface_hull");
    view->addPolygon<pcl::PointXYZ>(surface_hull, 0, .069 * 255, 0.2 * 255,
                                    "backview_hull_polyLine1", v2);

    int v3(0);
    view->createViewPort(0.66, 0.0, 1, 1, v3);
    view->setBackgroundColor(255, 255, 255, v3);
    view->addPointCloud(objects, "objects", v3);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0,
                                           "objects");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                           "objects");

    while (!view->wasStopped()) {
        view->spinOnce(1000);
    }

    system("pause");
    return 0;
}
