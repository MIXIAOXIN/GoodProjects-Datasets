//
// Created by mixiaoxin on 2018-11-23.
//

#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


typedef struct TXT_Point_XYZ
{
    double x;
    double y;
    double z;
}TOPOINT_XYZ;


int main(int argc, char* argv[] ) {
    std::cout << "Hello, World!" << std::endl;
    char* input_filename = argv[1];

    //读取txt文件
    int num_txt;
    std::vector<TXT_Point_XYZ> my_vTxtPoints;
    FILE *fp_txt = fopen(input_filename, "r");

    if (fp_txt)
    {
        float intensity;
        float r, g, b;
        while (!feof(fp_txt)){
            TXT_Point_XYZ txt_points;
            fscanf(fp_txt, "%lf %lf %lf %lf %lf %lf %lf", &txt_points.x, &txt_points.y, &txt_points.z,
                   &intensity, &r, &g, &b);
            my_vTxtPoints.push_back(txt_points);
        }
    }
    else{
        std::cout << "fail to load txt file"<< std::endl;
    }


    num_txt = my_vTxtPoints.size();
    //写入点云数据
    pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = num_txt;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width*cloud->height);
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = my_vTxtPoints[i].x;
        cloud->points[i].y = my_vTxtPoints[i].y;
        cloud->points[i].z = my_vTxtPoints[i].z;
    }

    pcl::io::savePCDFileASCII("test_txt_2_pcd_ASCLL.pcd", *cloud);
    pcl::io::savePCDFile("test_txt_2_pcd.pcd", *cloud);
    pcl::io::savePCDFileBinary("test_txt_2_pcd_binary.pcd", *cloud);

    std::cout << cloud->points.size() << std::endl;

    return 1;
}
