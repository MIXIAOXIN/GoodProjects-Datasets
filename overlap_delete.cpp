float CSuperVoxel2D::getMinimumnLengthFromPointToEntityXY(const CurbLINE &entity, const Eigen::Vector3f point) {
    pointCloudXYPtr cloud(new pointCloudXY);
    pcl::copyPointCloud(*cloud_ptr_, entity.pts_id, *cloud);
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXY>);
    tree->setInputCloud(cloud);

    pcl::PointXY searchP;
    searchP.x = point[0], searchP.y = point[1];


    std::vector<int> ids(1);
    std::vector<float> diss(1);
    float re_d = 0.0f;
    if(tree->nearestKSearch(searchP, 1, ids, diss)){
        re_d = std::sqrt(diss[0]);
    }

    std::vector<int>().swap(ids);
    std::vector<float>().swap(diss);
    ids.shrink_to_fit();
    diss.shrink_to_fit();
    tree.reset();
    cloud.reset();

    return re_d;
}
