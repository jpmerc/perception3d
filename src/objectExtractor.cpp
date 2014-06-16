#include <objectExtractor.h>
typedef pcl::PointXYZRGB PointT;

ObjectExtractor::ObjectExtractor(bool showViewer){
    cloud.reset(new pcl::PointCloud<PointT>);
    pclViewer.reset(new pcl::visualization::PCLVisualizer ("3DViewer"));
    set_showUI(showViewer);
    l_count = 0;
    index_to_track = 0;
}

void ObjectExtractor::extraction_callback(const pcl::PCLPointCloud2ConstPtr& input){

    pcl::PointCloud<PointT>::Ptr objects(new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*input,*objects);
    cloud = objects;

    //Extract objects
    if(!object_vector.empty()){object_vector.clear();}
    object_vector = segment_objects(objects,0.02,200,15000);

//    if(showUI){
//        printToPCLViewer();
//    }

}

std::vector<pcl::PointCloud<PointT>::Ptr> ObjectExtractor::segment_objects(pcl::PointCloud<PointT>::Ptr cloud_input, double tolerance, int minClusterSize, int maxClusterSize){
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (tolerance); // 2cm
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_input);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_input);
    ec.extract (cluster_indices);

    // returns a vector of all the objects
    std::vector<pcl::PointCloud<PointT>::Ptr> object_vector_temp;
    for (int i=0; i<cluster_indices.size(); i++){
        pcl::PointIndices cloud_indices = cluster_indices.at(i);
        pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        cloud_cluster = extract_object_from_indices(cloud_input,cloud_indices);
        object_vector_temp.push_back(cloud_cluster);
    }

    return object_vector_temp;
}


pcl::PointCloud<PointT>::Ptr ObjectExtractor::extract_object_from_indices(pcl::PointCloud<PointT>::Ptr cloud_input,pcl::PointIndices object_indices){
    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (int j=0; j<object_indices.indices.size(); j++){
        cloud_cluster->points.push_back (cloud_input->points[object_indices.indices[j]]);
    }
    return cloud_cluster;
}

// Get showUI
bool* ObjectExtractor::get_showUI(){
    return &showUI;
}

// Set the value of showUI
void ObjectExtractor::set_showUI(bool show){
    showUI = show;
}




