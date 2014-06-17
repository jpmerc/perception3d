#include <objectExtractor.h>
typedef pcl::PointXYZRGB PointT;

ObjectExtractor::ObjectExtractor(bool showViewer){
    cloud.reset(new pcl::PointCloud<PointT>);
    pclViewer.reset(new pcl::visualization::PCLVisualizer ("3DViewer"));
    pclViewer->registerKeyboardCallback(&ObjectExtractor::keyboard_callback,*this, (void*)&pclViewer);
    set_showUI(showViewer);
    l_count = 0;
    index_to_grasp = 0;
    object_to_grasp.reset(new pcl::PointCloud<PointT>);
    initialize_object_to_grasp = true;
    setPCLViewer();
}

void ObjectExtractor::extraction_callback(const pcl::PCLPointCloud2ConstPtr& input){

    pcl::PointCloud<PointT>::Ptr objects(new pcl::PointCloud<PointT>);
    pcl::fromPCLPointCloud2(*input,*objects);
    cloud = objects;

    //Extract objects
    if(!object_vector.empty()){object_vector.clear();}
    object_vector = segment_objects(objects,0.02,200,15000);


    // Initialize object to grasp
    if((!object_vector.empty()) && initialize_object_to_grasp){
        object_to_grasp = object_vector[index_to_grasp];
        initialize_object_to_grasp = false;
    }

    if(showUI){
        printToPCLViewer();
    }

}


void ObjectExtractor::printToPCLViewer(){
    pclViewer->removeAllPointClouds();

    for(int i=0; i < object_vector.size(); i++){
        pcl::PointCloud<PointT>::Ptr pc = object_vector[i];
        pcl::visualization::PointCloudColorHandlerRandom<PointT> randColor(pc);
        std::stringstream ss;
        ss << i;
        std::string ind = ss.str();
        std::string pc_name = "object_" + ind;
        pclViewer->addPointCloud<PointT>(pc,randColor,pc_name);
        pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, pc_name);
    }

//    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(object_to_grasp);
//    pclViewer->addPointCloud<PointT>(object_to_grasp,rgb,"object_to_grasp");
//    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object_to_grasp");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow_color(object_to_grasp, 255, 255, 102);
    pclViewer->addPointCloud<PointT>(object_to_grasp,yellow_color,"object_to_grasp");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object_to_grasp");
}

void ObjectExtractor::keyboard_callback(const pcl::visualization::KeyboardEvent &event, void* viewer_void){
    l_count = l_count + 1;
    if(l_count < 2){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym () == "p"){
            if(showUI){
                showUI=false;
            }
            else{
                showUI=true;
            }
        }
        else if(event.getKeySym () == "s"){
            if(index_to_grasp+1 < object_vector.size()){
                index_to_grasp++;
            }
            else{
                index_to_grasp = 0;
            }
            // object_to_track = addNormalsToPointCloud(object_vector[index_to_track]);
            object_to_grasp = object_vector[index_to_grasp];
        }
    }
    else{
        l_count = 0;
    }

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
bool ObjectExtractor::get_showUI(){
    return showUI;
}

// Set the value of showUI
void ObjectExtractor::set_showUI(bool show){
    showUI = show;
}

void ObjectExtractor::toggle_showUI(){
    set_showUI(!get_showUI());
    setPCLViewer();
}

void ObjectExtractor::setPCLViewer(){
    if(showUI){
        pclViewer->setBackgroundColor (0, 0, 0);
        pclViewer->initCameraParameters ();
        pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
        vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
        renderWindow->SetSize(800,450);
        renderWindow->Render();

    }
    else{ // Not very clean, but only solution I found to hide pclViewer
        vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
        renderWindow->SetSize(1,1);
        renderWindow->Render();
    }
}



