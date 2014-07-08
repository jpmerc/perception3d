#include <planSegmentor.h>

PlanSegmentor::PlanSegmentor(ros::NodeHandle p_nh)
{
    m_showUI = true;

    m_segmented_cloud.reset(new PCPointT);
    m_cloud.reset(new PCPointT);
    m_objects_cloud.reset(new PCPointT);
    m_loaded_cloud.reset(new pcl::PCLPointCloud2);

    m_pclViewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));

    m_pub = p_nh.advertise<sensor_msgs::PointCloud2> ("/custom/planes", 1);
    m_pub2 = p_nh.advertise<sensor_msgs::PointCloud2> ("/custom/filtered_data", 1);
    m_pub3 = p_nh.advertise<sensor_msgs::PointCloud2> ("/custom/not_planes", 1);
}


void PlanSegmentor::cloud_callback(const pcl::PCLPointCloud2ConstPtr &p_input)
{
    // Create the filtering object: downsample the dataset using a leaf size of 0.5cm
    pcl::PCLPointCloud2Ptr cloud_filtered = voxelgrid_filter(p_input, 0.005f);

    // Passthrough filter
    cloud_filtered = passthrough_filter(cloud_filtered,0,2.5);

    // Transform pc2 to pc
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud_filtered,*cloud_filtered2);

    // To show the filtered data in the pointcloud viewer
    *m_cloud = *cloud_filtered2;

    // Plane segmentation
    m_segmented_cloud = plane_segmentation(cloud_filtered2,5);

    if(m_segmented_cloud->size() == 0) return;
    // std::cout << "PointCloud representing the planar components: " << segmented_cloud->width * segmented_cloud->height << " data points." << std::endl;

    cloud_filtered2 = radius_outlier_removal_filter(cloud_filtered2,0.05,30);



    //   ==============  Euclidean Object Clustering  ==============  //
    // http://pointclouds.org/documentation/tutorials/cluster_extraction.php

    //    // Here, the extracted planes are not included in cloud_filtered2 (extracted by plane_segmentation function)
    //    std::vector<pcl::PointIndices> cluster_indices;
    //    euclidean_object_segmentation(cloud_filtered2,cluster_indices);
    //    std::cout << "Number of found objects: " << cluster_indices.size() << std::endl;

    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>);
    //    for (int i=0; i<cluster_indices.size(); i++){
    //        pcl::PointIndices cloud_indices = cluster_indices.at(i);
    //        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    //        cloud_cluster = extract_object_from_indices(cloud_filtered2,cloud_indices);
    //        *object_clusters += *cloud_cluster;
    //    }
    //    objects_cloud.swap(object_clusters);

    //   ===========================================================  //

    // pc to pc2
    pcl::PCLPointCloud2 planes_pcl,not_planes_pcl;
    pcl::toPCLPointCloud2(*m_segmented_cloud,planes_pcl);
    pcl::toPCLPointCloud2(*cloud_filtered2,not_planes_pcl);

    //From PCL pointclouds to ROS pointclouds (sensor_msgs)
    sensor_msgs::PointCloud2 filtered, planes,not_planes;
    pcl_conversions::fromPCL(planes_pcl,planes);
    pcl_conversions::fromPCL(*cloud_filtered,filtered);
    pcl_conversions::fromPCL(not_planes_pcl,not_planes);

    pcl_conversions::fromPCL(p_input->header,planes.header);
    pcl_conversions::fromPCL(p_input->header,filtered.header);
    pcl_conversions::fromPCL(p_input->header,not_planes.header);

    // Publish on given topics

    m_pub.publish (planes);
    m_pub2.publish(filtered);
    m_pub3.publish (not_planes);

    //Update PCL Viewer
    if(m_showUI){
        printToPCLViewer();
    }
}

pcl::PCLPointCloud2Ptr PlanSegmentor::voxelgrid_filter(const pcl::PCLPointCloud2ConstPtr p_cloud,
                                                       float p_leaf_size)
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> vx_grid;
    vx_grid.setInputCloud (p_cloud);
    vx_grid.setLeafSize (p_leaf_size, p_leaf_size, p_leaf_size);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    vx_grid.filter (*cloud_filtered);
    return cloud_filtered;
}

pcl::PCLPointCloud2Ptr PlanSegmentor::passthrough_filter(pcl::PCLPointCloud2Ptr p_input,
                                                         double p_min_distance,
                                                         double p_max_distance)
{
    pcl::PassThrough<pcl::PCLPointCloud2> pt_filter;
    pt_filter.setFilterFieldName ("z");
    pt_filter.setFilterLimits (p_min_distance, p_max_distance);
    pt_filter.setKeepOrganized (false);
    pt_filter.setInputCloud (p_input);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    pt_filter.filter (*cloud_filtered);

    //added by JeanJean
    pt_filter.setInputCloud(cloud_filtered);
    pt_filter.setFilterFieldName("x");
    pt_filter.setFilterLimits(-1.0, 1.0);
    pcl::PCLPointCloud2::Ptr ptr_cloud_filtered_x(new pcl::PCLPointCloud2);
    pt_filter.filter(*ptr_cloud_filtered_x);

    pt_filter.setInputCloud(ptr_cloud_filtered_x);
    pt_filter.setFilterFieldName("y");
    pt_filter.setFilterLimits(-1.0, 1.0);
    pcl::PCLPointCloud2::Ptr ptr_cloud_filtered_y(new pcl::PCLPointCloud2);
    pt_filter.filter(*ptr_cloud_filtered_y);
    /////////////////////////////////////////////////

    return ptr_cloud_filtered_y;
}

PCPointT::Ptr PlanSegmentor::plane_segmentation(PCPointT::Ptr p_cloud,
                                                int p_maxNumberOfPlanesToExtract)
{
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Instances used in planes extraction
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_seg_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_planes(new pcl::PointCloud<pcl::PointXYZRGB>);

    int iterationNumber = 1;
    while(iterationNumber <= p_maxNumberOfPlanesToExtract){
        seg.setInputCloud (p_cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_pointcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            return empty_pointcloud; //Returns empty pointcloud
        }
        // If the other planes (other than the principal) correspond to less than 20% of the pointcloud, they are not taken
        double pointcloud_proportion = double(inliers->indices.size())/double(p_cloud->size());
        if(iterationNumber >= 2 && pointcloud_proportion < 0.2){
            break;
        }

        // Extract the inliers
        extract.setInputCloud (p_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*temp_seg_cloud);
        *segmented_planes += *temp_seg_cloud;

        extract.setNegative (true);
        extract.filter (*temp_seg_cloud);
        p_cloud.swap (temp_seg_cloud);

        iterationNumber++;
    }

    return segmented_planes;
}

PCPointT::Ptr PlanSegmentor::radius_outlier_removal_filter(PCPointT::Ptr p_input,
                                                           double p_radius,
                                                           int p_minNN)
{
    //std::cout << "#of points before : " << cloud_input->size() << std::endl;

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(p_input);
    outrem.setRadiusSearch(p_radius);
    outrem.setMinNeighborsInRadius (p_minNN);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    outrem.filter (*cloud_filtered);
    // std::cout << "#of points after : " << cloud_filtered->size() << std::endl;
    return cloud_filtered;
}

void PlanSegmentor::printToPCLViewer()
{
    m_pclViewer->removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_cloud);
    m_pclViewer->addPointCloud<pcl::PointXYZRGB>(m_cloud,rgb,"source cloud");
    m_pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red_color(m_segmented_cloud, 255, 0, 0);
    m_pclViewer->addPointCloud<pcl::PointXYZRGB>(m_segmented_cloud,red_color,"segmented cloud");
    m_pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segmented cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue_color(m_objects_cloud, 0, 0, 255);
    m_pclViewer->addPointCloud<pcl::PointXYZRGB>(m_objects_cloud,blue_color,"objects cloud");
    m_pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "objects cloud");
}


void PlanSegmentor::setPCLViewer()
{
    if(m_showUI){
        m_pclViewer->setBackgroundColor (0, 0, 0);
        m_pclViewer->initCameraParameters ();
        m_pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
        vtkSmartPointer<vtkRenderWindow> renderWindow = m_pclViewer->getRenderWindow();
        renderWindow->SetSize(800,450);
        renderWindow->Render();
    }
    else{ // Not very clean, but only solution I found to not show pclViewer
        vtkSmartPointer<vtkRenderWindow> renderWindow = m_pclViewer->getRenderWindow();
        renderWindow->SetSize(1,1);
        renderWindow->Render();
    }
}

int PlanSegmentor::loadFile(const std::string &p_string)
{
    unsigned int stringSize = p_string.size();
    std::string extention = p_string.substr(stringSize-3);
    int confirmation;
    if(extention == "pcd")
    {
        if(pcl::io::loadPCDFile(p_string, *m_loaded_cloud) == -1)
        {
            PCL_ERROR("Not such file");
            PCL_ERROR(p_string.c_str());
            confirmation = -1;
        }
        std::cout << "File loaded complete" << std::endl;
        confirmation =1;
    }
    if(extention == "ply")
    {
        if(pcl::io::loadPLYFile(p_string, *m_loaded_cloud) == -1)
        {
            PCL_ERROR("Not such file");
            PCL_ERROR(p_string.c_str());
            confirmation = -1;
        }
        std::cout << "File loaded complete" << std::endl;
        confirmation = 1;
    }
    else
    {
        PCL_ERROR("The file is not a pcd or a ply file");
        std::stringstream ss;
        ss << "The extention is : " << extention;
        std::string error = ss.str();
        PCL_ERROR(error.c_str());
        confirmation = -1;
    }

    return confirmation;
}

void PlanSegmentor::setShowUi(bool p_input)
{
    m_showUI = p_input;
}

bool PlanSegmentor::viewerStoped() const
{
    return m_pclViewer->wasStopped();
}

void PlanSegmentor::viewerSpinOnce() const
{
    m_pclViewer->spinOnce(100);
}

void PlanSegmentor::spinOnceTestFile()
{
    cloud_callback(m_loaded_cloud);
}

void PlanSegmentor::showPointCloud(pcl::PCLPointCloud2Ptr p_ptr)
{
    PCPointT::Ptr cloud (new PCPointT);
    pcl::fromPCLPointCloud2(*p_ptr, *cloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("simple cloud"));
    viewer->addPointCloud<PointT>(cloud);
    viewer->setBackgroundColor(0,0,0);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}
