#include <ros/ros.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <vtkRenderWindow.h>
#include <pcl/common/transforms.h>
#include <stdlib.h>
#include <sstream>
#include <stdio.h>
#include <time.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/common/time.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;
using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//boost::shared_ptr<pcl::visualization::PCLVisualizer> pclViewer2 (new pcl::visualization::PCLVisualizer ("Tests"));


pcl::PointCloud<PointT>::Ptr in1(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr in2(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr in2_transformed(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);

pcl::PointCloud<PointT>::Ptr container_model(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr icp_aligned_cloud(new pcl::PointCloud<PointT>);

int l_count = 0;
bool sData = true;
bool tData = true;
bool mData = false;
bool rData = false;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

time_t timer_beginning;
time_t timer_end;



void initPCLViewer(){
    //PCL Viewer
    pclViewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&pclViewer);
    pclViewer->setBackgroundColor (0, 0, 0);
    pclViewer->initCameraParameters ();
    pclViewer->setCameraPosition(0,0,0,0,0,1,0,-1,0);
    vtkSmartPointer<vtkRenderWindow> renderWindow = pclViewer->getRenderWindow();
    renderWindow->SetSize(800,450);
    renderWindow->Render();

//    //PCL Viewer 2 (for testing)
//    pclViewer2->registerKeyboardCallback (keyboardEventOccurred, (void*)&pclViewer2);
//    pclViewer2->setBackgroundColor (0, 0, 0);
//    pclViewer2->initCameraParameters ();
//    pclViewer2->setCameraPosition(0,0,0,0,0,1,0,-1,0);
//    vtkSmartPointer<vtkRenderWindow> renderWindow2 = pclViewer2->getRenderWindow();
//    renderWindow2->SetSize(800,450);
//    renderWindow2->Render();
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    l_count = l_count + 1;
    if(l_count < 2){
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym () == "1" && tData){
            viewer->removePointCloud("in1");
            tData = false;
        }
        else if (event.getKeySym () == "1" && !tData){
            //viewer->removePointCloud("in1");
            pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow(in1, 255, 255, 0);
            viewer->addPointCloud<PointT>(in1, yellow ,"in1");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "in1");
            tData = true;
        }

        else if(event.getKeySym () == "2" && sData){
            viewer->removePointCloud("in2");
            sData = false;
        }
        else if(event.getKeySym () == "2" && !sData){
            //viewer->removePointCloud("in2");
            pclViewer->addPointCloud (in2_transformed, ColorHandlerT(in2_transformed, 255.0, 0.0, 0.0), "in2");
            pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "in2");
            sData = true;
        }
        else if(event.getKeySym () == "3" && mData){
            viewer->removePointCloud("merged");
            mData = false;
        }
        else if(event.getKeySym () == "3" && !mData){
            // viewer->removePointCloud("merged");
            pcl::visualization::PointCloudColorHandlerCustom<PointT> green(out, 0.0, 255.0, 0.0);
            pclViewer->addPointCloud (out, green, "merged");
            pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "merged");
            mData = true;
        }
        else if(event.getKeySym () == "4" && rData){
            viewer->removePointCloud("rgb");
            rData = false;
        }
        else if(event.getKeySym () == "4" && !rData){
            //viewer->removePointCloud("rgb");
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(out);
            pclViewer->addPointCloud<PointT>(out, rgb, "rgb");
            pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "rgb");
            rData = true;
        }

    }
    else{
        l_count = 0;
    }

}


Eigen::Matrix4f computeMatrixFromTransform(double x, double y, double z, double rx, double ry, double rz, double rw){

    Eigen::Vector3f vec(x, y, z);
    Eigen::Quaternionf quat(rw, rx, ry, rz);
    Eigen::Matrix3f rot = quat.toRotationMatrix();

    Eigen::Matrix4f mat;
    mat.setZero();
    mat.block(0, 0, 3, 3) = rot;
    mat.block(0, 3, 3, 1) = vec;
    mat(3,3) = 1;

    return mat;
}

pcl::PointCloud<PointT>::Ptr computeUniformSampling(pcl::PointCloud<PointT>::Ptr p_cloudIn, double radius)
{


    pcl::ScopeTime t("Uniform Sampling");

    std::cout << "US computation begin" << std::endl;

    pcl::UniformSampling<PointT> uniformSampling;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    boost::shared_ptr<std::vector<int> > point_cloud_indice (new std::vector<int> ());
    pcl::PointCloud<int> point_cloud_out;
    uniformSampling.setInputCloud(p_cloudIn);
    uniformSampling.setSearchMethod(tree);
    uniformSampling.setRadiusSearch(radius);
    uniformSampling.compute(point_cloud_out);

    for(int i = 0; i < point_cloud_out.size(); i++)
    {
        point_cloud_indice->push_back(point_cloud_out.at(i));
    }


    pcl::PointCloud<PointT>::Ptr sampled_cloud (new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(p_cloudIn);
    extract.setIndices(point_cloud_indice);
    extract.setNegative(false);
    extract.filter(*sampled_cloud);


    std::cout << "Pointcloud input size = " << p_cloudIn->size() << std::endl;
    std::cout << "Point cloud out size = " << point_cloud_out.size() << std::endl;
    std::cout << "Keypoints Size = " << sampled_cloud->size() << std::endl;

    return sampled_cloud;
}


pcl::PointCloud<PointT>::Ptr computeSurfaceNormals(pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::ScopeTime t("Normals");

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    // Estimate the normals.
    pcl::NormalEstimation<PointT, pcl::Normal> normalEstimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);

    normalEstimation.setInputCloud(cloud);
    normalEstimation.setKSearch(10);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    pcl::PointCloud<PointT>::Ptr merged_cloud (new pcl::PointCloud<PointT>);
    pcl::concatenateFields(*cloud, *normals, *merged_cloud);


    return merged_cloud;

}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH (pcl::PointCloud<PointT>::Ptr cloud){

    pcl::ScopeTime t("FPFH");


    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::search::KdTree<PointT>::Ptr tree;
    double feature_radius = 0.02;

    pcl::FPFHEstimation<PointT, PointT, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud (cloud);
    fpfh_est.setInputNormals (cloud);
    fpfh_est.setSearchMethod (tree);
    //fpfh_est.setKSearch(10);
    fpfh_est.setRadiusSearch (0.025);
    fpfh_est.compute (*features);

    return features;
}

Eigen::Matrix4f coarseAlignment(pcl::PointCloud<PointT>::Ptr src_cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features, pcl::PointCloud<PointT>::Ptr target_cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features){

    pcl::ScopeTime t("SAC-IA");

    pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
    float min_sample_distance = 0.1;
    float max_correspondence_distance = 1;
    int nr_iterations = 500;


    sac_ia.setMinSampleDistance (min_sample_distance);
    sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
    sac_ia.setMaximumIterations (nr_iterations);

    sac_ia.setInputTarget (target_cloud);
    sac_ia.setTargetFeatures (target_features);

    sac_ia.setInputSource(src_cloud);
    sac_ia.setSourceFeatures (src_features);

    pcl::PointCloud<PointT>::Ptr registration_output(new pcl::PointCloud<PointT>);
    sac_ia.align (*registration_output);

    double fitness_score = (float) sac_ia.getFitnessScore(max_correspondence_distance);
    cout << "SAC-IA Transformation Score = " << fitness_score << endl;
    Eigen::Matrix4f transformation = sac_ia.getFinalTransformation ();

    return transformation;
}

Eigen::Matrix4f ransac_prerejective(pcl::PointCloud<PointT>::Ptr src_cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features, pcl::PointCloud<PointT>::Ptr target_cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features){

    pcl::ScopeTime t("RANSAC Prerejective");

    // Object for pose estimation.
    pcl::SampleConsensusPrerejective<PointT, PointT, pcl::FPFHSignature33> pose;
    pose.setInputSource(src_cloud);
    pose.setInputTarget(target_cloud);
    pose.setSourceFeatures(src_features);
    pose.setTargetFeatures(target_features);

    // Instead of matching a descriptor with its nearest neighbor, choose randomly between
    // the N closest ones, making it more robust to outliers, but increasing time.
    pose.setCorrespondenceRandomness(10);

    // Set the fraction (0-1) of inlier points required for accepting a transformation.
    // At least this number of points will need to be aligned to accept a pose.
    pose.setInlierFraction(0.95f);

    // Set the number of samples to use during each iteration (minimum for 6 DoF is 3).
    pose.setNumberOfSamples(3);

    // Set the similarity threshold (0-1) between edge lengths of the polygons. The
    // closer to 1, the more strict the rejector will be, probably discarding acceptable poses.
    pose.setSimilarityThreshold(0.75f);

    // Set the maximum distance threshold between two correspondent points in source and target.
    // If the distance is larger, the points will be ignored in the alignment process.
    pose.setMaxCorrespondenceDistance(0.04f);

    pose.setMaximumIterations(100);

    pcl::PointCloud<PointT>::Ptr alignedModel(new pcl::PointCloud<PointT>);
    pose.align(*alignedModel);
    Eigen::Matrix4f transformation = pose.getFinalTransformation();


    std::cout << "Converged : " << pose.hasConverged() << std::endl;
    std::cout << "Transformation Matrix : " << std::endl << transformation << std::endl;
    pcl::console::print_info ("Inliers: %i/%i\n", pose.getInliers().size (), src_cloud->size ());

    return transformation;

}

Eigen::Matrix4f align_icp(pcl::PointCloud<PointT>::Ptr src_cloud, pcl::PointCloud<PointT>::Ptr target_cloud, Eigen::Matrix4f initial_transform, double maxCorrespondanceDistance){

    pcl::ScopeTime t("ICP");


    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(src_cloud);
    icp.setInputTarget(target_cloud);
    icp.setMaxCorrespondenceDistance(maxCorrespondanceDistance);
    icp.setMaximumIterations(40);
    pcl::PointCloud<PointT>::Ptr Final(new pcl::PointCloud<PointT>());
    icp.align(*Final,initial_transform);
    Eigen::Matrix4f icp_transform = icp.getFinalTransformation();

    return icp_transform;
}

void filterScenePointsFromAlignedModel(pcl::PointCloud<PointT>::Ptr aligned_model, pcl::PointCloud<PointT>::Ptr scene){

    pcl::PointCloud<PointT>::Ptr convexHull(new pcl::PointCloud<PointT>);
    pcl::ConvexHull<PointT> hull;
    std::vector<pcl::Vertices> polygons;
    hull.setInputCloud(aligned_model);
    hull.reconstruct(*convexHull, polygons);

    pcl::PointCloud<PointT>::Ptr objects (new pcl::PointCloud<PointT>);
    pcl::CropHull<PointT> bb_filter;

    bb_filter.setDim(3);
    bb_filter.setInputCloud(scene);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(convexHull);
    bb_filter.filter(*objects);
}

pcl::PointCloud<PointT>::Ptr downsample(pcl::PointCloud<PointT>::Ptr cloud){
    pcl::PointCloud<PointT>::Ptr returnCloud(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> grid;
    const float leaf = 0.008f;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (cloud);
    grid.filter (*returnCloud);

    return returnCloud;
}

int main (int argc, char** argv){

    // Initialize ROS
    ros::init (argc, argv, "multipleKinects");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");



    pcl::io::loadPCDFile("/home/jp/devel/src/perception3d/src/tests/device1_1.pcd", *in1);
    pcl::io::loadPCDFile("/home/jp/devel/src/perception3d/src/tests/device2_1.pcd", *in2);
    pcl::io::loadPCDFile("/home/jp/devel/src/perception3d/src/tests/container_model.pcd", *container_model);

    initPCLViewer();

    // Find the coarse transformation between the two scans of the scene
    Eigen::Matrix4f tf1_matrix = computeMatrixFromTransform(-0.012687, 0.2937498, 1.0124953, -0.36378023, -0.32528895, -0.56674915, 0.663812041);
    Eigen::Matrix4f tf2_matrix = computeMatrixFromTransform(-0.1223497, 0.28168088, 1.1013584, -0.5120674, -0.0235908, -0.04915027, 0.85721325);
    Eigen::Matrix4f coord_transform = tf1_matrix * tf2_matrix.inverse();

    // Uniform Sampling
    pcl::PointCloud<PointT>::Ptr in_sampled1(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr in_sampled2(new pcl::PointCloud<PointT>);
    in_sampled1 = computeUniformSampling(in1, 0.02);
    in_sampled2 = computeUniformSampling(in2, 0.02);
//    container_model = computeUniformSampling(container_model, 0.01);

    // Calculate Normals for every pointcloud
//    in1 = computeSurfaceNormals(in1);
//    in2 = computeSurfaceNormals(in2);
//    container_model = computeSurfaceNormals(container_model);


//    // downsample
    in1 = downsample(in1);
    in2 = downsample(in2);

    //    // Calculate FPFH Features for every pointcloud
    //    pcl::PointCloud<pcl::FPFHSignature33>::Ptr in1_fpfh = computeFPFH(in1);
    //    pcl::PointCloud<pcl::FPFHSignature33>::Ptr in2_fpfh = computeFPFH(in2);
    //    pcl::PointCloud<pcl::FPFHSignature33>::Ptr container_model_fpfh = computeFPFH(container_model);

    // Coarse Alignment

//    Eigen::Matrix4f model_to_scene_2 = coarseAlignment(container_model, container_model_fpfh, in2, in2_fpfh);
//    Eigen::Matrix4f model_to_scene_1 = ransac_prerejective(container_model, container_model_fpfh, in1, in1_fpfh);
   // Eigen::Matrix4f model_to_scene_2 = ransac_prerejective(container_model, container_model_fpfh, in2, in2_fpfh);

   // pcl::PointCloud<PointT>::Ptr container_scene2(new pcl::PointCloud<PointT>);

  //  pcl::transformPointCloud(*container_model, *container_scene2, model_to_scene_2);


//    Eigen::Matrix4f model_to_scene_1 = coarseAlignment(container_model, container_model_fpfh, in1, in1_fpfh);
//    pcl::PointCloud<PointT>::Ptr container_scene1(new pcl::PointCloud<PointT>);
//     pcl::transformPointCloud(*container_model, *container_scene1, model_to_scene_1);

//    pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow2(container_scene1, 255.0, 255.0, 0.0);
//    pclViewer2->addPointCloud (container_scene1, yellow2, "model");
//    pclViewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model");

////    pcl::visualization::PointCloudColorHandlerCustom<PointT> yellow3(container_scene2, 255.0, 255.0, 0.0);
////    pclViewer2->addPointCloud (container_scene2, yellow3, "model2");
////    pclViewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model2");


//    pcl::visualization::PointCloudColorHandlerCustom<PointT> red2(in1, 255.0, 0.0, 0.0);
//    pclViewer2->addPointCloud (in1, red2, "scene");
//    pclViewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene");

//    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(in2, 0.0, 0.0, 255.0);
//    pclViewer2->addPointCloud (in2, blue, "scene2");
//    pclViewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene2");


    pcl::transformPointCloud(*in2, *in2_transformed, coord_transform);



    Eigen::Matrix4f icp_transform = align_icp(in_sampled2, in_sampled1, coord_transform, 0.02);
    pcl::transformPointCloud(*in2, *icp_aligned_cloud, icp_transform);


    *out = *in1 + *icp_aligned_cloud;
    out = downsample(out);


    pclViewer->addPointCloud (in1, ColorHandlerT(in1, 255.0, 255.0, 0.0), "in1");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "in1");

    pclViewer->addPointCloud (in2_transformed, ColorHandlerT(in2_transformed, 255.0, 0.0, 0.0), "in2");
    pclViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "in2");


    // pc to pc2
    pcl::PCLPointCloud2 pcl_cloud1, pcl_cloud2;
    pcl::toPCLPointCloud2(*in1,                 pcl_cloud1);
    pcl::toPCLPointCloud2(*in2_transformed,   pcl_cloud2);

    //From PCL pointclouds to ROS pointclouds (sensor_msgs)
    sensor_msgs::PointCloud2 ros_cloud1, ros_cloud2;
    pcl_conversions::fromPCL(pcl_cloud1, ros_cloud1);
    pcl_conversions::fromPCL(pcl_cloud2, ros_cloud2);

    // Publish point clouds
    ros::Publisher scan_publisher = n.advertise<sensor_msgs::PointCloud2> ("/multipleKinectsScan", 1);
    scan_publisher.publish(ros_cloud1);
    sleep(1);
    scan_publisher.publish(ros_cloud2);



    ros::Rate r(30);
    while (ros::ok() && !pclViewer->wasStopped()) {
        pclViewer->spinOnce (100);
        r.sleep();
    }
    return 0;
}
