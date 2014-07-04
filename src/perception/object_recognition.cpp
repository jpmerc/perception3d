#include <object_recognition.h>


Object_recognition::Object_recognition()
{
    m_fpfh_persistence_scales[0] = 0.01;
    m_fpfh_persistence_scales[1] = 0.015;
    m_fpfh_persistence_scales[2] = 0.02;
    m_fpfh_persistence_alpha = 1.2f;
    m_sac_ia_maximum_distance = 1.0;
    m_sac_ia_minimum_sampling_distance = 0.02;
    m_sac_ia_maximum_iterations = 1000;
    m_sac_ia_number_of_samples = 15;
    m_sac_ia_correspondance_randomness = 20;
    m_normal_sphere_radius = 0.03;
}

void Object_recognition::compute_normal(pcl::PointCloud<PointT>::Ptr p_cloud, pcl::PointCloud<pcl::Normal>::Ptr p_normal)
{
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ()); //search method
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(.03);
    normal_estimation.setInputCloud(p_cloud);
    normal_estimation.compute(*p_normal);
}

Eigen::Matrix4f Object_recognition::mergePointClouds(   pcl::PointCloud<pcl::FPFHSignature33>::Ptr f_src,
                                                        pcl::PointCloud<pcl::FPFHSignature33>::Ptr f_target,
                                                        pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
                                                        pcl::PointCloud<PointT>::Ptr p_cloud_target_feature)
{
    pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia_;
    sac_ia_.setMinSampleDistance(m_sac_ia_minimum_sampling_distance);
    sac_ia_.setMaxCorrespondenceDistance(m_sac_ia_maximum_distance);
    sac_ia_.setMaximumIterations(m_sac_ia_maximum_iterations);
    sac_ia_.setNumberOfSamples(m_sac_ia_number_of_samples);
    sac_ia_.setCorrespondenceRandomness(m_sac_ia_correspondance_randomness);

    //Set target
    sac_ia_.setInputTarget(p_cloud_target_feature);
    sac_ia_.setTargetFeatures(f_target);

    //Set source
    sac_ia_.setInputSource(p_cloud_src_feature);
    sac_ia_.setSourceFeatures(f_src);

    std::cout << "Starting Alignment" << std::endl;
    //Output
    pcl::PointCloud<PointT> registration_output;
    sac_ia_.align (registration_output);
    std::cout << "Alignment Done" << std::endl;
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>(registration_output));

    float sac_score = sac_ia_.getFitnessScore(m_sac_ia_maximum_distance);
    Eigen::Matrix4f sac_transformation = sac_ia_.getFinalTransformation();
    //temp_transform = sac_transformation; //a deleter ser a rien??

    std::cout << "SAC-IA Transformation Score = " << sac_score << std::endl;

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    float maxDistanceICP = 0.2;
    icp.setInputSource(p_cloud_src_feature);
    icp.setInputTarget(p_cloud_target_feature);
    icp.setMaxCorrespondenceDistance(maxDistanceICP);
    icp.setMaximumIterations(20);
    pcl::PointCloud<PointT> Final;
    icp.align(Final,sac_transformation);
    m_icp_fitness_score = icp.getFitnessScore(maxDistanceICP);
    std::cout << "ICP Transformation Score = " << icp.getFitnessScore(maxDistanceICP) << std::endl;


    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>(Final));

    return icp_transformation;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr Object_recognition::calculateFPFH(pcl::PointCloud<PointT>::Ptr p_cloud,
                                                                             pcl::PointCloud<PointT>::Ptr p_feature,
                                                                             pcl::PointCloud<pcl::Normal>::Ptr p_normal)
{
    //FPFH Calculations
    std::cout << "Starting FPFH calculation!" << std::endl;
    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh_estimation(new pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33>);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    fpfh_estimation->setInputCloud (p_cloud);
    fpfh_estimation->setInputNormals (p_normal);
    fpfh_estimation->setSearchMethod (tree);
    fpfh_estimation->setRadiusSearch (0.01);

    //std::vector<float> scale_values(m_fpfh_persistence_scales,m_fpfh_persistence_scales + sizeof(m_fpfh_persistence_scales)/sizeof(float)); //set the multi scales
    std::vector<float> scale_values;
    scale_values.push_back(0.01);
    scale_values.push_back(0.015);
    scale_values.push_back(0.02);
    pcl::MultiscaleFeaturePersistence<PointT, pcl::FPFHSignature33> feature_persistence;
    feature_persistence.setScalesVector (scale_values);
    feature_persistence.setAlpha (m_fpfh_persistence_alpha);
    feature_persistence.setFeatureEstimator (fpfh_estimation);
    feature_persistence.setDistanceMetric (pcl::CS);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr output_features (new pcl::PointCloud<pcl::FPFHSignature33> ()); //Output
    boost::shared_ptr<std::vector<int> > output_indices (new std::vector<int> ());
    feature_persistence.determinePersistentFeatures (*output_features, output_indices);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (p_cloud);
    extract.setIndices (output_indices);
    extract.setNegative (false);
    extract.filter (*p_feature);

    std::cout << "Taille FPFH = " << output_features->points.size() << std::endl;
    return output_features;
}

bool Object_recognition::object_recon(pcl::PointCloud<PointT>::Ptr p_ptr_cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr ptr_point_cloud_normal(new pcl::PointCloud<pcl::Normal>);
    compute_normal(p_ptr_cloud, ptr_point_cloud_normal);

    pcl::PointCloud<PointT>::Ptr ptr_point_cloud_feature(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr ptr_point_cloud_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
    ptr_point_cloud_fpfh = calculateFPFH(p_ptr_cloud, ptr_point_cloud_feature, ptr_point_cloud_normal);

    pcl::PointCloud<PointT>::Ptr ptr_bd_point_cloud (new pcl::PointCloud<PointT>);//will contain the point cloud from the bd
    pcl::PointCloud<PointT>::Ptr ptr_bd_point_cloud_feature (new pcl::PointCloud<PointT>);//will contain the feature from the bd point cloud
    pcl::PointCloud<pcl::Normal>::Ptr ptr_bd_point_cloud_normal (new pcl::PointCloud<pcl::Normal>);//will contain the normal point cloud from the object in the bd
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr ptr_bd_point_cloud_fpfh (new pcl::PointCloud<pcl::FPFHSignature33>);//will contain the fpfh from the objects in the bd
    compute_normal(ptr_bd_point_cloud, ptr_bd_point_cloud_normal);
    ptr_bd_point_cloud_fpfh = calculateFPFH(ptr_bd_point_cloud,ptr_bd_point_cloud_feature, ptr_bd_point_cloud_normal);


    mergePointClouds(ptr_point_cloud_fpfh, ptr_bd_point_cloud_fpfh, ptr_point_cloud_feature, ptr_bd_point_cloud_feature);

    if(m_icp_fitness_score > 1)
    {
        //the object is recon
        //update an attribut with the grasp dans make it accessible to oder part of
        //the program
    }
    else
    {
        //the object is not recon
    }
}
