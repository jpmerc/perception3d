#include <object_recognition.h>


Object_recognition::Object_recognition()
{
    ros::Time::init();

    m_fpfh_persistence_scales[0] = 0.01;
    m_fpfh_persistence_scales[1] = 0.015;
    m_fpfh_persistence_scales[2] = 0.02;
    m_fpfh_persistence_alpha = 1.2f;
    m_sac_ia_maximum_distance = 1.0;
    m_sac_ia_minimum_sampling_distance = 0.005;
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
    normal_estimation.setRadiusSearch(0.03);
    normal_estimation.setInputCloud(p_cloud);
    normal_estimation.compute(*p_normal);
}



Eigen::Matrix4f Object_recognition::mergePointCVFH(pcl::PointCloud<pcl::VFHSignature308>::Ptr f_src,
                                                   pcl::PointCloud<pcl::VFHSignature308>::Ptr f_target,
                                                   pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
                                                   pcl::PointCloud<PointT>::Ptr p_cloud_target_feature)
{
    ros::Time begin = ros::Time::now();

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    float maxDistanceICP = 0.2;
    icp.setInputSource(p_cloud_src_feature);
    icp.setInputTarget(p_cloud_target_feature);
    icp.setMaxCorrespondenceDistance(maxDistanceICP);
    icp.setMaximumIterations(40);
    pcl::PointCloud<PointT> Final;
    icp.align(Final);
    m_icp_fitness_score = icp.getFitnessScore();

    ros::Time end = ros::Time::now();

    std::cout << GREEN << "ICP Time = " << end - begin << RESET << std::endl;
    std::cout << "ICP Transformation Score = " << icp.getFitnessScore(maxDistanceICP) << std::endl;


    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
    //pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>(Final));
    return icp_transformation;
}




void Object_recognition::computeUniformSampling(pcl::PointCloud<PointT>::Ptr p_cloudIn,
                                                pcl::PointCloud<PointT>::Ptr p_cloudOuput)
{
    ros::Time usBegin = ros::Time::now();

    std::cout << "US computation begin" << std::endl;

    pcl::UniformSampling<PointT> uniformSampling;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    boost::shared_ptr<std::vector<int> > point_cloud_indice (new std::vector<int> ());
    pcl::PointCloud<int> point_cloud_out;
    uniformSampling.setInputCloud(p_cloudIn);
    uniformSampling.setSearchMethod(tree);
    uniformSampling.setRadiusSearch(0.01);
    uniformSampling.compute(point_cloud_out);

    for(int i = 0; i < point_cloud_out.size(); i++)
    {
        point_cloud_indice->push_back(point_cloud_out.at(i));
    }


    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(p_cloudIn);
    extract.setIndices(point_cloud_indice);
    extract.setNegative(false);
    extract.filter(*p_cloudOuput);



    ros::Time usEnd = ros::Time::now();
    std::cout << GREEN << "US time taken = " << usEnd - usBegin << RESET << std::endl;
    std::cout << "Point cloud out size = " << point_cloud_out.size() << std::endl;
    std::cout << "Keypoints Size = " << p_cloudOuput->size() << std::endl;

    //showPointCloud(p_cloudOuput);
}


// Returns the OUR-CVFH Histograms (1 per surface) and also outputs the transform of each of these surfaces in argument tf
pcl::PointCloud<pcl::VFHSignature308>::Ptr Object_recognition::calculateCVFHUS(pcl::PointCloud<PointT>::Ptr p_cloud,
                                                                               pcl::PointCloud<pcl::Normal>::Ptr p_normal,
                                                                               std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf
                                                                               )
{
    pcl::OURCVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> ourCVFH;
    pcl::search::KdTree<PointT>::Ptr ktree (new pcl::search::KdTree<PointT>);
    ourCVFH.setInputCloud(p_cloud);
    ourCVFH.setInputNormals(p_normal);
    ourCVFH.setSearchMethod(ktree);
    ourCVFH.setKSearch(0);
    ourCVFH.setRadiusSearch(0.05);
    //ourCVFH.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
    //ourCVFH.setCurvatureThreshold(1.0);
    // ourcvfh.setClusterTolerance (0.015f); //1.5cm, three times the leaf size
    ourCVFH.setNormalizeBins(false);
    //ourCVFH.setAxisRatio(0.8);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr returnCloud(new pcl::PointCloud<pcl::VFHSignature308>);
    ourCVFH.compute(*returnCloud);


    ourCVFH.getTransforms(tf);

    std::cout << "CVFH size = " << returnCloud->size() << std::endl;

    return returnCloud;
}


int Object_recognition::object_recon(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
                                     pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd)
{

    std::vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> bd_vector;

    //fpfhProcessing(p_ptr_cloud, p_bd_cloud_ptr);

    //shotColorProcessing(p_ptr_cloud, p_bd_cloud_ptr);

    //usProcessingFpfh(p_ptr_cloud, p_bd_cloud_ptr);

    //usProcessingShot(p_ptr_cloud, p_bd_cloud_ptr);

    //usProcessingCVFH(p_ptr_cloud, p_bd_cloud_ptr);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr object_cvfh = makeCVFH(p_ptr_cloud);


    int positionObject = histogramComparison(object_cvfh, p_bd);

    return positionObject;

}


// Merge 2 pointclouds based on OUR-CVFH Features
void Object_recognition::usProcessingCVFH(pcl::PointCloud<PointT>::Ptr p_ptr_cloud, pcl::PointCloud<PointT>::Ptr p_bd_cloud_ptr)
{

    Eigen::Matrix4f transformationMatrix;
    pcl::PointCloud<PointT>::Ptr transformedPC_ptr(new pcl::PointCloud<PointT>);

    ros::Time begin = ros::Time::now();

    // Sampling + Normal of both pc
    pcl::PointCloud<PointT>::Ptr cloud_us_ptr(new pcl::PointCloud<PointT>);
    computeUniformSampling(p_ptr_cloud, cloud_us_ptr);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_us_normal_ptr (new pcl::PointCloud<pcl::Normal>);
    compute_normal(cloud_us_ptr, cloud_us_normal_ptr);


    pcl::PointCloud<PointT>::Ptr cloud_us_bd_ptr(new pcl::PointCloud<PointT>);
    computeUniformSampling(p_bd_cloud_ptr, cloud_us_bd_ptr);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_us_bd_normal_ptr (new pcl::PointCloud<pcl::Normal>);
    compute_normal(cloud_us_bd_ptr, cloud_us_bd_normal_ptr);

    // Calculation of the signature
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > cloud_vfh_tf;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud_vfh_ptr (new pcl::PointCloud<pcl::VFHSignature308>);
    *cloud_vfh_ptr = *(calculateCVFHUS(cloud_us_ptr, cloud_us_normal_ptr, cloud_vfh_tf));

    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > cloud_bd_tf;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud_bd_vfh_ptr (new pcl::PointCloud<pcl::VFHSignature308>);
    *cloud_bd_vfh_ptr = *(calculateCVFHUS(cloud_us_bd_ptr, cloud_us_bd_normal_ptr, cloud_bd_tf));

    // Merge (ICP)
    transformationMatrix = mergePointCVFH(cloud_vfh_ptr, cloud_bd_vfh_ptr, cloud_us_ptr, cloud_us_bd_ptr);


    //histogramComparison(cloud_vfh_ptr, cloud_bd_vfh_ptr);

    ros::Time end = ros::Time::now();

    pcl::transformPointCloud(*p_ptr_cloud, *transformedPC_ptr, transformationMatrix);
    showPointCloud(transformedPC_ptr);

    std::cout << GREEN << "US Total time CVFH = " << end - begin << RESET
              << std::endl << std::endl;
}


ObjectBd Object_recognition::OURCVFHRecognition(pcl::PointCloud<PointT>::Ptr in_pc, FileAPI *fileAPI){

    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > surface_transforms;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr surface_histograms = makeCVFH(in_pc,surface_transforms);

    // Get a certain number of the NN surface hypotheses from the object database
    pcl::PointCloud<pcl::VFHSignature308>::Ptr signature_database = fileAPI->getAllHistograms();
    std::vector<std::vector<int> > NN_object_indices = getNNSurfaces(surface_histograms,signature_database,5);

    // Get object hypotheses and initial transforms
    std::vector<std::vector<ObjectBd> > object_hypotheses;
    for(int i=0; i<NN_object_indices.size(); i++){
        std::vector<ObjectBd> obj_vector = fileAPI->retrieveObjectFromHistogram(NN_object_indices.at(i));
        object_hypotheses.push_back(obj_vector);
    }

    // Recalculate hypotheses features to get the transform
    int surfaceNumber = 0;
    std::vector<std::vector<Eigen::Matrix4f> > transform_matrix;
    while(surfaceNumber < surface_histograms->size()){
        for(int i=0; i<object_hypotheses.size(); i++){
            std::vector<ObjectBd> obj_vector = object_hypotheses.at(i);
            for(int j=0; j< obj_vector.size(); j++){
                ObjectBd obj = obj_vector.at(j);
                pcl::PointCloud<PointT>::Ptr model_pc = obj.getPointCloud();
                std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > model_surface_transforms;
                pcl::PointCloud<pcl::VFHSignature308>::Ptr model_signature = makeCVFH(model_pc,model_surface_transforms);

                // TO CONTINUE :
                // 1) Take the good histogram and transform (surfaceNumber variable)  -- Make sure that the histogram is the same that we have chosen previously when calculating the distance between histograms
                // 2) Compute total transform and store it in a vector

            }

        }
        surfaceNumber++;
    }

    //Output the object with the best score


}






std::vector<std::vector<int> > Object_recognition::getNNSurfaces(pcl::PointCloud<pcl::VFHSignature308>::Ptr p_cloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd_cloud, int NNnumber)
{
    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::VFHSignature308>);

    kdtree->setInputCloud(p_bd_cloud);

    std::vector<int> index;
    std::vector<float> sqrDistance;

    std::vector<std::vector<int> > memoryIndex;
    std::vector<std::vector<float> > memoryDistance;

    for(int i = 0; i < p_cloud->size(); i++)
    {
        int numberOfReturnedNN = kdtree->nearestKSearch(p_cloud->at(i), NNnumber, index, sqrDistance);
        if(numberOfReturnedNN > 0){
            memoryIndex.push_back(index);
            memoryDistance.push_back(sqrDistance);
        }
        else{

        }

    }

    return memoryIndex;
}



int Object_recognition::histogramComparison(pcl::PointCloud<pcl::VFHSignature308>::Ptr p_cloud,
                                            pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd_cloud)

{
    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::VFHSignature308>);

    kdtree->setInputCloud(p_bd_cloud);

    std::vector<int> index(1);
    std::vector<float> sqrDistance(1);

    std::vector<int> memoryIndex;
    std::vector<float> memoryDistance;

    for(int i = 0; i < p_cloud->size(); i++)
    {
        kdtree->nearestKSearch(p_cloud->at(i), 1, index, sqrDistance);
        memoryIndex.push_back(index[0]);
        memoryDistance.push_back(sqrDistance[0]);
    }

    int smallestDistance = sqrDistance[0];
    int smallestDistanceIndex = memoryIndex[0];
    for(int i = 0; i < sqrDistance.size(); i++)
    {
        if (smallestDistance > sqrDistance[i])
        {
            smallestDistance = sqrDistance[i];
            smallestDistanceIndex = memoryIndex[i];
        }
    }

    std::cout << "The best match is = " << smallestDistanceIndex << std::endl;
    std::cout << "The sqrt distance is = " << smallestDistance << std::endl;
    return smallestDistanceIndex;
}

/*
  A new comparison just to use in the test, its a copy of histogramComparison but with a different
  return type.
  */

std::vector<float> Object_recognition::histogramComparisonVector(pcl::PointCloud<pcl::VFHSignature308>::Ptr p_cloud,
                                            pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd_cloud)

{
    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::VFHSignature308>);

    kdtree->setInputCloud(p_bd_cloud);

    std::vector<int> index(1);
    std::vector<float> sqrDistance(1);

    std::vector<int> memoryIndex;
    std::vector<float> memoryDistance;

    for(int i = 0; i < p_cloud->size(); i++)
    {
        kdtree->nearestKSearch(p_cloud->at(i), 1, index, sqrDistance);
        memoryIndex.push_back(index[0]);
        memoryDistance.push_back(sqrDistance[0]);
    }

    int smallestDistance = sqrDistance[0];
    int smallestDistanceIndex = memoryIndex[0];
    for(int i = 0; i < sqrDistance.size(); i++)
    {
        if (smallestDistance > sqrDistance[i])
        {
            smallestDistance = sqrDistance[i];
            smallestDistanceIndex = memoryIndex[i];
        }
    }

    std::vector<float> returnVector;
    returnVector.push_back(smallestDistanceIndex);
    returnVector.push_back(smallestDistance);

    std::cout << "The best match is = " << smallestDistanceIndex << std::endl;
    std::cout << "The sqrt distance is = " << smallestDistance << std::endl;
    return returnVector;
}


pcl::PointCloud<pcl::VFHSignature308>::Ptr Object_recognition::makeCVFH(pcl::PointCloud<PointT>::Ptr p_ptr_cloud)
{
    pcl::PointCloud<PointT>::Ptr cloud_us_ptr(new pcl::PointCloud<PointT>);
    computeUniformSampling(p_ptr_cloud, cloud_us_ptr);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_us_normal_ptr (new pcl::PointCloud<pcl::Normal>);
    compute_normal(cloud_us_ptr, cloud_us_normal_ptr);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud_vfh_ptr (new pcl::PointCloud<pcl::VFHSignature308>);
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > tf_;
    *cloud_vfh_ptr = *(calculateCVFHUS(cloud_us_ptr, cloud_us_normal_ptr,tf_));

    return cloud_vfh_ptr;

}

pcl::PointCloud<pcl::VFHSignature308>::Ptr Object_recognition::makeCVFH(pcl::PointCloud<PointT>::Ptr p_ptr_cloud, std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf_)
{
    pcl::PointCloud<PointT>::Ptr cloud_us_ptr(new pcl::PointCloud<PointT>);
    computeUniformSampling(p_ptr_cloud, cloud_us_ptr);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_us_normal_ptr (new pcl::PointCloud<pcl::Normal>);
    compute_normal(cloud_us_ptr, cloud_us_normal_ptr);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud_vfh_ptr (new pcl::PointCloud<pcl::VFHSignature308>);
    *cloud_vfh_ptr = *(calculateCVFHUS(cloud_us_ptr, cloud_us_normal_ptr,tf_));

    return cloud_vfh_ptr;

}


void Object_recognition::showPointCloud(pcl::PointCloud<PointT>::Ptr p_cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("simple cloud"));
    viewer->addPointCloud<PointT>(p_cloud);
    viewer->setBackgroundColor(0,0,0);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}

//Eigen::Matrix4f Object_recognition::mergePointClouds(   pcl::PointCloud<pcl::FPFHSignature33>::Ptr f_src,
//                                                        pcl::PointCloud<pcl::FPFHSignature33>::Ptr f_target,
//                                                        pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
//                                                        pcl::PointCloud<PointT>::Ptr p_cloud_target_feature)
//{

//    ros::Time begin = ros::Time::now();

//    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia_;
//    sac_ia_.setMinSampleDistance(m_sac_ia_minimum_sampling_distance);
//    sac_ia_.setMaxCorrespondenceDistance(m_sac_ia_maximum_distance);
//    sac_ia_.setMaximumIterations(m_sac_ia_maximum_iterations);
//    sac_ia_.setNumberOfSamples(m_sac_ia_number_of_samples);
//    sac_ia_.setCorrespondenceRandomness(m_sac_ia_correspondance_randomness);

//    //Set target
//    sac_ia_.setInputTarget(p_cloud_target_feature);
//    sac_ia_.setTargetFeatures(f_target);


//    //Set source
//    sac_ia_.setInputSource(p_cloud_src_feature);
//    sac_ia_.setSourceFeatures(f_src);

//    std::cout << "Starting Alignment" << std::endl;
//    //Output
//    pcl::PointCloud<PointT> registration_output;
//    sac_ia_.align (registration_output);
//    std::cout << "Alignment Done" << std::endl;
//    //   pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>(registration_output));

//    float sac_score = sac_ia_.getFitnessScore();
//    Eigen::Matrix4f sac_transformation = sac_ia_.getFinalTransformation();
//    //temp_transform = sac_transformation; //a deleter ser a rien??

//    ros::Time end = ros::Time::now();

//    std::cout << GREEN << "SAC Time = " << end - begin << RESET << std::endl;
//    std::cout << "SAC-IA Transformation Score = " << sac_score << std::endl;

//    begin = ros::Time::now();

//    pcl::IterativeClosestPoint<PointT, PointT> icp;
//    float maxDistanceICP = 0.2;
//    icp.setInputSource(p_cloud_src_feature);
//    icp.setInputTarget(p_cloud_target_feature);
//    icp.setMaxCorrespondenceDistance(maxDistanceICP);
//    icp.setMaximumIterations(40);
//    pcl::PointCloud<PointT> Final;
//    icp.align(Final);
//    m_icp_fitness_score = icp.getFitnessScore();

//    end = ros::Time::now();

//    std::cout << GREEN << "ICP Time = " << end - begin << RESET << std::endl;
//    std::cout << "ICP Transformation Score = " << icp.getFitnessScore(maxDistanceICP) << std::endl;


//    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>(Final));
//    return icp_transformation;
//}

//Eigen::Matrix4f Object_recognition::mergePointCloudsShot(pcl::PointCloud<pcl::SHOT1344>::Ptr f_src,
//                                                         pcl::PointCloud<pcl::SHOT1344>::Ptr f_target,
//                                                         pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
//                                                         pcl::PointCloud<PointT>::Ptr p_cloud_target_feature)
//{
//    ros::Time begin = ros::Time::now();

//    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::SHOT1344> sac_ia_;
//    sac_ia_.setMinSampleDistance(m_sac_ia_minimum_sampling_distance);
//    sac_ia_.setMaxCorrespondenceDistance(m_sac_ia_maximum_distance);
//    sac_ia_.setMaximumIterations(m_sac_ia_maximum_iterations);
//    sac_ia_.setNumberOfSamples(m_sac_ia_number_of_samples);
//    sac_ia_.setCorrespondenceRandomness(m_sac_ia_correspondance_randomness);

//    //Set target
//    sac_ia_.setInputTarget(p_cloud_target_feature);
//    sac_ia_.setTargetFeatures(f_target);


//    //Set source
//    sac_ia_.setInputSource(p_cloud_src_feature);
//    sac_ia_.setSourceFeatures(f_src);

//    std::cout << "Starting Alignment" << std::endl;
//    //Output
//    pcl::PointCloud<PointT> registration_output;
//    sac_ia_.align (registration_output);
//    std::cout << "Alignment Done" << std::endl;
//    //   pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>(registration_output));

//    float sac_score = sac_ia_.getFitnessScore();
//    Eigen::Matrix4f sac_transformation = sac_ia_.getFinalTransformation();
//    //temp_transform = sac_transformation; //a deleter ser a rien??

//    ros::Time end = ros::Time::now();

//    std::cout << GREEN << "SAC-IA Time = " << end - begin << RESET << std::endl;
//    std::cout << "SAC-IA Transformation Score = " << sac_score << std::endl;

//    begin = ros::Time::now();

//    pcl::IterativeClosestPoint<PointT, PointT> icp;
//    float maxDistanceICP = 0.2;
//    icp.setInputSource(p_cloud_src_feature);
//    icp.setInputTarget(p_cloud_target_feature);
//    icp.setMaxCorrespondenceDistance(maxDistanceICP);
//    icp.setMaximumIterations(40);
//    pcl::PointCloud<PointT> Final;
//    icp.align(Final);
//    m_icp_fitness_score = icp.getFitnessScore();

//    end = ros::Time::now();

//    std::cout << GREEN << "ICP Time = " << end - begin << RESET << std::endl;
//    std::cout << "ICP Transformation Score = " << icp.getFitnessScore(maxDistanceICP) << std::endl;


//    Eigen::Matrix4f icp_transformation = icp.getFinalTransformation();
//    //pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>(Final));
//    return icp_transformation;
//}

//pcl::PointCloud<pcl::FPFHSignature33>::Ptr Object_recognition::calculateFPFH(pcl::PointCloud<PointT>::Ptr p_cloud,
//                                                                             pcl::PointCloud<PointT>::Ptr p_feature,
//                                                                             pcl::PointCloud<pcl::Normal>::Ptr p_normal)
//{
//    ros::Time fpfhBegin = ros::Time::now();

//    //FPFH Calculations
//    std::cout << "Starting FPFH calculation!" << std::endl;
//    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh_estimation(new pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33>);
//    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//    fpfh_estimation->setInputCloud (p_cloud);
//    fpfh_estimation->setInputNormals (p_normal);
//    fpfh_estimation->setSearchMethod (tree);
//    fpfh_estimation->setRadiusSearch (0.01);

//    std::vector<float> scale_values(m_fpfh_persistence_scales,m_fpfh_persistence_scales + sizeof(m_fpfh_persistence_scales)/sizeof(float)); //set the multi scales
//    scale_values.push_back(0.01);
//    scale_values.push_back(0.015);
//    scale_values.push_back(0.02);
//    pcl::MultiscaleFeaturePersistence<PointT, pcl::FPFHSignature33> feature_persistence;
//    feature_persistence.setScalesVector (scale_values);
//    feature_persistence.setAlpha (m_fpfh_persistence_alpha);
//    feature_persistence.setFeatureEstimator (fpfh_estimation);
//    feature_persistence.setDistanceMetric (pcl::CS);

//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr output_features (new pcl::PointCloud<pcl::FPFHSignature33> ()); //Output
//    boost::shared_ptr<std::vector<int> > output_indices (new std::vector<int> ());
//    feature_persistence.determinePersistentFeatures (*output_features, output_indices);

//    pcl::ExtractIndices<PointT> extract;
//    extract.setInputCloud (p_cloud);
//    extract.setIndices (output_indices);
//    extract.setNegative (false);
//    extract.filter (*p_feature);

//    std::cout << "Taille feature = " << p_feature->size() << std::endl;
//    std::cout << "Taille FPFH = " << output_features->points.size() << std::endl;


//    ros::Time fpfhEnd = ros::Time::now();

//    std::cout << GREEN << "FPFHCompute time = " << fpfhEnd - fpfhBegin << RESET << std::endl;

//    return output_features;
//}

//pcl::PointCloud<pcl::FPFHSignature33>::Ptr Object_recognition::calculateFPFHUS(pcl::PointCloud<PointT>::Ptr p_cloud,
//                                                                               pcl::PointCloud<pcl::Normal>::Ptr p_normal)
//{
//    ros::Time fpfhBegin = ros::Time::now();

//    //FPFH Calculations
//    std::cout << "Starting FPFH calculation!" << std::endl;
//    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh_estimation(new pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33>);
//    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr return_fpfh (new pcl::PointCloud<pcl::FPFHSignature33>);
//    fpfh_estimation->setInputCloud (p_cloud);
//    fpfh_estimation->setInputNormals (p_normal);
//    fpfh_estimation->setSearchMethod (tree);
//    fpfh_estimation->setRadiusSearch (0.01);
//    fpfh_estimation->compute(*return_fpfh);

//    return return_fpfh;
//}

//pcl::PointCloud<pcl::SHOT1344>::Ptr Object_recognition::calculateShotColor(pcl::PointCloud<PointT>::Ptr p_cloud,
//                                                                           pcl::PointCloud<PointT>::Ptr p_feature,
//                                                                           pcl::PointCloud<pcl::Normal>::Ptr p_normal)
//{
//    ros::Time shotBegin = ros::Time::now();

//    std::cout << "Starting SHOTColor estimation" << std::endl;
//    pcl::SHOTColorEstimation<PointT, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>::Ptr shotColor (new pcl::SHOTColorEstimation<PointT, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>);
//    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//    pcl::PointCloud<pcl::SHOT1344>::Ptr cloud_shot(new pcl::PointCloud<pcl::SHOT1344>);
//    shotColor->setInputCloud(p_cloud);
//    shotColor->setInputNormals(p_normal);
//    shotColor->setSearchMethod(tree);
//    shotColor->setRadiusSearch(0.04);
//    shotColor->compute(*cloud_shot);

//    std::vector<float> scale_values;
//    scale_values.push_back(0.01);
//    scale_values.push_back(0.015);
//    scale_values.push_back(0.02);

//    pcl::MultiscaleFeaturePersistence<PointT, pcl::SHOT1344> feature_persistence;
//    feature_persistence.setFeatureEstimator(shotColor);
//    feature_persistence.setDistanceMetric(pcl::CS);
//    feature_persistence.setAlpha(1.2f);

//    feature_persistence.setScalesVector(scale_values);

//    pcl::PointCloud<pcl::SHOT1344>::Ptr cloud_return (new pcl::PointCloud<pcl::SHOT1344>);
//    boost::shared_ptr<std::vector<int> > output_indices (new std::vector<int> ());
//    feature_persistence.determinePersistentFeatures(*cloud_return, output_indices);

//    pcl::ExtractIndices<PointT> extract;
//    extract.setInputCloud(p_cloud);
//    extract.setIndices(output_indices);
//    extract.setNegative(false);
//    extract.filter(*p_feature);


//    ros::Time shotEnd = ros::Time::now();
//    if(cloud_return->size() == 0)
//    {
//        cloud_return = cloud_shot;
//        *p_feature = *p_cloud;
//    }
//    std::cout << "Taille SHOTColor " << cloud_return->size() << std::endl;
//    std::cout << "Taille feature " << p_feature->size() << std::endl;
//    std::cout << GREEN << "Shot computation time = " << shotEnd - shotBegin << RESET << std::endl;

//    return cloud_return;
//}

//pcl::PointCloud<pcl::SHOT1344>::Ptr Object_recognition::calculateShotColorUS(pcl::PointCloud<PointT>::Ptr p_cloud,
//                                                                             pcl::PointCloud<pcl::Normal>::Ptr p_normal)
//{
//    ros::Time shotBegin = ros::Time::now();

//    std::cout << "Starting SHOTColor estimation" << std::endl;
//    pcl::SHOTColorEstimation<PointT, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>::Ptr shotColor (new pcl::SHOTColorEstimation<PointT, pcl::Normal, pcl::SHOT1344, pcl::ReferenceFrame>);
//    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//    pcl::PointCloud<pcl::SHOT1344>::Ptr cloud_shot(new pcl::PointCloud<pcl::SHOT1344>);
//    shotColor->setInputCloud(p_cloud);
//    shotColor->setInputNormals(p_normal);
//    shotColor->setSearchMethod(tree);
//    shotColor->setRadiusSearch(0.04);
//    shotColor->compute(*cloud_shot);

//    ros::Time shotEnd = ros::Time::now();
//    std::cout << GREEN << "Shot computation time = " << shotEnd - shotBegin << RESET << std::endl;

//    return cloud_shot;
//}

//void Object_recognition::fpfhProcessing(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
//                                        pcl::PointCloud<PointT>::Ptr p_bd_cloud_ptr)
//{
//    Eigen::Matrix4f transformationMatrix;
//    pcl::PointCloud<PointT>::Ptr transformedPC_ptr(new pcl::PointCloud<PointT>);


//    pcl::PointCloud<pcl::Normal>::Ptr ptr_point_cloud_normal(new pcl::PointCloud<pcl::Normal>);
//    compute_normal(p_ptr_cloud, ptr_point_cloud_normal);


//    pcl::PointCloud<pcl::Normal>::Ptr ptr_bd_point_cloud_normal (new pcl::PointCloud<pcl::Normal>);//will contain the normal point cloud from the object in the bd
//    compute_normal(p_bd_cloud_ptr, ptr_bd_point_cloud_normal);

//    //FPFH computation

//    ros::Time begin = ros::Time::now();

//    pcl::PointCloud<PointT>::Ptr ptr_point_cloud_feature(new pcl::PointCloud<PointT>);
//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr ptr_point_cloud_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
//    *ptr_point_cloud_fpfh = *(calculateFPFH(p_ptr_cloud, ptr_point_cloud_feature, ptr_point_cloud_normal));

//    pcl::PointCloud<PointT>::Ptr ptr_bd_point_cloud_feature (new pcl::PointCloud<PointT>);//will contain the feature from the bd point cloud
//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr ptr_bd_point_cloud_fpfh (new pcl::PointCloud<pcl::FPFHSignature33>);//will contain the fpfh from the objects in the bd
//    *ptr_bd_point_cloud_fpfh = *(calculateFPFH(p_bd_cloud_ptr,ptr_bd_point_cloud_feature, ptr_bd_point_cloud_normal));

//    transformationMatrix = mergePointClouds(ptr_point_cloud_fpfh, ptr_bd_point_cloud_fpfh, ptr_point_cloud_feature, ptr_bd_point_cloud_feature);

//    ros::Time end = ros::Time::now();
//    std::cout << GREEN << "FPFH total time = " << end - begin << RESET << std::endl << std::endl;

//    pcl::transformPointCloud(*p_ptr_cloud, *transformedPC_ptr, transformationMatrix);
//    showPointCloud(transformedPC_ptr);
//}


//void Object_recognition::shotColorProcessing(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
//                                             pcl::PointCloud<PointT>::Ptr p_bd_cloud_ptr)
//{
//    Eigen::Matrix4f transformationMatrix;
//    pcl::PointCloud<PointT>::Ptr transformedPC_ptr(new pcl::PointCloud<PointT>);

//    //shot color computation
//    ros::Time begin = ros::Time::now();

//    pcl::PointCloud<pcl::Normal>::Ptr ptr_point_cloud_normal(new pcl::PointCloud<pcl::Normal>);
//    compute_normal(p_ptr_cloud, ptr_point_cloud_normal);


//    pcl::PointCloud<pcl::Normal>::Ptr ptr_bd_point_cloud_normal (new pcl::PointCloud<pcl::Normal>);//will contain the normal point cloud from the object in the bd
//    compute_normal(p_bd_cloud_ptr, ptr_bd_point_cloud_normal);


//    pcl::PointCloud<PointT>::Ptr ptr_point_cloud_shot_feature(new pcl::PointCloud<PointT>);
//    pcl::PointCloud<PointT>::Ptr ptr_bd_point_cloud_shot_feature(new pcl::PointCloud<PointT>);

//    pcl::PointCloud<pcl::SHOT1344>::Ptr ptr_point_cloud_shot (new pcl::PointCloud<pcl::SHOT1344>);
//    *ptr_point_cloud_shot = *(calculateShotColor(p_ptr_cloud, ptr_point_cloud_shot_feature, ptr_point_cloud_normal));

//    pcl::PointCloud<pcl::SHOT1344>::Ptr ptr_bd_point_cloud_shot (new pcl::PointCloud<pcl::SHOT1344>);
//    *ptr_bd_point_cloud_shot = *(calculateShotColor(p_bd_cloud_ptr, ptr_bd_point_cloud_shot_feature, ptr_bd_point_cloud_normal));


//    transformationMatrix = mergePointCloudsShot(ptr_point_cloud_shot, ptr_bd_point_cloud_shot, ptr_point_cloud_shot_feature, ptr_bd_point_cloud_shot_feature);

//    ros::Time end = ros::Time::now();
//    std::cout << GREEN << "SHOT total Time = " << end - begin << RESET << std::endl << std::endl;

//    pcl::transformPointCloud(*p_ptr_cloud, *transformedPC_ptr, transformationMatrix);
//    showPointCloud(transformedPC_ptr);
//}

//void Object_recognition::usProcessingFpfh(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
//                                          pcl::PointCloud<PointT>::Ptr p_bd_cloud_ptr)
//{
//    Eigen::Matrix4f transformationMatrix;
//    pcl::PointCloud<PointT>::Ptr transformedPC_ptr(new pcl::PointCloud<PointT>);

//    ros::Time begin = ros::Time::now();

//    pcl::PointCloud<PointT>::Ptr cloud_us_ptr(new pcl::PointCloud<PointT>);
//    computeUniformSampling(p_ptr_cloud, cloud_us_ptr);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_us_normal_ptr (new pcl::PointCloud<pcl::Normal>);
//    compute_normal(cloud_us_ptr, cloud_us_normal_ptr);

//    pcl::PointCloud<PointT>::Ptr cloud_us_bd_ptr(new pcl::PointCloud<PointT>);
//    computeUniformSampling(p_bd_cloud_ptr, cloud_us_bd_ptr);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_us_bd_normal_ptr (new pcl::PointCloud<pcl::Normal>);
//    compute_normal(cloud_us_bd_ptr, cloud_us_bd_normal_ptr);


//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_us_ptr (new pcl::PointCloud<pcl::FPFHSignature33>);
//    *fpfh_us_ptr = *(calculateFPFHUS(cloud_us_ptr,cloud_us_normal_ptr));

//    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_us_bd_ptr (new pcl::PointCloud<pcl::FPFHSignature33>);
//    *fpfh_us_bd_ptr = *(calculateFPFHUS(cloud_us_bd_ptr, cloud_us_bd_normal_ptr));

//    transformationMatrix = mergePointClouds(fpfh_us_ptr, fpfh_us_bd_ptr, cloud_us_ptr, cloud_us_bd_ptr);

//    ros::Time end = ros::Time::now();

//    std::cout << GREEN << "US Total time FPFH = " << end - begin << RESET
//              << std::endl << std::endl;


//    pcl::transformPointCloud(*p_ptr_cloud, *transformedPC_ptr, transformationMatrix);
//    showPointCloud(transformedPC_ptr);


//}

//void Object_recognition::usProcessingShot(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
//                                          pcl::PointCloud<PointT>::Ptr p_bd_cloud_ptr)
//{
//    Eigen::Matrix4f transformationMatrix;
//    pcl::PointCloud<PointT>::Ptr transformedPC_ptr(new pcl::PointCloud<PointT>);

//    ros::Time begin = ros::Time::now();

//    pcl::PointCloud<PointT>::Ptr cloud_us_ptr(new pcl::PointCloud<PointT>);
//    computeUniformSampling(p_ptr_cloud, cloud_us_ptr);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_us_normal_ptr (new pcl::PointCloud<pcl::Normal>);
//    compute_normal(cloud_us_ptr, cloud_us_normal_ptr);

//    pcl::PointCloud<PointT>::Ptr cloud_us_bd_ptr(new pcl::PointCloud<PointT>);
//    computeUniformSampling(p_bd_cloud_ptr, cloud_us_bd_ptr);
//    pcl::PointCloud<pcl::Normal>::Ptr cloud_us_bd_normal_ptr (new pcl::PointCloud<pcl::Normal>);
//    compute_normal(cloud_us_bd_ptr, cloud_us_bd_normal_ptr);

//    pcl::PointCloud<pcl::SHOT1344>::Ptr shot_us_ptr (new pcl::PointCloud<pcl::SHOT1344>);
//    *shot_us_ptr = *calculateShotColorUS(cloud_us_ptr, cloud_us_normal_ptr);

//    pcl::PointCloud<pcl::SHOT1344>::Ptr shot_us_bd_ptr (new pcl::PointCloud<pcl::SHOT1344>);
//    *shot_us_bd_ptr = * calculateShotColorUS(cloud_us_bd_ptr, cloud_us_bd_normal_ptr);

//    transformationMatrix = mergePointCloudsShot(shot_us_ptr, shot_us_bd_ptr, cloud_us_ptr, cloud_us_bd_ptr);

//    ros::Time end = ros::Time::now();

//    std::cout << GREEN << "US Total time SHOT = " << end -begin << RESET
//              << std::endl << std::endl;

//    pcl::transformPointCloud(*p_ptr_cloud, *transformedPC_ptr, transformationMatrix);
//    showPointCloud(transformedPC_ptr);

//}
