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

    m_rmse_recognition_threshold = 0.005;
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

// Takes initial guess as input argument. It modifies it to return the final transformation.
// The function returns the score.
double Object_recognition::mergePointCVFH(pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
                                          pcl::PointCloud<PointT>::Ptr p_cloud_target_feature,
                                          Eigen::Matrix4f &transform_guess)
{
    double time;
    return mergePointCVFH(p_cloud_src_feature,
                          p_cloud_target_feature,
                          transform_guess,
                          time);
}

double Object_recognition::mergePointCVFH(pcl::PointCloud<PointT>::Ptr p_cloud_src_feature,
                                          pcl::PointCloud<PointT>::Ptr p_cloud_target_feature,
                                          Eigen::Matrix4f &transform_guess,
                                          double &executionTime)
{
    ros::Time begin = ros::Time::now();

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    //float maxDistanceICP = 0.2;
    icp.setInputSource(p_cloud_src_feature);
    icp.setInputTarget(p_cloud_target_feature);
    //icp.setMaxCorrespondenceDistance(maxDistanceICP);
    icp.setMaximumIterations(40);
    pcl::PointCloud<PointT> Final;
    icp.align(Final,transform_guess);
    m_icp_fitness_score = icp.getFitnessScore();

    ros::Time end = ros::Time::now();

    executionTime = (end-begin).toSec();
    transform_guess = icp.getFinalTransformation();
    return m_icp_fitness_score;
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


/*
  This is a version with lest parameter to compute OURVFH.  It call the one with the more parameter.
  */
pcl::PointCloud<pcl::VFHSignature308>::Ptr Object_recognition::calculateCVFH(pcl::PointCloud<PointT>::Ptr p_cloud,
                                                                               pcl::PointCloud<pcl::Normal>::Ptr p_normal,
                                                                               std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf)
{
    std::vector<Eigen::Vector3f> centroid;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr returnSig(new pcl::PointCloud<pcl::VFHSignature308>);
    std::vector<pcl::PointIndices> indicesVect;
    returnSig = calculateCVFH(p_cloud, p_normal, tf, centroid, indicesVect);
    return returnSig;
}

/*
  This is a version with lest parameter to compute OURVFH.  It call the one with the more parameter.
  */
pcl::PointCloud<pcl::VFHSignature308>::Ptr Object_recognition::calculateCVFH(pcl::PointCloud<PointT>::Ptr p_cloud,
                                                                              pcl::PointCloud<pcl::Normal>::Ptr p_normal,
                                                                              std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf,
                                                                              std::vector<Eigen::Vector3f> &p_centroid)
{
    pcl::PointCloud<pcl::VFHSignature308>::Ptr returnSig(new pcl::PointCloud<pcl::VFHSignature308>);
    std::vector<pcl::PointIndices> indicesVect;
    returnSig = calculateCVFH(p_cloud, p_normal, tf, p_centroid,indicesVect);
    return returnSig;

}

/*
  This is the main function to compute OURCVFH.
  param[in] p_cloud the point cloud you want to compute the signature.
  param[in] p_normal the point cloud normal.
  param[out] tf the transform from SGURF to camare returned by the OURCVFH algorithm.
  param[out] p_centroid the centroid of every cluster used by the algorithm.
  param[out] p_indice the indice that represent every cluster used by the algoritm.  It should be
  in the same order that the centroid and the tf.
  */
pcl::PointCloud<pcl::VFHSignature308>::Ptr Object_recognition::calculateCVFH(pcl::PointCloud<PointT>::Ptr p_cloud,
                                                                              pcl::PointCloud<pcl::Normal>::Ptr p_normal,
                                                                              std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf,
                                                                              std::vector<Eigen::Vector3f> &p_centroid,
                                                                              std::vector<pcl::PointIndices>& p_indice)
{
    pcl::OURCVFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> ourCVFH;
    pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT>);
    ourCVFH.setInputCloud(p_cloud);
    ourCVFH.setInputNormals(p_normal);
    ourCVFH.setSearchMethod(kdtree);
    ourCVFH.setKSearch(10);
    ourCVFH.setRadiusSearch(0);
    //ourCVFH.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
    //ourCVFH.setCurvatureThreshold(1.0);
    // ourcvfh.setClusterTolerance (0.015f); //1.5cm, three times the leaf size
    ourCVFH.setNormalizeBins(false);
    ourCVFH.setAxisRatio(0.98);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr returnCloud(new pcl::PointCloud<pcl::VFHSignature308>);
    ourCVFH.compute(*returnCloud);

    std::vector< Eigen::Vector3f > normal_centroids;

    ourCVFH.getTransforms(tf);
    ourCVFH.getCentroidClusters(p_centroid);
    ourCVFH.getCentroidNormalClusters(normal_centroids);
    ourCVFH.getClusterIndices(p_indice);


//        cout << "---Centroid---" << endl;
//        for(int i=0; i < p_centroid.size() ; i++){
//            Eigen::Vector3f vec = p_centroid.at(i);
//            cout << "x: " <<  vec[2] << endl;
//            cout << "y: " << -vec[0] << endl;
//            cout << "z: " << -vec[1] << endl;
//            cout << endl;
//        }

//        cout << "---Normal Centroids---" << endl;
//        for(int i=0; i < normal_centroids.size() ; i++){
//            Eigen::Vector3f vec = normal_centroids.at(i);
//            cout << "x: " <<  vec[2] << endl;
//            cout << "y: " << -vec[0] << endl;
//            cout << "z: " << -vec[1] << endl;
//            cout << endl;
//        }

//        cout << "---Transforms---" << endl;
//        for(int i=0; i < tf.size() ; i++){
//            Eigen::Matrix4d md(tf.at(i).cast<double>());
//            Eigen::Affine3d affine(md);
//            tf::Transform transform_;
//            tf::transformEigenToTF(affine,transform_);
//            tf::Vector3 vec = transform_.getOrigin();
//            cout << "x: " <<  vec.getZ() << endl;
//            cout << "y: " << -vec.getX() << endl;
//            cout << "z: " << -vec.getY() << endl;
//            cout << endl;
//        }
//    std::cout << "CVFH size = " << returnCloud->size() << std::endl;

    return returnCloud;

}

/*
  A function that extract point from a point cloud.
  param[in] p_cloud the initial point cloud.
  param[in] p_indices the indice you want to extract the the p_cloud.
  param[out] p_cloudOut the cloud that have been extract.
  */

void Object_recognition::pointCloudExtractor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud,
                                             pcl::PointIndices p_indices,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloudOut)
{
    pcl::PointIndicesPtr indicePtr(new pcl::PointIndices);
    *indicePtr = p_indices;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(p_cloud);
    extract.setIndices(indicePtr);
    extract.setNegative(false);
    extract.filter(*p_cloudOut);
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
    *cloud_vfh_ptr = *(calculateCVFH(cloud_us_ptr, cloud_us_normal_ptr, cloud_vfh_tf));

    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > cloud_bd_tf;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud_bd_vfh_ptr (new pcl::PointCloud<pcl::VFHSignature308>);
    *cloud_bd_vfh_ptr = *(calculateCVFH(cloud_us_bd_ptr, cloud_us_bd_normal_ptr, cloud_bd_tf));

    // Merge (ICP)
    transformationMatrix = mergePointCVFH(cloud_vfh_ptr, cloud_bd_vfh_ptr, cloud_us_ptr, cloud_us_bd_ptr);


    //histogramComparison(cloud_vfh_ptr, cloud_bd_vfh_ptr);

    ros::Time end = ros::Time::now();

    pcl::transformPointCloud(*p_ptr_cloud, *transformedPC_ptr, transformationMatrix);
    showPointCloud(transformedPC_ptr);

    std::cout << GREEN << "US Total time CVFH = " << end - begin << RESET
              << std::endl << std::endl;
}


// Returns the index of the histogram corresponding to the recognized object.
// 'guess' variable is updated with transform between input and recognized object;
int Object_recognition::OURCVFHRecognition(pcl::PointCloud<PointT>::Ptr in_pc, FileAPI *fileAPI, Eigen::Matrix4f &trans){

    // Calculate the surface histograms for the input pointcloud
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > surface_transforms;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr surface_histograms = makeCVFH(in_pc,surface_transforms);

    // Get a certain number of the NN surface hypotheses from the object database
    pcl::PointCloud<pcl::VFHSignature308>::Ptr signature_database = fileAPI->getAllHistograms();
    std::vector<std::vector<int> > NN_object_indices = getNNSurfaces(surface_histograms,signature_database,5);

    // Get object hypotheses and initial transforms

    std::vector<ObjectBd> object_hypotheses = fileAPI->retrieveObjectFromHistogram( NN_object_indices.at(1) );

    std::vector<double> whole_PC_scores;
    std::vector<double> whole_PC_times;
    std::vector<Eigen::Matrix4f> whole_PC_transforms;

    std::vector<double> sampled_PC_scores;
    std::vector<double> sampled_PC_times;
    std::vector<Eigen::Matrix4f> sampled_PC_transforms;

    //Sample Pointclouds
    pcl::PointCloud<PointT>::Ptr in_pc_sampled(new pcl::PointCloud<PointT>);
    computeUniformSampling(in_pc,in_pc_sampled);

    // Loop over object hypotheses
    for(int i=0; i < object_hypotheses.size(); i++){
        ObjectBd obj = object_hypotheses.at(i);
        pcl::PointCloud<PointT>::Ptr obj_sampled(new pcl::PointCloud<PointT>);
        computeUniformSampling(obj.getPointCloud(),obj_sampled);

        double smallestScore_fine = 99999;
        Eigen::Matrix4f smallestTransform_fine;
        double smallestScore_coarse = 99999;
        Eigen::Matrix4f smallestTransform_coarse;

        double avgTime_fine = 0;
        double avgTime_coarse = 0;
        int loopIteration = 0;

        // Loop for different initial transforms (only yaw angle)
        for(double j=0; j <= 360; j=j+5){
            tf::Transform initial_guess_tf;
            initial_guess_tf.setIdentity();
            tf::Quaternion quat;
            quat.setRPY(0,0,angles::from_degrees(j));
            initial_guess_tf.setRotation(quat);
            Eigen::Matrix4f initial_guess_matrix;
            pcl_ros::transformAsMatrix(initial_guess_tf,initial_guess_matrix);

            Eigen::Matrix4f fine_transform   = initial_guess_matrix;
            Eigen::Matrix4f coarse_transform = initial_guess_matrix;

            double time_fine   = 0;
            double time_coarse = 0;

            double score_fine   = mergePointCVFH(in_pc,obj.getPointCloud(),fine_transform,time_fine);
            double score_coarse = mergePointCVFH(in_pc_sampled,obj_sampled,coarse_transform,time_coarse);

            avgTime_fine   += time_fine;
            avgTime_coarse += time_coarse;
            loopIteration++;

            if(score_fine < smallestScore_fine){
                smallestScore_fine = score_fine;
                smallestTransform_fine = fine_transform;
            }

            if(score_coarse < smallestScore_coarse){
                smallestScore_coarse = score_coarse;
                smallestTransform_coarse = coarse_transform;
            }
        }

        //avgTime_fine   = avgTime_fine   / loopIteration;
        //avgTime_coarse = avgTime_coarse / loopIteration;

        whole_PC_times.push_back(avgTime_fine);
        whole_PC_scores.push_back(smallestScore_fine);
        whole_PC_transforms.push_back(smallestTransform_fine);

        sampled_PC_times.push_back(avgTime_coarse);
        sampled_PC_scores.push_back(smallestScore_coarse);
        sampled_PC_transforms.push_back(smallestTransform_coarse);

    }

    // Retrieve the object with the best score (Object is recognized)
    double fine_time_avg   = std::accumulate(whole_PC_times.begin(),whole_PC_times.end(),0);
    //fine_time_avg = fine_time_avg / whole_PC_times.size();

    double coarse_time_avg = std::accumulate(sampled_PC_times.begin(),sampled_PC_times.end(),0);
    //coarse_time_avg = coarse_time_avg / sampled_PC_times.size();


    int    fine_smallest_distance_index   = -1;
    int    coarse_smallest_distance_index = -1;
    double fine_smallest_distance         = 99999.0;
    double coarse_smallest_distance       = 99999.0;

    for(int i=0; i < sampled_PC_scores.size(); i++){
        double fine_distance = whole_PC_scores.at(i);
        if(fine_distance < fine_smallest_distance){
            fine_smallest_distance = fine_distance;
            fine_smallest_distance_index = i;
            trans = whole_PC_transforms.at(i);
        }

        double coarse_distance = sampled_PC_scores.at(i);
        if(coarse_distance < coarse_smallest_distance){
            coarse_smallest_distance = coarse_distance;
            coarse_smallest_distance_index = i;
        }
    }

    std::cout << "Object # retrieved with ICP on WHOLE Pointcloud : "   << fine_smallest_distance_index << std::endl;
    std::cout << "Object # retrieved with ICP on SAMPLED Pointcloud : " << coarse_smallest_distance     << std::endl;

    if(fine_smallest_distance < m_rmse_recognition_threshold){
        return NN_object_indices.at(1).at(fine_smallest_distance_index);
    }

    else{
        return -1;
    }
}


// Returns a vector of the form [ObjectIndexFromFineTransform ObjectIndexFromCoarseTransform AverageTimeForFineTransforms AverageTimeForCoarseTransforms]
std::vector<double> Object_recognition::OURCVFHRecognition(pcl::PointCloud<PointT>::Ptr in_pc, std::vector<pcl::PointCloud<PointT>::Ptr> hypotheses){

    std::cout << "There are " << hypotheses.size() << " hypotheses to check for recognition" << std::endl;

    std::vector<double> whole_PC_scores;
    std::vector<double> whole_PC_times;
    std::vector<Eigen::Matrix4f> whole_PC_transforms;

    std::vector<double> sampled_PC_scores;
    std::vector<double> sampled_PC_times;
    std::vector<Eigen::Matrix4f> sampled_PC_transforms;

    //Sample Pointclouds
    pcl::PointCloud<PointT>::Ptr in_pc_sampled(new pcl::PointCloud<PointT>);
    computeUniformSampling(in_pc,in_pc_sampled);

    // Loop over object hypotheses
    for(int i=0; i < hypotheses.size(); i++){
        std::cout << "Looking at object #" << i << std::endl;
        pcl::PointCloud<PointT>::Ptr obj = hypotheses.at(i);
        pcl::PointCloud<PointT>::Ptr obj_sampled(new pcl::PointCloud<PointT>);
        computeUniformSampling(obj,obj_sampled);

        double smallestScore_fine = 99999;
        Eigen::Matrix4f smallestTransform_fine;
        double smallestScore_coarse = 99999;
        Eigen::Matrix4f smallestTransform_coarse;

        double avgTime_fine = 0;
        double avgTime_coarse = 0;
        int loopIteration = 0;

        // Loop for different initial transforms (only yaw angle)
        for(double j=0; j <= 360; j=j+5){
            std::cout << "Guess angles : " << j << std::endl;
            tf::Transform initial_guess_tf;
            initial_guess_tf.setIdentity();
            tf::Quaternion quat;
            quat.setRPY(0,0,angles::from_degrees(j));
            initial_guess_tf.setRotation(quat);
            Eigen::Matrix4f initial_guess_matrix;
            pcl_ros::transformAsMatrix(initial_guess_tf,initial_guess_matrix);

            Eigen::Matrix4f fine_transform   = initial_guess_matrix;
            Eigen::Matrix4f coarse_transform = initial_guess_matrix;

            double time_fine   = 0;
            double time_coarse = 0;

            double score_coarse = mergePointCVFH(in_pc_sampled,obj_sampled,coarse_transform,time_coarse);
            std::cout << "It took " << time_coarse << " seconds to do ICP with sampled pointcloud" << std::endl;

            double score_fine   = mergePointCVFH(in_pc,obj,fine_transform,time_fine);
            std::cout << "It took " << time_fine   << " seconds to do ICP with full pointcloud"    << std::endl;


            avgTime_fine   += time_fine;
            avgTime_coarse += time_coarse;
            loopIteration++;

            if(score_fine < smallestScore_fine){
                smallestScore_fine = score_fine;
                smallestTransform_fine = fine_transform;
            }

            if(score_coarse < smallestScore_coarse){
                smallestScore_coarse = score_coarse;
                smallestTransform_coarse = coarse_transform;
            }
        }

        //avgTime_fine   = avgTime_fine   / loopIteration;
        //avgTime_coarse = avgTime_coarse / loopIteration;

        std::cout << "It took " << avgTime_coarse << " seconds to do ICP with sampled pointclouds of this object" << std::endl;
        std::cout << "It took " << avgTime_fine   << " seconds to do ICP with full pointclouds of this object"    << std::endl;

        whole_PC_times.push_back(avgTime_fine);
        whole_PC_scores.push_back(smallestScore_fine);
        whole_PC_transforms.push_back(smallestTransform_fine);

        sampled_PC_times.push_back(avgTime_coarse);
        sampled_PC_scores.push_back(smallestScore_coarse);
        sampled_PC_transforms.push_back(smallestTransform_coarse);

    }

    // Retrieve the object with the best score (Object is recognized)
    double fine_time_avg = std::accumulate(whole_PC_times.begin(),whole_PC_times.end(),0);
    //fine_time_avg = fine_time_avg / whole_PC_times.size();

    double coarse_time_avg = std::accumulate(sampled_PC_times.begin(),sampled_PC_times.end(),0);
    //coarse_time_avg = coarse_time_avg / sampled_PC_times.size();


    int    fine_smallest_distance_index   = -1;
    int    coarse_smallest_distance_index = -1;
    double fine_smallest_distance         = 99999.0;
    double coarse_smallest_distance       = 99999.0;

    for(int i=0; i < sampled_PC_scores.size(); i++){
        double fine_distance = whole_PC_scores.at(i);
        if(fine_distance < fine_smallest_distance){
            fine_smallest_distance = fine_distance;
            fine_smallest_distance_index = i;
            //trans = whole_PC_transforms.at(i);
        }

        double coarse_distance = sampled_PC_scores.at(i);
        if(coarse_distance < coarse_smallest_distance){
            coarse_smallest_distance = coarse_distance;
            coarse_smallest_distance_index = i;
        }
    }

    //std::cout << "Object # retrieved with ICP on WHOLE Pointcloud : "   << fine_smallest_distance_index << std::endl;
    //std::cout << "Object # retrieved with ICP on SAMPLED Pointcloud : " << coarse_smallest_distance     << std::endl;


    std::vector<double> returnVector;


    if(fine_smallest_distance < m_rmse_recognition_threshold){
        returnVector.push_back(fine_smallest_distance_index);
    }

    else{
        returnVector.push_back(-1.0);
    }

    if(coarse_smallest_distance < m_rmse_recognition_threshold){
        returnVector.push_back(coarse_smallest_distance);
    }

    else{
        returnVector.push_back(-1.0);
    }

    returnVector.push_back(fine_time_avg);
    returnVector.push_back(coarse_time_avg);

    return returnVector;

}


// Returns a matrix of the form :
// Column 0 : Surface index of the input pointcloud
// Column 1 : Histogram index in the database
// Column 2 : Surface index of the object the histogram belongs to (not used)

std::vector<std::vector<int> > Object_recognition::getNNSurfaces(pcl::PointCloud<pcl::VFHSignature308>::Ptr p_cloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd_cloud, int NNnumber)
{
    pcl::KdTreeFLANN<pcl::VFHSignature308>::Ptr kdtree (new pcl::KdTreeFLANN<pcl::VFHSignature308>);

    kdtree->setInputCloud(p_bd_cloud);

    std::vector<int> index;
    std::vector<float> sqrDistance;

    std::vector<int> input_surface_index;
    std::vector<int> output_histogram_index;
    std::vector<int> output_surface_index;

    std::vector<std::vector<int> > memoryIndex;
    //std::vector<std::vector<float> > memoryDistance;


    for(int i = 0; i < p_cloud->size(); i++)
    {
        kdtree->nearestKSearch(p_cloud->at(i), NNnumber, index, sqrDistance);
        for(int j=0; j<index.size(); j++){
            input_surface_index.push_back(i);
            output_histogram_index.push_back(index.at(j));
            //output_surface_index = functionToGetSurfaceIndex();
        }
    }

    memoryIndex.push_back(input_surface_index);
    memoryIndex.push_back(output_histogram_index);
    memoryIndex.push_back(output_surface_index);

    return memoryIndex;
}


/*
  The function will search for the best math for the signature and the signature hold in the bd.
  This function will only return the index found.
  param[in] p_cloud the signature you want to match.
  param[in] p_bd_cloud the cloud that containt the signature db.
  return int the best math index.
  */

int Object_recognition::histogramComparison(pcl::PointCloud<pcl::VFHSignature308>::Ptr p_cloud,
                                            pcl::PointCloud<pcl::VFHSignature308>::Ptr p_bd_cloud)
{
    std::vector<float> returnVector = histogramComparisonVector(p_cloud, p_bd_cloud);
    return returnVector[0];
}


/*
  The function will search for the best math for the signature and the signature hold in the bd.
  param[in] p_cloud the signature you want to match.
  param[in] p_bd_cloud the cloud that containt the signature db.
  return std::vector<float> the firt value is the best math index.  The second is the smallest
  distance.
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
    //computeUniformSampling(p_ptr_cloud, cloud_us_ptr);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_us_normal_ptr (new pcl::PointCloud<pcl::Normal>);
    compute_normal(p_ptr_cloud, cloud_us_normal_ptr);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud_vfh_ptr (new pcl::PointCloud<pcl::VFHSignature308>);
    std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > tf_;
    cloud_vfh_ptr = calculateCVFH(p_ptr_cloud, cloud_us_normal_ptr,tf_);

    return cloud_vfh_ptr;

}

/*
  Its a version with less parameter of makeCVFH mainly used for backward compability.  Its basicaly
  call the makeCVFH with more parameter.
  */
pcl::PointCloud<pcl::VFHSignature308>::Ptr Object_recognition::makeCVFH(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
                                                                        std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf_)
{
    std::vector<Eigen::Vector3f> centroidVec;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> indicesVec;

    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud_vfh_ptr (new pcl::PointCloud<pcl::VFHSignature308>);
    cloud_vfh_ptr = makeCVFH(p_ptr_cloud,
                             tf_,
                             centroidVec,
                             indicesVec);

    return cloud_vfh_ptr;

}

/*
  Its a version with less parameter of makeCVFH mainly used for backward compability.  Its basicaly
  call the makeCVFH with more parameter.
  */
pcl::PointCloud<pcl::VFHSignature308>::Ptr Object_recognition::makeCVFH(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
                                                                        std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf_,
                                                                        std::vector<Eigen::Vector3f>& p_centroid)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> indicesVec;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud_vfh_ptr(new pcl::PointCloud<pcl::VFHSignature308>);
    cloud_vfh_ptr = makeCVFH(p_ptr_cloud,
                             tf_,
                             p_centroid,
                             indicesVec);

    return cloud_vfh_ptr;
}

/*
  Compute the OURVFH for a point cloud.
  param[in] p_ptr_cloud the point cloud you want to compute the signature.
  param[out] tf_ the transform for SGURF that the algorithm return.
  param[out] p_centroid the centroid of every cluster used in the computation.
  param[out] p_surface a vector that containt point cloud.  The point cloud are the different surface
  used in the computation.
  */
pcl::PointCloud<pcl::VFHSignature308>::Ptr Object_recognition::makeCVFH(pcl::PointCloud<PointT>::Ptr p_ptr_cloud,
                                                                        std::vector<Eigen::Matrix4f,Eigen::aligned_allocator<Eigen::Matrix4f> > &tf_,
                                                                        std::vector<Eigen::Vector3f>& p_centroid,
                                                                        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& p_surface)
{
    pcl::PointCloud<PointT>::Ptr cloud_us_ptr(new pcl::PointCloud<PointT>);
    //computeUniformSampling(p_ptr_cloud, cloud_us_ptr);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_us_normal_ptr (new pcl::PointCloud<pcl::Normal>);
    compute_normal(p_ptr_cloud, cloud_us_normal_ptr);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud_vfh_ptr (new pcl::PointCloud<pcl::VFHSignature308>);
    std::vector<pcl::PointIndices> pointIndice_vec;
    cloud_vfh_ptr = calculateCVFH(p_ptr_cloud, cloud_us_normal_ptr,tf_, p_centroid, pointIndice_vec);

    for(int i = 0; i < pointIndice_vec.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointSurface(new pcl::PointCloud<pcl::PointXYZRGB>);
        pointCloudExtractor(p_ptr_cloud, pointIndice_vec.at(i), pointSurface);
        p_surface.push_back(pointSurface);
    }

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


pcl::PointCloud<PointT>::Ptr Object_recognition::transformAndVoxelizePointCloud(pcl::PointCloud<PointT>::Ptr in_source, pcl::PointCloud<PointT>::Ptr in_target, Eigen::Matrix4f in_transform){

    pcl::PointCloud<PointT>::Ptr transformed_pc(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*in_source,*transformed_pc,in_transform);



    pcl::PointCloud<PointT>::Ptr merged_pc(new pcl::PointCloud<PointT>());
    *merged_pc  = *transformed_pc + *in_target;
    //*merged_pc += *in_target;

    pcl::VoxelGrid<PointT> vx_grid;
    vx_grid.setInputCloud (merged_pc);
    vx_grid.setLeafSize (0.005f, 0.005f, 0.005f);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    vx_grid.filter(*cloud_filtered);

    return cloud_filtered;
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
