/**
 * 
 * Refactor by: Pedro Corçaque
 * Date:        30/01/2024
 * 
*/
#include "LargePointCloudPreprocessing.hpp"

LargePointCloudPreprocessing::LargePointCloudPreprocessing(Params theParams)
{
    readParameters(theParams);
}

bool LargePointCloudPreprocessing::execute(std::string filename, std::string out_filename)
{
    auto start = std::chrono::steady_clock::now();

    pcl::PCLPointCloud2::Ptr cloud_pc2(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr aux_pc2(new pcl::PCLPointCloud2);
    PointCloud<pcl::PointNormal>::Ptr cloud_normal(new PointCloud<pcl::PointNormal>);
    PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new PointCloud<pcl::PointXYZ>);
    PointCloud<pcl::PointXYZ>::Ptr search_xyz(new PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    loadPCD(filename, *cloud_pc2);

    if (use_search_surface_param)
    {
        pcl::fromPCLPointCloud2(*cloud_pc2, *search_xyz);
    }

    as::Filters<pcl::PCLPointCloud2> filters(cb_min, cb_max, rs_param, vg_params, sor_params);
    filters.filter(cloud_pc2);

    bool has_normal = false;
    for(auto it = cloud_pc2->fields.begin(); it != cloud_pc2->fields.end(); it++) 
    {
        if(it->name == "normal_x") 
        {
            has_normal = true;
        }
    }

    pcl::fromPCLPointCloud2(*cloud_pc2, *cloud_xyz);
    if (!use_search_surface_param) 
    {
        search_xyz = cloud_xyz;
    }
    as::NormalEstimation<pcl::PointXYZ> ne(neomp_param);

    if (ne.compute(cloud_xyz, search_xyz, normals)) 
    {
        if (use_search_surface_param) 
        {
            search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        }
        pcl::concatenateFields(*cloud_xyz, *normals, *cloud_normal);
        normals.reset(new pcl::PointCloud<pcl::Normal>);

        has_normal = true;
    }
    else if (has_normal) 
    {
        if (use_search_surface_param) 
        {
            search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        }
        pcl::fromPCLPointCloud2(*cloud_pc2, *cloud_normal);
    }
    else 
    {
        if (use_search_surface_param) 
        {
            search_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
        }
        pcl::fromPCLPointCloud2(*cloud_pc2, *cloud_xyz);
    }

    if (has_normal) 
    {
        as::Normalization<pcl::PointNormal> normalization(reescale_param, centralize_param, align_param, noise_add_param, cube_reescale_param);
        normalization.normalize(cloud_normal);
        pcl::toPCLPointCloud2(*cloud_normal, *aux_pc2);
        cloud_normal.reset(new pcl::PointCloud<pcl::PointNormal>);
    }
    else 
    {
        as::Normalization<pcl::PointXYZ> normalization(reescale_param, centralize_param, align_param, noise_add_param, cube_reescale_param);
        normalization.normalize(cloud_xyz);
        pcl::toPCLPointCloud2(*cloud_xyz, *aux_pc2);
        cloud_xyz.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    pcl::PCLPointCloud2::Ptr out_pc2(new pcl::PCLPointCloud2);

    concatenateFields(*cloud_pc2, *aux_pc2, *out_pc2);
    cloud_pc2.reset(new pcl::PCLPointCloud2);
    aux_pc2.reset(new pcl::PCLPointCloud2);

    savePCD(out_filename, *out_pc2);

    auto end = std::chrono::steady_clock::now();
    print_info("\n\nThe overall process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end - start).count());

    return EXIT_SUCCESS;
}

void LargePointCloudPreprocessing::savePCD(const std::string &filename, const pcl::PCLPointCloud2 &output)
{
    auto start_local = std::chrono::steady_clock::now();
    print_info("\n\nWriting Point Cloud...\n");
    PCDWriter w;
    w.writeBinaryCompressed (filename, output);
    auto end_local = std::chrono::steady_clock::now();
    print_info("The writing process took: "); print_value("%lf sec\n", static_cast<std::chrono::duration<double>>(end_local - start_local).count());
}

void LargePointCloudPreprocessing::loadPCD(std::string filename, pcl::PCLPointCloud2& cloud)
{
    auto start_local = std::chrono::steady_clock::now();
    print_info ("\nReading Point Cloud...\n");
    loadPCDFile(filename, cloud);
    print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList(cloud).c_str ());
    print_info("PointCloud before filtering: "); print_value("%d data points\n", cloud.width * cloud.height);
    auto end_local = std::chrono::steady_clock::now();
    print_info("The reading process took: "); print_value("%f sec\n", static_cast<std::chrono::duration<double>>(end_local - start_local).count());
}

void LargePointCloudPreprocessing::readParameters(Params theParams)
{
    /* To not use the original cloud as search surface for normal estimation */
    use_search_surface_param = theParams.use_search_surface_param;

    /* Minimum x, y and z values to use. */
    cb_min = theParams.cb_min;
    if (cb_min.size() != 3)
    {
        if (!cb_min.empty())
        {
            std::string theError = "Cut off minimum must be specified with 3 numbers (" + std::to_string(cb_min.size()) + " given).\n";
            throw new std::length_error(theError);
        }
    }

    /* Maximum x, y and z values to use. */
    cb_max = theParams.cb_max;
    if (cb_max.size() != 3)
    {
        if (!cb_max.empty())
        {
            std::string theError = "Cut off maximum must be specified with 3 numbers ("+ std::to_string(cb_max.size()) +" given).\n";
            throw new std::length_error(theError);
        }
    }

    /* Number of points to random sample in the input point cloud. */
    rs_param = theParams.rs_param;

    /* Leaf size for voxel grid for x, y and z cordinates, if just leaf is passed, it is used for all cordinates. */
    vg_params = theParams.vg_params;
    if (vg_params.size() == 1)
    {
        vg_params = std::vector<double>(3, vg_params[0]);
    }
    else if (vg_params.size() != 3)
    {
        if (!vg_params.empty())
        {
            std::string theError = "Voxel Grid leaf size must be specified with either 1 or 3 numbers ("+ std::to_string(vg_params.size()) +" given).\n";
            throw new std::length_error(theError);
        }
    }

    /* Mean and std for a statistical outlier removal filter. */
    sor_params = theParams.sor_params;
    if (sor_params.size()!= 2)
    {
        if (!sor_params.empty())
        {
            std::string theError = "Statistical outlier removal maximum must be specified with 2 numbers ("+ std::to_string(sor_params.size()) +" given).\n";
            throw new std::length_error(theError);
        }
    }

    /* Radius of the sphere used to estimate the normal of each point. */
    neomp_param = theParams.neomp_param;

    /* Factor that will reescale all the points (change measurament unity). */
    reescale_param = theParams.reescale_param;

    /* Use it to put the origin of the pointcloud at the geometric center of the points. */
    centralize_param = theParams.centralize_param;

    /* Use it to aling the x axis of the coordinate system with the axis of minor variation on point cloud. */
    align_param = theParams.align_param;

    /* Limit of a random uniform noise applied at the normal direction for each point. */
    noise_add_param = theParams.noise_add_param;

    /* Make all the point cloud lies in a cube of edge size equal factor. */
    cube_reescale_param = theParams.cube_reescale_param;
}