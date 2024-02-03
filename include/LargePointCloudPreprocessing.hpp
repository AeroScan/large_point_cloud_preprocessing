/**
 * 
 * Refactor by: Pedro Corçaque
 * Date:        30/01/2024
 * 
*/

#pragma once

#define PCL_NO_PRECOMPILE
#include <iostream>
#include <string>
#include <chrono>
#include <limits>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

#include "preprocessing/filters.h"
#include "preprocessing/normal_estimation.h"
#include "preprocessing/normalization.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

class LargePointCloudPreprocessing
{
public:
    struct Params
    {
        bool use_search_surface_param   = true;
        std::vector<double> cb_min;
        std::vector<double> cb_max;
        unsigned int rs_param           = 0;
        std::vector<double> vg_params;
        std::vector<double> sor_params;
        double neomp_param              = 0.0;
        double reescale_param           = 0.0;
        bool centralize_param           = false;
        bool align_param                = false;
        double noise_add_param          = 0.0;
        double cube_reescale_param      = 0.0;
    };
private:
    bool use_search_surface_param   = true;
    std::vector<double> cb_min;
    std::vector<double> cb_max;
    unsigned int rs_param           = 0;
    std::vector<double> vg_params;
    std::vector<double> sor_params;
    double neomp_param              = 0.0;
    double reescale_param           = 0.0;
    bool centralize_param           = false;
    bool align_param                = false;
    double noise_add_param          = 0.0;
    double cube_reescale_param      = 0.0;
private:
    void readParameters(Params theParams);
    void loadPCD(std::string filename, pcl::PCLPointCloud2& cloud);
    void savePCD(const std::string &filename, const pcl::PCLPointCloud2 &output);
public:
    explicit LargePointCloudPreprocessing(Params theParams);

    bool execute(std::string filename, std::string out_filename);
};