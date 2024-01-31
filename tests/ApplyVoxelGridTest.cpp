#include "LargePointCloudPreprocessing.hpp"

int main(int argc, char** argv)
{
    std::string filename        = TEST_CLOUD;
    std::string out_filename    = "../tests/output_voxel_grid.pcd";

    LargePointCloudPreprocessing::Params theParams;
    theParams.vg_params = std::vector<double> (0.1283);

    LargePointCloudPreprocessing theLargePointCloudPreprocessing(theParams);
   return theLargePointCloudPreprocessing.execute(filename, out_filename);
}