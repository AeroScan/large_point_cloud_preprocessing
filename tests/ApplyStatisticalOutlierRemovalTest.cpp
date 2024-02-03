#include "LargePointCloudPreprocessing.hpp"

int main(int argc, char** argv)
{
    std::string filename        = TEST_CLOUD;
    std::string out_filename    = "../tests/output_statistical_outlier_removal.pcd";

    LargePointCloudPreprocessing::Params theParams;
    theParams.sor_params = std::vector<double> ({0.2, 1.0});

    LargePointCloudPreprocessing theLargePointCloudPreprocessing(theParams);
   return theLargePointCloudPreprocessing.execute(filename, out_filename);
}