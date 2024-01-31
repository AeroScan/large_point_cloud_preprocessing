#include "LargePointCloudPreprocessing.hpp"

int main(int argc, char** argv)
{
    std::string filename        = TEST_CLOUD;
    std::string out_filename    = "../tests/output_statistical_outlier_removal.pcd";

    LargePointCloudPreprocessing::Params theParams;
    theParams.sor_params = std::vector<double> ();

    LargePointCloudPreprocessing theLargePointCloudPreprocessing(theParams);
   return theLargePointCloudPreprocessing.execute(filename, out_filename);
}