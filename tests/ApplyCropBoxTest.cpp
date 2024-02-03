#include "LargePointCloudPreprocessing.hpp"

int main(int argc, char** argv)
{
    std::string filename        = TEST_CLOUD;
    std::string out_filename    = "../tests/output_crop_box.pcd";

    LargePointCloudPreprocessing::Params theParams;
    theParams.cb_min = std::vector<double> ({-1000, -1000, -1000});
    theParams.cb_max = std::vector<double> ({1000, 1000, 1000});

    LargePointCloudPreprocessing theLargePointCloudPreprocessing(theParams);
   return theLargePointCloudPreprocessing.execute(filename, out_filename);
}