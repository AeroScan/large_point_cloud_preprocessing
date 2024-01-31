#include "LargePointCloudPreprocessing.hpp"

int main(int argc, char** argv)
{
    std::string filename        = TEST_CLOUD;
    std::string out_filename    = "../tests/output_reescale.pcd";

    LargePointCloudPreprocessing::Params theParams;
    theParams.reescale_param = 0.5;

    LargePointCloudPreprocessing theLargePointCloudPreprocessing(theParams);

    return theLargePointCloudPreprocessing.execute(filename, out_filename);
}