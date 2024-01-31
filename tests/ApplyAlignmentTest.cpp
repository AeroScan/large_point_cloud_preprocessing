#include "LargePointCloudPreprocessing.hpp"

int main(int argc, char** argv)
{
    std::string filename        = TEST_CLOUD;
    std::string out_filename    = "../tests/output_alignment.pcd";

    LargePointCloudPreprocessing::Params theParams;
    theParams.align_param = true;

    LargePointCloudPreprocessing theLargePointCloudPreprocessing(theParams);
   return theLargePointCloudPreprocessing.execute(filename, out_filename);
}