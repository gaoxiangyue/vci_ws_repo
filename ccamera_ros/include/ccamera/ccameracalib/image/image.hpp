#ifndef IMAGE
#define IMAGE

#include <memory>
#include <vector>
#include <set>

#include <opencv2/core/core.hpp>

#include "bitmap.hpp"

namespace ce {

struct CoverageReport
{
    enum GridType
    {
        SPHERICAL,
        RECTANGULAR
    } gridType;

    double validFOV;
    std::vector<std::vector<double>> coverageHistogram;
    std::vector<std::vector<cv::Point2f>> pointGrid2d;
    std::vector<std::vector<cv::Point2f>> pointGrid2dUn;

    std::vector<cv::Point2f> features;
    std::vector<cv::Point2f> featuresUn;

    CoverageReport(GridType t, const double& fov, const std::vector<std::vector<double>>& hist);
};

struct ImageQualityReport
{
    static const float DISTORTED_CELL_AREA_T;       // minimum distorted cell area
    static const uchar CONTRAST_RANGE_THRESH;       // minimum white and black difference
    static const int   FEATURES_PER_AREA_UNIT;      // minimum features density

    //TODO: move it somewhere else
    static const int   MINIMUM_CALIBRATION_IMAGES;  // minimum calibration images required for acceptable result
    static const float MINIMUM_COVERAGE_RATE;       // minimum image area which has to be covered

    bool enoughDistance;                            // reflects checkerboard cell's size on an image
    bool enoughContrast;                            // reflects feature local contrast

    std::shared_ptr<CoverageReport> coverage;       // holds coverage histogram
};

struct ImageInfo
{
    ImageInfo();
    std::string path;

    std::vector<cv::Point2f> features;
    std::vector<cv::Point2f> featuresUn;

    std::vector<cv::Point2f> featuresRep;
    std::vector<cv::Point2f> featuresRepUn;

    double reprojectionError;
    double reconstructionError;
    ImageQualityReport qualityReport;
};

class  Image;
typedef std::vector<std::shared_ptr<Image>> ImageList;
typedef std::set<std::shared_ptr<Image>>    ImageSet;

class Image
{
public:
    Image(Bitmap bmp);
    Image(Bitmap bmp, const std::string &path);
    Image(Bitmap bmp, ImageInfo info);

    Image clone();

    cv::Mat asCvMat();

    ImageInfo &info();

    int save(const std::string &path);

    int width()                  { return mBmp.width(); }
    int height()                 { return mBmp.height(); }
    int bitDepth()               { return mBmp.bitDepth(); }
    std::shared_ptr<char> data() { return mBmp.data(); }

protected:
    Bitmap    mBmp;

    ImageInfo mInfo;
};


}
#endif // IMAGE
