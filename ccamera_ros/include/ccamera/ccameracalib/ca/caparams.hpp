#ifndef CALIBRATION_ALGORITHM_PARAMS
#define CALIBRATION_ALGORITHM_PARAMS

#include <cereal/cereal.hpp>

namespace ce {

enum EPatternType
{
    CHESSBOARD
};

enum ECameraType
{
    FISHEYE,
    RECTILINEAR
};

struct CalibrationAlgorithmParams
{
    ECameraType cameraType = FISHEYE;

    struct Pattern {
        EPatternType type = CHESSBOARD;
        uint  width;
        uint  height;
        float cell;

    private:
        friend class cereal::access;

        template<class Archive>
        void serialize(Archive &ar)
        {
            ar( CEREAL_NVP(type),
                CEREAL_NVP(width),
                CEREAL_NVP(height),
                CEREAL_NVP(cell));
        }
    } pattern;

    int smoothingFilterSize = 3;

    bool useCheckerboardQualityFilter = true;

private:
    friend class cereal::access;

    template<class Archive>
    void serialize(Archive &ar)
    {
        ar( CEREAL_NVP(pattern),
            CEREAL_NVP(smoothingFilterSize));
    }
};

}
#endif // CALIBRATION_ALGORITHM_PARAMS
