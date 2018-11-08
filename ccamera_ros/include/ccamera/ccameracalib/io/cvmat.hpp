#ifndef CE_TYPES_CVMAT_HPP_
#define CE_TYPES_CVMAT_HPP_

#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>

#include <opencv2/core/core.hpp>

#include "io/yaml.hpp"

namespace ce
{


  //! Prologue for cv::Mat for YAML archives
  inline void prologue( YAMLOutputArchive & ar, cv::Mat const & )
  {
    ar.startNode();
    ar.setNextTag("opencv-matrix");
  }

  //! Serializing (save) for cv::Mat
  template <class Archive> inline
  void CEREAL_SAVE_FUNCTION_NAME( Archive & ar, cv::Mat const & mat )
  {
    assert(mat.type() == CV_64F || mat.type() == CV_32F);

    if(mat.type() == CV_64F)
    {
        std::vector<double> data;
        data.assign((double *)mat.data, (double *)mat.data + mat.total());

        ar( CEREAL_NVP_("rows", mat.rows),
            CEREAL_NVP_("cols", mat.cols),
            CEREAL_NVP_("dt", std::string("d")),
            CEREAL_NVP_("data", data) );
    }
    else
    {
        std::vector<float> data;
        data.assign((float *)mat.data, (float *)mat.data + mat.total());

        ar( CEREAL_NVP_("rows", mat.rows),
            CEREAL_NVP_("cols", mat.cols),
            CEREAL_NVP_("dt", std::string("f")),
            CEREAL_NVP_("data", data) );
    }
  }

  //! Serializing (load) for cv::Mat
  template <class Archive> inline
  void CEREAL_LOAD_FUNCTION_NAME( Archive & ar, cv::Mat & mat )
  {
    int rows, cols;
    std::string dt;
    ar( CEREAL_NVP_("rows", rows),
        CEREAL_NVP_("cols", cols),
        CEREAL_NVP_("dt", dt));

    if(dt == "d")
    {
        std::vector<double> data;
        ar( CEREAL_NVP_("data", data));

        mat = cv::Mat(rows, cols, CV_64F, data.data()).clone();
        return;
    }
    else //dt == "f"
    {
        std::vector<float> data;
        ar( CEREAL_NVP_("data", data));

        mat = cv::Mat(rows, cols, CV_32F, data.data()).clone();
        return;
    }

  }

} // namespace ce


#endif // CE_TYPES_CVMAT_HPP_
