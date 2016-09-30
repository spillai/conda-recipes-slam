/* ----------------------------------------------------------------------------
 * Author(s): Sudeep Pillai (spillai@csail.mit.edu)

 * This code wraps:
 *  @inproceedings{olson2011tags,
 *    TITLE      = {{AprilTag}: A robust and flexible visual fiducial system},
 *    AUTHOR     = {Edwin Olson},
 *    BOOKTITLE  = {Proceedings of the {IEEE} International Conference on Robotics and
 *    Automation ({ICRA})},
 *    YEAR       = {2011},
 *    MONTH      = {May},
 *    PAGES      = {3400-3407},
 *    KEYWORDS   = {Robot navigation, SLAM, Visual Fiducial, ARToolkit},
 *    PUBLISHER  = {IEEE},
 *  }
 *
 * -------------------------------------------------------------------------- */

// pybot_types
#include "pybot_types.hpp"
#include "pybot_eigen_types.hpp"
#include "pybot_cv_types.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <boost/thread.hpp>

#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>

#include "wrappers/apriltags_wrapper.hpp"
namespace bot { namespace vision {

class AprilTagsWrapper {
 public:
  AprilTagsWrapper(float tagSize=0, float fx=0, float fy=0, float cx=0, float cy=0) :
      tagDetector_(NULL),
      tagCodes_(AprilTags::tagCodes36h11),
      fx_(fx), fy_(fy), cx_(cx), cy_(cy),
      tagSize_(tagSize)
  {
    tagDetector_ = new AprilTags::TagDetector(tagCodes_);
    set_calib(tagSize, fx, fy, cx, cy);
  }
  
  ~AprilTagsWrapper() {
    delete tagDetector_;
  }

  void set_calib(float tagSize, float fx, float fy, float cx, float cy) {
    tagSize_ = tagSize;
    fx_ = fx, fy_ = fy, cx_ = cx, cy_ = cy;    
  }
  
  std::vector< AprilTag > process(const cv::Mat& img, bool return_poses=false) {

    // Convert to grayscale
    cv::Mat1b gray;
    if (img.channels() == 3)
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else
      gray = img;

    // Detect tags
    std::vector<AprilTags::TagDetection> detections = tagDetector_->extractTags(gray);    
    std::vector<AprilTag> tags(detections.size());
    for (int j=0; j<detections.size(); j++) {
      tags[j].id_ = detections[j].id;
      tags[j].features_ = cv::Mat1f::zeros(4, 2);
      for (int k=0; k<4; k++) {
        tags[j].features_(k,0) = detections[j].p[k].first,
        tags[j].features_(k,1) = detections[j].p[k].second;
      }

      // Optionally output poses
      if (return_poses) {
        if (!fx_ || !fy_)
          throw std::runtime_error("Cannot return poses, invalid calibration!");
      
        // Get Pose estimates
        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        Eigen::Matrix4d T = detections[j].getRelativeTransform(tagSize_,fx_,fy_,cx_,cy_);
        cv::eigen2cv(T, tags[j].pose_);
      }
    }
    return tags;
  }
  
  AprilTags::TagCodes tagCodes_; 
  AprilTags::TagDetector* tagDetector_;

  double tagSize_; // April tag side length in meters of square black frame
  double fx_; // camera focal length in pixels
  double fy_;
  double cx_; // camera principal point
  double cy_;

};
} // namespace vision
} // namespace bot


 namespace py = boost::python;
 namespace bot { namespace python { 

 BOOST_PYTHON_MODULE(pybot_apriltags)
 {
   // Main types export
   bot::python::init_and_export_converters();
   py::scope scope = py::scope();

  // Class def
  py::class_<bot::vision::AprilTag>("AprilTag")
      .def_readonly("id", &bot::vision::AprilTag::id_)
      .def("getFeatures", &bot::vision::AprilTag::getFeatures)
      .def("getPose", &bot::vision::AprilTag::getPose)
      .def("getPoseBody", &bot::vision::AprilTag::getPoseBody)
      .def("setPose", &bot::vision::AprilTag::setPose)
      ;
  expose_template_type<std::vector<bot::vision::AprilTag> >();
  
  // Class def
  py::class_<bot::vision::AprilTagsWrapper>("AprilTagsWrapper")
      .def(py::init<py::optional<float, float, float, float, float> >(
          (py::arg("tag_size")=0, py::arg("fx")=0, py::arg("fy")=0, py::arg("cx")=0, py::arg("cy")=0)))
            .def("process", &bot::vision::AprilTagsWrapper::process,
           (py::arg("img"), py::arg("return_poses")=false))
      .def("set_calib", &bot::vision::AprilTagsWrapper::set_calib,
           (py::arg("tag_size"), py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy")))
      ;

  py::def("draw_tags", &bot::vision::draw_tags);
  py::def("project_tags", &bot::vision::project_tags);

   
}

} // namespace bot
} // namespace python

