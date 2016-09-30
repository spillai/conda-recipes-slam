// Author(s): Sudeep Pillai (spillai@csail.mit.edu)
// License: MIT

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

// pybot_types
#include "pybot_types.hpp"

// Apriltags
#include "wrappers/apriltags_wrapper.hpp"

// gtsam
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

// Vertigo for MaxMix, and Switchable Constraints
#include <vertigo/betweenFactorMaxMix.h>
#include <vertigo/betweenFactorSwitchable.h>
#include <vertigo/switchVariableLinear.h>
#include <vertigo/switchVariableSigmoid.h>

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the structure-from-motion problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Projection factors to model the camera's landmark observations.
// Also, we will initialize the robot at some location using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

namespace py = boost::python;
namespace gt = gtsam;
namespace NM = gt::noiseModel;

namespace bot { namespace vision { 

// ====================================================================
// TAGCORNERS
// ====================================================================

TagCorners toTagCorners(gt::Point2 bl, gt::Point2 br, gt::Point2 tr, gt::Point2 tl) // : bl(bl), br(br), tr(tr), tl(tl) {}
{
  Eigen::VectorXd vec(8);
  vec << bl.vector(), br.vector(), tr.vector(), tl.vector();
  return TagCorners(vec);
}

// ====================================================================
// GenericObjectProjectionFactor
// ====================================================================

template<class POSE, class LANDMARK, class CALIBRATION = gt::Cal3_S2>
class GenericObjectProjectionFactor: public gt::NoiseModelFactor2<POSE, LANDMARK> {
 protected:

  // Keep a copy of measurement and calibration for I/O
  Eigen::VectorXd measured_;                    ///< 2D * 4 measurements
  boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object
  boost::optional<POSE> body_P_sensor_; ///< The pose of the sensor in the body frame
  float tag_size_;  ///< shared pointer to calibration object

 public:

  // typedef LANDMARK Measurement;                                 ///< typedef for the measurement

  /// shorthand for base class type
  typedef gt::NoiseModelFactor2<POSE, LANDMARK> Base;

  /// shorthand for this class
  typedef GenericObjectProjectionFactor<POSE, LANDMARK, CALIBRATION> This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  GenericObjectProjectionFactor() {} // : throwCheirality_(false), verboseCheirality_(false) {}


  /**
   * Constructor
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation
   * @param pose_key is the index of the camera
   * @param landmark_key is the index of the landmark
   * @param K shared pointer to the constant calibration
   * @param body_P_sensor is the transform from body to sensor frame (default identity)
   */
  GenericObjectProjectionFactor(const Eigen::VectorXd& measured, const gt::SharedNoiseModel& model,
                                gt::Key pose_key, gt::Key landmark_key, const boost::shared_ptr<CALIBRATION>& K,
                                const float& tag_size, boost::optional<POSE> body_P_sensor = boost::none) :
      Base(model, pose_key, landmark_key),
      measured_(measured), K_(K), tag_size_(tag_size), body_P_sensor_(body_P_sensor) {

    std::cout << "model: " << model->dim() << std::endl;
    // throwCheirality_(false), verboseCheirality_(false) {}
    
    
  } 
  /// Evaluate error h(x)-z and optionally derivatives
  gt::Vector evaluateError(const POSE& pose, const LANDMARK& measurement,
                           boost::optional<gt::Matrix&> H1 = boost::none, boost::optional<gt::Matrix&> H2 = boost::none) const {
    // try {
      // if(body_P_sensor_) {
      //   if(H1) {
      //     gt::Matrix H0;
      //     PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_, H0), *K_);
      //     Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
      //     *H1 = *H1 * H0;
      //     return reprojectionError.vector();
      //   } else {
      //     PinholeCamera<CALIBRATION> camera(pose.compose(*body_P_sensor_), *K_);
      //     Point2 reprojectionError(camera.project(point, H1, H2) - measured_);
      //     return reprojectionError.vector();
      //   }
      // } else {
    
    // measurement.ominus(pose)
    gt::PinholeCamera<CALIBRATION> camera(pose.between(measurement), *K_);

    double h = tag_size_/2;

    gt::Point2 p1, p2, p3, p4;
    p1 = camera.project(gt::Point3(-h,-h,0), H1, H2);
    p2 = camera.project(gt::Point3(h,-h,0), H1, H2);
    p3 = camera.project(gt::Point3(h,h,0), H1, H2);
    p4 = camera.project(gt::Point3(-h,h,0), H1, H2);
         
    Eigen::VectorXd predicted(8);
    predicted << p1.x(), p1.y(), p2.x(), p2.y(), p3.x(), p3.y(), p4.x(), p4.y();
    
    Eigen::VectorXd err = predicted - measured_;
    std::cerr << " shape: " << predicted.size() << " " << measured_.size() << std::endl;
    std::cerr << "err: " << predicted << " " << measured_ << std::endl;
    
    return err;
  }

      //     // }
  //   } catch( CheiralityException& e) {
  //     if (H1) *H1 = zeros(2,6);
  //     if (H2) *H2 = zeros(2,3);
  //     if (verboseCheirality_)
  //       std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
  //           " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
  //     if (throwCheirality_)
  //       throw e;
  //   }
  //   return ones(2) * 2.0 * K_->fx();
  // }
  
  // //Overrides the virtual clone() method in the NonlinearFactor base class.
  // boost::shared_ptr<NonlinearFactor> clone() const;
  
  // void print(const std::string& s= " ", const gt::KeyFormatter& keyFormatter= gt::DefaultKeyFormatter) const;
  
  
  ~GenericObjectProjectionFactor()
  {
    //Nothing to do here
  }
  
};

// ====================================================================
// GTSAMWrapper
// ====================================================================

class GTSAMWrapper {
 public:
  
  const NM::Base::shared_ptr priorNoise_ = NM::Isotropic::Sigma(6, 0.01);
  const NM::Base::shared_ptr odoNoise_ = NM::Isotropic::Sigma(6, 0.01);
  const NM::Base::shared_ptr measurementNoise_ = NM::Isotropic::Sigma(6, 0.4);

  // Define the camera observation noise model
  // const NM::Base::shared_ptr measurementNoise_ = gtsam::noiseModel::Gaussian::SqrtInformation(1 * gtsam::eye(8,8));
  
  GTSAMWrapper(bool incremental=true):
      incremental_(incremental) {
    gt::ISAM2Params params;

    params.optimizationParams = gt::ISAM2DoglegParams();
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 10;
    params.enablePartialRelinearizationCheck = true;
      
    // Properties prop;
    // prop.mod_update = 1;
    // prop.mod_batch = 10;
    // prop.method = DOG_LEG; // LEVENBERG_MARQUARDT;
    
    slam_ = boost::shared_ptr<gtsam::ISAM2>(new gt::ISAM2(params));
    idx_ = -1;
  }
  
  GTSAMWrapper(const GTSAMWrapper& slam) {
  }
  
  ~GTSAMWrapper() {
  }

  int latest() const { return idx_; }
  int index() const { return idx_; }

  // save factor graph as graphviz dot file
  void saveGraph(const std::string& fn) {
    slam_->saveGraph(fn);
  }
  
  void set_calib(const Eigen::Matrix<double, 3,4>& K) {
    K_ = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(K(0,0), K(1,1), 0.0, K(0,2), K(1,2)));
  }

  // Add odometry incrementally (initialize if not previously done)
  int add_odom_incremental(const gt::Pose3& delta) {

    if (this->latest() < 0)
    { 
      // std::cerr << "Initializing factor graph" << std::endl;

      // Add prior on first pose
      gt::Pose3 pose0;
      graph_.push_back(gt::PriorFactor<gt::Pose3>(gt::Symbol('x', 0), pose0, priorNoise_));
      initial_.insert(gt::Symbol('x', 0), pose0);
      prev_pose_ = pose0;
      idx_ = 0;
    }     

    // add odometry factor
    // std::cerr << "adding: x" << this->latest() << " -> x" << this->latest()+1 << std::endl;
    graph_.push_back(gt::BetweenFactor<gt::Pose3>(gt::Symbol('x', this->latest()),
                                                  gt::Symbol('x', this->latest()+1), delta, odoNoise_));
      
    // predict pose and add as initial estimate
    gt::Pose3 predictedPose = prev_pose_.compose(delta);
    initial_.insert(gt::Symbol('x', this->latest()+1), predictedPose);
    prev_pose_ = predictedPose;
    
    idx_++;
  
    return this->latest();
  }

  int add_odom_incremental(const Eigen::Matrix4d& delta) {
    return add_odom_incremental(gt::Pose3(delta));
  }

  // Add landmark observation to the last instantiated pose 
  int add_observation_incremental(int id, const gt::Pose3& delta) {

    // std::cerr << "adding landmark: x" << this->latest() << " -> l" << id << std::endl;
    // Add Pose3d->Pose3d pose-landmark factor 
    gt::Key x_id = gt::Symbol('x', this->latest());
    gt::Key l_id = gt::Symbol('l', id);

    // Add landmark pose 
    graph_.push_back(gt::BetweenFactor<gt::Pose3>(x_id, l_id, delta, measurementNoise_));

    // Add to landmark measurements
    landmark_edges_.push_back(std::pair<int,int>(this->latest(), id));
    
    // Predict pose and add as initial estimate
    if (landmarks_.find(id) == landmarks_.end()) { 
      gt::Pose3 predictedPose = prev_pose_.compose(delta);
      initial_.insert(l_id, predictedPose);
      landmarks_.insert(id);
    }
    
    return this->latest();
  }

  int add_observation_incremental(int id, const Eigen::Matrix4d& delta) {
    return add_observation_incremental(id, gt::Pose3(delta));
  }

  // Add landmark observation to the last instantiated pose 
  int add_observation(int xid, int lid, const gt::Pose3& delta) {
    
    // Add Pose3d->Pose3d pose-landmark factor 
    gt::Key x_id = gt::Symbol('x', xid);
    gt::Key l_id = gt::Symbol('l', lid);

    // Add landmark pose 
    graph_.push_back(gt::BetweenFactor<gt::Pose3>(x_id, l_id, delta, measurementNoise_));

    // Add to landmark measurements
    landmark_edges_.push_back(std::pair<int,int>(xid, lid));
    
    // Predict pose and add as initial estimate
    if (landmarks_.find(lid) == landmarks_.end()) { 
      gt::Pose3 predictedPose = prev_pose_.compose(delta);
      initial_.insert(l_id, predictedPose);
      landmarks_.insert(lid);
    }
    
    return this->latest();
  }

  int add_observation(int xid, int lid, const Eigen::Matrix4d& delta) {
    return add_observation(xid, lid, gt::Pose3(delta));
  }

  
  
  // int add_prior(uint32_t idx=-1, const gt::Pose3& pose0 = gt::Pose3()) {
  //   // Add prior on first pose
  //   std::cerr << "Initializing with " << idx << std::endl;
  //   graph_.push_back(gt::PriorFactor<gt::Pose3>(gt::Symbol('x', idx), pose0, priorNoise_));
  //   initial_.insert(gt::Symbol('x', idx), pose0);
  //   prev_pose_ = pose0;
  // }
  
  // int add_odom_relative(uint32_t from_idx=-1, uint32_t to_idx=-1, const gt::Pose3& delta = gt::Pose3()) {
  // }    
  //   if (idx_ == 0) {
  //     add_prior(idx_);
  //   }
  //   else
  //   {
  //     // add odometry factor
  //     // std::cerr << "adding: x" << idx_-1 << " -> x" << idx_ << std::endl;
  //     graph_.push_back(gt::BetweenFactor<gt::Pose3>(gt::Symbol('x', idx_-1), gt::Symbol('x', idx_), delta, odoNoise_));
      
  //     // predict pose and add as initial estimate
  //     gt::Pose3 predictedPose = prev_pose_.compose(delta);
  //     initial_.insert(gt::Symbol('x', idx_), predictedPose);
  //     prev_pose_ = predictedPose;
      
  //   }

  //   idx_++;
    
  //   return idx_ - 1;
  // }

  // void add_observation(int32_t id, const Eigen::Matrix4d& delta, const gt::Point2& p) {
  //   // Add Pose3d->Point2 pose-landmark factor 
  //   gt::Key x_id = gt::Symbol('x', idx_-1);
  //   gt::Key l_id = gt::Symbol('l', id);

  //   // Add landmark pose 
  //   graph_.push_back(gt::BetweenFactor<gt::Pose3>(x_id, l_id, gt::Pose3(delta), measurementNoise_));
    
  //   // Predict pose and add as initial estimate
  //   if (landmarks_.find(id) == landmarks_.end()) { 
  //     gt::Pose3 predictedPose = prev_pose_.compose(gt::Pose3(delta));
  //     initial_.insert(l_id, predictedPose);
  //     landmarks_.insert(id);
  //   }

  // }

  
  int add_tag_incremental(int32_t tag_id, const Eigen::Matrix4d& delta, const TagCorners& tag, const float& tag_size) {

    // POSE-TAG2D CONSTRAINTS (REPROJECTION ERROR)
    // Create the set of ground-truth landmarks
    double s = tag_size/2;
    std::vector<gt::Point3> points;
    points.push_back(gt::Point3(s,s,0));
    points.push_back(gt::Point3(s,-s,0));
    points.push_back(gt::Point3(-s,-s,0));
    points.push_back(gt::Point3(-s,s,0));
    
    // Add factors for each landmark observation
    for (size_t j = 0; j < points.size(); ++j) {
      gt::SimpleCamera camera(prev_pose_, *K_);
      gt::Point2 measurement = camera.project(points[j]);
      graph_.push_back(
          gt::GenericProjectionFactor
          <gt::Pose3, gt::Point3, gt::Cal3_S2>(measurement, measurementNoise_,
                                               gt::Symbol('x', this->latest()), gt::Symbol('l', 4 * tag_id + j), K_));

      // Add to landmark measurements
      landmark_edges_.push_back(std::make_pair<int, int>(this->latest(), 4 * tag_id + j));
    }
    
    // Add initial guesses to all observed landmarks
    for (size_t j = 0; j < points.size(); ++j) {
      Eigen::Vector4d v; v << points[j].x(), points[j].y(), points[j].z(), 1;
      Eigen::Vector4d vt = (prev_pose_.compose(gt::Pose3(delta))).matrix() * v;
      initial_.insert(gt::Symbol('l', 4 * tag_id + j), gt::Point3(vt(0), vt(1), vt(2)));
    }

    // gt::Key x_id = gt::Symbol('x', idx_-1);
    // gt::Key l_id = gt::Symbol('l', id);

    // // std::cerr << measurementNoise_ << " " << tag.vector() << std::endl;
    // graph_.push_back(GenericObjectProjectionFactor<gt::Pose3, gt::Pose3>
    //                  (tag.vector(), measurementNoise_, x_id, l_id, K_, tag_size));
    
    
    return this->latest();
  }

  // void add_observation(int32_t id, const cv::Point2f& p) {
  //   add_observation(id, gt::Point2(p.x, p.y));
  // }

  
  void update() {
    if (this->latest() < 1)
      return;
    
    // Update iSAM with the new factors
    slam_->update(graph_, initial_);

    // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
    // If accuracy is desired at the expense of time, update(*) can be called additional times
    // to perform multiple optimizer iterations every step.
    slam_->update();

    // Update poses, and targets
    gt::Values current = slam_->calculateEstimate();

    targets_.clear();
    poses_.clear();
    BOOST_FOREACH(const gt::Values::ConstKeyValuePair& kv, current) {
      const gt::Pose3* pose3D = dynamic_cast<const gt::Pose3*>(&kv.value);
      PoseID3d p(gt::symbolIndex(kv.key), pose3D->matrix());

      // Update optimized robot poses
      if (gt::symbolChr(kv.key) == 'x') {
        poses_.push_back(p);

        // // Update prev_pose
        // if (gt::symbolIndex(kv.key) == this->latest())
        //   prev_pose_ = *pose3D;
      }


      // Update optimized landmarks
      if (gt::symbolChr(kv.key) == 'l') {
        targets_.push_back(p);
      }
    }
    
    // Clear the factor graph and values for the next iteration
    graph_.resize(0);
    initial_.clear();
  }

  std::vector<std::pair<int, int> > getLandmarkEdges() {
    return landmark_edges_;
  }
  
  std::vector<PoseID3d> getPoses() {
    return poses_;
  }

  std::vector<PoseID3d> getTargets() {
    return targets_;
  }

 private:

  // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
  // and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
  // structure is available that allows the user to set various properties, such as the relinearization threshold
  // and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
  // will approach the batch result.
  boost::shared_ptr<gt::ISAM2> slam_;

  // Create a Factor Graph and Values to hold the new data
  gt::NonlinearFactorGraph graph_;
  gt::Values initial_;
  std::vector<PoseID3d> poses_;
  std::vector<PoseID3d> targets_;
  std::vector<std::pair<int, int> > landmark_edges_;
  std::set<size_t> landmarks_;
  
  // Define the camera calibration parameters
  gt::Cal3_S2::shared_ptr K_; 

  gt::Pose3 prev_pose_;

  bool incremental_;
  int idx_;
};

// ====================================================================
// GTSAMTags
// ====================================================================

class GTSAMTags {
 public: 
  GTSAMTags(const float tag_size=0.166)
      : tag_size_(tag_size){}

  GTSAMTags(const GTSAMTags& isam_tags) {
  }

  void set_calib(const Eigen::Matrix<double, 3, 4>& K) {
    vslam_.set_calib(K); 
  }
  
  ~GTSAMTags() {
  }
  
  void saveGraph(const std::string& fn) {
    // save factor graph as graphviz dot file
    vslam_.saveGraph(fn);
  }
  
  int on_tags(double timestamp, const std::vector<AprilTag>& tags) {
    
    // iSAM
    // int pose_id = vslam_.add_pose(timestamp);

    // Add all detections 
    for (size_t k=0; k<tags.size(); ++k) {

      Eigen::Matrix4d T;
      cv::cv2eigen(tags[k].getPose(), T);
      
      // add to iSAM
      // convert from camera pose to robot pose
      gt::Pose3 delta(T);

      // cv::Mat1f pts = tags[k].getFeatures();
      // TagCorners corners = toTagCorners(gtsam::Point2(pts(0,0), pts(0,1)),
      //                                   gtsam::Point2(pts(1,0), pts(1,1)),
      //                                   gtsam::Point2(pts(2,0), pts(2,1)),
      //                                   gtsam::Point2(pts(3,0), pts(3,1)));
      // vslam_.add_observation(tags[k].getId(), delta) ; // , corners, tag_size_);

      // Add observations as 3D pose
      vslam_.add_observation_incremental(tags[k].getId(), delta);

    }

    // iSAM
    vslam_.update();

    return vslam_.latest();
  }

  int on_pose_ids(double timestamp, const std::vector<int>& ids, const std::vector<Eigen::Matrix4d>& poses) {
    
    // iSAM
    // int pose_id = vslam_.add_pose(timestamp);

    // Add all detections 
    for (size_t k=0; k<poses.size(); ++k) {
      
      // add to iSAM
      gt::Pose3 delta(poses[k]);

      // Add observations as 3D pose
      vslam_.add_observation_incremental(ids[k], delta);

    }

    // iSAM
    vslam_.update();

    return vslam_.latest();
  }

  int on_odom(double timestamp, const Eigen::Matrix4d& odom) {

    // iSAM
    vslam_.add_odom_incremental(gt::Pose3(odom));

    // iSAM
    vslam_.update();

    return vslam_.latest();
  }
  
  std::vector<PoseID3d> getPoses() {
    return vslam_.getPoses(); 
  }

  std::vector<PoseID3d> getTargets() {
    return vslam_.getTargets(); 
  }

  std::vector<std::pair<int, int> > getLandmarkEdges() {
    return vslam_.getLandmarkEdges(); 
  }

  // std::vector<PoseID3d> getTargets() {
  //   std::vector<PoseID3d> targets;
  //   for (std::map<int32_t, isam::Pose3d_Node*>::const_iterator it = vslam_.targets().begin();
  //        it != vslam_.targets().end(); it++) {
  //     PoseID3d p(it->first, fromPose3d(it->second->value()));
  //     targets.push_back(p);
  //   }
  //   return targets;
  // }

  void update() {
    vslam_.update();
  }
  
 private:
  GTSAMWrapper vslam_;

  float tag_size_;
};

}
}

namespace bot { namespace python { 

// Key - size_t
// SharedNoiseModel - shared_ptr<gt::noiseModel::Base>

#define DEFINE_DERIVED_VALUE(NAME, TYPE)                \
  py::class_<gt::DerivedValue<TYPE>,                    \
             py::bases<gt::Value>,                      \
             boost::noncopyable >(NAME, py::no_init)    \
  // .def("get", &gt::DerivedValue<TYPE>::operator())   \
  ;                                                     \


BOOST_PYTHON_MODULE(pybot_gtsam)
{
  // Main types export
  bot::python::init_and_export_converters();
  py::scope scope = py::scope();

  // ====================================================================
  // GTSAMWrapper
  py::class_<bot::vision::GTSAMWrapper>("GTSAMWrapper")
      .def(py::init<py::optional<bool> >((py::arg("incremental")=true)))
      // .def("update", &bot::vision::GTSAMWrapper::update)
      // .def("on_tags", &bot::vision::GTSAMWrapper::on_tags)
      // .def("on_pose_ids", &bot::vision::GTSAMWrapper::on_pose_ids)
      // .def("on_odom", &bot::vision::GTSAMWrapper::on_odom)
      // .def("set_calib", &bot::vision::GTSAMWrapper::set_calib)
      // .def("get_targets", &bot::vision::GTSAMWrapper::getTargets)
      // .def("get_poses", &bot::vision::GTSAMWrapper::getPoses)
      // .def("save_graph", &bot::vision::GTSAMWrapper::saveGraph)
      ;
  
  // GTSAMTags
  py::class_<bot::vision::GTSAMTags>("GTSAMTags")
      .def(py::init<py::optional<float> >((py::arg("tag_size")=0.166)))
      .def("update", &bot::vision::GTSAMTags::update)
      .def("on_tags", &bot::vision::GTSAMTags::on_tags)
      .def("on_pose_ids", &bot::vision::GTSAMTags::on_pose_ids)
      .def("on_odom", &bot::vision::GTSAMTags::on_odom)
      .def("set_calib", &bot::vision::GTSAMTags::set_calib)
      .def("get_targets", &bot::vision::GTSAMTags::getTargets)
      .def("get_poses", &bot::vision::GTSAMTags::getPoses)
      .def("get_landmark_edges", &bot::vision::GTSAMTags::getLandmarkEdges)
      .def("save_graph", &bot::vision::GTSAMTags::saveGraph)
      ;



  // --------------------------------------------------------------------
  // Switchable Constraints for Pose2, Pose3

  DEFINE_DERIVED_VALUE("SwitchVariableLinear", vertigo::SwitchVariableLinear);
  DEFINE_DERIVED_VALUE("SwitchVariableSigmoid", vertigo::SwitchVariableSigmoid);

  // PriorFactos for SwitchVariableLinear, SwitchVariableSigmoid
  py::class_<gt::NoiseModelFactor1<vertigo::SwitchVariableLinear>,
             py::bases<gt::NoiseModelFactor>, 
             boost::shared_ptr<gt::NoiseModelFactor1<vertigo::SwitchVariableLinear> >,
             boost::noncopyable > 
      ("NoiseModelFactor1SwitchVariableLinear", py::no_init)
      ;

  py::class_<gt::PriorFactor<vertigo::SwitchVariableLinear>,
             py::bases<gt::NoiseModelFactor1<vertigo::SwitchVariableLinear> >, 
             boost::shared_ptr<gt::PriorFactor<vertigo::SwitchVariableLinear> >,
             boost::noncopyable> 
      ("PriorFactorSwitchVariableLinear",
       py::init<gt::Key, vertigo::SwitchVariableLinear, gt::SharedNoiseModel>())
      ;

  
  py::class_<gt::NoiseModelFactor1<vertigo::SwitchVariableSigmoid>,
             py::bases<gt::NoiseModelFactor>, 
             boost::shared_ptr<gt::NoiseModelFactor1<vertigo::SwitchVariableSigmoid> >,
             boost::noncopyable > 
      ("NoiseModelFactor1SwitchVariableSigmoid", py::no_init)
      ;

  py::class_<gt::PriorFactor<vertigo::SwitchVariableSigmoid>,
             py::bases<gt::NoiseModelFactor1<vertigo::SwitchVariableSigmoid> >, 
             boost::shared_ptr<gt::PriorFactor<vertigo::SwitchVariableSigmoid> >,
             boost::noncopyable> 
      ("PriorFactorSwitchVariableSigmoid",
       py::init<gt::Key, vertigo::SwitchVariableSigmoid, gt::SharedNoiseModel>())
      ;


  // BetweenFactors for SwitchVariableLinear, SwitchVariableSigmoid
  py::class_<gt::NoiseModelFactor3<gt::Pose2, gt::Pose2,
                                   vertigo::SwitchVariableLinear>,
             py::bases<gt::NoiseModelFactor>, 
             boost::shared_ptr<gt::NoiseModelFactor3<gt::Pose2, gt::Pose2,
                                                     vertigo::SwitchVariableLinear> >,
             boost::noncopyable > 
      ("NoiseModelFactor2Pose2Pose2SwitchVariableLinear", py::no_init)
      ;

  py::class_<vertigo::BetweenFactorSwitchableLinear<gt::Pose2>,
             py::bases<gt::NoiseModelFactor3<gt::Pose2, gt::Pose2,
                                             vertigo::SwitchVariableLinear> >, 
             boost::shared_ptr<vertigo::BetweenFactorSwitchableLinear<gt::Pose2> >,
             boost::noncopyable> 
      ("BetweenFactorSwitchableLinearPose2",
       py::init<gt::Key, gt::Key, gt::Key, gt::Pose2, gt::SharedNoiseModel>())
      ;


  py::class_<gt::NoiseModelFactor3<gt::Pose2, gt::Pose2,
                                   vertigo::SwitchVariableSigmoid>,
             py::bases<gt::NoiseModelFactor>, 
             boost::shared_ptr<gt::NoiseModelFactor3<gt::Pose2, gt::Pose2,
                                                     vertigo::SwitchVariableSigmoid> >,
             boost::noncopyable > 
      ("NoiseModelFactor2Pose2Pose2SwitchVariableSigmoid", py::no_init)
      ;

  py::class_<vertigo::BetweenFactorSwitchableSigmoid<gt::Pose2>,
             py::bases<gt::NoiseModelFactor3<gt::Pose2, gt::Pose2,
                                             vertigo::SwitchVariableSigmoid> >, 
             boost::shared_ptr<vertigo::BetweenFactorSwitchableSigmoid<gt::Pose2> >,
             boost::noncopyable> 
      ("BetweenFactorSwitchableSigmoidPose2",
       py::init<gt::Key, gt::Key, gt::Key, gt::Pose2, gt::SharedNoiseModel>())
      ;

  
  



  
}

} // namespace python
} // namespace bot


