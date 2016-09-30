// Author(s): Sudeep Pillai (spillai@csail.mit.edu)
// License: MIT

#ifndef  __APRILTAGS_WRAPPER__
#define  __APRILTAGS_WRAPPER__

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace bot { namespace vision {

// B: x (fwd), y (left), z (up)
// CV: x (right), y (down), z (fwd)

// Old: 
// Desired output (OpenCV): x (right), y (down), z (ahead)

// Takes from tag to body
static const cv::Mat1d p_b_tag_ = (cv::Mat1d(4,4) << 0, 0, 1, 0,
                                  1, 0, 0, 0,
                                  0, 1, 0, 0,
                                  0, 0, 0, 1
                                  );
static const cv::Mat1d p_tag_b_ = p_b_tag_.t();

class AprilTag {
 public: 
  AprilTag(): id_(-1) {
  }

  AprilTag(const int id, const cv::Mat1f& features, const cv::Mat1f& pose)
      : id_(id), features_(features), pose_(pose) {
  }

  int getId() const { return id_; }
  
  cv::Mat1f getFeatures() const {
    return features_;
  }

  cv::Mat1d getPose() const {
    return pose_;
  }
  
  cv::Mat1d getPoseBody() const {
    return p_b_tag_ * pose_;
  }

  void setPose(const cv::Mat1d pose) {
    pose_ = pose;
  }

  int id_;
  cv::Mat1f features_;
  cv::Mat1d pose_;
};

// april tag detection, contains image coordinates of four corners in fixed order
class TagCorners {
public:
  Eigen::VectorXd vec_;

  TagCorners(const TagCorners& corners) {
    vec_ = corners.vector();
  }

  TagCorners(Eigen::VectorXd& corners) {
    // N x 8 (bl, br, tr, tl)
    vec_ = corners;
  }
  
  friend std::ostream& operator<<(std::ostream& out, const TagCorners& t) {
    t.write(out);
    return out;
  }

  void write(std::ostream &out) const {
    out << "(" << bl() << " ; " << br() << " ; " << tr() << " ; " << tl() << ")";
  }

  Eigen::VectorXd vector() const {
    return vec_;
  }

  Eigen::Vector2d bl() const {
    return Eigen::Vector2d(vec_(0), vec_(1));
  }

  Eigen::Vector2d br() const {
    return Eigen::Vector2d(vec_(2), vec_(3));
  }

  Eigen::Vector2d tr() const {
    return Eigen::Vector2d(vec_(4), vec_(5));
  }

  Eigen::Vector2d tl() const {
    return Eigen::Vector2d(vec_(6), vec_(7));
  }
  
};

class PoseID3d {
 public: 
  PoseID3d(): id_(-1) {
  }

  PoseID3d(const int id, const Eigen::MatrixXd& pose)
      : id_(id), pose_(pose) {
  }

  int getId() const { return id_; }

  Eigen::MatrixXd getPose() const {
    return pose_;
  }

  int id_;
  Eigen::MatrixXd pose_;
};



static void draw_detection(cv::Mat& vis, const AprilTag& tag, const Eigen::Matrix<double, 3,4>& P, const float tag_size) {
    
  // draw corner points as detected by line intersection
  const cv::Mat1f& features = tag.getFeatures();
  std::pair<float, float> p1 = std::make_pair(features(0,0), features(0,1));
  std::pair<float, float> p2 = std::make_pair(features(1,0), features(1,1));
  std::pair<float, float> p3 = std::make_pair(features(2,0), features(2,1));
  std::pair<float, float> p4 = std::make_pair(features(3,0), features(3,1));

  cv::line(vis, cv::Point2f(p1.first, p1.second), cv::Point2f(p2.first, p2.second), cv::Scalar(255,0,0,0), 2);
  cv::line(vis, cv::Point2f(p2.first, p2.second), cv::Point2f(p3.first, p3.second), cv::Scalar(0,255,0,0), 2 );
  cv::line(vis, cv::Point2f(p3.first, p3.second), cv::Point2f(p4.first, p4.second), cv::Scalar(0,0,255,0), 2);
  cv::line(vis, cv::Point2f(p4.first, p4.second), cv::Point2f(p1.first, p1.second), cv::Scalar(255,0,255,0), 2);
  // cv::circle(vis, cv::Point2f(detection.cxy.first, detection.cxy.second), 8, cv::Scalar(0,0,255,0), 2);
   
  Eigen::MatrixXd corners(4,4);
  double s = tag_size/2;
  corners << s,s,-s,-s, s,-s,-s,s, 0,0,0,0, 1,1,1,1;
  Eigen::Vector3d x;
  for (int i=0; i<4; i++) {
    x = P * corners.col(i);
    x = x/x(2);
    cv::circle(vis, cv::Point2f(x(0), x(1)), 3, cv::Scalar(0,255,0,0), 2);
  }

  // print ID
  std::ostringstream strSt;
  strSt << "#" << tag.getId();
  cv::putText(vis, strSt.str(),
              cv::Point2f(p1.first * 0.5 + p3.first * 0.5 + 10, p1.second * 0.5 + p3.second * 0.5 + 10),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
}


static cv::Mat3b project_tags(const cv::Mat3b& img, const std::vector<AprilTag>& tags, const Eigen::Matrix<double, 3,4>& K, const float tag_size) {

  // Add all detections 
  cv::Mat3b vis = img.clone();
    
  for (size_t k=0; k<tags.size(); ++k) {

    Eigen::Matrix4d T;
    cv::cv2eigen(tags[k].getPose(), T);
    Eigen::Matrix<double, 3,4> P = K*T;

    // local image
    draw_detection(vis, tags[k], P, tag_size);
  }
  return vis;
}

static cv::Mat3b draw_tags(const cv::Mat3b& img, const std::vector<AprilTag>& tags) {

  // Add all detections 
  cv::Mat3b vis = img.clone();
  cv::Mat3b zero = img.clone();
  
  for (size_t k=0; k<tags.size(); ++k) {
    const cv::Mat1f& features = tags[k].getFeatures();
    cv::line(zero, cv::Point2f(features.row(0)), cv::Point2f(features.row(1)), cv::Scalar(255,0,0), 2);
    cv::line(zero, cv::Point2f(features.row(1)), cv::Point2f(features.row(2)), cv::Scalar(0,255,0), 2);
    cv::line(zero, cv::Point2f(features.row(2)), cv::Point2f(features.row(3)), cv::Scalar(0,0,255), 2);
    cv::line(zero, cv::Point2f(features.row(3)), cv::Point2f(features.row(0)), cv::Scalar(255,0,255), 2);


    const int count = 4;
    cv::Point ppt[4] = {cv::Point(features.row(0)), cv::Point(features.row(1)),
                        cv::Point(features.row(2)), cv::Point(features.row(3))};
    const cv::Point* pts = &ppt[0];
    cv::fillPoly(zero, &pts, &count, 1, cv::Scalar(0,255,0), CV_AA );
    
  // print ID
    std::ostringstream strSt;
    strSt << "#" << tags[k].getId();
    cv::putText(vis, strSt.str(),
                cv::Point2f((features(0,0) + features(2,0))/2, (features(0,1) + features(2,1))/2),
                cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0,0,255), 2);
  }
  return vis / 4 * 3 + zero / 4;
}

}
}

#endif  // __APRILTAGS_WRAPPER__
