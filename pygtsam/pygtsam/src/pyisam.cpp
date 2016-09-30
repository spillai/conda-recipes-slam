// Author(s): Sudeep Pillai (spillai@csail.mit.edu)
// License: MIT

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

// pybot_types
#include "pybot_types.hpp"

// isam
#include <isam/isam.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace isam;

namespace py = boost::python;

namespace bot { namespace vision {

} // namespace vision
} // namespace bot

namespace bot { namespace python {

BOOST_PYTHON_MODULE(pyisam)
{
  // Main types export
  bot::python::init_and_export_converters();
  py::scope scope = py::scope();

  // --------------------------------------------------------------------
  // Noise
  py::class_<isam::Noise>("Noise", py::no_init)
      .def("sqrtinf", &isam::Noise::sqrtinf,
           py::return_value_policy<py::copy_const_reference>())
      ;

  py::class_<isam::SqrtInformation, py::bases<isam::Noise> >
      ("SqrtInformation", py::init<const Eigen::MatrixXd&>(py::arg("sqrtinf")))
      ;

  py::class_<isam::Information, py::bases<isam::Noise> >
      ("Information", py::init<const Eigen::MatrixXd&>(py::arg("inf")))
      ;

  py::class_<isam::Covariance, py::bases<isam::Noise> >
      ("Covariance", py::init<const Eigen::MatrixXd&>(py::arg("cov")))
      ;

  // --------------------------------------------------------------------
  // Graph, Slam

  py::class_<isam::Graph, boost::noncopyable>("Graph", py::init<>())
      .def("add_node", &isam::Graph::add_node)
      .def("add_factor", &isam::Graph::add_factor)
      .def("remove_node", &isam::Graph::remove_node)
      .def("remove_factor", &isam::Graph::remove_factor)

      .def("get_nodes", &isam::Graph::get_nodes,
           py::return_value_policy<py::copy_const_reference>())
      .def("get_factors", &isam::Graph::get_factors,
           py::return_value_policy<py::copy_const_reference>())

      .def("num_nodes", &isam::Graph::num_nodes)
      .def("num_factors", &isam::Graph::num_factors)
      .def("erase_marked", &isam::Graph::erase_marked)
      .def("print_graph", &isam::Graph::print_graph)
      ;


  py::class_<isam::OptimizationInterface, boost::noncopyable>
      ("OptimizationInterface", py::no_init)
      .def("jacobian",
           py::pure_virtual(&isam::OptimizationInterface::jacobian))
      .def("apply_exmap",
           py::pure_virtual(&isam::OptimizationInterface::apply_exmap))
      .def("self_exmap",
           py::pure_virtual(&isam::OptimizationInterface::self_exmap))
      .def("estimate_to_linpoint",
           py::pure_virtual(&isam::OptimizationInterface::estimate_to_linpoint))
      .def("linpoint_to_estimate",
           py::pure_virtual(&isam::OptimizationInterface::linpoint_to_estimate))
      .def("swap_estimates",
           py::pure_virtual(&isam::OptimizationInterface::swap_estimates))
      .def("weighted_errors",
           py::pure_virtual(&isam::OptimizationInterface::weighted_errors))
      ;
  
  py::class_<isam::UpdateStats>
      ("UpdateStats", py::no_init)
      .def_readwrite("step", &isam::UpdateStats::step)
      .def_readwrite("batch", &isam::UpdateStats::batch)
      .def_readwrite("solve", &isam::UpdateStats::solve)
      ;

  py::class_<isam::Slam, py::bases<isam::Graph>, boost::noncopyable >
      ("Slam", py::init<>())
      .def("batch_optimization", &isam::Slam::batch_optimization)
      .def("update", &isam::Slam::update)
      .def("print_stats", &isam::Slam::print_stats)
      ;

  // py::class_<isam::Slam, py::bases<isam::Graph, isam::OptimizationInterface>, boost::noncopyable >
  //     ("Slam", py::init<>())
  //     ;
  
  // --------------------------------------------------------------------
  // Element

  py::enum_<isam::Selector>("Selector")
      .value("LINPOINT", isam::LINPOINT)
      .value("ESTIMATE", isam::ESTIMATE)
      .export_values()
      ;

  py::class_<isam::Element, boost::noncopyable>
      ("Element", py::init<const char*, int>((py::arg("name"), py::arg("dim"))))
      // .def("dim", &isam::Element::dim,
      //      py::return_value_policy<py::copy_const_reference>())
      // .def("start", &isam::Element::start,
      //      py::return_value_policy<py::copy_const_reference>())
      ;

  // Function
  py::class_<isam::Function, boost::noncopyable>
      ("Function", py::no_init)
      ;


  // Factor
  py::class_<isam::Factor, py::bases<isam::Element>,
             boost::noncopyable >
      ("Factor", py::no_init) // py::init<const char*, int, isam::Noise>())
      // (py::arg("name"), py::arg("dim"), py::arg("noise"))
      ;

  // Node
  py::class_<isam::Node, py::bases<isam::Element>, boost::noncopyable >
      ("Node", py::no_init)
      .def("add_factor", &isam::Node::add_factor)
      .def("remove_factor", &isam::Node::remove_factor)
      .def("deleted", &isam::Node::deleted)
      .def("mark_deleted", &isam::Node::mark_deleted)
      .def("erase_marked_factors", &isam::Node::erase_marked_factors)
      // .def("factors", &isam::Node::factors)
      ;

  // NodeT<Point2d>
  py::class_<Point2d_Node, py::bases<isam::Node>, boost::noncopyable >
      ("Point2d_Node", py::init<>())
      .def(py::init<const char*>(py::arg("name")))
      .def("value", &isam::Point2d_Node::value,
           py::arg("s")=isam::ESTIMATE)
      .def("value0", &isam::Point2d_Node::value0)
      ;

  // NodeT<Pose2d>
  py::class_<Pose2d_Node, py::bases<isam::Node>, boost::noncopyable >
      ("Pose2d_Node", py::init<>())
      .def(py::init<const char*>(py::arg("name")))
      .def("value", &isam::Pose2d_Node::value,
           py::arg("s")=isam::ESTIMATE)
      .def("value0", &isam::Pose2d_Node::value0)
      ;

  // NodeT<Anchor2d>
  py::class_<Anchor2d_Node, py::bases<isam::Pose2d_Node>, boost::noncopyable >
      ("Anchor2d_Node", py::init<isam::Slam*>())
      ;

  // FactorT<Point2d>
  py::class_<isam::FactorT<isam::Point2d>,
             py::bases<isam::Factor>, boost::noncopyable>
      ("__FactorT_Point2d", py::no_init);

  // Point2d_Factor
  py::class_<isam::Point2d_Factor,
             py::bases<isam::FactorT<isam::Point2d> >,
             boost::noncopyable >
      ("Point2d_Factor", py::init<
       isam::Point2d_Node*,
       const isam::Point2d&, const isam::Noise&>())
      ;

  // FactorT<Pose2d>
  py::class_<isam::FactorT<isam::Pose2d>,
             py::bases<isam::Factor>, boost::noncopyable>
      ("__FactorT_Pose2d", py::no_init);

  // Pose2d_Factor
  py::class_<isam::Pose2d_Factor,
             py::bases<isam::FactorT<isam::Pose2d> >,
             boost::noncopyable >
      ("Pose2d_Factor", py::init<
       isam::Pose2d_Node*,
       const isam::Pose2d&, const isam::Noise&>())
      ;

  // Pose2d_Pose2d_Factor
  py::class_<isam::Pose2d_Pose2d_Factor,
             py::bases<isam::FactorT<isam::Pose2d> >,
             boost::noncopyable >
      ("Pose2d_Pose2d_Factor", py::init<
       isam::Pose2d_Node*, isam::Pose2d_Node*,
       const isam::Pose2d&, const isam::Noise&,
       py::optional<isam::Anchor2d_Node*, isam::Anchor2d_Node*> >())
      ;

  // Pose2d_Point2d_Factor
  py::class_<isam::Pose2d_Point2d_Factor,
             py::bases<isam::FactorT<isam::Point2d> >,
             boost::noncopyable >
      ("Pose2d_Point2d_Factor", py::init<
       isam::Pose2d_Node*, isam::Point2d_Node*,
       const isam::Point2d&, const isam::Noise&>())
      ;


  // --------------------------------------------------------------------
  // Point2d
  py::class_<isam::Point2d,
             boost::shared_ptr<isam::Point2d> >("Point2d", py::init<>())
      .def(py::init<double, double>(py::args("x","y")))
      .def(py::init<Eigen::Vector2d>())
      // .def("expmap", &isam::Point2d::expmap)
      .add_property("x", &isam::Point2d::x, &isam::Point2d::set_x)
      .add_property("y", &isam::Point2d::y, &isam::Point2d::set_y)
      .def("vector", &isam::Point2d::vector)
      // .def("set", &isam::Point2d::set)
      // .def("wTo", &isam::Point2d::wTo)
      // .def("oTw", &isam::Point2d::oTw)
      // .def("oplus", &isam::Point2d::oplus)
      // .def("ominus", &isam::Point2d::ominus)
      // .def("transform_to", &isam::Point2d::transform_to)
      // .def("transform_from", &isam::Point2d::transform_from)
      ;

  // Point3d
  py::class_<isam::Point3d,
             boost::shared_ptr<isam::Point3d> >("Point3d", py::init<>())
      .def(py::init<double, double, double>(py::args("x","y","z")))
      .def(py::init<Eigen::Vector3d>())
      // .def("expmap", &isam::Point3d::expmap)
      .add_property("x", &isam::Point3d::x, &isam::Point3d::set_x)
      .add_property("y", &isam::Point3d::y, &isam::Point3d::set_y)
      .add_property("z", &isam::Point3d::z, &isam::Point3d::set_z)
      // .def("trans", &isam::Point3d::trans)
      .def("vector", &isam::Point3d::vector)
      // .def("set", &isam::Point3d::set)
      .def("to_point3d", &isam::Point3d::to_point3d)
      .def("of_point2d", &isam::Point3d::of_point2d)
      ;


  // Pose2d
  py::class_<isam::Pose2d,
             boost::shared_ptr<isam::Pose2d> >("Pose2d", py::init<>())
      .def(py::init<double, double, double>(py::args("x","y","t")))
      .def(py::init<Eigen::Vector3d>())
      // .def("expmap", &isam::Pose2d::expmap)
      .add_property("x", &isam::Pose2d::x, &isam::Pose2d::set_x)
      .add_property("y", &isam::Pose2d::y, &isam::Pose2d::set_y)
      .add_property("t", &isam::Pose2d::t, &isam::Pose2d::set_t)
      .def("vector", &isam::Pose2d::vector)
      // .def("set", &isam::Pose2d::set)
      // .def("wTo", &isam::Pose2d::wTo)
      // .def("oTw", &isam::Pose2d::oTw)
      .def("oplus", &isam::Pose2d::oplus)
      .def("ominus", &isam::Pose2d::ominus)
      .def("transform_to", &isam::Pose2d::transform_to)
      .def("transform_from", &isam::Pose2d::transform_from)
      ;

  // Rot3d
  py::class_<isam::Rot3d,
             boost::shared_ptr<isam::Rot3d> >("Rot3d", py::init<>())
      .def(py::init<double, double, double>(py::args("yaw","pitch","roll")))
      // .def(py::init<Eigen::Quaterniond>())
      // .def(py::init<Eigen::Matrix3d>())
      // .add_property("yaw", &isam::Rot3d::yaw, &isam::Rot3d::set_yaw)
      // .add_property("pitch", &isam::Rot3d::pitch, &isam::Rot3d::set_pitch)
      // .add_property("roll", &isam::Rot3d::roll, &isam::Rot3d::set_roll)
      .def("x", &isam::Rot3d::x)
      .def("y", &isam::Rot3d::y)
      .def("z", &isam::Rot3d::z)
      .def("w", &isam::Rot3d::w)
      // .def("ypr", &isam::Rot3d::ypr)
      // .def("set", &isam::Rot3d::set)
      // .def("wRo", &isam::Rot3d::wRo)
      ;

  // isam::Pose3d (isam::Pose3d::*isam_Pose3d_oplus) (const isam::Pose3d& d) const = &isam::Pose3d::oplus;
  
  // Pose3d
  py::class_<isam::Pose3d,
             boost::shared_ptr<isam::Pose3d> >("Pose3d", py::init<>())
      .def(py::init<double, double, double, double, double, double>(py::args("x","y","z","yaw","pitch","roll")))
      .def(py::init<Eigen::MatrixXd>())
      .def(py::init<isam::Point3d, isam::Rot3d>())
      // .add_property("x", &isam::Pose3d::x, &isam::Pose3d::set_x)
      // .add_property("y", &isam::Pose3d::y, &isam::Pose3d::set_y)
      // .add_property("z", &isam::Pose3d::z, &isam::Pose3d::set_z)
      // .add_property("yaw", &isam::Pose3d::yaw, &isam::Pose3d::set_yaw)
      // .add_property("pitch", &isam::Pose3d::pitch, &isam::Pose3d::set_pitch)
      // .add_property("roll", &isam::Pose3d::roll, &isam::Pose3d::set_roll)
      .def("trans", &isam::Pose3d::trans)
      .def("rot", &isam::Pose3d::rot)
      .def("vector", &isam::Pose3d::vector)
      // .def("set", &isam::Pose3d::set)
      .def("wTo", &isam::Pose3d::wTo)
      .def("oTw", &isam::Pose3d::oTw)
      .def("oplus", &isam::Pose3d::oplus) // isam_Pose3d_oplus)
      .def("ominus", &isam::Pose3d::ominus)
      // .def("transform_to", &isam::Pose3d::transform_to)
      // .def("transform_from", &isam::Pose3d::transform_from)
      ;



}

} // namespace python
} // namespace bot


  // py::class_<isam::FactorT<isam::Point2d>,
  //            py::bases<isam::Factor>,
  //            boost::noncopyable>
  //     ("FactorT_Point2d", py::no_init)
  //     // py::init<const char*, int, const isam::Noise&, const isam::Point2d&>())
  //     // .def("measurement", &isam::FactorT<isam::Point2d>::measurement,
  //     //      py::return_value_policy<py::copy_const_reference>())
  //     ;



  // // Pose2d_Node
  // py::class_<isam::Pose2d_Node, py::bases<isam::Node>,
  //            isam::Pose2d_Node*, boost::noncopyable >
  //     ("Pose2d_Node", py::init<>())
  //     // .def(py::init<char*>())
  //     .def("init", &isam::NodeT<Pose2d>::init)
  //     .def("value", &isam::NodeT<Pose2d>::value)
  //     .def("value0", &isam::NodeT<Pose2d>::value0)
  //     .def("vector", &isam::NodeT<Pose2d>::vector)
  //     .def("vector0", &isam::NodeT<Pose2d>::vector0)
  //     ;

  // // Pose3d_Node
  // py::class_<isam::Pose3d_Node, py::bases<isam::Node>,
  //            isam::Pose3d_Node*, boost::noncopyable >
  //     ("Pose3d_Node", py::init<>())
  //     // .def(py::init<char*>())
  //     .def("init", &isam::NodeT<Pose3d>::init)
  //     .def("value", &isam::NodeT<Pose3d>::value)
  //     .def("value0", &isam::NodeT<Pose3d>::value0)
  //     .def("vector", &isam::NodeT<Pose3d>::vector)
  //     .def("vector0", &isam::NodeT<Pose3d>::vector0)
  //     ;


  // // Pose3d_Node
  // py::class_<isam::Pose3d_Pose3d_Factor, py::bases<isam::Factor>,
  //            isam::Pose3d_Pose3d_Factor*, boost::noncopyable >
  //     ("Pose3d_Pose3d_Factor",
  //      py::init<Pose3d_Node*, Pose3d_Node*, isam::Pose3d, isam::Noise,
  //      py::optional<Anchor3d_Node*, Anchor3d_Node*>())
  //     .def("initialize", &Pose3d_Pose3d_Factor::initialize)
  //     .def("basic_error", &Pose3d_Pose3d_Factor::basic_error)
  //     ;
