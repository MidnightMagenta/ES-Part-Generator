#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <scene_builder.h>

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(esjson, m) {
    nb::enum_<Shape>(m, "Shape")
            .value("Sphere", Shape::SPHERE)
            .value("Box", Shape::BLOCK)
            .value("Cylinder", Shape::CYLINDER)
            .value("FixedRotation", Shape::ROT_CONSTRAINED);

    nb::class_<dvec3_s>(m, "Vec3").def("__init__",
                                       [](dvec3_s *v, double x, double y, double z) { new (v) dvec3_s{x, y, z}; }),
            nb::arg("x") = 0.0, nb::arg("y") = 0.0, nb::arg("z") = 0.0;

    nb::class_<SceneBuilder>(m, "SceneBuilder")
            .def(nb::init<>())

            .def("create_attachment",
                 nb::overload_cast<uint64_t, const dvec3_s &, double, const Attachment::AttachmentDetail &>(
                         &SceneBuilder::create_attachment),
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("detail"))

            .def("create_rigid_body",
                 nb::overload_cast<uint64_t,
                                   bool,
                                   double,
                                   const dvec3_s &,
                                   double,
                                   double,
                                   const dvec3_s &,
                                   const dvec3_s &>(&SceneBuilder::create_rigid_body),
                 nb::arg("parent"),
                 nb::arg("infinite_mass")   = false,
                 nb::arg("angle")           = 0.0,
                 nb::arg("position")        = dvec3_s{0, 0, 0},
                 nb::arg("reset_angle")     = 0.0,
                 nb::arg("reset_angular_v") = 0.0,
                 nb::arg("reset_position")  = dvec3_s{0, 0, 0},
                 nb::arg("velocity")        = dvec3_s{0, 0, 0})

            .def("create_rigid_body_element",
                 nb::overload_cast<uint64_t, const dvec3_s &, double, const std::vector<double> &, Shape, bool>(
                         &SceneBuilder::create_rigid_body_element),
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("mass"),
                 nb::arg("params"),
                 nb::arg("type"),
                 nb::arg("visible"))

            .def("create_gas_reservoir",
                 nb::overload_cast<uint64_t>(&SceneBuilder::create_gas_reservoir),
                 nb::arg("parent"))

            .def("create_tube",
                 nb::overload_cast<uint64_t,
                                   size_t,
                                   const std::vector<double> &,
                                   const std::vector<double> &,
                                   int,
                                   int,
                                   int>(&SceneBuilder::create_tube),
                 nb::arg("parent"),
                 nb::arg("segment_count"),
                 nb::arg("dx"),
                 nb::arg("area"),
                 nb::arg("precision") = 1,
                 nb::arg("solverid")  = 4,
                 nb::arg("limiterid") = 7)

            .def("create_valve",
                 nb::overload_cast<uint64_t, double, double, double, double, double>(&SceneBuilder::create_valve),
                 nb::arg("parent"),
                 nb::arg("s_initial"),
                 nb::arg("s_max"),
                 nb::arg("s_min"),
                 nb::arg("s")        = 0.0,
                 nb::arg("s_target") = 0.0)

            .def("create_spark_source",
                 nb::overload_cast<uint64_t>(&SceneBuilder::create_spark_source),
                 nb::arg("parent"))

            .def("create_engine_controller",
                 nb::overload_cast<uint64_t, int, const std::vector<double>, double, double>(
                         &SceneBuilder::create_engine_controller),
                 nb::arg("parent"),
                 nb::arg("cylinder_count"),
                 nb::arg("firing_angles"),
                 nb::arg("rev_max"),
                 nb::arg("rev_release"))

            .def("create_connection",
                 nb::overload_cast<uint64_t, uint64_t, uint64_t>(&SceneBuilder::create_connection),
                 nb::arg("parent"),
                 nb::arg("p0"),
                 nb::arg("p1"))

            .def("root_id",
                 [](SceneBuilder &self) {
                     Object *r = self.root();
                     return r ? r->id() : Object::nullid;
                 })

            .def("serialize",
                 [](SceneBuilder &self) {
                     nlohmann::ordered_json j;
                     self.serialize(j);
                     return j.dump();
                 })
            .def("create_attachment_sensor",
                 &SceneBuilder::create_attachment_sensor,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("sensor_type"),
                 nb::arg("radius"))
            .def("create_attachment_free",
                 &SceneBuilder::create_attachment_free,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("radius"))
            .def("create_attachment_rigid",
                 &SceneBuilder::create_attachment_rigid,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("radius"))
            .def("create_attachment_spring",
                 &SceneBuilder::create_attachment_spring,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("ks"),
                 nb::arg("kd"),
                 nb::arg("rest_len"),
                 nb::arg("radius"))
            .def("create_attachment_bearing_inner",
                 &SceneBuilder::create_attachment_bearing_inner,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("radius"),
                 nb::arg("depth"))
            .def("create_attachment_bearing_outer",
                 &SceneBuilder::create_attachment_bearing_outer,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("inner_radius"),
                 nb::arg("depth"),
                 nb::arg("friction"))
            .def("create_attachment_sliding_inner",
                 &SceneBuilder::create_attachment_sliding_inner,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("shape"),
                 nb::arg("radius"),
                 nb::arg("depth"))
            .def("create_attachment_sliding_outer",
                 &SceneBuilder::create_attachment_sliding_outer,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("shape"),
                 nb::arg("radius"),
                 nb::arg("depth"))
            .def("create_attachment_reservoir_inner",
                 &SceneBuilder::create_attachment_reservoir_inner,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("direction"))
            .def("create_attachment_reservoir_outer",
                 &SceneBuilder::create_attachment_reservoir_outer,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("shape"),
                 nb::arg("radius"),
                 nb::arg("volume"))
            .def("create_attachment_reservoir_skin",
                 &SceneBuilder::create_attachment_reservoir_skin,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"))
            .def("create_attachment_fluid",
                 &SceneBuilder::create_attachment_fluid,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("direction"),
                 nb::arg("radius"))
            .def("create_attachment_logic",
                 &SceneBuilder::create_attachment_logic,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("radius"),
                 nb::arg("port"))
            .def("create_attachment_logic_input",
                 &SceneBuilder::create_attachment_logic_input,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("radius"),
                 nb::arg("port"))
            .def("create_attachment_spark",
                 &SceneBuilder::create_attachment_spark,
                 nb::arg("parent"),
                 nb::arg("position"),
                 nb::arg("angle"),
                 nb::arg("radius"));
}
