#ifndef SCENE_BUILDER_H
#define SCENE_BUILDER_H

#include <nlohmann/json.hpp>
#include <object_tree.h>
#include <objects.h>
#include <vector>

class SceneBuilder {
public:
    SceneBuilder() {
        initialize_tree();
    }

    Object *create_instance(Object *parent, Object *spec, const dvec3_s &position = {0, 0, 0}, bool primary = false);
    Object *create_attachment(Object                             *parent,
                              const dvec3_s                      &position,
                              double                              angle,
                              const Attachment::AttachmentDetail &detail);
    Object *create_rigid_body(Object        *parent,
                              bool           infinite_mass   = false,
                              double         angle           = 0.0,
                              const dvec3_s &position        = {0.0, 0.0, 0.0},
                              double         reset_angle     = 0.0,
                              double         reset_angular_v = 0.0,
                              const dvec3_s &reset_position  = {0.0, 0.0, 0.0},
                              const dvec3_s &velocity        = {0.0, 0.0, 0.0});
    Object *create_rigid_body_element(Object                    *parent,
                                      const dvec3_s             &position,
                                      double                     mass,
                                      const std::vector<double> &params,
                                      Shape                      type,
                                      bool                       visible);
    Object *create_gas_reservoir(Object *parent);
    Object *create_tube(Object                    *parent,
                        size_t                     segment_count,
                        const std::vector<double> &dx,
                        const std::vector<double> &area,
                        int                        precision = 1,
                        int                        solverid  = 4,
                        int                        limiterid = 7);
    Object *create_valve(Object *parent,
                         double  s_initial,
                         double  s_max,
                         double  s_min,
                         double  s        = 0.0,
                         double  s_target = 0.0);
    Object *create_spark_source(Object *parent);
    Object *create_engine_controller(Object                   *parent,
                                     int                       cylidner_count,
                                     const std::vector<double> firing_angles,
                                     double                    rev_max,
                                     double                    rev_release);
    Object *create_connection(Object *parent, Object *p0, Object *p1);

    inline uint64_t create_attachment(uint64_t                            parent,
                                      const dvec3_s                      &position,
                                      double                              angle,
                                      const Attachment::AttachmentDetail &detail) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, detail);
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_rigid_body(uint64_t       parent,
                                      bool           infinite_mass   = false,
                                      double         angle           = 0.0,
                                      const dvec3_s &position        = {0.0, 0.0, 0.0},
                                      double         reset_angle     = 0.0,
                                      double         reset_angular_v = 0.0,
                                      const dvec3_s &reset_position  = {0.0, 0.0, 0.0},
                                      const dvec3_s &velocity        = {0.0, 0.0, 0.0}) {
        Object *p   = find_object(parent);
        Object *obj = create_rigid_body(p,
                                        infinite_mass,
                                        angle,
                                        position,
                                        reset_angle,
                                        reset_angular_v,
                                        reset_position,
                                        velocity);
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_rigid_body_element(uint64_t                   parent,
                                              const dvec3_s             &position,
                                              double                     mass,
                                              const std::vector<double> &params,
                                              Shape                      type,
                                              bool                       visible) {
        Object *p   = find_object(parent);
        Object *obj = create_rigid_body_element(p, position, mass, params, type, visible);
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_gas_reservoir(uint64_t parent) {
        Object *p   = find_object(parent);
        Object *obj = create_gas_reservoir(p);
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_tube(uint64_t                   parent,
                                size_t                     segment_count,
                                const std::vector<double> &dx,
                                const std::vector<double> &area,
                                int                        precision = 1,
                                int                        solverid  = 4,
                                int                        limiterid = 7) {
        Object *p   = find_object(parent);
        Object *obj = create_tube(p, segment_count, dx, area, precision, solverid, limiterid);
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_valve(uint64_t parent,
                                 double   s_initial,
                                 double   s_max,
                                 double   s_min,
                                 double   s        = 0.0,
                                 double   s_target = 0.0) {
        Object *p   = find_object(parent);
        Object *obj = create_valve(p, s_initial, s_max, s_min, s, s_target);
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_spark_source(uint64_t parent) {
        Object *p   = find_object(parent);
        Object *obj = create_spark_source(p);
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_engine_controller(uint64_t                  parent,
                                             int                       cylidner_count,
                                             const std::vector<double> firing_angles,
                                             double                    rev_max,
                                             double                    rev_release) {
        Object *p   = find_object(parent);
        Object *obj = create_engine_controller(p, cylidner_count, firing_angles, rev_max, rev_release);
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_connection(uint64_t parent, uint64_t p0, uint64_t p1) {
        Object *par = find_object(parent);
        Object *o0  = find_object(p0);
        Object *o1  = find_object(p1);
        Object *obj = create_connection(par, o0, o1);
        return obj ? obj->id() : Object::nullid;
    }

    // attachment adapters
    inline uint64_t create_attachment_sensor(uint64_t       parent,
                                             const dvec3_s &position,
                                             double         angle,
                                             int            sensor_type,
                                             double         radius) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::Sensor(sensor_type, radius));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_free(uint64_t parent, const dvec3_s &position, double angle, double radius) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::FreeAttachment(radius));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_rigid(uint64_t parent, const dvec3_s &position, double angle, double radius) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::RigidAttachment(radius));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_spring(uint64_t       parent,
                                             const dvec3_s &position,
                                             double         angle,
                                             double         ks,
                                             double         kd,
                                             double         rest_len,
                                             double         radius) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::SpringAttachment(ks, kd, rest_len, radius));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_bearing_inner(uint64_t       parent,
                                                    const dvec3_s &position,
                                                    double         angle,
                                                    double         radius,
                                                    double         depth) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::BearingInner(radius, depth));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_bearing_outer(uint64_t       parent,
                                                    const dvec3_s &position,
                                                    double         angle,
                                                    double         inner_radius,
                                                    double         depth,
                                                    double         friction) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::BearingOuter(inner_radius, depth, friction));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_sliding_inner(uint64_t       parent,
                                                    const dvec3_s &position,
                                                    double         angle,
                                                    Shape          shape,
                                                    double         radius,
                                                    double         depth) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::SlidingInner(shape, radius, depth));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_sliding_outer(uint64_t       parent,
                                                    const dvec3_s &position,
                                                    double         angle,
                                                    Shape          shape,
                                                    double         radius,
                                                    double         depth) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::SlidingOuter(shape, radius, depth));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_reservoir_inner(uint64_t       parent,
                                                      const dvec3_s &position,
                                                      double         angle,
                                                      int            direction) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::ReservoirInner(direction));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_reservoir_outer(uint64_t       parent,
                                                      const dvec3_s &position,
                                                      double         angle,
                                                      Shape          shape,
                                                      double         radius,
                                                      double         volume) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::ReservoirOuter(shape, radius, volume));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_reservoir_skin(uint64_t parent, const dvec3_s &position, double angle) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::ReservoirSkin());
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_fluid(uint64_t       parent,
                                            const dvec3_s &position,
                                            double         angle,
                                            int            direction,
                                            double         radius) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::FluidAttachment(direction, radius));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_logic(uint64_t       parent,
                                            const dvec3_s &position,
                                            double         angle,
                                            double         radius,
                                            int            port) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::LogicAttachment(radius, port));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_logic_input(uint64_t       parent,
                                                  const dvec3_s &position,
                                                  double         angle,
                                                  double         radius,
                                                  int            port) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::LogicInputAttachment(radius, port));
        return obj ? obj->id() : Object::nullid;
    }

    inline uint64_t create_attachment_spark(uint64_t parent, const dvec3_s &position, double angle, double radius) {
        Object *p   = find_object(parent);
        Object *obj = create_attachment(p, position, angle, Attachment::SparkSource(radius));
        return obj ? obj->id() : Object::nullid;
    }
    // !attachment adapters

    Object *find_object(uint64_t id) {
        return m_tree.find_object(id);
    }

    uint64_t get_obj_id(Object *obj) {
        return obj->id();
    }

    ObjectTree *tree() {
        return &m_tree;
    }

    Object *root() {
        return m_root_component;
    }
    const Object *root() const {
        return m_root_component;
    }

    inline void serialize(nlohmann::ordered_json &json) {
        m_tree.serialize(json);
    }

private:
    void initialize_tree();

    Component         *m_root_component;
    ComponentInstance *m_root_component_instance;
    InstanceMapping   *m_mapping;
    ObjectTree         m_tree;
};

#endif
