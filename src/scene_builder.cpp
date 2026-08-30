#include <iostream>
#include <objects.h>
#include <scene_builder.h>

void SceneBuilder::initialize_tree() {
    Component         *root, *assembly_root, *instance_root;
    Instance          *root_instance, *scene_instnace;
    ComponentInstance *root_cinstance;
    InstanceMapping   *root_mapping;

    root                      = m_tree.create_object<ObjType::COMPONENT>();
    root_instance             = m_tree.create_object<ObjType::INSTANCE>();
    assembly_root             = m_tree.create_object<ObjType::COMPONENT>();
    instance_root             = m_tree.create_object<ObjType::COMPONENT>();
    scene_instnace            = m_tree.create_object<ObjType::INSTANCE>();
    m_root_component          = m_tree.create_object<ObjType::COMPONENT>();
    root_mapping              = m_tree.create_object<ObjType::ISNTANCE_MAPPING>();
    m_mapping                 = m_tree.create_object<ObjType::ISNTANCE_MAPPING>();
    root_cinstance            = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    m_root_component_instance = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();

    root->add_detail(root_instance);
    root_instance->specification() = assembly_root->id();

    assembly_root->add_detail(m_tree.create_object<ObjType::ASSEMBLY>());
    assembly_root->add_child(instance_root);

    instance_root->add_detail(scene_instnace);
    scene_instnace->specification() = m_root_component->id();

    m_root_component->add_detail(m_tree.create_object<ObjType::ASSEMBLY>());

    root_cinstance->specification()    = root->id();
    root_cinstance->instance_mapping() = root_mapping->id();
    root_cinstance->type()             = ObjType::INSTANCE;
    root_cinstance->referenced_type()  = ObjType::ASSEMBLY;

    m_root_component_instance->specification()    = instance_root->id();
    m_root_component_instance->context()          = root_cinstance->id();
    m_root_component_instance->instance_mapping() = m_mapping->id();
    m_root_component_instance->type()             = ObjType::INSTANCE;
    m_root_component_instance->referenced_type()  = ObjType::ASSEMBLY;
    root_mapping->add_mapping(instance_root->id(), m_root_component_instance->id());
}

Object *SceneBuilder::create_instance(Object *parent, Object *spec, const dvec3_s &position, bool primary) {
    std::cerr << "Unimplemented\n";
    return nullptr;
}


Object *SceneBuilder::create_attachment(Object                             *parent,
                                        const dvec3_s                      &position,
                                        double                              angle,
                                        const Attachment::AttachmentDetail &detail) {
    Component         *comp, *parent_comp;
    ComponentInstance *comp_instance;
    Attachment        *attachment;

    parent_comp = dynamic_cast<Component *>(parent);
    if (!parent_comp) { return nullptr; }

    comp          = m_tree.create_object<ObjType::COMPONENT>();
    comp_instance = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    attachment    = m_tree.create_object<ObjType::ATTACHMENT>();

    parent_comp->add_child(comp);
    comp->add_detail(attachment);
    attachment->position() = position;
    attachment->angle()    = angle;
    std::visit(
            [&](auto &d) {
                using T = std::decay_t<decltype(d)>;
                attachment->raw_detail().template emplace<T>(std::forward<decltype(d)>(d));
            },
            detail);

    comp_instance->context()         = m_root_component_instance->id();
    comp_instance->specification()   = comp->id();
    comp_instance->type()            = comp->type();
    comp_instance->referenced_type() = comp->type();

    m_mapping->add_mapping(comp->id(), comp_instance->id());

    return comp;
}

Object *SceneBuilder::create_rigid_body(Object        *parent,
                                        bool           infinite_mass,
                                        double         angle,
                                        const dvec3_s &position,
                                        double         reset_angle,
                                        double         reset_angular_v,
                                        const dvec3_s &reset_position,
                                        const dvec3_s &velocity) {
    Component         *comp, *parent_comp;
    ComponentInstance *comp_instance;
    RigidBody         *rb;
    RigidBodyState    *rb_state;

    parent_comp = dynamic_cast<Component *>(parent);
    if (!parent_comp) { return nullptr; }

    comp          = m_tree.create_object<ObjType::COMPONENT>();
    comp_instance = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    rb            = m_tree.create_object<ObjType::RIGID_BODY>();
    rb_state      = m_tree.create_object<ObjType::RIGID_BODY_STATE>();

    parent_comp->add_child(comp);
    comp->add_detail(rb);
    rb->infinite_mass()    = infinite_mass;
    rb->default_angle()    = angle;
    rb->default_position() = position;

    comp_instance->specification()   = comp->id();
    comp_instance->context()         = m_root_component_instance->id();
    comp_instance->detail()          = rb_state->id();
    comp_instance->type()            = ObjType::RIGID_BODY;
    comp_instance->referenced_type() = ObjType::RIGID_BODY;

    rb_state->angle()            = reset_angle;
    rb_state->position()         = reset_position;
    rb_state->angular_velocity() = reset_angular_v;
    rb_state->velocity()         = velocity;

    m_mapping->add_mapping(comp->id(), comp_instance->id());

    return comp;
}


Object *SceneBuilder::create_rigid_body_element(Object                    *parent,
                                                const dvec3_s             &position,
                                                double                     mass,
                                                const std::vector<double> &params,
                                                Shape                      type,
                                                bool                       visible) {
    Component         *comp, *parent_comp;
    ComponentInstance *comp_instance;
    RigidBodyElement  *rb;

    if (params.size() > RigidBodyElement::params_size()) { return nullptr; }

    parent_comp = dynamic_cast<Component *>(parent);
    if (!parent_comp) { return nullptr; }

    comp          = m_tree.create_object<ObjType::COMPONENT>();
    comp_instance = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    rb            = m_tree.create_object<ObjType::RIGID_BODY_ELEMENT>();

    parent_comp->add_child(comp);
    comp->add_detail(rb);
    comp->required() = true;
    rb->position()   = position;
    rb->mass()       = mass;
    rb->type()       = type;
    rb->visible()    = visible;
    for (size_t i = 0; i < params.size(); i++) { rb->set_param(i, params[i]); }

    comp_instance->specification()   = comp->id();
    comp_instance->context()         = m_root_component_instance->id();
    comp_instance->type()            = ObjType::RIGID_BODY_ELEMENT;
    comp_instance->referenced_type() = ObjType::RIGID_BODY_ELEMENT;

    m_mapping->add_mapping(comp->id(), comp_instance->id());

    return comp;
}

Object *SceneBuilder::create_gas_reservoir(Object *parent) {

    Component              *comp, *parent_comp;
    ComponentInstance      *comp_instance;
    GasReservoirModel      *gr;
    GasReservoirModelState *gr_state;

    parent_comp = dynamic_cast<Component *>(parent);
    if (!parent_comp) { return nullptr; }

    comp          = m_tree.create_object<ObjType::COMPONENT>();
    comp_instance = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    gr            = m_tree.create_object<ObjType::GAS_RESERVOIR_MODEL>();
    gr_state      = m_tree.create_object<ObjType::GAS_RESERVOIR_MODEL_STATE>();

    parent_comp->add_child(comp);
    comp->add_detail(gr);

    comp_instance->specification()   = comp->id();
    comp_instance->context()         = m_root_component_instance->id();
    comp_instance->detail()          = gr_state->id();
    comp_instance->type()            = ObjType::GAS_RESERVOIR_MODEL;
    comp_instance->referenced_type() = ObjType::GAS_RESERVOIR_MODEL;

    gr_state->volume()                            = 0.00010000000000000002;
    gr_state->density()                           = 1.293;
    gr_state->internal_energy()                   = 256709.0;
    gr_state->burned_volume()                     = 0;
    gr_state->unburned_volume()                   = 0.00010000000000000002;
    gr_state->turbulence_intensity()              = 0.0;
    gr_state->unburned_fraction_internal_energy() = 256709.0;
    gr_state->burned_fraction_internal_energy()   = 256709.0;
    gr_state->combustion_active()                 = false;

    m_mapping->add_mapping(comp->id(), comp_instance->id());
    return comp;
}

Object *SceneBuilder::create_tube(Object                    *parent,
                                  size_t                     segment_count,
                                  const std::vector<double> &dx,
                                  const std::vector<double> &area,
                                  int                        precision,
                                  int                        solverid,
                                  int                        limiterid) {

    Component         *comp, *parent_comp;
    ComponentInstance *comp_instance;
    Tube              *tube;
    TubeState         *tube_state;

    if (segment_count < 5) { return nullptr; }
    if (dx.size() != segment_count || area.size() != segment_count) { return nullptr; }

    parent_comp = dynamic_cast<Component *>(parent);
    if (!parent_comp) { return nullptr; }

    comp          = m_tree.create_object<ObjType::COMPONENT>();
    comp_instance = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    tube          = m_tree.create_object<ObjType::TUBE>();
    tube_state    = m_tree.create_object<ObjType::TUBE_STATE>();

    parent_comp->add_child(comp);
    comp->add_detail(tube);
    tube->set_cell_count(segment_count);
    tube->precision()  = precision;
    tube->solver_id()  = solverid;
    tube->limiter_id() = limiterid;
    for (size_t i = 0; i < segment_count; i++) { *tube->dx(i) = dx[i]; }
    for (size_t i = 0; i < segment_count; i++) { *tube->area(i) = area[i]; }

    comp_instance->specification()   = comp->id();
    comp_instance->context()         = m_root_component_instance->id();
    comp_instance->detail()          = tube_state->id();
    comp_instance->type()            = ObjType::TUBE;
    comp_instance->referenced_type() = ObjType::TUBE;

    tube_state->set_cell_count(segment_count);

    for (size_t i = 0; i < segment_count; i++) {
        *tube_state->density(i)  = 1.1841210698479999;
        *tube_state->velocity(i) = 0.0;
        *tube_state->energy(i)   = 256709.0;
        *tube_state->T_wall(i)   = 298.15;
    }

    m_mapping->add_mapping(comp->id(), comp_instance->id());
    return comp;
}

Object *SceneBuilder::create_valve(Object *parent,
                                   double  s_initial,
                                   double  s_max,
                                   double  s_min,
                                   double  s,
                                   double  s_target) {

    Component         *comp, *parent_comp;
    ComponentInstance *comp_instance;
    Valve             *valve;
    ValveState        *valve_state;

    parent_comp = dynamic_cast<Component *>(parent);
    if (!parent_comp) { return nullptr; }

    comp          = m_tree.create_object<ObjType::COMPONENT>();
    comp_instance = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    valve         = m_tree.create_object<ObjType::VALVE>();
    valve_state   = m_tree.create_object<ObjType::VALVE_STATE>();

    parent_comp->add_child(comp);
    comp->add_detail(valve);
    valve->initial() = s_initial;
    valve->max()     = s_max;
    valve->min()     = s_min;

    comp_instance->specification()   = comp->id();
    comp_instance->context()         = m_root_component_instance->id();
    comp_instance->detail()          = valve_state->id();
    comp_instance->type()            = ObjType::VALVE;
    comp_instance->referenced_type() = ObjType::VALVE;

    valve_state->s()        = s;
    valve_state->s_target() = s_target;

    m_mapping->add_mapping(comp->id(), comp_instance->id());
    return comp;
}

Object *SceneBuilder::create_spark_source(Object *parent) {

    Component         *comp, *parent_comp;
    ComponentInstance *comp_instance;
    SparkSource       *spark;

    parent_comp = dynamic_cast<Component *>(parent);
    if (!parent_comp) { return nullptr; }

    comp          = m_tree.create_object<ObjType::COMPONENT>();
    comp_instance = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    spark         = m_tree.create_object<ObjType::SPARK_SRC>();

    parent_comp->add_child(comp);
    comp->add_detail(spark);

    comp_instance->specification()   = comp->id();
    comp_instance->context()         = m_root_component_instance->id();
    comp_instance->type()            = ObjType::SPARK_SRC;
    comp_instance->referenced_type() = ObjType::SPARK_SRC;

    m_mapping->add_mapping(comp->id(), comp_instance->id());
    return comp;
}

Object *SceneBuilder::create_engine_controller(Object                   *parent,
                                               int                       cylidner_count,
                                               const std::vector<double> firing_angles,
                                               double                    rev_max,
                                               double                    rev_release) {

    Component         *comp, *parent_comp;
    ComponentInstance *comp_instance;
    EngineController  *ec;

    parent_comp = dynamic_cast<Component *>(parent);
    if (!parent_comp) { return nullptr; }

    comp          = m_tree.create_object<ObjType::COMPONENT>();
    comp_instance = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    ec            = m_tree.create_object<ObjType::ENGINE_CONTROLLER>();

    parent_comp->add_child(comp);
    comp->add_detail(ec);

    ec->set_cylinder_count(cylidner_count);
    ec->rev_max()     = rev_max;
    ec->rev_release() = rev_release;
    for (size_t i = 0; i < cylidner_count; i++) { *ec->firing_angle(i) = firing_angles[i]; }

    comp_instance->specification()   = comp->id();
    comp_instance->context()         = m_root_component_instance->id();
    comp_instance->type()            = ObjType::ENGINE_CONTROLLER;
    comp_instance->referenced_type() = ObjType::ENGINE_CONTROLLER;

    m_mapping->add_mapping(comp->id(), comp_instance->id());
    return comp;
}

Object *SceneBuilder::create_connection(Object *parent, Object *p0, Object *p1) {

    Component         *comp, *parent_comp;
    ComponentInstance *comp_instance;
    Connection        *con;

    parent_comp = dynamic_cast<Component *>(parent);
    if (!parent_comp) { return nullptr; }

    comp          = m_tree.create_object<ObjType::COMPONENT>();
    comp_instance = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    con           = m_tree.create_object<ObjType::CONNECTION>();

    parent_comp->add_child(comp);
    comp->add_detail(con);
    con->connect(p0->id(), p1->id());

    comp_instance->specification()   = comp->id();
    comp_instance->context()         = m_root_component_instance->id();
    comp_instance->type()            = ObjType::CONNECTION;
    comp_instance->referenced_type() = ObjType::CONNECTION;

    m_mapping->add_mapping(comp->id(), comp_instance->id());
    return comp;
}
