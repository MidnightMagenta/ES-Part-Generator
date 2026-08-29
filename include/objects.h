#ifndef ESJSON_OBJECTS
#define ESJSON_OBJECTS

#include <array>
#include <cstdint>
#include <format>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#define SIMPLE_ACCESSOR(type, name, member)                                                                            \
    inline type &name() {                                                                                              \
        return member;                                                                                                 \
    }                                                                                                                  \
    inline type name() const {                                                                                         \
        return member;                                                                                                 \
    }

#define SIMPLE_ACCESSOR_REF(type, name, member)                                                                        \
    inline type &name() {                                                                                              \
        return member;                                                                                                 \
    }                                                                                                                  \
    inline const type &name() const {                                                                                  \
        return member;                                                                                                 \
    }

class ObjectTree;

enum ObjType : int {
    VALVE_STATE               = -7,
    TUBE_STATE                = -6,
    GAS_RESERVOIR_MODEL_STATE = -5,
    RIGID_BODY_STATE          = -4,
    ISNTANCE_MAPPING          = -3,
    COMPONENT_INSTANCE        = -2,
    COMPONENT                 = -1,
    ASSEMBLY                  = 0,
    INSTANCE                  = 1,
    ATTACHMENT                = 2,
    RIGID_BODY                = 3,
    RIGID_BODY_ELEMENT        = 4,
    GAS_RESERVOIR_MODEL       = 5,
    TUBE                      = 6,
    VALVE                     = 7,
    SPARK_SRC                 = 8,
    ENGINE_CONTROLLER         = 9,
    CONNECTION                = 10,
};

struct dvec3_s {
    double data[3];
};

class Object {
public:
    static constexpr uint64_t nullid = (uint64_t) -1;

public:
    Object() = delete;
    explicit Object(int type, ObjectTree *tree)
        : m_type(type),
          m_tree(tree),
          m_id(next_id++) {}

    virtual ~Object() {}

    virtual void serialize(nlohmann::json &json) = 0;

    uint64_t id() const {
        return m_id;
    }

    int type() const {
        return m_type;
    }

protected:
    static uint64_t next_id;

protected:
    ObjectTree *m_tree;

    int      m_type;
    uint64_t m_id;
};

class Component : public Object {
public:
    Component(ObjectTree *tree)
        : Object(-1, tree) {}
    ~Component() {}

    inline void reparent(Object *new_parent) {
        Component *parent = dynamic_cast<Component *>(new_parent);
        if (!new_parent) { throw std::runtime_error("Invalid parent type"); }

        if (m_parent) { dynamic_cast<Component *>(m_parent)->m_children.erase(this); }
        m_parent = new_parent;
        parent->m_children.insert(this);
    }

    inline void add_child(Object *child) {
        Component *cchild = dynamic_cast<Component *>(child);
        if (!child) { throw std::runtime_error("Invalid child type"); }

        cchild->reparent(this);
    }

    inline const std::unordered_set<Object *> &get_children() const {
        return m_children;
    }

    inline const Object *get_parent() const {
        return m_parent;
    }

    inline void add_detail(Object *detail) {
        // TODO: should probably check if the detail type is actually valid
        m_detail = detail;
        m_type   = detail->type();
    }

    inline Object *get_detail() {
        return m_detail;
    }
    inline const Object *get_detail() const {
        return m_detail;
    }

    SIMPLE_ACCESSOR(bool, required, m_required);

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["Component::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]           = m_id;
        data["parent"]["id"] = m_parent ? m_parent->id() : nullid;
        for (const auto child : m_children) {
            nlohmann::json children = data["children"];
            children.push_back({{"id", child->id()}});
        }
        data["type"]         = m_type;
        data["detail"]["id"] = m_detail ? m_detail->id() : nullid;
        data["required"]     = m_required;

        state.push_back(elem);
    }

private:
    Object                      *m_parent = nullptr;
    std::unordered_set<Object *> m_children;

    Object *m_detail;
    int     m_type;

    bool m_required = false;
};

class RigidBody : public Object {
public:
public:
    RigidBody(ObjectTree *tree)
        : Object(3, tree) {}

    SIMPLE_ACCESSOR(bool, infinite_mass, m_infinite_mass);
    SIMPLE_ACCESSOR(double, default_angle, m_default_angle);
    SIMPLE_ACCESSOR_REF(dvec3_s, default_position, m_default_position);

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["RigidBody::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]              = m_id;
        data["infiniteMass"]    = m_infinite_mass;
        data["defaultPosition"] = m_default_position.data;
        data["defaultAngle"]    = m_default_angle;

        state.push_back(elem);
    }

private:
    bool    m_infinite_mass = false;
    double  m_default_angle;
    dvec3_s m_default_position;
};

enum class Shape {
    CYLINDER,
    BLOCK,
    SPHERE,
    ROT_CONSTRAINED,
};

class RigidBodyElement : public Object {
public:
    RigidBodyElement(ObjectTree *tree)
        : Object(4, tree) {
        m_orient_c0 = {1.0, 0.0, 0.0};
        m_orient_c1 = {0.0, 1.0, 0.0};
        m_orient_c2 = {0.0, 0.0, 1.0};
    }
    ~RigidBodyElement() {}

    SIMPLE_ACCESSOR_REF(dvec3_s, position, m_position);
    SIMPLE_ACCESSOR_REF(dvec3_s, orient_c0, m_orient_c0);
    SIMPLE_ACCESSOR_REF(dvec3_s, orient_c1, m_orient_c1);
    SIMPLE_ACCESSOR_REF(dvec3_s, orient_c2, m_orient_c2);
    SIMPLE_ACCESSOR(double, mass, m_mass);
    SIMPLE_ACCESSOR(Shape, type, m_type);
    SIMPLE_ACCESSOR(bool, visible, m_visible);

    inline void set_param(size_t param, double v) {
        if (param >= m_params.size()) { throw std::runtime_error("Invalid parameter index"); }
        m_params[param] = v;
    }
    inline const double get_param(size_t param) const {
        if (param >= m_params.size()) { throw std::runtime_error("Invalid parameter index"); }
        return m_params[param];
    }

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["RigidBodyElement::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]       = m_id;
        data["position"] = m_position.data;

        {
            nlohmann::json &orient = data["orientation"];
            orient["c0"]           = m_orient_c0.data;
            orient["c1"]           = m_orient_c1.data;
            orient["c2"]           = m_orient_c2.data;
        }

        data["mass"] = m_mass;

        for (size_t i = 0; i < m_params.size(); i++) { data["parameters"][std::format("value{}", i)] = m_params[i]; }

        data["invisible"] = !m_visible;
        data["type"]      = (int) m_type;

        state.push_back(elem);
    }

private:
    dvec3_s                m_position;
    dvec3_s                m_orient_c0;
    dvec3_s                m_orient_c1;
    dvec3_s                m_orient_c2;
    double                 m_mass;
    std::array<double, 16> m_params;
    Shape                  m_type;
    bool                   m_visible;
};

class GasReservoirModel : public Object {
public:
    GasReservoirModel(ObjectTree *tree)
        : Object(5, tree) {}
    ~GasReservoirModel() {}

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["GasReservoirModel::Specification"];
        nlohmann::json  elem;

        elem["id"]   = m_id;
        elem["data"] = nlohmann::json::object();

        state.push_back(elem);
    }

private:
};

class Tube : public Object {
public:
    Tube(ObjectTree *tree)
        : Object(6, tree) {}
    ~Tube() {}

    SIMPLE_ACCESSOR(int, precision, m_precision);
    SIMPLE_ACCESSOR(int, solver_id, m_solverid);
    SIMPLE_ACCESSOR(int, limiter_id, m_limiterid);

    inline void set_cell_count(size_t c) {
        m_dx.resize(c);
        m_area.resize(c);
    }

    inline size_t cell_count() const {
        return m_dx.size();
    }

    inline double *dx(size_t idx) {
        if (idx >= m_dx.size()) { return nullptr; }
        return &m_dx[idx];
    }
    inline const double *dx(size_t idx) const {
        if (idx >= m_dx.size()) { return nullptr; }
        return &m_dx[idx];
    }

    inline double *area(size_t idx) {
        if (idx >= m_area.size()) { return nullptr; }
        return &m_area[idx];
    }
    inline const double *area(size_t idx) const {
        if (idx >= m_area.size()) { return nullptr; }
        return &m_area[idx];
    }

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["Tube::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"] = m_id;

        data["precision"] = m_precision;
        data["solverId"]  = m_solverid;
        data["limiterId"] = m_limiterid;
        data["dx"]        = m_dx;
        data["area"]      = m_area;

        state.push_back(elem);
    }

private:
    int                 m_precision = 1;
    int                 m_solverid  = 4;
    int                 m_limiterid = 7;
    std::vector<double> m_dx;
    std::vector<double> m_area;
};

class Attachment : public Object {
public:
    enum class AttachmentType : int {
        SENSOR           = 0,
        FREE_ATTACHMENT  = 1,
        BEARING_INNER    = 2,
        BEARING_OUTER    = 3,
        SLIDING_OUTER    = 4,
        SLIDING_INNER    = 5,
        RESERVOIR_OUTER  = 6,
        RIGID_ATTACHMENT = 7,
        SPRING           = 8,
        RESERVOIR_SKIN   = 9,
        RESERVOIR_INNER  = 10,
        FLUID            = 11,
        LOGIC            = 12,
        LOGIC_IN         = 13,
        SPARK            = 14,
    };

    struct Detail {
        const AttachmentType m_type;
    };

    struct Sensor : public Detail {
        Sensor()
            : Detail(AttachmentType::SENSOR) {}
        Sensor(int sensorType, double radius)
            : Detail(AttachmentType::SENSOR),
              m_sensor_type(sensorType),
              m_radius(radius) {}

        int    m_sensor_type;
        double m_radius;

        void serialize(nlohmann::json &json) {
            json["sensorType"] = m_sensor_type;
            json["radius"]     = m_radius;
        }
    };

    struct FreeAttachment : public Detail {
        FreeAttachment()
            : Detail(AttachmentType::FREE_ATTACHMENT) {}
        FreeAttachment(double radius)
            : Detail(AttachmentType::FREE_ATTACHMENT),
              m_radius(radius) {}

        double m_radius;

        void serialize(nlohmann::json &json) {
            json["radius"] = m_radius;
        }
    };

    struct RigidAttachment : public Detail {
        RigidAttachment()
            : Detail(AttachmentType::RIGID_ATTACHMENT) {}
        RigidAttachment(double radius)
            : Detail(AttachmentType::RIGID_ATTACHMENT),
              m_radius(radius) {}

        double m_radius;

        void serialize(nlohmann::json &json) {
            json["radius"] = m_radius;
        }
    };

    struct SpringAttachment : public Detail {
        SpringAttachment()
            : Detail(AttachmentType::SPRING) {}
        SpringAttachment(double ks, double kd, double rest_len, double radius)
            : Detail(AttachmentType::SPRING),
              m_ks(ks),
              m_kd(kd),
              m_rest_len(rest_len),
              m_radius(radius) {}

        double m_ks;
        double m_kd;
        double m_rest_len;
        double m_radius;

        void serialize(nlohmann::json &json) {
            json["radius"]     = m_radius;
            json["ks"]         = m_ks;
            json["kd"]         = m_kd;
            json["restLength"] = m_rest_len;
        }
    };

    struct BearingInner : public Detail {
        BearingInner()
            : Detail(AttachmentType::BEARING_INNER) {}
        BearingInner(double radius, double depth)
            : Detail(AttachmentType::BEARING_INNER),
              m_radius(radius),
              m_depth(depth) {}

        double m_radius;
        double m_depth;

        void serialize(nlohmann::json &json) {
            json["depth"]  = m_depth;
            json["radius"] = m_radius;
        }
    };

    struct BearingOuter : public Detail {
        BearingOuter()
            : Detail(AttachmentType::BEARING_OUTER) {}
        BearingOuter(double inner_radius, double depth, double friction)
            : Detail(AttachmentType::BEARING_OUTER),
              m_inner_radius(inner_radius),
              m_depth(depth),
              m_friction(friction) {}

        double m_inner_radius;
        double m_depth;
        double m_friction;

        void serialize(nlohmann::json &json) {
            json["depth"]       = m_depth;
            json["innerRadius"] = m_inner_radius;
            json["friction"]    = m_friction;
        }
    };

    struct SlidingInner : public Detail {
        SlidingInner()
            : Detail(AttachmentType::SLIDING_INNER) {}
        SlidingInner(Shape shape, double radius, double depth)
            : Detail(AttachmentType::SLIDING_INNER),
              m_shape(shape),
              m_radius(radius),
              m_depth(depth) {}

        Shape  m_shape;
        double m_radius;
        double m_depth;

        void serialize(nlohmann::json &json) {
            json["radius"] = m_radius;
            json["shape"]  = (int) m_shape;
            json["depth"]  = m_depth;
        }
    };

    struct SlidingOuter : public Detail {
        SlidingOuter()
            : Detail(AttachmentType::SLIDING_OUTER) {}
        SlidingOuter(Shape shape, double radius, double depth)
            : Detail(AttachmentType::SLIDING_OUTER),
              m_shape(shape),
              m_radius(radius),
              m_depth(depth) {}

        Shape  m_shape;
        double m_radius;
        double m_depth;

        void serialize(nlohmann::json &json) {
            json["radius"] = m_radius;
            json["shape"]  = (int) m_shape;
            json["depth"]  = m_depth;
        }
    };

    struct ReservoirInner : public Detail {
        ReservoirInner()
            : Detail(AttachmentType::RESERVOIR_INNER) {}
        ReservoirInner(int direction)
            : Detail(AttachmentType::RESERVOIR_INNER),
              m_direction(direction) {}

        int m_direction;

        void serialize(nlohmann::json &json) {
            json["direction"] = m_direction;
        }
    };

    struct ReservoirOuter : public Detail {
        ReservoirOuter()
            : Detail(AttachmentType::RESERVOIR_OUTER) {}
        ReservoirOuter(Shape shape, double radius, double volume)
            : Detail(AttachmentType::RESERVOIR_OUTER),
              m_shape(shape),
              m_radius(radius),
              m_volume(volume) {}

        Shape  m_shape;
        double m_radius;
        double m_volume;

        void serialize(nlohmann::json &json) {
            json["radius"] = m_radius;
            json["shape"]  = (int) m_shape;
            json["volume"] = m_volume;
        }
    };

    struct ReservoirSkin : public Detail {
        ReservoirSkin()
            : Detail(AttachmentType::RESERVOIR_SKIN) {}

        void serialize(nlohmann::json &json) {
            json = nlohmann::json::object();
        }
    };

    struct FluidAttachment : public Detail {
        FluidAttachment()
            : Detail(AttachmentType::FLUID) {}
        FluidAttachment(int direction, double radius)
            : Detail(AttachmentType::FLUID),
              m_direction(direction),
              m_radius(radius) {}

        int    m_direction;
        double m_radius;

        void serialize(nlohmann::json &json) {
            json["radius"]    = m_radius;
            json["direction"] = m_direction;
        }
    };

    struct LogicAttachment : public Detail {
        LogicAttachment()
            : Detail(AttachmentType::LOGIC) {}
        LogicAttachment(double radius, int port)
            : Detail(AttachmentType::LOGIC),
              m_radius(radius),
              m_port(port) {}

        double m_radius;
        int    m_port;

        void serialize(nlohmann::json &json) {
            json["port"]   = m_port;
            json["radius"] = m_radius;
        }
    };

    struct LogicInputAttachment : public Detail {
        LogicInputAttachment()
            : Detail(AttachmentType::LOGIC_IN) {}
        LogicInputAttachment(double radius, int port)
            : Detail(AttachmentType::LOGIC_IN),
              m_radius(radius),
              m_port(port) {}

        double m_radius;
        int    m_port;

        void serialize(nlohmann::json &json) {
            json["port"]   = m_port;
            json["radius"] = m_radius;
        }
    };

    struct SparkSource : public Detail {
        SparkSource()
            : Detail(AttachmentType::SPARK) {}
        SparkSource(double radius)
            : Detail(AttachmentType::SPARK),
              m_radius(radius) {}

        double m_radius;

        void serialize(nlohmann::json &json) {
            json["radius"] = m_radius;
        }
    };

    typedef std::variant<Sensor,
                         FreeAttachment,
                         RigidAttachment,
                         SpringAttachment,
                         BearingInner,
                         BearingOuter,
                         SlidingInner,
                         SlidingOuter,
                         ReservoirInner,
                         ReservoirOuter,
                         ReservoirSkin,
                         FluidAttachment,
                         LogicAttachment,
                         LogicInputAttachment,
                         SparkSource>
            AttachmentDetail;

public:
    Attachment(ObjectTree *tree)
        : Object(2, tree) {}
    ~Attachment() {}

    SIMPLE_ACCESSOR_REF(dvec3_s, position, m_local_pos);
    SIMPLE_ACCESSOR(double, angle, m_local_angle);
    SIMPLE_ACCESSOR_REF(AttachmentDetail, raw_detail, m_detail);

    template<typename T>
    inline T &detail() {
        return std::get<T>(m_detail);
    }

    template<typename T>
    inline const T detail() const {
        return std::get<T>(m_detail);
    }

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["AttachmentPoint::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]            = m_id;
        data["localPosition"] = m_local_pos.data;
        data["localAngle"]    = m_local_angle;
        data["type"]          = std::visit([](auto &obj) { return obj.m_type; }, m_detail);
        std::visit([&data](auto &obj) { return obj.serialize(data["detail"]); }, m_detail);

        state.push_back(elem);
    }

private:
    dvec3_s          m_local_pos;
    double           m_local_angle;
    AttachmentDetail m_detail;
};

class Connection : public Object {
public:
    Connection(ObjectTree *tree)
        : Object(10, tree) {}
    ~Connection() {}

    inline void connect(uint64_t p0id, uint64_t p1id) {
        m_p0 = p0id;
        m_p1 = p1id;
    }

    SIMPLE_ACCESSOR(uint64_t, p0id, m_p0);
    SIMPLE_ACCESSOR(uint64_t, p1id, m_p1);

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["Connection::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]       = m_id;
        data["p0"]["id"] = m_p0;
        data["p1"]["id"] = m_p1;

        state.push_back(elem);
    }

private:
    uint64_t m_p0;
    uint64_t m_p1;
};

class Assembly : public Object {
public:
    Assembly(ObjectTree *tree)
        : Object(0, tree) {}
    ~Assembly() {}

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["Assembly::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"] = m_id;
        data       = nlohmann::json::object();

        state.push_back(elem);
    }

private:
};

class Instance : public Object {
public:
    Instance(ObjectTree *tree)
        : Object(1, tree) {
        m_orient_c0 = {1.0, 0.0, 0.0};
        m_orient_c1 = {0.0, 1.0, 0.0};
        m_orient_c2 = {0.0, 0.0, 1.0};
    }
    ~Instance() {}

    SIMPLE_ACCESSOR(uint64_t, specification, m_specification);
    SIMPLE_ACCESSOR_REF(dvec3_s, position, m_position);
    SIMPLE_ACCESSOR_REF(dvec3_s, orient_c0, m_orient_c0);
    SIMPLE_ACCESSOR_REF(dvec3_s, orient_c1, m_orient_c1);
    SIMPLE_ACCESSOR_REF(dvec3_s, orient_c2, m_orient_c2);
    SIMPLE_ACCESSOR(bool, primary, m_primary)

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["Instance::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]                  = m_id;
        data["specification"]["id"] = m_specification;
        data["position"]            = m_position.data;

        {
            nlohmann::json &orient = data["orientation"];
            orient["c0"]           = m_orient_c0.data;
            orient["c1"]           = m_orient_c1.data;
            orient["c2"]           = m_orient_c2.data;
        }

        data["primary"] = m_primary;

        state.push_back(elem);
    }

private:
    uint64_t m_specification;
    dvec3_s  m_position;
    dvec3_s  m_orient_c0;
    dvec3_s  m_orient_c1;
    dvec3_s  m_orient_c2;
    bool     m_primary;
};

class Valve : public Object {
public:
    Valve(ObjectTree *tree)
        : Object(7, tree) {}
    ~Valve() {}

    SIMPLE_ACCESSOR(double, initial, m_s_initial);
    SIMPLE_ACCESSOR(double, max, m_s_max);
    SIMPLE_ACCESSOR(double, min, m_s_min);

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["Valve::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]        = m_id;
        data["s_initial"] = m_s_initial;
        data["s_min"]     = m_s_min;
        data["s_max"]     = m_s_max;

        state.push_back(elem);
    }


private:
    double m_s_initial;
    double m_s_min;
    double m_s_max;
};

class SparkSource : public Object {
public:
    SparkSource(ObjectTree *tree)
        : Object(8, tree) {}
    ~SparkSource() {}

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["SparkSource::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"] = m_id;
        data       = nlohmann::json::object();

        state.push_back(elem);
    }


private:
};

class EngineController : public Object {
public:
    EngineController(ObjectTree *tree)
        : Object(9, tree) {}
    ~EngineController() {}

    SIMPLE_ACCESSOR(double, rev_max, m_rev_limit_max_speed);
    SIMPLE_ACCESSOR(double, rev_release, m_rev_limit_release_speed);

    inline void set_cylinder_count(size_t cnt) {
        m_cylinder_count = cnt;
        m_firing_angles.resize(cnt);
    }

    inline int cylinder_count() const {
        return m_cylinder_count;
    }

    inline double *firing_angle(size_t cyl) {
        if (cyl >= m_firing_angles.size()) { return nullptr; }
        return &m_firing_angles[cyl];
    }
    inline const double *firing_angle(size_t cyl) const {
        if (cyl >= m_firing_angles.size()) { return nullptr; }
        return &m_firing_angles[cyl];
    }

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["model"]["EngineController::Specification"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]                     = m_id;
        data["cylinderCount"]          = m_cylinder_count;
        data["firingAngles"]           = m_firing_angles;
        data["revLimiterMaxSpeed"]     = m_rev_limit_max_speed;
        data["revLimiterReleaseSpeed"] = m_rev_limit_release_speed;

        state.push_back(elem);
    }


private:
    int                 m_cylinder_count;
    std::vector<double> m_firing_angles;
    double              m_rev_limit_max_speed;
    double              m_rev_limit_release_speed;
};

// state/reset

class ComponentInstance : public Object {
public:
    ComponentInstance(ObjectTree *tree)
        : Object(-2, tree) {}
    ~ComponentInstance() {}

    SIMPLE_ACCESSOR(uint64_t, specification, m_specification);
    SIMPLE_ACCESSOR(uint64_t, context, m_context);
    SIMPLE_ACCESSOR(uint64_t, detail, m_detail);
    SIMPLE_ACCESSOR(uint64_t, instance_mapping, m_instance_mapping);
    SIMPLE_ACCESSOR(int, type, m_type);
    SIMPLE_ACCESSOR(int, referenced_type, m_referenced_type);

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["state"]["Component::Instance"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]                  = m_id;
        data["specification"]["id"] = m_specification;
        data["context"]["id"]       = m_context;
        data["detail"]["id"]        = m_detail;
        data["instanceMapping"]     = m_instance_mapping;
        data["type"]                = m_type;
        data["referencedType"]      = m_referenced_type;

        state.push_back(elem);
    }

private:
    uint64_t m_specification    = nullid;
    uint64_t m_context          = nullid;
    uint64_t m_detail           = nullid;
    uint64_t m_instance_mapping = nullid;
    int      m_type             = 0;
    int      m_referenced_type  = 0;
};

class InstanceMapping : public Object {
public:
    InstanceMapping(ObjectTree *tree)
        : Object(-3, tree) {}
    ~InstanceMapping() {}

    void add_mapping(uint64_t key, uint64_t value) {
        m_object_map.insert({key, value});
    }

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["state"]["InstanceMapping"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"] = m_id;

        {
            nlohmann::json &obj_map = data["objectMap"];
            for (const auto &[key, value] : m_object_map) { obj_map.push_back({{"key", key}, {"value", value}}); }
        }

        state.push_back(elem);
    }

private:
    std::unordered_map<uint64_t, uint64_t> m_object_map;
};

class RigidBodyState : public Object {
public:
    RigidBodyState(ObjectTree *tree)
        : Object(-4, tree) {}
    ~RigidBodyState() {}

    SIMPLE_ACCESSOR_REF(dvec3_s, position, m_position);
    SIMPLE_ACCESSOR_REF(dvec3_s, velocity, m_velocity);
    SIMPLE_ACCESSOR(double, angle, m_angle);
    SIMPLE_ACCESSOR(double, angular_velocity, m_angular_velocity);

    inline void serialize(nlohmann::json &json) override {
        serialize(json, true);
        serialize(json, false);
    }

private:
    void serialize(nlohmann::json &json, bool reset) {
        nlohmann::json &state = json[reset ? "reset" : "state"]["RigidBody::State"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]              = m_id;
        data["position"]        = m_position.data;
        data["velocity"]        = m_velocity.data;
        data["angle"]           = m_angle;
        data["angularVelocity"] = m_angular_velocity;

        state.push_back(elem);
    }

    dvec3_s m_position;
    dvec3_s m_velocity;
    double  m_angle;
    double  m_angular_velocity;
};

class GasReservoirModelState : public Object {
public:
    GasReservoirModelState(ObjectTree *tree)
        : Object(-5, tree) {}
    ~GasReservoirModelState() {}

    SIMPLE_ACCESSOR(double, volume, m_volume);
    SIMPLE_ACCESSOR(double, density, m_density);
    SIMPLE_ACCESSOR(double, internal_energy, m_internal_energy);
    SIMPLE_ACCESSOR(double, burned_volume, m_burned_volume);
    SIMPLE_ACCESSOR(double, unburned_volume, m_unburned_volume);
    SIMPLE_ACCESSOR(double, turbulence_intensity, m_turbulence_intensity);
    SIMPLE_ACCESSOR(double, unburned_fraction_internal_energy, m_unburned_fraction_internal_energy);
    SIMPLE_ACCESSOR(double, burned_fraction_internal_energy, m_burned_fraction_internal_energy);
    SIMPLE_ACCESSOR(bool, combustion_active, m_combustion_active);

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["state"]["GasReservoirModel::State"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]                             = m_id;
        data["volume"]                         = m_volume;
        data["density"]                        = m_density;
        data["internalEnergy"]                 = m_internal_energy;
        data["burnedVolume"]                   = m_burned_volume;
        data["unburnedVolume"]                 = m_unburned_volume;
        data["turbulenceIntensity"]            = m_turbulence_intensity;
        data["unburnedFractionInternalEnergy"] = m_unburned_fraction_internal_energy;
        data["burnedFractionInternalEnergy"]   = m_burned_fraction_internal_energy;
        data["combustionActive"]               = m_combustion_active;

        state.push_back(elem);
    }


private:
    double m_volume;
    double m_density;
    double m_internal_energy;
    double m_burned_volume;
    double m_unburned_volume;
    double m_turbulence_intensity;
    double m_unburned_fraction_internal_energy;
    double m_burned_fraction_internal_energy;
    bool   m_combustion_active;
};

class TubeState : public Object {
public:
    struct OutletState {
        double                m_entropy;
        double                m_sound_pressure;
        std::array<double, 5> m_outlet_velocity;

        void serialize(nlohmann::json &json) {
            json["entropy"]       = m_entropy;
            json["soundPressure"] = m_sound_pressure;

            {
                for (size_t i = 0; i < m_outlet_velocity.size(); i++) {
                    json["outletVelocity"][std::format("value{}", i)] = m_outlet_velocity[i];
                }
            }
        }
    };

public:
    TubeState(ObjectTree *tree)
        : Object(-6, tree) {}
    ~TubeState() {}

    inline OutletState *outlet_state(size_t idx) {
        if (idx >= m_outlet_state.size()) { return nullptr; }
        return &m_outlet_state[idx];
    }
    inline const OutletState *outlet_state(size_t idx) const {
        if (idx >= m_outlet_state.size()) { return nullptr; }
        return &m_outlet_state[idx];
    }

    inline double *density(size_t cyl) {
        if (cyl >= m_density.size()) { return nullptr; }
        return &m_density[cyl];
    }
    inline const double *density(size_t cyl) const {
        if (cyl >= m_density.size()) { return nullptr; }
        return &m_density[cyl];
    }

    inline double *velocity(size_t cyl) {
        if (cyl >= m_velocity.size()) { return nullptr; }
        return &m_velocity[cyl];
    }
    inline const double *velocity(size_t cyl) const {
        if (cyl >= m_velocity.size()) { return nullptr; }
        return &m_velocity[cyl];
    }

    inline double *energy(size_t cyl) {
        if (cyl >= m_energy.size()) { return nullptr; }
        return &m_energy[cyl];
    }
    inline const double *energy(size_t cyl) const {
        if (cyl >= m_energy.size()) { return nullptr; }
        return &m_energy[cyl];
    }

    inline double *T_wall(size_t cyl) {
        if (cyl >= m_T_wall.size()) { return nullptr; }
        return &m_T_wall[cyl];
    }
    inline const double *T_wall(size_t cyl) const {
        if (cyl >= m_T_wall.size()) { return nullptr; }
        return &m_T_wall[cyl];
    }

    void serialize(nlohmann::json &json) override {
        serialize(json, true);
        serialize(json, false);
    }

private:
    void serialize(nlohmann::json &json, bool reset) {
        nlohmann::json &state = json[reset ? "reset" : "state"]["Tube::State"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"] = m_id;

        for (size_t i = 0; i < m_outlet_state.size(); i++) {
            m_outlet_state[i].serialize(data["outletState"][std::format("value{}", i)]);
        }

        data["density"]  = m_density;
        data["velocity"] = m_velocity;
        data["energy"]   = m_energy;
        data["T_wall"]   = m_T_wall;

        state.push_back(elem);
    }

    std::array<OutletState, 2> m_outlet_state;
    std::vector<double>        m_density;
    std::vector<double>        m_velocity;
    std::vector<double>        m_energy;
    std::vector<double>        m_T_wall;
};

class ValveState : public Object {
public:
    ValveState(ObjectTree *tree)
        : Object(-7, tree) {}
    ~ValveState() {}

    SIMPLE_ACCESSOR(double, s, m_s);
    SIMPLE_ACCESSOR(double, s_target, m_s_target);

    void serialize(nlohmann::json &json) override {
        serialize(json, true);
        serialize(json, false);
    }

private:
    void serialize(nlohmann::json &json, bool reset) {
        nlohmann::json &state = json[reset ? "reset" : "state"]["Valve::State"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]       = m_id;
        data["s"]        = m_s;
        data["s_target"] = m_s_target;

        state.push_back(elem);
    }

    double m_s;
    double m_s_target;
};

template<ObjType T>
struct ObjEnumToType;

#define OBJ_ENUM_TO_TYPE(e, t)                                                                                         \
    template<>                                                                                                         \
    struct ObjEnumToType<e> {                                                                                          \
        using type = t;                                                                                                \
    }

OBJ_ENUM_TO_TYPE(ObjType::VALVE_STATE, ValveState);
OBJ_ENUM_TO_TYPE(ObjType::TUBE_STATE, TubeState);
OBJ_ENUM_TO_TYPE(ObjType::GAS_RESERVOIR_MODEL_STATE, GasReservoirModelState);
OBJ_ENUM_TO_TYPE(ObjType::RIGID_BODY_STATE, RigidBodyState);
OBJ_ENUM_TO_TYPE(ObjType::ISNTANCE_MAPPING, InstanceMapping);
OBJ_ENUM_TO_TYPE(ObjType::COMPONENT_INSTANCE, ComponentInstance);
OBJ_ENUM_TO_TYPE(ObjType::COMPONENT, Component);
OBJ_ENUM_TO_TYPE(ObjType::ASSEMBLY, Assembly);
OBJ_ENUM_TO_TYPE(ObjType::INSTANCE, Instance);
OBJ_ENUM_TO_TYPE(ObjType::ATTACHMENT, Attachment);
OBJ_ENUM_TO_TYPE(ObjType::RIGID_BODY, RigidBody);
OBJ_ENUM_TO_TYPE(ObjType::RIGID_BODY_ELEMENT, RigidBodyElement);
OBJ_ENUM_TO_TYPE(ObjType::GAS_RESERVOIR_MODEL, GasReservoirModel);
OBJ_ENUM_TO_TYPE(ObjType::TUBE, Tube);
OBJ_ENUM_TO_TYPE(ObjType::VALVE, Valve);
OBJ_ENUM_TO_TYPE(ObjType::SPARK_SRC, SparkSource);
OBJ_ENUM_TO_TYPE(ObjType::ENGINE_CONTROLLER, EngineController);
OBJ_ENUM_TO_TYPE(ObjType::CONNECTION, Connection);

template<ObjType T>
Object *impl_make_object(ObjectTree *tree) {
    return new typename ObjEnumToType<T>::type(tree);
}

#endif
