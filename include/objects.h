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

    inline void add_detail(Object *detail) {
        // TODO: should probably check if the detail type is actually valid
        m_detail = detail;
        m_type   = detail->type();
    }

    inline void set_required(bool r) {
        m_required = r;
    }

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

    inline void set_infinite_mass(bool im) {
        m_infinite_mass = im;
    }

    inline void set_default_angle(double da) {
        m_default_angle = da;
    }

    inline void set_detault_position(double x, double y, double z) {
        m_default_position = {x, y, z};
    }

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

    inline void set_position(double x, double y, double z) {
        m_position = {x, y, z};
    }

    inline void set_mass(double mass) {
        m_mass = mass;
    }

    inline void set_param(size_t param, double v) {
        m_params[param] = v;
    }

    inline void set_shape(Shape s) {
        m_type = s;
    }

    inline void set_visible(bool v) {
        m_visible = v;
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

        double m_radius;

        void serialize(nlohmann::json &json) {
            json["radius"] = m_radius;
        }
    };

    struct RigidAttachment : public Detail {
        RigidAttachment()
            : Detail(AttachmentType::RIGID_ATTACHMENT) {}

        double m_radius;

        void serialize(nlohmann::json &json) {
            json["radius"] = m_radius;
        }
    };

    struct SpringAttachment : public Detail {
        SpringAttachment()
            : Detail(AttachmentType::SPRING) {}

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

        int m_direction;

        void serialize(nlohmann::json &json) {
            json["direction"] = m_direction;
        }
    };

    struct ReservoirOuter : public Detail {
        ReservoirOuter()
            : Detail(AttachmentType::RESERVOIR_OUTER) {}

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

    inline void set_position(double x, double y, double z) {
        m_local_pos = {x, y, z};
    }

    inline void set_angle(double angle) {
        m_local_angle = angle;
    }

    inline void set_detail(const AttachmentDetail &detail) {
        std::visit(
                [this](auto &&d) {
                    using T = std::decay_t<decltype(d)>;
                    m_detail.emplace<T>(d);
                },
                detail);
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

    inline void set_connection(uint64_t p0id, uint64_t p1id) {
        m_p0 = p0id;
        m_p1 = p1id;
    }

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

    inline void set_specification(uint64_t specid) {
        m_specification = specid;
    }

    inline void set_position(double x, double y, double z) {
        m_position = {x, y, z};
    }

    inline void set_primary(bool p) {
        m_primary = p;
    }

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

    inline void set_initial(double v) {
        m_s_initial = v;
    }

    inline void set_max(double v) {
        m_s_max = v;
    }

    inline void set_min(double v) {
        m_s_min = v;
    }

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

    inline void set_cylinder_count(int count) {
        m_cylinder_count = count;
        m_firing_angles.resize(count);
    }

    inline void set_firing_angle(size_t cylinder, double angle) {
        if (cylinder >= m_cylinder_count) { return; }
        m_firing_angles[cylinder] = angle;
    }

    inline void set_rev_limit(double max, double release) {
        m_rev_limit_max_speed     = max;
        m_rev_limit_release_speed = release;
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
    uint64_t m_specification;
    uint64_t m_context;
    uint64_t m_detail;
    uint64_t m_instance_mapping;
    int      m_type;
    int      m_referenced_type;
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

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["state"]["RigidBody::State"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]              = m_id;
        data["position"]        = m_position.data;
        data["velocity"]        = m_velocity.data;
        data["angle"]           = m_angle;
        data["angularVelocity"] = m_angular_velocity;

        state.push_back(elem);
    }


private:
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
    double m_combustion_active;
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

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["state"]["Tube::State"];
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


public:
    TubeState(ObjectTree *tree)
        : Object(-6, tree) {}
    ~TubeState() {}

private:
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

    void serialize(nlohmann::json &json) override {
        nlohmann::json &state = json["state"]["Valve::State"];
        nlohmann::json  elem;
        nlohmann::json &data = elem["data"];

        elem["id"]       = m_id;
        data["s"]        = m_s;
        data["s_target"] = m_s_target;

        state.push_back(elem);
    }


private:
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
