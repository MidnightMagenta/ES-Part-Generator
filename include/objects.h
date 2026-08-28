#ifndef ESJSON_OBJECTS
#define ESJSON_OBJECTS

#include <array>
#include <cstdint>
#include <md_math.hpp>
#include <unordered_map>
#include <variant>
#include <vector>

class Object {
public:
    static constexpr int64_t nullid = -1;

public:
    Object() = delete;
    explicit Object(int type)
        : m_type(type),
          m_id(next_id++) {}

    virtual ~Object() {}

    int type() const {
        return m_type;
    }

protected:
    int            m_type;
    static int64_t next_id;
    int64_t        m_id;
};

class Component : public Object {
public:
    Component(Object *detail)
        : Object(-1) {
        m_detail = detail;
        m_type   = detail->type();
    }
    ~Component() {}

private:
    Object               *m_parent = nullptr;
    std::vector<Object *> m_children;

    Object *m_detail;
    int     m_type;

    bool m_required = false;
};

class RigidBody : public Object {
public:
public:
    RigidBody()
        : Object(3) {}

private:
    bool         m_infinite_mass = false;
    double       m_default_angle;
    mdm::dvec3_s m_default_position;
};

enum class Shape {
    CYLINDER,
    BLOCK,
    SPHERE,
    ROT_CONSTRAINED,
};

class RigidBodyElement : public Object {
public:
    RigidBodyElement()
        : Object(4) {}
    ~RigidBodyElement() {}

private:
    mdm::dvec3_s           m_position;
    mdm::dvec3_s           m_orient_c0;
    mdm::dvec3_s           m_orient_c1;
    mdm::dvec3_s           m_orient_c2;
    double                 m_mass;
    std::array<double, 16> m_params;
    Shape                  m_type;
    bool                   m_visible;
};

class GasReservoirModel : public Object {
public:
    GasReservoirModel()
        : Object(5) {}
    ~GasReservoirModel() {}

private:
};

class Tube : public Object {
public:
    Tube()
        : Object(6) {}
    ~Tube() {}

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
        SPARK            = 14,
    };

    struct Detail {
        const AttachmentType m_type;
    };

    struct Sensor : public Detail {
        Sensor()
            : Detail(AttachmentType::SENSOR) {}

        int    m_sensor_type;
        double m_radius;
    };

    struct FreeAttachment : public Detail {
        FreeAttachment()
            : Detail(AttachmentType::FREE_ATTACHMENT) {}

        double m_radius;
    };

    struct RigidAttachment : public Detail {
        RigidAttachment()
            : Detail(AttachmentType::RIGID_ATTACHMENT) {}

        double m_radius;
    };

    struct SpringAttachment : public Detail {
        SpringAttachment()
            : Detail(AttachmentType::SPRING) {}

        double m_ks;
        double m_kd;
        double m_rest_len;
        double m_radius;
    };

    struct BearingInner : public Detail {
        BearingInner()
            : Detail(AttachmentType::BEARING_INNER) {}

        double m_radius;
        double m_depth;
    };

    struct BearingOuter : public Detail {
        BearingOuter()
            : Detail(AttachmentType::BEARING_OUTER) {}

        double m_inner_radius;
        double m_depth;
        double m_friction;
    };

    struct SlidingInner : public Detail {
        SlidingInner()
            : Detail(AttachmentType::SLIDING_INNER) {}

        Shape  m_shape;
        double m_radius;
        double m_depth;
    };

    struct SlidingOuter : public Detail {
        SlidingOuter()
            : Detail(AttachmentType::SLIDING_OUTER) {}

        Shape  m_shape;
        double m_radius;
        double m_depth;
    };

    struct ReservoirInner : public Detail {
        ReservoirInner()
            : Detail(AttachmentType::RESERVOIR_INNER) {}

        int m_direction;
    };

    struct ReservoirOuter : public Detail {
        ReservoirOuter()
            : Detail(AttachmentType::RESERVOIR_OUTER) {}

        Shape  m_shape;
        double m_radius;
        double m_volume;
    };

    struct ReservoirSkin : public Detail {
        ReservoirSkin()
            : Detail(AttachmentType::RESERVOIR_SKIN) {}
    };

    struct FluidAttachment : public Detail {
        FluidAttachment()
            : Detail(AttachmentType::FLUID) {}

        int    m_direction;
        double m_radius;
    };

    struct LogicAttachment : public Detail {
        LogicAttachment()
            : Detail(AttachmentType::LOGIC) {}

        double m_radius;
        int    m_port;
    };

    struct SparkSource : public Detail {
        SparkSource()
            : Detail(AttachmentType::SPARK) {}

        double m_radius;
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
                         SparkSource>
            AttachmentDetail;

public:
    Attachment()
        : Object(2) {}
    ~Attachment() {}

private:
    mdm::dvec3_s     m_local_pos;
    double           m_local_angle;
    AttachmentDetail m_detail;
};

class Connection : public Object {
public:
    Connection()
        : Object(10) {}
    ~Connection() {}

private:
    int64_t m_p0;
    int64_t m_p1;
};

class Assembly : public Object {
public:
    Assembly()
        : Object(0) {}
    ~Assembly() {}

private:
};

class Instance : public Object {
public:
    Instance()
        : Object(1) {}
    ~Instance() {}

private:
    int64_t      m_specification;
    mdm::dvec3_s m_position;
    mdm::dvec3_s m_orient_c0;
    mdm::dvec3_s m_orient_c1;
    mdm::dvec3_s m_orient_c2;
    bool         m_primary;
};

class Valve : public Object {
public:
    Valve()
        : Object(7) {}
    ~Valve() {}

private:
    double m_s_initial;
    double m_s_min;
    double m_s_max;
};

class SparkSource : public Object {
public:
    SparkSource()
        : Object(8) {}
    ~SparkSource() {}

private:
};

class EngineController : public Object {
public:
    EngineController()
        : Object(9) {}
    ~EngineController() {}

private:
    int                 m_cylinder_count;
    std::vector<double> m_firing_angles;
};

// state/reset

class ComponentInstance : public Object {
public:
    ComponentInstance()
        : Object(-2) {}
    ~ComponentInstance() {}

private:
    int64_t m_specification;
    int64_t m_context;
    int64_t m_detail;
    int64_t m_instance_mapping;
    int     m_type;
    int     m_referenced_type;
};

class InstanceMapping : public Object {
public:
    InstanceMapping()
        : Object(-3) {}
    ~InstanceMapping() {}

private:
    std::unordered_map<int64_t, int64_t> m_object_map;
};

class RigidBodyState : public Object {
public:
    RigidBodyState()
        : Object(-4) {}
    ~RigidBodyState() {}

private:
    mdm::dvec3_s m_position;
    mdm::dvec3_s m_velocity;
    double       m_angle;
    double       m_angular_velocity;
};

class GasReservoirModelState : public Object {
public:
    GasReservoirModelState()
        : Object(-5) {}
    ~GasReservoirModelState() {}

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
    };

public:
    TubeState()
        : Object(-6) {}
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
    ValveState()
        : Object(-7) {}
    ~ValveState() {}

private:
    double m_s;
    double m_s_target;
};

#endif
