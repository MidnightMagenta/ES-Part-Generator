#ifndef OBJECT_TREE_H
#define OBJECT_TREE_H

#include <nlohmann/json.hpp>
#include <objects.h>
#include <typeinfo>

#define JSON_VERSION 4

class ObjectTree {
public:
    ObjectTree() {}
    ~ObjectTree() {
        for (auto o : m_objects) { delete o; }
    }

    template<ObjType T>
    inline Object *make_object() {
        Object *new_object = impl_make_object<T>(this);
        m_objects.push_back(new_object);
        return new_object;
    }

    template<ObjType T>
    inline typename ObjEnumToType<T>::type *create_object() {
        return dynamic_cast<typename ObjEnumToType<T>::type *>(make_object<T>());
    }

    Object *find_object(int64_t id) {
        Object *obj = nullptr;

        for (auto o : m_objects) {
            if (o->id() == id) {
                obj = o;
                break;
            }
        }

        return obj;
    }

    template<ObjType T>
    typename ObjEnumToType<T>::type *find_object(int64_t id) {
        Object                          *obj = nullptr;
        typename ObjEnumToType<T>::type *cast_object;

        for (auto o : m_objects) {
            if (o->id() == id) {
                obj = o;
                break;
            }
        }

        if (!obj) { return nullptr; }
        cast_object = dynamic_cast<typename ObjEnumToType<T>::type *>(obj);
        if (!cast_object) { throw std::bad_cast(); }

        return cast_object;
    }

    inline void serialize(nlohmann::ordered_json &json) {
        setup_json_objects(json);

        for (const auto &o : m_objects) { o->serialize(json); }
    }

private:
    inline void setup_json_objects(nlohmann::ordered_json &json) {
        json["model"]       = nlohmann::ordered_json::object();
        json["state"]       = nlohmann::ordered_json::object();
        json["reset"]       = nlohmann::ordered_json::object();
        json["fileVersion"] = JSON_VERSION;

        nlohmann::ordered_json &model = json["model"];
        nlohmann::ordered_json &state = json["state"];
        nlohmann::ordered_json &reset = json["reset"];

        json["fileVersion"] = JSON_VERSION;

        model["Component::Specification"]         = nlohmann::ordered_json::array();
        model["RigidBody::Specification"]         = nlohmann::ordered_json::array();
        model["AttachmentPoint::Specification"]   = nlohmann::ordered_json::array();
        model["RigidBodyElement::Specification"]  = nlohmann::ordered_json::array();
        model["GasReservoirModel::Specification"] = nlohmann::ordered_json::array();
        model["Tube::Specification"]              = nlohmann::ordered_json::array();
        model["Connection::Specification"]        = nlohmann::ordered_json::array();
        model["Assembly::Specification"]          = nlohmann::ordered_json::array();
        model["Instance::Specification"]          = nlohmann::ordered_json::array();
        model["Reference::Specification"]         = nlohmann::ordered_json::array();
        model["Valve::Specification"]             = nlohmann::ordered_json::array();
        model["SparkSource::Specification"]       = nlohmann::ordered_json::array();
        model["EngineController::Specification"]  = nlohmann::ordered_json::array();


        state["Component::Instance"]      = nlohmann::ordered_json::array();
        state["InstanceMapping"]          = nlohmann::ordered_json::array();
        state["RigidBody::State"]         = nlohmann::ordered_json::array();
        state["Assembly::Instance"]       = nlohmann::ordered_json::array();
        state["GasReservoirModel::State"] = nlohmann::ordered_json::array();
        state["Tube::State"]              = nlohmann::ordered_json::array();
        state["Valve::State"]             = nlohmann::ordered_json::array();

        reset["RigidBody::State"]         = nlohmann::ordered_json::array();
        reset["GasReservoirModel::State"] = nlohmann::ordered_json::array();
        reset["Tube::State"]              = nlohmann::ordered_json::array();
    }

private:
    std::vector<Object *> m_objects;
};

#endif
