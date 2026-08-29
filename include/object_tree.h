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
    typename ObjEnumToType<T>::type *find_object(int64_t id) {
        Object                          *obj;
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

    inline void serialize(nlohmann::json &json) {
        json["fileVersion"] = JSON_VERSION;
        for (const auto &o : m_objects) { o->serialize(json); }
    }

private:
    std::vector<Object *> m_objects;
};

#endif
