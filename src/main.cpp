#include <iostream>
#include <nlohmann/json.hpp>
#include <object_tree.h>

int main() {
    nlohmann::json test;
    ObjectTree     tree;
    tree.make_object<ObjType::VALVE_STATE>();
    tree.make_object<ObjType::TUBE_STATE>();
    tree.make_object<ObjType::GAS_RESERVOIR_MODEL_STATE>();
    tree.make_object<ObjType::RIGID_BODY_STATE>();
    dynamic_cast<InstanceMapping *>(tree.make_object<ObjType::ISNTANCE_MAPPING>())->add_mapping(1, 2);
    tree.make_object<ObjType::COMPONENT_INSTANCE>();
    tree.make_object<ObjType::COMPONENT>();
    tree.make_object<ObjType::ASSEMBLY>();
    dynamic_cast<Attachment *>(tree.make_object<ObjType::ATTACHMENT>())->set_detail(Attachment::Sensor(1, 2));
    tree.make_object<ObjType::RIGID_BODY>();
    tree.make_object<ObjType::RIGID_BODY_ELEMENT>();
    tree.make_object<ObjType::GAS_RESERVOIR_MODEL>();
    tree.make_object<ObjType::TUBE>();
    tree.make_object<ObjType::VALVE>();
    tree.make_object<ObjType::SPARK_SRC>();
    tree.make_object<ObjType::ENGINE_CONTROLLER>();
    tree.make_object<ObjType::CONNECTION>();

    tree.serialize(test);

    std::cout << test;
}
