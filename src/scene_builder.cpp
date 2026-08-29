#include <objects.h>
#include <scene_builder.h>

void SceneBuilder::initialize_tree() {
    Component         *root, *assembly_root, *instance_root, *root_component;
    Instance          *root_instance, *scene_instnace;
    ComponentInstance *root_cinstance, *root_comp_cinstance;
    InstanceMapping   *root_mapping;

    root          = m_tree.create_object<ObjType::COMPONENT>();
    root_instance = m_tree.create_object<ObjType::INSTANCE>();
    root->add_detail(root_instance);

    assembly_root                  = m_tree.create_object<ObjType::COMPONENT>();
    root_instance->specification() = assembly_root->id();
    assembly_root->add_detail(m_tree.create_object<ObjType::ASSEMBLY>());

    instance_root = m_tree.create_object<ObjType::COMPONENT>();
    assembly_root->add_child(instance_root);
    scene_instnace = m_tree.create_object<ObjType::INSTANCE>();
    instance_root->add_detail(scene_instnace);
    m_root_component                = m_tree.create_object<ObjType::COMPONENT>();
    scene_instnace->specification() = m_root_component->id();
    m_root_component->add_detail(m_tree.create_object<ObjType::ASSEMBLY>());

    root_mapping                       = m_tree.create_object<ObjType::ISNTANCE_MAPPING>();
    root_cinstance                     = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    root_cinstance->specification()    = root->id();
    root_cinstance->instance_mapping() = root_mapping->id();
    root_cinstance->type()             = ObjType::INSTANCE;
    root_cinstance->referenced_type()  = ObjType::ASSEMBLY;
    root_mapping->add_mapping(root_instance->id(), root_cinstance->id());

    root_comp_cinstance                     = m_tree.create_object<ObjType::COMPONENT_INSTANCE>();
    root_comp_cinstance->specification()    = root_component->id();
    root_comp_cinstance->context()          = root_cinstance->id();
    root_comp_cinstance->instance_mapping() = root_mapping->id();
    root_comp_cinstance->type()             = ObjType::INSTANCE;
    root_comp_cinstance->referenced_type()  = ObjType::ASSEMBLY;
    root_mapping->add_mapping(root_component->id(), root_comp_cinstance->id());
}
