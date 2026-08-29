#ifndef SCENE_BUILDER_H
#define SCENE_BUILDER_H

#include <object_tree.h>
#include <objects.h>

class SceneBuilder {
public:
    SceneBuilder() {}

private:
    void initialize_tree();

    Component *m_root_component;
    ObjectTree m_tree;
};

#endif
