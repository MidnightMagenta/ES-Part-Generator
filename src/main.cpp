#include <iostream>
#include <nlohmann/json.hpp>
#include <object_tree.h>
#include <scene_builder.h>

int main() {
    try {
        nlohmann::ordered_json test;
        SceneBuilder           sb;

        Object *rb = sb.create_rigid_body(sb.root());
        sb.create_rigid_body_element(rb, {0.0, 0.0, 0.0}, 1.0, {1.0, 1.0, 1.0}, Shape::BLOCK, true);

        sb.serialize(test);

        std::cout << test;
    } catch (std::exception &e) { std::cerr << e.what() << '\n'; } catch (...) {
        std::cerr << "unkown error\n";
    }
}
