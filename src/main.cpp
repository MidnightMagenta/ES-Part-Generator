#include <iostream>
#include <nlohmann/json.hpp>
#include <scene_builder.h>

int main() {
    SceneBuilder sb;

    Object *rb = sb.create_rigid_body(sb.root());
    sb.create_rigid_body_element(rb, {0.0, 0.0, 0.0}, 0.01, {0.2, 0.05}, Shape::CYLINDER, true);
    sb.create_attachment(rb, {0.0, 0.0, 0.1}, 0.0, Attachment::BearingOuter(0.025, 0.01, 0.0));
    sb.create_attachment(rb, {0.0, 0.0, -0.1}, 0.0, Attachment::BearingOuter(0.025, 0.01, 0.0));

    nlohmann::ordered_json json;
    sb.serialize(json);

    std::cout << json;
}
