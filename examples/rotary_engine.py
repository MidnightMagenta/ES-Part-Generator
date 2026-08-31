import esjson

scene = esjson.SceneBuilder()
root = scene.root_id()


def CreateBlock(num_cylinders):
    root_rb = scene.create_rigid_body(root)
    scene.create_attachment_bearing_outer(
        root_rb, esjson.Vec3(0, 0, 0), 0, 0.025, 0.127, 0.0
    )


CreateBlock(1)

with open("build/rotary_engine.json", "w+") as f:
    f.write(scene.serialize())
