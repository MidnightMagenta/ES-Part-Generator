# Engine Simulator JSON Format Specification (reverse engineered)

## Index

- [model](#model)
- [Component::Specification](#Component::Specification)
- [RigidBody::Specification](#RigidBody::Specification)
- [RigidBodyElement::Specification](#RigidBodyElement::Specification)
- [AttachmentPoint::Specification](#AttachmentPoint::Specification)
- [GasReservoirModel::Specification](#GasReservoirModel::Specification)
- [Instance::Specification](#Instance::Specification)
- [Valve::Specification](#Valve::Specification)
- [SparkSource::Specification](#SparkSource::Specificationa)
- [Connection::Specification](#Connection::Specification)

## model

The `model` object contains the top level arrays for specification objects, which specify the layout of the scene.

### Component::Specification

An example `Component::Specification` object

```json
"Component::Specification": [
    {
        "id": 0,
        "data": {
            "parent": {
                "id": 18446744073709551615
            },
            "children": [
                {
                    "id": 4
                }
            ],
            "type": 0,
            "detail": {
                "id": 1
            },
            "required": false
        }
    },
]
```

| field                     | type     | description                                              |
| ------------------------- | -------- | -------------------------------------------------------- |
| id                        | uint64_t | The ID of the object                                     |
| data                      | object   | Object specifying the Component data                     |
| data::parent              | object   | Object holding the ID of the Component's parent          |
| data::parent::id          | uint64_t | The ID of the parent `Component::Specification`          |
| data::children            | array    | Array of objects holding the ID's of children components |
| data::children::\[n\]::id | uint64_t | The ID of the child `Component::Specification`           |
| data::type                | int      | The type of the `defail` object                          |
| data::detail              | object   | Object holding the ID of the detail object               |
| data::detail::id          | uint64_t | The ID of the detail object                              |
| required                  | bool     | unknown function                                         |

### RigidBody::Specification

An example `RigidBody::Specification` object

```json
"RigidBody::Specification": [
    {
        "id": 35,
        "data": {
            "infiniteMass": false,
            "defaultPosition": [
                -0.06189250946044922,
                0.054803382605314258,
                0.0
            ],
            "defaultAngle": 0.0
        }
    },
]
```

Object type: 3

| field                 | type     | description                                                 |
| --------------------- | -------- | ----------------------------------------------------------- |
| id                    | uint64_t | The ID of the object                                        |
| data                  | object   | Object specifying the Component data                        |
| data::infiniteMass    | bool     | Flag for infinite mass (immovable) rigid body               |
| data::defaultPosition | Vec3     | 3D Vector specifying the default position of the rigid body |
| data::defaultAngle    | f64      | Default angle of the rigid body                             |

### RigidBodyElement::Specification

An example `RigidBodyElement::Specification` object

```json
"RigidBodyElement::Specification": [
    {
        "id": 37,
        "data": {
            "position": [
                0.0,
                0.0,
                0.0
            ],
            "orientation": {
                "c0": [
                    0.0,
                    0.0,
                    -1.0
                ],
                "c1": [
                    0.0,
                    1.0,
                    0.0
                ],
                "c2": [
                    1.0,
                    0.0,
                    0.0
                ]
            },
            "mass": 0.2309070600388497,
            "parameters": {
                "value0": 0.0034999999999999998,
                "value1": 0.15,
                "value2": 0.0,
                "value3": 0.0,
                "value4": 0.0,
                "value5": 0.0,
                "value6": 0.0,
                "value7": 0.0,
                "value8": 0.0,
                "value9": 0.0,
                "value10": 0.0,
                "value11": 0.0,
                "value12": 0.0,
                "value13": 0.0,
                "value14": 0.0,
                "value15": 0.0
            },
            "invisible": false,
            "type": 0
        }
    },
]
```

Object type: 4

| field             | type     | description                                                                   |
| ----------------- | -------- | ----------------------------------------------------------------------------- |
| id                | uint64_t | The ID of the object                                                          |
| data              | object   | Object specifying the Component data                                          |
| data::position    | Vec3     | Position of the rigid body element relative to the parent object              |
| data::orientation | Mat3x3   | Orientation of the rigid body element relative to the parent object           |
| data::mass        | f64      | Mass of the object                                                            |
| data::parameters  | array    | Parameters of the object - meaning depends on the type, usually object's size |
| data::invisible   | bool     | Whether an object is visible in the scene                                     |
| data::type        | int      | Type of the element (usually shape)                                           |

### AttachmentPoint::Specification

An example `AttachmentPoint::Specification` object

```json
"AttachmentPoint::Specification": [
    {
        "id": 39,
        "data": {
            "localPosition": [
                -0.075,
                0.0,
                0.0
            ],
            "localAngle": 0.0,
            "type": 1,
            "detail": {
                "radius": 0.005
            }
        }
    },
]
```

Object type: 2

| field               | type     | description                                       |
| ------------------- | -------- | ------------------------------------------------- |
| id                  | uint64_t | The ID of the object                              |
| data                | object   | Object specifying the Component data              |
| data::localPosition | Vec3     | Position of the attachment relative to the parent |
| data::localAngle    | f64      | Angle of the attachment relative to the parent    |
| data::type          | int      | Type of the attachment                            |
| data::detail        | Object   | Object specifying the attachment parameters       |

| type | purpose                    | values (detail)                                           |
| ---- | -------------------------- | --------------------------------------------------------- |
| 0    | sensor                     | sensorType (int), radius (float)                          |
| 1    | free attachment            | radius (float)                                            |
| 2    | brearing (inner)           | radius (float), depth (float)                             |
| 3    | bearing (outer)            | innerRadius (float), depth (float), friction (float)      |
| 4    | sliding attachment (outer) | shape (int), radius (float), depth (float)                |
| 5    | sliding attachment (inner) | shape (int), radius (float), depth (float)                |
| 6    | reservoir tip ("outer")    | shape (int), radius (float), volume (float)               |
| 7    | rigid attachment           | radius (float)                                            |
| 8    | spring attachment          | ks (float), kd (float) restLength (float), radius (float) |
| 9    | reservoir "skin"           | none                                                      |
| 10   | reservoir "tips" ("inner") | direction (int)                                           |
| 11   | fluid attachment           | direction (int), radius (float)                           |
| 12   | logic attachment (out)     | radius (float), port (int)                                |
| 13   | logic attachment (in)      | radius (float), port (int)                                |
| 14   | spark source               | radius (float)                                            |

### GasReservoirModel::Specification

An example `GasReservoirModel::Specification` object

```json
"GasReservoirModel::Specification": [
    {
        "id": 85,
        "data": {}
    },
]
```

Object type: 5

| field | type     | description                          |
| ----- | -------- | ------------------------------------ |
| id    | uint64_t | The ID of the object                 |
| data  | object   | Object specifying the Component data |

### Instance::Specification

An example `Instance::Specification` object

```json
"Instance::Specification": [
    {
        "id": 5,
        "data": {
            "specification": {
                "id": 2
            },
            "position": [
                0.0,
                0.0,
                0.0
            ],
            "orientation": {
                "c0": [
                    -1.0,
                    0.0,
                    0.0
                ],
                "c1": [
                    0.0,
                    0.0,
                    1.0
                ],
                "c2": [
                    0.0,
                    1.0,
                    0.0
                ]
            },
            "primary": false
        }
    },
]
```

Object type: 1

| field                   | type     | description                                                                       |
| ----------------------- | -------- | --------------------------------------------------------------------------------- |
| id                      | uint64_t | The ID of the object                                                              |
| data                    | object   | Object specifying the Component data                                              |
| data::specification     | object   | Object holding the ID of the specification component                              |
| data::specification::id | uint64_t | The ID of the root component of the instance                                      |
| data::position          | Vec3     | Position of the instance                                                          |
| data::orientation       | Mat3x3   | Orientation of the instance                                                       |
| data::pirmary           | bool     | unkonwn, possibly whether this is the "main" instance, as in, the "root"/"parent" |

### Assembly::Specification

An example `Assembly::Specification` object

```json
"Assembly::Specification": [
    {
        "id": 1,
        "data": {}
    },
]
```

Object type: 0

| field | type     | description                          |
| ----- | -------- | ------------------------------------ |
| id    | uint64_t | The ID of the object                 |
| data  | object   | Object specifying the Component data |

### Valve::Specification

An example `Valve::Specification` object

```json
"Valve::Specification": [
    {
        "id": 1,
        "data": {
            "s_initial": 1.0,
            "s_min": 0.0,
            "s_max": 1.0
        }
    },
]
```

Object type: 7

| field           | type     | description                              |
| --------------- | -------- | ---------------------------------------- |
| id              | uint64_t | The ID of the object                     |
| data            | object   | Object specifying the Component data     |
| data::s_initial | f64      | Initial value of the valve's opening     |
| data::s_min     | f64      | Minimum valve opening (aka fully open)   |
| data::s_max     | f64      | Maximum valve opening (aka fully closed) |

### SparkSource::Specification

An example `SparkSource::Specification` object

```json
"SparkSource::Specification": [
    {
        "id": 11,
        "data": {}
    },
]
```

Object type: 8

| field | type     | description                          |
| ----- | -------- | ------------------------------------ |
| id    | uint64_t | The ID of the object                 |
| data  | object   | Object specifying the Component data |

### EngineController::Specification

An example `EngineController::Specification` object

```json
"EngineController::Specification": [
    {
        "id": 754,
        "data": {
            "cylinderCount": 8,
            "firingAngles": [
                0.0,
                1.5707963267948966,
                3.141592653589793,
                4.71238898038469,
                6.283185307179586,
                7.853981633974483,
                9.42477796076938,
                10.995574287564276
            ],
            "revLimiterMaxSpeed": 837.7580409572781,
            "revLimiterReleaseSpeed": 827.2860654453122
        }
    }
]
```

Object type: 9

| field                        | type     | description                                                              |
| ---------------------------- | -------- | ------------------------------------------------------------------------ |
| id                           | uint64_t | The ID of the object                                                     |
| data                         | object   | Object specifying the Component data                                     |
| data::cylinderCount          | int      | Number of cylinders                                                      |
| data::firingAngles           | array    | Array of f64\'s of firing angles in radians (when the spark will happen) |
| data::revLimiterMaxSpeed     | f64      | Max speed for rev limiter                                                |
| data::revLimiterReleaseSpeed | f64      | Release speed for limiter                                                |

### Connection::Specification

An example `Connection::Specification` object

```json
"Connection::Specification": [
    {
        "id": 754,
        "data": {
            "p0": {
                "id": 142
            },
            "p1": {
                "id": 152
            }
        }
    }
]
```

Object type: 10

| field        | type     | description                          |
| ------------ | -------- | ------------------------------------ |
| id           | uint64_t | The ID of the object                 |
| data         | object   | Object specifying the Component data |
| data::p0::id | uint64_t | ID of the first connection           |
| data::p1::id | uint64_t | ID of the second connection          |
