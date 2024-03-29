# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: perception_sensor_object.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='perception_sensor_object.proto',
  package='adu.common.perception.viz',
  serialized_pb=_b('\n\x1eperception_sensor_object.proto\x12\x19\x61\x64u.common.perception.viz\"\x1f\n\x07Point2D\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\"*\n\x07Point3D\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\t\n\x01z\x18\x03 \x01(\x01\"\xc0\x01\n\x10\x43\x61meraSupplement\x12\x36\n\nupper_left\x18\x01 \x01(\x0b\x32\".adu.common.perception.viz.Point2D\x12\x37\n\x0blower_right\x18\x02 \x01(\x0b\x32\".adu.common.perception.viz.Point2D\x12\x1a\n\x0elocal_track_id\x18\x03 \x01(\x05:\x02-1\x12\x10\n\x04pts8\x18\x04 \x03(\x01\x42\x02\x10\x01\x12\r\n\x05\x61lpha\x18\x05 \x01(\x01\"\xfe\x04\n\x0cSensorObject\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x32\n\x06\x63\x65nter\x18\x02 \x01(\x0b\x32\".adu.common.perception.viz.Point3D\x12\r\n\x05theta\x18\x03 \x01(\x01\x12\x35\n\tdirection\x18\x04 \x01(\x0b\x32\".adu.common.perception.viz.Point3D\x12\x0e\n\x06length\x18\x05 \x01(\x01\x12\r\n\x05width\x18\x06 \x01(\x01\x12\x0e\n\x06height\x18\x07 \x01(\x01\x12\x43\n\x04type\x18\x08 \x01(\x0e\x32,.adu.common.perception.viz.SensorObject.Type:\x07UNKNOWN\x12\x10\n\x08track_id\x18\t \x01(\x05\x12\x34\n\x08velocity\x18\n \x01(\x0b\x32\".adu.common.perception.viz.Point3D\x12\x19\n\rpolygon_point\x18\x0b \x03(\x01\x42\x02\x10\x01\x12\x11\n\x05\x63loud\x18\x0c \x03(\x01\x42\x02\x10\x01\x12\x17\n\x0b\x63loud_world\x18\r \x03(\x01\x42\x02\x10\x01\x12\x15\n\rtracking_time\x18\x0e \x01(\x01\x12\x1b\n\x13latest_tracked_time\x18\x0f \x01(\x01\x12\x46\n\x11\x63\x61mera_supplement\x18\x10 \x01(\x0b\x32+.adu.common.perception.viz.CameraSupplement\"i\n\x04Type\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x13\n\x0fUNKNOWN_MOVABLE\x10\x01\x12\x15\n\x11UNKNOWN_UNMOVABLE\x10\x02\x12\x0e\n\nPEDESTRIAN\x10\x03\x12\x0b\n\x07\x42ICYCLE\x10\x04\x12\x0b\n\x07VEHICLE\x10\x05\"\x89\x01\n\rSensorObjects\x12\x11\n\ttimestamp\x18\x01 \x01(\x01\x12\x38\n\x07objects\x18\x02 \x03(\x0b\x32\'.adu.common.perception.viz.SensorObject\x12\x1d\n\x11sensor2world_pose\x18\x03 \x03(\x01\x42\x02\x10\x01\x12\x0c\n\x04name\x18\x04 \x01(\t')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_SENSOROBJECT_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='adu.common.perception.viz.SensorObject.Type',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_MOVABLE', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_UNMOVABLE', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PEDESTRIAN', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BICYCLE', index=4, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VEHICLE', index=5, number=5,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=867,
  serialized_end=972,
)
_sym_db.RegisterEnumDescriptor(_SENSOROBJECT_TYPE)


_POINT2D = _descriptor.Descriptor(
  name='Point2D',
  full_name='adu.common.perception.viz.Point2D',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='adu.common.perception.viz.Point2D.x', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='adu.common.perception.viz.Point2D.y', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=61,
  serialized_end=92,
)


_POINT3D = _descriptor.Descriptor(
  name='Point3D',
  full_name='adu.common.perception.viz.Point3D',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='adu.common.perception.viz.Point3D.x', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='adu.common.perception.viz.Point3D.y', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='z', full_name='adu.common.perception.viz.Point3D.z', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=94,
  serialized_end=136,
)


_CAMERASUPPLEMENT = _descriptor.Descriptor(
  name='CameraSupplement',
  full_name='adu.common.perception.viz.CameraSupplement',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='upper_left', full_name='adu.common.perception.viz.CameraSupplement.upper_left', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lower_right', full_name='adu.common.perception.viz.CameraSupplement.lower_right', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='local_track_id', full_name='adu.common.perception.viz.CameraSupplement.local_track_id', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pts8', full_name='adu.common.perception.viz.CameraSupplement.pts8', index=3,
      number=4, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))),
    _descriptor.FieldDescriptor(
      name='alpha', full_name='adu.common.perception.viz.CameraSupplement.alpha', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=139,
  serialized_end=331,
)


_SENSOROBJECT = _descriptor.Descriptor(
  name='SensorObject',
  full_name='adu.common.perception.viz.SensorObject',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.common.perception.viz.SensorObject.id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='center', full_name='adu.common.perception.viz.SensorObject.center', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='theta', full_name='adu.common.perception.viz.SensorObject.theta', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='direction', full_name='adu.common.perception.viz.SensorObject.direction', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='length', full_name='adu.common.perception.viz.SensorObject.length', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='width', full_name='adu.common.perception.viz.SensorObject.width', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height', full_name='adu.common.perception.viz.SensorObject.height', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='type', full_name='adu.common.perception.viz.SensorObject.type', index=7,
      number=8, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='track_id', full_name='adu.common.perception.viz.SensorObject.track_id', index=8,
      number=9, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='velocity', full_name='adu.common.perception.viz.SensorObject.velocity', index=9,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='polygon_point', full_name='adu.common.perception.viz.SensorObject.polygon_point', index=10,
      number=11, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))),
    _descriptor.FieldDescriptor(
      name='cloud', full_name='adu.common.perception.viz.SensorObject.cloud', index=11,
      number=12, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))),
    _descriptor.FieldDescriptor(
      name='cloud_world', full_name='adu.common.perception.viz.SensorObject.cloud_world', index=12,
      number=13, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))),
    _descriptor.FieldDescriptor(
      name='tracking_time', full_name='adu.common.perception.viz.SensorObject.tracking_time', index=13,
      number=14, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='latest_tracked_time', full_name='adu.common.perception.viz.SensorObject.latest_tracked_time', index=14,
      number=15, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='camera_supplement', full_name='adu.common.perception.viz.SensorObject.camera_supplement', index=15,
      number=16, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _SENSOROBJECT_TYPE,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=334,
  serialized_end=972,
)


_SENSOROBJECTS = _descriptor.Descriptor(
  name='SensorObjects',
  full_name='adu.common.perception.viz.SensorObjects',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='adu.common.perception.viz.SensorObjects.timestamp', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='objects', full_name='adu.common.perception.viz.SensorObjects.objects', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sensor2world_pose', full_name='adu.common.perception.viz.SensorObjects.sensor2world_pose', index=2,
      number=3, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))),
    _descriptor.FieldDescriptor(
      name='name', full_name='adu.common.perception.viz.SensorObjects.name', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=975,
  serialized_end=1112,
)

_CAMERASUPPLEMENT.fields_by_name['upper_left'].message_type = _POINT2D
_CAMERASUPPLEMENT.fields_by_name['lower_right'].message_type = _POINT2D
_SENSOROBJECT.fields_by_name['center'].message_type = _POINT3D
_SENSOROBJECT.fields_by_name['direction'].message_type = _POINT3D
_SENSOROBJECT.fields_by_name['type'].enum_type = _SENSOROBJECT_TYPE
_SENSOROBJECT.fields_by_name['velocity'].message_type = _POINT3D
_SENSOROBJECT.fields_by_name['camera_supplement'].message_type = _CAMERASUPPLEMENT
_SENSOROBJECT_TYPE.containing_type = _SENSOROBJECT
_SENSOROBJECTS.fields_by_name['objects'].message_type = _SENSOROBJECT
DESCRIPTOR.message_types_by_name['Point2D'] = _POINT2D
DESCRIPTOR.message_types_by_name['Point3D'] = _POINT3D
DESCRIPTOR.message_types_by_name['CameraSupplement'] = _CAMERASUPPLEMENT
DESCRIPTOR.message_types_by_name['SensorObject'] = _SENSOROBJECT
DESCRIPTOR.message_types_by_name['SensorObjects'] = _SENSOROBJECTS

Point2D = _reflection.GeneratedProtocolMessageType('Point2D', (_message.Message,), dict(
  DESCRIPTOR = _POINT2D,
  __module__ = 'perception_sensor_object_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.perception.viz.Point2D)
  ))
_sym_db.RegisterMessage(Point2D)

Point3D = _reflection.GeneratedProtocolMessageType('Point3D', (_message.Message,), dict(
  DESCRIPTOR = _POINT3D,
  __module__ = 'perception_sensor_object_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.perception.viz.Point3D)
  ))
_sym_db.RegisterMessage(Point3D)

CameraSupplement = _reflection.GeneratedProtocolMessageType('CameraSupplement', (_message.Message,), dict(
  DESCRIPTOR = _CAMERASUPPLEMENT,
  __module__ = 'perception_sensor_object_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.perception.viz.CameraSupplement)
  ))
_sym_db.RegisterMessage(CameraSupplement)

SensorObject = _reflection.GeneratedProtocolMessageType('SensorObject', (_message.Message,), dict(
  DESCRIPTOR = _SENSOROBJECT,
  __module__ = 'perception_sensor_object_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.perception.viz.SensorObject)
  ))
_sym_db.RegisterMessage(SensorObject)

SensorObjects = _reflection.GeneratedProtocolMessageType('SensorObjects', (_message.Message,), dict(
  DESCRIPTOR = _SENSOROBJECTS,
  __module__ = 'perception_sensor_object_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.perception.viz.SensorObjects)
  ))
_sym_db.RegisterMessage(SensorObjects)


_CAMERASUPPLEMENT.fields_by_name['pts8'].has_options = True
_CAMERASUPPLEMENT.fields_by_name['pts8']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))
_SENSOROBJECT.fields_by_name['polygon_point'].has_options = True
_SENSOROBJECT.fields_by_name['polygon_point']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))
_SENSOROBJECT.fields_by_name['cloud'].has_options = True
_SENSOROBJECT.fields_by_name['cloud']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))
_SENSOROBJECT.fields_by_name['cloud_world'].has_options = True
_SENSOROBJECT.fields_by_name['cloud_world']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))
_SENSOROBJECTS.fields_by_name['sensor2world_pose'].has_options = True
_SENSOROBJECTS.fields_by_name['sensor2world_pose']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\020\001'))
# @@protoc_insertion_point(module_scope)
