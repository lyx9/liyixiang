# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: velodyne_config.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import sensor_velodyne_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='velodyne_config.proto',
  package='cybertron.proto',
  serialized_pb=_b('\n\x15velodyne_config.proto\x12\x0f\x63ybertron.proto\x1a\x15sensor_velodyne.proto\"\x99\x06\n\x0eVelodyneConfig\x12\x10\n\x08\x66rame_id\x18\x01 \x02(\t\x12\'\n\x05model\x18\x02 \x02(\x0e\x32\x18.adu.common.sensor.Model\x12%\n\x04mode\x18\x03 \x02(\x0e\x32\x17.adu.common.sensor.Mode\x12\x11\n\tpcap_file\x18\x04 \x01(\t\x12\x10\n\x08npackets\x18\x05 \x01(\r\x12\x0b\n\x03rpm\x18\x06 \x02(\x02\x12\x1a\n\x0cuse_gps_time\x18\x07 \x02(\x08:\x04true\x12\x14\n\ttime_zone\x18\x08 \x02(\x05:\x01\x38\x12\x18\n\x10\x66iring_data_port\x18\t \x02(\r\x12\x1d\n\x15positioning_data_port\x18\n \x01(\r\x12\x11\n\tmax_range\x18\x0b \x02(\x02\x12\x11\n\tmin_range\x18\x0c \x02(\x02\x12\x11\n\tmax_angle\x18\r \x01(\x02\x12\x11\n\tmin_angle\x18\x0e \x01(\x02\x12\x16\n\x0eview_direction\x18\x0f \x01(\x02\x12\x12\n\nview_width\x18\x10 \x01(\x02\x12\x18\n\x10\x63\x61libration_file\x18\x11 \x02(\t\x12\x18\n\torganized\x18\x12 \x02(\x08:\x05\x66\x61lse\x12\x14\n\x0cprefix_angle\x18\x13 \x01(\x05\x12\x17\n\x0f\x63\x61r_config_path\x18\x14 \x01(\t\x12\x1b\n\x13\x63\x61r_distance_buffer\x18\x15 \x01(\x02\x12$\n\x1c\x64\x65\x66\x61ult_front_edge_to_center\x18\x16 \x01(\x02\x12#\n\x1b\x64\x65\x66\x61ult_back_edge_to_center\x18\x17 \x01(\x02\x12#\n\x1b\x64\x65\x66\x61ult_left_edge_to_center\x18\x18 \x01(\x02\x12$\n\x1c\x64\x65\x66\x61ult_right_edge_to_center\x18\x19 \x01(\x02\x12\x1a\n\x12gpu_sched_strategy\x18\x1a \x01(\t\x12\x10\n\x08priority\x18\x1b \x01(\r\x12\x1b\n\x0cis_set_sched\x18\x1c \x01(\x08:\x05\x66\x61lse\x12\x18\n\x10gpu_scan_channel\x18\x1d \x01(\t\x12\x15\n\rgpu_scan_node\x18\x1e \x01(\t')
  ,
  dependencies=[sensor_velodyne_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_VELODYNECONFIG = _descriptor.Descriptor(
  name='VelodyneConfig',
  full_name='cybertron.proto.VelodyneConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='cybertron.proto.VelodyneConfig.frame_id', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='model', full_name='cybertron.proto.VelodyneConfig.model', index=1,
      number=2, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='mode', full_name='cybertron.proto.VelodyneConfig.mode', index=2,
      number=3, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pcap_file', full_name='cybertron.proto.VelodyneConfig.pcap_file', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='npackets', full_name='cybertron.proto.VelodyneConfig.npackets', index=4,
      number=5, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='rpm', full_name='cybertron.proto.VelodyneConfig.rpm', index=5,
      number=6, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='use_gps_time', full_name='cybertron.proto.VelodyneConfig.use_gps_time', index=6,
      number=7, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='time_zone', full_name='cybertron.proto.VelodyneConfig.time_zone', index=7,
      number=8, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=8,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='firing_data_port', full_name='cybertron.proto.VelodyneConfig.firing_data_port', index=8,
      number=9, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='positioning_data_port', full_name='cybertron.proto.VelodyneConfig.positioning_data_port', index=9,
      number=10, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_range', full_name='cybertron.proto.VelodyneConfig.max_range', index=10,
      number=11, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='min_range', full_name='cybertron.proto.VelodyneConfig.min_range', index=11,
      number=12, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='max_angle', full_name='cybertron.proto.VelodyneConfig.max_angle', index=12,
      number=13, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='min_angle', full_name='cybertron.proto.VelodyneConfig.min_angle', index=13,
      number=14, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='view_direction', full_name='cybertron.proto.VelodyneConfig.view_direction', index=14,
      number=15, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='view_width', full_name='cybertron.proto.VelodyneConfig.view_width', index=15,
      number=16, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='calibration_file', full_name='cybertron.proto.VelodyneConfig.calibration_file', index=16,
      number=17, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='organized', full_name='cybertron.proto.VelodyneConfig.organized', index=17,
      number=18, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='prefix_angle', full_name='cybertron.proto.VelodyneConfig.prefix_angle', index=18,
      number=19, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='car_config_path', full_name='cybertron.proto.VelodyneConfig.car_config_path', index=19,
      number=20, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='car_distance_buffer', full_name='cybertron.proto.VelodyneConfig.car_distance_buffer', index=20,
      number=21, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='default_front_edge_to_center', full_name='cybertron.proto.VelodyneConfig.default_front_edge_to_center', index=21,
      number=22, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='default_back_edge_to_center', full_name='cybertron.proto.VelodyneConfig.default_back_edge_to_center', index=22,
      number=23, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='default_left_edge_to_center', full_name='cybertron.proto.VelodyneConfig.default_left_edge_to_center', index=23,
      number=24, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='default_right_edge_to_center', full_name='cybertron.proto.VelodyneConfig.default_right_edge_to_center', index=24,
      number=25, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gpu_sched_strategy', full_name='cybertron.proto.VelodyneConfig.gpu_sched_strategy', index=25,
      number=26, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='priority', full_name='cybertron.proto.VelodyneConfig.priority', index=26,
      number=27, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='is_set_sched', full_name='cybertron.proto.VelodyneConfig.is_set_sched', index=27,
      number=28, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gpu_scan_channel', full_name='cybertron.proto.VelodyneConfig.gpu_scan_channel', index=28,
      number=29, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gpu_scan_node', full_name='cybertron.proto.VelodyneConfig.gpu_scan_node', index=29,
      number=30, type=9, cpp_type=9, label=1,
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
  serialized_start=66,
  serialized_end=859,
)

_VELODYNECONFIG.fields_by_name['model'].enum_type = sensor_velodyne_pb2._MODEL
_VELODYNECONFIG.fields_by_name['mode'].enum_type = sensor_velodyne_pb2._MODE
DESCRIPTOR.message_types_by_name['VelodyneConfig'] = _VELODYNECONFIG

VelodyneConfig = _reflection.GeneratedProtocolMessageType('VelodyneConfig', (_message.Message,), dict(
  DESCRIPTOR = _VELODYNECONFIG,
  __module__ = 'velodyne_config_pb2'
  # @@protoc_insertion_point(class_scope:cybertron.proto.VelodyneConfig)
  ))
_sym_db.RegisterMessage(VelodyneConfig)


# @@protoc_insertion_point(module_scope)
