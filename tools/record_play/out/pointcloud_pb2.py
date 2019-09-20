# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: pointcloud.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import simulation_common_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='pointcloud.proto',
  package='adu.simulation',
  serialized_pb=_b('\n\x10pointcloud.proto\x12\x0e\x61\x64u.simulation\x1a\x17simulation_common.proto\"V\n\nPointCloud\x12\x15\n\rtimestamp_sec\x18\x01 \x01(\x01\x12\x31\n\x0bpoint_cloud\x18\x02 \x03(\x0b\x32\x1c.adu.simulation.PolygonPoint')
  ,
  dependencies=[simulation_common_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_POINTCLOUD = _descriptor.Descriptor(
  name='PointCloud',
  full_name='adu.simulation.PointCloud',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp_sec', full_name='adu.simulation.PointCloud.timestamp_sec', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='point_cloud', full_name='adu.simulation.PointCloud.point_cloud', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_end=147,
)

_POINTCLOUD.fields_by_name['point_cloud'].message_type = simulation_common_pb2._POLYGONPOINT
DESCRIPTOR.message_types_by_name['PointCloud'] = _POINTCLOUD

PointCloud = _reflection.GeneratedProtocolMessageType('PointCloud', (_message.Message,), dict(
  DESCRIPTOR = _POINTCLOUD,
  __module__ = 'pointcloud_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.PointCloud)
  ))
_sym_db.RegisterMessage(PointCloud)


# @@protoc_insertion_point(module_scope)
