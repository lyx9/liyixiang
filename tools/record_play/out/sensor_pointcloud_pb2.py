# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sensor_pointcloud.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import header_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='sensor_pointcloud.proto',
  package='adu.common.sensor',
  serialized_pb=_b('\n\x17sensor_pointcloud.proto\x12\x11\x61\x64u.common.sensor\x1a\x0cheader.proto\"d\n\nPointXYZIT\x12\x0e\n\x01x\x18\x01 \x02(\x02:\x03nan\x12\x0e\n\x01y\x18\x02 \x02(\x02:\x03nan\x12\x0e\n\x01z\x18\x03 \x02(\x02:\x03nan\x12\x14\n\tintensity\x18\x04 \x02(\r:\x01\x30\x12\x10\n\x05stamp\x18\x05 \x02(\x04:\x01\x30\"\xc2\x01\n\nPointCloud\x12)\n\x06header\x18\x01 \x01(\x0b\x32\x19.adu.common.header.Header\x12\x10\n\x08\x66rame_id\x18\x02 \x01(\t\x12\x10\n\x08is_dense\x18\x03 \x01(\x08\x12,\n\x05point\x18\x04 \x03(\x0b\x32\x1d.adu.common.sensor.PointXYZIT\x12\x18\n\x10measurement_time\x18\x05 \x01(\x01\x12\r\n\x05width\x18\x06 \x01(\r\x12\x0e\n\x06height\x18\x07 \x01(\r')
  ,
  dependencies=[header_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_POINTXYZIT = _descriptor.Descriptor(
  name='PointXYZIT',
  full_name='adu.common.sensor.PointXYZIT',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='adu.common.sensor.PointXYZIT.x', index=0,
      number=1, type=2, cpp_type=6, label=2,
      has_default_value=True, default_value=(1e10000 * 0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='adu.common.sensor.PointXYZIT.y', index=1,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=True, default_value=(1e10000 * 0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='z', full_name='adu.common.sensor.PointXYZIT.z', index=2,
      number=3, type=2, cpp_type=6, label=2,
      has_default_value=True, default_value=(1e10000 * 0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='intensity', full_name='adu.common.sensor.PointXYZIT.intensity', index=3,
      number=4, type=13, cpp_type=3, label=2,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stamp', full_name='adu.common.sensor.PointXYZIT.stamp', index=4,
      number=5, type=4, cpp_type=4, label=2,
      has_default_value=True, default_value=0,
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
  serialized_start=60,
  serialized_end=160,
)


_POINTCLOUD = _descriptor.Descriptor(
  name='PointCloud',
  full_name='adu.common.sensor.PointCloud',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='adu.common.sensor.PointCloud.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='adu.common.sensor.PointCloud.frame_id', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='is_dense', full_name='adu.common.sensor.PointCloud.is_dense', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='point', full_name='adu.common.sensor.PointCloud.point', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='measurement_time', full_name='adu.common.sensor.PointCloud.measurement_time', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='width', full_name='adu.common.sensor.PointCloud.width', index=5,
      number=6, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height', full_name='adu.common.sensor.PointCloud.height', index=6,
      number=7, type=13, cpp_type=3, label=1,
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
  serialized_start=163,
  serialized_end=357,
)

_POINTCLOUD.fields_by_name['header'].message_type = header_pb2._HEADER
_POINTCLOUD.fields_by_name['point'].message_type = _POINTXYZIT
DESCRIPTOR.message_types_by_name['PointXYZIT'] = _POINTXYZIT
DESCRIPTOR.message_types_by_name['PointCloud'] = _POINTCLOUD

PointXYZIT = _reflection.GeneratedProtocolMessageType('PointXYZIT', (_message.Message,), dict(
  DESCRIPTOR = _POINTXYZIT,
  __module__ = 'sensor_pointcloud_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.sensor.PointXYZIT)
  ))
_sym_db.RegisterMessage(PointXYZIT)

PointCloud = _reflection.GeneratedProtocolMessageType('PointCloud', (_message.Message,), dict(
  DESCRIPTOR = _POINTCLOUD,
  __module__ = 'sensor_pointcloud_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.sensor.PointCloud)
  ))
_sym_db.RegisterMessage(PointCloud)


# @@protoc_insertion_point(module_scope)
