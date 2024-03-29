# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: camera_info.proto

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
import camera_region_of_interest_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='camera_info.proto',
  package='adu.common.sensor',
  serialized_pb=_b('\n\x11\x63\x61mera_info.proto\x12\x11\x61\x64u.common.sensor\x1a\x0cheader.proto\x1a\x1f\x63\x61mera_region_of_interest.proto\"\xf4\x01\n\nCameraInfo\x12)\n\x06header\x18\x01 \x01(\x0b\x32\x19.adu.common.header.Header\x12\x0e\n\x06height\x18\x02 \x01(\r\x12\r\n\x05width\x18\x03 \x01(\r\x12\x18\n\x10\x64istortion_model\x18\x04 \x01(\t\x12\t\n\x01\x44\x18\x05 \x03(\x01\x12\t\n\x01K\x18\x06 \x03(\x01\x12\t\n\x01R\x18\x07 \x03(\x01\x12\t\n\x01P\x18\x08 \x03(\x01\x12\x11\n\tbinning_x\x18\t \x01(\r\x12\x11\n\tbinning_y\x18\n \x01(\r\x12\x30\n\x03roi\x18\x0b \x01(\x0b\x32#.adu.common.sensor.RegionOfInterest')
  ,
  dependencies=[header_pb2.DESCRIPTOR,camera_region_of_interest_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_CAMERAINFO = _descriptor.Descriptor(
  name='CameraInfo',
  full_name='adu.common.sensor.CameraInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='adu.common.sensor.CameraInfo.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height', full_name='adu.common.sensor.CameraInfo.height', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='width', full_name='adu.common.sensor.CameraInfo.width', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='distortion_model', full_name='adu.common.sensor.CameraInfo.distortion_model', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='D', full_name='adu.common.sensor.CameraInfo.D', index=4,
      number=5, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='K', full_name='adu.common.sensor.CameraInfo.K', index=5,
      number=6, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='R', full_name='adu.common.sensor.CameraInfo.R', index=6,
      number=7, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='P', full_name='adu.common.sensor.CameraInfo.P', index=7,
      number=8, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='binning_x', full_name='adu.common.sensor.CameraInfo.binning_x', index=8,
      number=9, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='binning_y', full_name='adu.common.sensor.CameraInfo.binning_y', index=9,
      number=10, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='roi', full_name='adu.common.sensor.CameraInfo.roi', index=10,
      number=11, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=88,
  serialized_end=332,
)

_CAMERAINFO.fields_by_name['header'].message_type = header_pb2._HEADER
_CAMERAINFO.fields_by_name['roi'].message_type = camera_region_of_interest_pb2._REGIONOFINTEREST
DESCRIPTOR.message_types_by_name['CameraInfo'] = _CAMERAINFO

CameraInfo = _reflection.GeneratedProtocolMessageType('CameraInfo', (_message.Message,), dict(
  DESCRIPTOR = _CAMERAINFO,
  __module__ = 'camera_info_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.sensor.CameraInfo)
  ))
_sym_db.RegisterMessage(CameraInfo)


# @@protoc_insertion_point(module_scope)
