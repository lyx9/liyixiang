# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: camera_config.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='camera_config.proto',
  package='cybertron.proto',
  serialized_pb=_b('\n\x13\x63\x61mera_config.proto\x12\x0f\x63ybertron.proto\"\xf0\x04\n\x0c\x43\x61meraConfig\x12\x12\n\ncamera_dev\x18\x01 \x02(\t\x12\x10\n\x08\x66rame_id\x18\x02 \x02(\t\x12\x1a\n\x0cpixel_format\x18\x03 \x02(\t:\x04yuyv\x12,\n\tio_method\x18\x04 \x02(\x0e\x32\x19.cybertron.proto.IOMethod\x12\r\n\x05width\x18\x05 \x02(\r\x12\x0e\n\x06height\x18\x06 \x02(\r\x12\x12\n\nframe_rate\x18\x07 \x02(\r\x12\x19\n\nmonochrome\x18\x08 \x02(\x08:\x05\x66\x61lse\x12\x16\n\nbrightness\x18\t \x02(\x05:\x02-1\x12\x14\n\x08\x63ontrast\x18\n \x02(\x05:\x02-1\x12\x16\n\nsaturation\x18\x0b \x02(\x05:\x02-1\x12\x15\n\tsharpness\x18\x0c \x02(\x05:\x02-1\x12\x10\n\x04gain\x18\r \x02(\x05:\x02-1\x12\x19\n\nauto_focus\x18\x0e \x02(\x08:\x05\x66\x61lse\x12\x11\n\x05\x66ocus\x18\x0f \x02(\x05:\x02-1\x12\x1b\n\rauto_exposure\x18\x10 \x02(\x08:\x04true\x12\x15\n\x08\x65xposure\x18\x11 \x02(\x05:\x03\x31\x30\x30\x12 \n\x12\x61uto_white_balance\x18\x12 \x02(\x08:\x04true\x12\x1b\n\rwhite_balance\x18\x13 \x02(\x05:\x04\x34\x30\x30\x30\x12\x1a\n\x0f\x62ytes_per_pixel\x18\x14 \x02(\r:\x01\x33\x12\x1b\n\rtrigger_param\x18\x15 \x02(\t:\x04\x66\x32\x66\x66\x12\x1d\n\x11metric_error_code\x18\x16 \x02(\r:\x02\x31\x31\x12\x1b\n\x0f\x66pga_dev_number\x18\x17 \x02(\x05:\x02-1\x12\x1d\n\x11\x63\x61mera_seq_number\x18\x18 \x02(\x05:\x02-1*`\n\x08IOMethod\x12\x15\n\x11IO_METHOD_UNKNOWN\x10\x00\x12\x12\n\x0eIO_METHOD_READ\x10\x01\x12\x12\n\x0eIO_METHOD_MMAP\x10\x02\x12\x15\n\x11IO_METHOD_USERPTR\x10\x03')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

_IOMETHOD = _descriptor.EnumDescriptor(
  name='IOMethod',
  full_name='cybertron.proto.IOMethod',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='IO_METHOD_UNKNOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IO_METHOD_READ', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IO_METHOD_MMAP', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IO_METHOD_USERPTR', index=3, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=667,
  serialized_end=763,
)
_sym_db.RegisterEnumDescriptor(_IOMETHOD)

IOMethod = enum_type_wrapper.EnumTypeWrapper(_IOMETHOD)
IO_METHOD_UNKNOWN = 0
IO_METHOD_READ = 1
IO_METHOD_MMAP = 2
IO_METHOD_USERPTR = 3



_CAMERACONFIG = _descriptor.Descriptor(
  name='CameraConfig',
  full_name='cybertron.proto.CameraConfig',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='camera_dev', full_name='cybertron.proto.CameraConfig.camera_dev', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='frame_id', full_name='cybertron.proto.CameraConfig.frame_id', index=1,
      number=2, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pixel_format', full_name='cybertron.proto.CameraConfig.pixel_format', index=2,
      number=3, type=9, cpp_type=9, label=2,
      has_default_value=True, default_value=_b("yuyv").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='io_method', full_name='cybertron.proto.CameraConfig.io_method', index=3,
      number=4, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='width', full_name='cybertron.proto.CameraConfig.width', index=4,
      number=5, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height', full_name='cybertron.proto.CameraConfig.height', index=5,
      number=6, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='frame_rate', full_name='cybertron.proto.CameraConfig.frame_rate', index=6,
      number=7, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='monochrome', full_name='cybertron.proto.CameraConfig.monochrome', index=7,
      number=8, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='brightness', full_name='cybertron.proto.CameraConfig.brightness', index=8,
      number=9, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='contrast', full_name='cybertron.proto.CameraConfig.contrast', index=9,
      number=10, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='saturation', full_name='cybertron.proto.CameraConfig.saturation', index=10,
      number=11, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sharpness', full_name='cybertron.proto.CameraConfig.sharpness', index=11,
      number=12, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gain', full_name='cybertron.proto.CameraConfig.gain', index=12,
      number=13, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='auto_focus', full_name='cybertron.proto.CameraConfig.auto_focus', index=13,
      number=14, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='focus', full_name='cybertron.proto.CameraConfig.focus', index=14,
      number=15, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='auto_exposure', full_name='cybertron.proto.CameraConfig.auto_exposure', index=15,
      number=16, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='exposure', full_name='cybertron.proto.CameraConfig.exposure', index=16,
      number=17, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=100,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='auto_white_balance', full_name='cybertron.proto.CameraConfig.auto_white_balance', index=17,
      number=18, type=8, cpp_type=7, label=2,
      has_default_value=True, default_value=True,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='white_balance', full_name='cybertron.proto.CameraConfig.white_balance', index=18,
      number=19, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=4000,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='bytes_per_pixel', full_name='cybertron.proto.CameraConfig.bytes_per_pixel', index=19,
      number=20, type=13, cpp_type=3, label=2,
      has_default_value=True, default_value=3,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='trigger_param', full_name='cybertron.proto.CameraConfig.trigger_param', index=20,
      number=21, type=9, cpp_type=9, label=2,
      has_default_value=True, default_value=_b("f2ff").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='metric_error_code', full_name='cybertron.proto.CameraConfig.metric_error_code', index=21,
      number=22, type=13, cpp_type=3, label=2,
      has_default_value=True, default_value=11,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='fpga_dev_number', full_name='cybertron.proto.CameraConfig.fpga_dev_number', index=22,
      number=23, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='camera_seq_number', full_name='cybertron.proto.CameraConfig.camera_seq_number', index=23,
      number=24, type=5, cpp_type=1, label=2,
      has_default_value=True, default_value=-1,
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
  serialized_start=41,
  serialized_end=665,
)

_CAMERACONFIG.fields_by_name['io_method'].enum_type = _IOMETHOD
DESCRIPTOR.message_types_by_name['CameraConfig'] = _CAMERACONFIG
DESCRIPTOR.enum_types_by_name['IOMethod'] = _IOMETHOD

CameraConfig = _reflection.GeneratedProtocolMessageType('CameraConfig', (_message.Message,), dict(
  DESCRIPTOR = _CAMERACONFIG,
  __module__ = 'camera_config_pb2'
  # @@protoc_insertion_point(class_scope:cybertron.proto.CameraConfig)
  ))
_sym_db.RegisterMessage(CameraConfig)


# @@protoc_insertion_point(module_scope)
