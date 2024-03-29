# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sensor_velodyne.proto

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


import header_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='sensor_velodyne.proto',
  package='adu.common.sensor',
  serialized_pb=_b('\n\x15sensor_velodyne.proto\x12\x11\x61\x64u.common.sensor\x1a\x0cheader.proto\"\x1e\n\x0eVelodynePacket\x12\x0c\n\x04\x64\x61ta\x18\x01 \x02(\x0c\"\x9f\x02\n\x0cVelodyneScan\x12)\n\x06header\x18\x01 \x01(\x0b\x32\x19.adu.common.header.Header\x12\'\n\x05model\x18\x02 \x02(\x0e\x32\x18.adu.common.sensor.Model\x12%\n\x04mode\x18\x03 \x02(\x0e\x32\x17.adu.common.sensor.Mode\x12\x36\n\x0b\x66iring_pkts\x18\x04 \x03(\x0b\x32!.adu.common.sensor.VelodynePacket\x12;\n\x10positioning_pkts\x18\x05 \x03(\x0b\x32!.adu.common.sensor.VelodynePacket\x12\n\n\x02sn\x18\x06 \x01(\t\x12\x13\n\x08\x62\x61setime\x18\x07 \x02(\x04:\x01\x30*q\n\x05Model\x12\n\n\x06UNKOWN\x10\x00\x12\x0e\n\nHDL64E_S3S\x10\x01\x12\x0e\n\nHDL64E_S3D\x10\x02\x12\r\n\tHDL64E_S2\x10\x03\x12\n\n\x06HDL32E\x10\x04\x12\t\n\x05VLP16\x10\x05\x12\n\n\x06VLP32C\x10\x06\x12\n\n\x06VLS128\x10\x07*)\n\x04Mode\x12\r\n\tSTRONGEST\x10\x01\x12\x08\n\x04LAST\x10\x02\x12\x08\n\x04\x44UAL\x10\x03')
  ,
  dependencies=[header_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

_MODEL = _descriptor.EnumDescriptor(
  name='Model',
  full_name='adu.common.sensor.Model',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKOWN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HDL64E_S3S', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HDL64E_S3D', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HDL64E_S2', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HDL32E', index=4, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VLP16', index=5, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VLP32C', index=6, number=6,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='VLS128', index=7, number=7,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=380,
  serialized_end=493,
)
_sym_db.RegisterEnumDescriptor(_MODEL)

Model = enum_type_wrapper.EnumTypeWrapper(_MODEL)
_MODE = _descriptor.EnumDescriptor(
  name='Mode',
  full_name='adu.common.sensor.Mode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='STRONGEST', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LAST', index=1, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DUAL', index=2, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=495,
  serialized_end=536,
)
_sym_db.RegisterEnumDescriptor(_MODE)

Mode = enum_type_wrapper.EnumTypeWrapper(_MODE)
UNKOWN = 0
HDL64E_S3S = 1
HDL64E_S3D = 2
HDL64E_S2 = 3
HDL32E = 4
VLP16 = 5
VLP32C = 6
VLS128 = 7
STRONGEST = 1
LAST = 2
DUAL = 3



_VELODYNEPACKET = _descriptor.Descriptor(
  name='VelodynePacket',
  full_name='adu.common.sensor.VelodynePacket',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='data', full_name='adu.common.sensor.VelodynePacket.data', index=0,
      number=1, type=12, cpp_type=9, label=2,
      has_default_value=False, default_value=_b(""),
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
  serialized_start=58,
  serialized_end=88,
)


_VELODYNESCAN = _descriptor.Descriptor(
  name='VelodyneScan',
  full_name='adu.common.sensor.VelodyneScan',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='adu.common.sensor.VelodyneScan.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='model', full_name='adu.common.sensor.VelodyneScan.model', index=1,
      number=2, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='mode', full_name='adu.common.sensor.VelodyneScan.mode', index=2,
      number=3, type=14, cpp_type=8, label=2,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='firing_pkts', full_name='adu.common.sensor.VelodyneScan.firing_pkts', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='positioning_pkts', full_name='adu.common.sensor.VelodyneScan.positioning_pkts', index=4,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sn', full_name='adu.common.sensor.VelodyneScan.sn', index=5,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='basetime', full_name='adu.common.sensor.VelodyneScan.basetime', index=6,
      number=7, type=4, cpp_type=4, label=2,
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
  serialized_start=91,
  serialized_end=378,
)

_VELODYNESCAN.fields_by_name['header'].message_type = header_pb2._HEADER
_VELODYNESCAN.fields_by_name['model'].enum_type = _MODEL
_VELODYNESCAN.fields_by_name['mode'].enum_type = _MODE
_VELODYNESCAN.fields_by_name['firing_pkts'].message_type = _VELODYNEPACKET
_VELODYNESCAN.fields_by_name['positioning_pkts'].message_type = _VELODYNEPACKET
DESCRIPTOR.message_types_by_name['VelodynePacket'] = _VELODYNEPACKET
DESCRIPTOR.message_types_by_name['VelodyneScan'] = _VELODYNESCAN
DESCRIPTOR.enum_types_by_name['Model'] = _MODEL
DESCRIPTOR.enum_types_by_name['Mode'] = _MODE

VelodynePacket = _reflection.GeneratedProtocolMessageType('VelodynePacket', (_message.Message,), dict(
  DESCRIPTOR = _VELODYNEPACKET,
  __module__ = 'sensor_velodyne_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.sensor.VelodynePacket)
  ))
_sym_db.RegisterMessage(VelodynePacket)

VelodyneScan = _reflection.GeneratedProtocolMessageType('VelodyneScan', (_message.Message,), dict(
  DESCRIPTOR = _VELODYNESCAN,
  __module__ = 'sensor_velodyne_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.sensor.VelodyneScan)
  ))
_sym_db.RegisterMessage(VelodyneScan)


# @@protoc_insertion_point(module_scope)
