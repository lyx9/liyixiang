# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: novatel_header.proto

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
  name='novatel_header.proto',
  package='adu.common.sensor',
  serialized_pb=_b('\n\x14novatel_header.proto\x12\x11\x61\x64u.common.sensor\"\x80\x02\n\rNovatelHeader\x12\n\n\x02id\x18\x01 \x01(\r\x12\x10\n\x08msg_type\x18\x02 \x01(\r\x12\x11\n\tport_addr\x18\x03 \x01(\r\x12\x0e\n\x06length\x18\x04 \x01(\r\x12\x10\n\x08sequence\x18\x05 \x01(\r\x12\x11\n\tidle_time\x18\x06 \x01(\r\x12\x13\n\x0btime_status\x18\x07 \x01(\r\x12\x10\n\x08gps_week\x18\x08 \x01(\r\x12\x1d\n\x15gps_week_milliseconds\x18\t \x01(\r\x12\x17\n\x0freceiver_status\x18\n \x01(\r\x12\x10\n\x08reserved\x18\x0b \x01(\r\x12\x18\n\x10software_version\x18\x0c \x01(\r')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_NOVATELHEADER = _descriptor.Descriptor(
  name='NovatelHeader',
  full_name='adu.common.sensor.NovatelHeader',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.common.sensor.NovatelHeader.id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='msg_type', full_name='adu.common.sensor.NovatelHeader.msg_type', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='port_addr', full_name='adu.common.sensor.NovatelHeader.port_addr', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='length', full_name='adu.common.sensor.NovatelHeader.length', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='sequence', full_name='adu.common.sensor.NovatelHeader.sequence', index=4,
      number=5, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='idle_time', full_name='adu.common.sensor.NovatelHeader.idle_time', index=5,
      number=6, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='time_status', full_name='adu.common.sensor.NovatelHeader.time_status', index=6,
      number=7, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gps_week', full_name='adu.common.sensor.NovatelHeader.gps_week', index=7,
      number=8, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gps_week_milliseconds', full_name='adu.common.sensor.NovatelHeader.gps_week_milliseconds', index=8,
      number=9, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='receiver_status', full_name='adu.common.sensor.NovatelHeader.receiver_status', index=9,
      number=10, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='reserved', full_name='adu.common.sensor.NovatelHeader.reserved', index=10,
      number=11, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='software_version', full_name='adu.common.sensor.NovatelHeader.software_version', index=11,
      number=12, type=13, cpp_type=3, label=1,
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
  serialized_start=44,
  serialized_end=300,
)

DESCRIPTOR.message_types_by_name['NovatelHeader'] = _NOVATELHEADER

NovatelHeader = _reflection.GeneratedProtocolMessageType('NovatelHeader', (_message.Message,), dict(
  DESCRIPTOR = _NOVATELHEADER,
  __module__ = 'novatel_header_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.sensor.NovatelHeader)
  ))
_sym_db.RegisterMessage(NovatelHeader)


# @@protoc_insertion_point(module_scope)
