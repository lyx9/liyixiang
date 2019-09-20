# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: pad_msg.proto

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
import global_adc_status_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='pad_msg.proto',
  package='adu.common.control',
  serialized_pb=_b('\n\rpad_msg.proto\x12\x12\x61\x64u.common.control\x1a\x0cheader.proto\x1a\x17global_adc_status.proto\"\xa0\x01\n\nPadMessage\x12)\n\x06header\x18\x01 \x01(\x0b\x32\x19.adu.common.header.Header\x12\x34\n\x0c\x64riving_mode\x18\x02 \x01(\x0e\x32\x1e.adu.common.status.DrivingMode\x12\x31\n\x06\x61\x63tion\x18\x03 \x01(\x0e\x32!.adu.common.control.DrivingAction*/\n\rDrivingAction\x12\x08\n\x04STOP\x10\x00\x12\t\n\x05START\x10\x01\x12\t\n\x05RESET\x10\x02')
  ,
  dependencies=[header_pb2.DESCRIPTOR,global_adc_status_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

_DRIVINGACTION = _descriptor.EnumDescriptor(
  name='DrivingAction',
  full_name='adu.common.control.DrivingAction',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='STOP', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='START', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RESET', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=239,
  serialized_end=286,
)
_sym_db.RegisterEnumDescriptor(_DRIVINGACTION)

DrivingAction = enum_type_wrapper.EnumTypeWrapper(_DRIVINGACTION)
STOP = 0
START = 1
RESET = 2



_PADMESSAGE = _descriptor.Descriptor(
  name='PadMessage',
  full_name='adu.common.control.PadMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='adu.common.control.PadMessage.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='driving_mode', full_name='adu.common.control.PadMessage.driving_mode', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='action', full_name='adu.common.control.PadMessage.action', index=2,
      number=3, type=14, cpp_type=8, label=1,
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
  serialized_start=77,
  serialized_end=237,
)

_PADMESSAGE.fields_by_name['header'].message_type = header_pb2._HEADER
_PADMESSAGE.fields_by_name['driving_mode'].enum_type = global_adc_status_pb2._DRIVINGMODE
_PADMESSAGE.fields_by_name['action'].enum_type = _DRIVINGACTION
DESCRIPTOR.message_types_by_name['PadMessage'] = _PADMESSAGE
DESCRIPTOR.enum_types_by_name['DrivingAction'] = _DRIVINGACTION

PadMessage = _reflection.GeneratedProtocolMessageType('PadMessage', (_message.Message,), dict(
  DESCRIPTOR = _PADMESSAGE,
  __module__ = 'pad_msg_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.control.PadMessage)
  ))
_sym_db.RegisterMessage(PadMessage)


# @@protoc_insertion_point(module_scope)