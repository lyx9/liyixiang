# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: recorder_cmd.proto

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
  name='recorder_cmd.proto',
  package='adu.common.data_recorder',
  serialized_pb=_b('\n\x12recorder_cmd.proto\x12\x18\x61\x64u.common.data_recorder\"M\n\x07\x43ommand\x12\x14\n\x0c\x63ommand_name\x18\x01 \x01(\t\x12\x13\n\x0b\x63ommand_cmd\x18\x02 \x01(\t\x12\x17\n\x0f\x63ommand_purpose\x18\x03 \x01(\t')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_COMMAND = _descriptor.Descriptor(
  name='Command',
  full_name='adu.common.data_recorder.Command',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='command_name', full_name='adu.common.data_recorder.Command.command_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='command_cmd', full_name='adu.common.data_recorder.Command.command_cmd', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='command_purpose', full_name='adu.common.data_recorder.Command.command_purpose', index=2,
      number=3, type=9, cpp_type=9, label=1,
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
  serialized_start=48,
  serialized_end=125,
)

DESCRIPTOR.message_types_by_name['Command'] = _COMMAND

Command = _reflection.GeneratedProtocolMessageType('Command', (_message.Message,), dict(
  DESCRIPTOR = _COMMAND,
  __module__ = 'recorder_cmd_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.data_recorder.Command)
  ))
_sym_db.RegisterMessage(Command)


# @@protoc_insertion_point(module_scope)
