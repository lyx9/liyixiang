# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: node_exception.proto

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
  name='node_exception.proto',
  package='adu.common.monitor',
  serialized_pb=_b('\n\x14node_exception.proto\x12\x12\x61\x64u.common.monitor\"r\n\rNodeException\x12\x11\n\tnode_name\x18\x01 \x01(\t\x12\x10\n\x08mode_num\x18\x02 \x01(\x05\x12\x11\n\tfault_num\x18\x03 \x01(\x05\x12\x16\n\x0emessage_detail\x18\x04 \x01(\t\x12\x11\n\ttimestamp\x18\x05 \x01(\t')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_NODEEXCEPTION = _descriptor.Descriptor(
  name='NodeException',
  full_name='adu.common.monitor.NodeException',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='node_name', full_name='adu.common.monitor.NodeException.node_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='mode_num', full_name='adu.common.monitor.NodeException.mode_num', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='fault_num', full_name='adu.common.monitor.NodeException.fault_num', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='message_detail', full_name='adu.common.monitor.NodeException.message_detail', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='adu.common.monitor.NodeException.timestamp', index=4,
      number=5, type=9, cpp_type=9, label=1,
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
  serialized_start=44,
  serialized_end=158,
)

DESCRIPTOR.message_types_by_name['NodeException'] = _NODEEXCEPTION

NodeException = _reflection.GeneratedProtocolMessageType('NodeException', (_message.Message,), dict(
  DESCRIPTOR = _NODEEXCEPTION,
  __module__ = 'node_exception_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.monitor.NodeException)
  ))
_sym_db.RegisterMessage(NodeException)


# @@protoc_insertion_point(module_scope)
