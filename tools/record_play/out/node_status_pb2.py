# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: node_status.proto

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
  name='node_status.proto',
  package='adu.common.monitor',
  serialized_pb=_b('\n\x11node_status.proto\x12\x12\x61\x64u.common.monitor\"\x85\x01\n\nNodeStatus\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x11\n\ttimestamp\x18\x02 \x01(\x01\x12\x0e\n\x06status\x18\x03 \x01(\x05\x12.\n\x06\x64\x65tail\x18\x04 \x03(\x0b\x32\x1e.adu.common.monitor.NodeStatus\x12\x16\n\x0ereal_frequency\x18\x05 \x01(\t')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_NODESTATUS = _descriptor.Descriptor(
  name='NodeStatus',
  full_name='adu.common.monitor.NodeStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='adu.common.monitor.NodeStatus.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='adu.common.monitor.NodeStatus.timestamp', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='adu.common.monitor.NodeStatus.status', index=2,
      number=3, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='detail', full_name='adu.common.monitor.NodeStatus.detail', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='real_frequency', full_name='adu.common.monitor.NodeStatus.real_frequency', index=4,
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
  serialized_start=42,
  serialized_end=175,
)

_NODESTATUS.fields_by_name['detail'].message_type = _NODESTATUS
DESCRIPTOR.message_types_by_name['NodeStatus'] = _NODESTATUS

NodeStatus = _reflection.GeneratedProtocolMessageType('NodeStatus', (_message.Message,), dict(
  DESCRIPTOR = _NODESTATUS,
  __module__ = 'node_status_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.monitor.NodeStatus)
  ))
_sym_db.RegisterMessage(NodeStatus)


# @@protoc_insertion_point(module_scope)
