# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: monitor_result.proto

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
  name='monitor_result.proto',
  package='adu.common.monitor',
  serialized_pb=_b('\n\x14monitor_result.proto\x12\x12\x61\x64u.common.monitor\"\x93\x01\n\x0eMonitortResult\x12\n\n\x02id\x18\x01 \x01(\r\x12\x0e\n\x06status\x18\x02 \x01(\r\x12\r\n\x05value\x18\x03 \x01(\t\x12\x0e\n\x06module\x18\x04 \x01(\t\x12\x12\n\ntime_stamp\x18\x05 \x01(\x04\x12\x32\n\x06\x64\x65tail\x18\x06 \x03(\x0b\x32\".adu.common.monitor.MonitortResult')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_MONITORTRESULT = _descriptor.Descriptor(
  name='MonitortResult',
  full_name='adu.common.monitor.MonitortResult',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.common.monitor.MonitortResult.id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='adu.common.monitor.MonitortResult.status', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='value', full_name='adu.common.monitor.MonitortResult.value', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='module', full_name='adu.common.monitor.MonitortResult.module', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='time_stamp', full_name='adu.common.monitor.MonitortResult.time_stamp', index=4,
      number=5, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='detail', full_name='adu.common.monitor.MonitortResult.detail', index=5,
      number=6, type=11, cpp_type=10, label=3,
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
  serialized_start=45,
  serialized_end=192,
)

_MONITORTRESULT.fields_by_name['detail'].message_type = _MONITORTRESULT
DESCRIPTOR.message_types_by_name['MonitortResult'] = _MONITORTRESULT

MonitortResult = _reflection.GeneratedProtocolMessageType('MonitortResult', (_message.Message,), dict(
  DESCRIPTOR = _MONITORTRESULT,
  __module__ = 'monitor_result_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.monitor.MonitortResult)
  ))
_sym_db.RegisterMessage(MonitortResult)


# @@protoc_insertion_point(module_scope)
