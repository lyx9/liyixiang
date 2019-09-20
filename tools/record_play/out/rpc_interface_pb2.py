# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: rpc_interface.proto

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
  name='rpc_interface.proto',
  package='adu.common.rpc',
  serialized_pb=_b('\n\x13rpc_interface.proto\x12\x0e\x61\x64u.common.rpc\"\xbf\x01\n\x07Request\x12\x32\n\x0creceive_time\x18\x01 \x01(\x0b\x32\x1c.adu.common.rpc.Request.Time\x12\x30\n\x07payload\x18\x02 \x03(\x0b\x32\x1f.adu.common.rpc.Request.Message\x1a&\n\x04Time\x12\x0f\n\x07seconds\x18\x01 \x01(\r\x12\r\n\x05nanos\x18\x02 \x01(\r\x1a&\n\x07Message\x12\r\n\x05topic\x18\x01 \x01(\t\x12\x0c\n\x04\x64\x61ta\x18\x02 \x01(\x0c')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_REQUEST_TIME = _descriptor.Descriptor(
  name='Time',
  full_name='adu.common.rpc.Request.Time',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='seconds', full_name='adu.common.rpc.Request.Time.seconds', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='nanos', full_name='adu.common.rpc.Request.Time.nanos', index=1,
      number=2, type=13, cpp_type=3, label=1,
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
  serialized_start=153,
  serialized_end=191,
)

_REQUEST_MESSAGE = _descriptor.Descriptor(
  name='Message',
  full_name='adu.common.rpc.Request.Message',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='topic', full_name='adu.common.rpc.Request.Message.topic', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='data', full_name='adu.common.rpc.Request.Message.data', index=1,
      number=2, type=12, cpp_type=9, label=1,
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
  serialized_start=193,
  serialized_end=231,
)

_REQUEST = _descriptor.Descriptor(
  name='Request',
  full_name='adu.common.rpc.Request',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='receive_time', full_name='adu.common.rpc.Request.receive_time', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='payload', full_name='adu.common.rpc.Request.payload', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_REQUEST_TIME, _REQUEST_MESSAGE, ],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=40,
  serialized_end=231,
)

_REQUEST_TIME.containing_type = _REQUEST
_REQUEST_MESSAGE.containing_type = _REQUEST
_REQUEST.fields_by_name['receive_time'].message_type = _REQUEST_TIME
_REQUEST.fields_by_name['payload'].message_type = _REQUEST_MESSAGE
DESCRIPTOR.message_types_by_name['Request'] = _REQUEST

Request = _reflection.GeneratedProtocolMessageType('Request', (_message.Message,), dict(

  Time = _reflection.GeneratedProtocolMessageType('Time', (_message.Message,), dict(
    DESCRIPTOR = _REQUEST_TIME,
    __module__ = 'rpc_interface_pb2'
    # @@protoc_insertion_point(class_scope:adu.common.rpc.Request.Time)
    ))
  ,

  Message = _reflection.GeneratedProtocolMessageType('Message', (_message.Message,), dict(
    DESCRIPTOR = _REQUEST_MESSAGE,
    __module__ = 'rpc_interface_pb2'
    # @@protoc_insertion_point(class_scope:adu.common.rpc.Request.Message)
    ))
  ,
  DESCRIPTOR = _REQUEST,
  __module__ = 'rpc_interface_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.rpc.Request)
  ))
_sym_db.RegisterMessage(Request)
_sym_db.RegisterMessage(Request.Time)
_sym_db.RegisterMessage(Request.Message)


# @@protoc_insertion_point(module_scope)
