# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: meta_message.proto

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
  name='meta_message.proto',
  package='',
  serialized_pb=_b('\n\x12meta_message.proto\"/\n\x0bMetaMessage\x12\x12\n\nmessage_id\x18\x01 \x02(\x04\x12\x0c\n\x04\x64\x61ta\x18\x02 \x02(\t')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_METAMESSAGE = _descriptor.Descriptor(
  name='MetaMessage',
  full_name='MetaMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='message_id', full_name='MetaMessage.message_id', index=0,
      number=1, type=4, cpp_type=4, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='data', full_name='MetaMessage.data', index=1,
      number=2, type=9, cpp_type=9, label=2,
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
  serialized_start=22,
  serialized_end=69,
)

DESCRIPTOR.message_types_by_name['MetaMessage'] = _METAMESSAGE

MetaMessage = _reflection.GeneratedProtocolMessageType('MetaMessage', (_message.Message,), dict(
  DESCRIPTOR = _METAMESSAGE,
  __module__ = 'meta_message_pb2'
  # @@protoc_insertion_point(class_scope:MetaMessage)
  ))
_sym_db.RegisterMessage(MetaMessage)


# @@protoc_insertion_point(module_scope)