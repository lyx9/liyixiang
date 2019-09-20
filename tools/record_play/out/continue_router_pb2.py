# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: continue_router.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import header_pb2
import common_geometry_pb2
import new_router_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='continue_router.proto',
  package='adu.common.continue_router',
  serialized_pb=_b('\n\x15\x63ontinue_router.proto\x12\x1a\x61\x64u.common.continue_router\x1a\x0cheader.proto\x1a\x15\x63ommon_geometry.proto\x1a\x10new_router.proto\"|\n\x15\x43ontinueRouterRequest\x12)\n\x06header\x18\x01 \x01(\x0b\x32\x19.adu.common.header.Header\x12\x38\n\x0bstart_point\x18\x02 \x01(\x0b\x32#.adu.common.new_router.LaneWaypoint')
  ,
  dependencies=[header_pb2.DESCRIPTOR,common_geometry_pb2.DESCRIPTOR,new_router_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_CONTINUEROUTERREQUEST = _descriptor.Descriptor(
  name='ContinueRouterRequest',
  full_name='adu.common.continue_router.ContinueRouterRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='adu.common.continue_router.ContinueRouterRequest.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start_point', full_name='adu.common.continue_router.ContinueRouterRequest.start_point', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=108,
  serialized_end=232,
)

_CONTINUEROUTERREQUEST.fields_by_name['header'].message_type = header_pb2._HEADER
_CONTINUEROUTERREQUEST.fields_by_name['start_point'].message_type = new_router_pb2._LANEWAYPOINT
DESCRIPTOR.message_types_by_name['ContinueRouterRequest'] = _CONTINUEROUTERREQUEST

ContinueRouterRequest = _reflection.GeneratedProtocolMessageType('ContinueRouterRequest', (_message.Message,), dict(
  DESCRIPTOR = _CONTINUEROUTERREQUEST,
  __module__ = 'continue_router_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.continue_router.ContinueRouterRequest)
  ))
_sym_db.RegisterMessage(ContinueRouterRequest)


# @@protoc_insertion_point(module_scope)