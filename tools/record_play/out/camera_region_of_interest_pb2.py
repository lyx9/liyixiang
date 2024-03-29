# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: camera_region_of_interest.proto

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
  name='camera_region_of_interest.proto',
  package='adu.common.sensor',
  serialized_pb=_b('\n\x1f\x63\x61mera_region_of_interest.proto\x12\x11\x61\x64u.common.sensor\"i\n\x10RegionOfInterest\x12\x10\n\x08x_offset\x18\x01 \x01(\r\x12\x10\n\x08y_offset\x18\x02 \x01(\r\x12\x0e\n\x06height\x18\x03 \x01(\r\x12\r\n\x05width\x18\x04 \x01(\r\x12\x12\n\ndo_rectify\x18\x05 \x01(\x08')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_REGIONOFINTEREST = _descriptor.Descriptor(
  name='RegionOfInterest',
  full_name='adu.common.sensor.RegionOfInterest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x_offset', full_name='adu.common.sensor.RegionOfInterest.x_offset', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y_offset', full_name='adu.common.sensor.RegionOfInterest.y_offset', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height', full_name='adu.common.sensor.RegionOfInterest.height', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='width', full_name='adu.common.sensor.RegionOfInterest.width', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='do_rectify', full_name='adu.common.sensor.RegionOfInterest.do_rectify', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=54,
  serialized_end=159,
)

DESCRIPTOR.message_types_by_name['RegionOfInterest'] = _REGIONOFINTEREST

RegionOfInterest = _reflection.GeneratedProtocolMessageType('RegionOfInterest', (_message.Message,), dict(
  DESCRIPTOR = _REGIONOFINTEREST,
  __module__ = 'camera_region_of_interest_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.sensor.RegionOfInterest)
  ))
_sym_db.RegisterMessage(RegionOfInterest)


# @@protoc_insertion_point(module_scope)
