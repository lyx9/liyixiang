# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: map_signal.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import map_geometry_pb2
import map_id_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='map_signal.proto',
  package='adu.common.hdmap',
  serialized_pb=_b('\n\x10map_signal.proto\x12\x10\x61\x64u.common.hdmap\x1a\x12map_geometry.proto\x1a\x0cmap_id.proto\"\xc4\x02\n\tSubsignal\x12 \n\x02id\x18\x01 \x01(\x0b\x32\x14.adu.common.hdmap.Id\x12.\n\x04type\x18\x02 \x01(\x0e\x32 .adu.common.hdmap.Subsignal.Type\x12)\n\x08location\x18\x03 \x01(\x0b\x32\x17.adu.common.hdmap.Point\"\xb9\x01\n\x04Type\x12\x0b\n\x07UNKNOWN\x10\x01\x12\n\n\x06\x43IRCLE\x10\x02\x12\x0e\n\nARROW_LEFT\x10\x03\x12\x11\n\rARROW_FORWARD\x10\x04\x12\x0f\n\x0b\x41RROW_RIGHT\x10\x05\x12\x1a\n\x16\x41RROW_LEFT_AND_FORWARD\x10\x06\x12\x1b\n\x17\x41RROW_RIGHT_AND_FORWARD\x10\x07\x12\x10\n\x0c\x41RROW_U_TURN\x10\x08\x12\x19\n\x15\x41RROW_LEFT_AND_U_TURN\x10\t\"\x83\x04\n\x06Signal\x12 \n\x02id\x18\x01 \x01(\x0b\x32\x14.adu.common.hdmap.Id\x12+\n\x08\x62oundary\x18\x02 \x01(\x0b\x32\x19.adu.common.hdmap.Polygon\x12.\n\tsubsignal\x18\x03 \x03(\x0b\x32\x1b.adu.common.hdmap.Subsignal\x12(\n\noverlap_id\x18\x04 \x03(\x0b\x32\x14.adu.common.hdmap.Id\x12+\n\x04type\x18\x05 \x01(\x0e\x32\x1d.adu.common.hdmap.Signal.Type\x12*\n\tstop_line\x18\x06 \x03(\x0b\x32\x17.adu.common.hdmap.Curve\x12=\n\x11\x63ontrol_direction\x18\x07 \x03(\x0e\x32\".adu.common.hdmap.Signal.Direction\"s\n\x04Type\x12\x0b\n\x07UNKNOWN\x10\x01\x12\x14\n\x10MIX_2_HORIZONTAL\x10\x02\x12\x12\n\x0eMIX_2_VERTICAL\x10\x03\x12\x14\n\x10MIX_3_HORIZONTAL\x10\x04\x12\x12\n\x0eMIX_3_VERTICAL\x10\x05\x12\n\n\x06SINGLE\x10\x06\"C\n\tDirection\x12\x0b\n\x07NO_TURN\x10\x01\x12\r\n\tLEFT_TURN\x10\x02\x12\x0e\n\nRIGHT_TURN\x10\x03\x12\n\n\x06U_TURN\x10\x04')
  ,
  dependencies=[map_geometry_pb2.DESCRIPTOR,map_id_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_SUBSIGNAL_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='adu.common.hdmap.Subsignal.Type',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CIRCLE', index=1, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARROW_LEFT', index=2, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARROW_FORWARD', index=3, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARROW_RIGHT', index=4, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARROW_LEFT_AND_FORWARD', index=5, number=6,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARROW_RIGHT_AND_FORWARD', index=6, number=7,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARROW_U_TURN', index=7, number=8,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ARROW_LEFT_AND_U_TURN', index=8, number=9,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=212,
  serialized_end=397,
)
_sym_db.RegisterEnumDescriptor(_SUBSIGNAL_TYPE)

_SIGNAL_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='adu.common.hdmap.Signal.Type',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MIX_2_HORIZONTAL', index=1, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MIX_2_VERTICAL', index=2, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MIX_3_HORIZONTAL', index=3, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MIX_3_VERTICAL', index=4, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SINGLE', index=5, number=6,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=731,
  serialized_end=846,
)
_sym_db.RegisterEnumDescriptor(_SIGNAL_TYPE)

_SIGNAL_DIRECTION = _descriptor.EnumDescriptor(
  name='Direction',
  full_name='adu.common.hdmap.Signal.Direction',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_TURN', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LEFT_TURN', index=1, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RIGHT_TURN', index=2, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='U_TURN', index=3, number=4,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=848,
  serialized_end=915,
)
_sym_db.RegisterEnumDescriptor(_SIGNAL_DIRECTION)


_SUBSIGNAL = _descriptor.Descriptor(
  name='Subsignal',
  full_name='adu.common.hdmap.Subsignal',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.common.hdmap.Subsignal.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='type', full_name='adu.common.hdmap.Subsignal.type', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='location', full_name='adu.common.hdmap.Subsignal.location', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _SUBSIGNAL_TYPE,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=73,
  serialized_end=397,
)


_SIGNAL = _descriptor.Descriptor(
  name='Signal',
  full_name='adu.common.hdmap.Signal',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.common.hdmap.Signal.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='boundary', full_name='adu.common.hdmap.Signal.boundary', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='subsignal', full_name='adu.common.hdmap.Signal.subsignal', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='overlap_id', full_name='adu.common.hdmap.Signal.overlap_id', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='type', full_name='adu.common.hdmap.Signal.type', index=4,
      number=5, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stop_line', full_name='adu.common.hdmap.Signal.stop_line', index=5,
      number=6, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='control_direction', full_name='adu.common.hdmap.Signal.control_direction', index=6,
      number=7, type=14, cpp_type=8, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _SIGNAL_TYPE,
    _SIGNAL_DIRECTION,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=400,
  serialized_end=915,
)

_SUBSIGNAL.fields_by_name['id'].message_type = map_id_pb2._ID
_SUBSIGNAL.fields_by_name['type'].enum_type = _SUBSIGNAL_TYPE
_SUBSIGNAL.fields_by_name['location'].message_type = map_geometry_pb2._POINT
_SUBSIGNAL_TYPE.containing_type = _SUBSIGNAL
_SIGNAL.fields_by_name['id'].message_type = map_id_pb2._ID
_SIGNAL.fields_by_name['boundary'].message_type = map_geometry_pb2._POLYGON
_SIGNAL.fields_by_name['subsignal'].message_type = _SUBSIGNAL
_SIGNAL.fields_by_name['overlap_id'].message_type = map_id_pb2._ID
_SIGNAL.fields_by_name['type'].enum_type = _SIGNAL_TYPE
_SIGNAL.fields_by_name['stop_line'].message_type = map_geometry_pb2._CURVE
_SIGNAL.fields_by_name['control_direction'].enum_type = _SIGNAL_DIRECTION
_SIGNAL_TYPE.containing_type = _SIGNAL
_SIGNAL_DIRECTION.containing_type = _SIGNAL
DESCRIPTOR.message_types_by_name['Subsignal'] = _SUBSIGNAL
DESCRIPTOR.message_types_by_name['Signal'] = _SIGNAL

Subsignal = _reflection.GeneratedProtocolMessageType('Subsignal', (_message.Message,), dict(
  DESCRIPTOR = _SUBSIGNAL,
  __module__ = 'map_signal_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.hdmap.Subsignal)
  ))
_sym_db.RegisterMessage(Subsignal)

Signal = _reflection.GeneratedProtocolMessageType('Signal', (_message.Message,), dict(
  DESCRIPTOR = _SIGNAL,
  __module__ = 'map_signal_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.hdmap.Signal)
  ))
_sym_db.RegisterMessage(Signal)


# @@protoc_insertion_point(module_scope)
