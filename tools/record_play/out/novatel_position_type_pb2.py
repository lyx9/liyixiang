# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: novatel_position_type.proto

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




DESCRIPTOR = _descriptor.FileDescriptor(
  name='novatel_position_type.proto',
  package='adu.common.sensor',
  serialized_pb=_b('\n\x1bnovatel_position_type.proto\x12\x11\x61\x64u.common.sensor*\x9c\x04\n\x0cPositionType\x12\x08\n\x04NONE\x10\x00\x12\t\n\x05\x46IXED\x10\x01\x12\x0f\n\x0b\x46IXEDHEIGHT\x10\x02\x12\r\n\tFLOATCONV\x10\x04\x12\x0c\n\x08WIDELANE\x10\x05\x12\x0e\n\nNARROWLANE\x10\x06\x12\x14\n\x10\x44OPPLER_VELOCITY\x10\x08\x12\n\n\x06SINGLE\x10\x10\x12\x0b\n\x07PSRDIFF\x10\x11\x12\x08\n\x04WAAS\x10\x12\x12\x0e\n\nPROPAGATED\x10\x13\x12\x0c\n\x08OMNISTAR\x10\x14\x12\x0c\n\x08L1_FLOAT\x10 \x12\x12\n\x0eIONOFREE_FLOAT\x10!\x12\x10\n\x0cNARROW_FLOAT\x10\"\x12\n\n\x06L1_INT\x10\x30\x12\x0c\n\x08WIDE_INT\x10\x31\x12\x0e\n\nNARROW_INT\x10\x32\x12\x12\n\x0eRTK_DIRECT_INS\x10\x33\x12\x0c\n\x08INS_SBAS\x10\x34\x12\r\n\tINS_PSRSP\x10\x35\x12\x0f\n\x0bINS_PSRDIFF\x10\x36\x12\x10\n\x0cINS_RTKFLOAT\x10\x37\x12\x10\n\x0cINS_RTKFIXED\x10\x38\x12\x10\n\x0cINS_OMNISTAR\x10\x39\x12\x13\n\x0fINS_OMNISTAR_HP\x10:\x12\x13\n\x0fINS_OMNISTAR_XP\x10;\x12\x0f\n\x0bOMNISTAR_HP\x10@\x12\x0f\n\x0bOMNISTAR_XP\x10\x41\x12\x12\n\x0ePPP_CONVERGING\x10\x44\x12\x07\n\x03PPP\x10\x45\x12\x16\n\x12INS_PPP_CONVERGING\x10I\x12\x0b\n\x07INS_PPP\x10J')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

_POSITIONTYPE = _descriptor.EnumDescriptor(
  name='PositionType',
  full_name='adu.common.sensor.PositionType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FIXED', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FIXEDHEIGHT', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='FLOATCONV', index=3, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WIDELANE', index=4, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NARROWLANE', index=5, number=6,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DOPPLER_VELOCITY', index=6, number=8,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='SINGLE', index=7, number=16,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PSRDIFF', index=8, number=17,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WAAS', index=9, number=18,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PROPAGATED', index=10, number=19,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OMNISTAR', index=11, number=20,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='L1_FLOAT', index=12, number=32,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='IONOFREE_FLOAT', index=13, number=33,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NARROW_FLOAT', index=14, number=34,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='L1_INT', index=15, number=48,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='WIDE_INT', index=16, number=49,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='NARROW_INT', index=17, number=50,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RTK_DIRECT_INS', index=18, number=51,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_SBAS', index=19, number=52,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_PSRSP', index=20, number=53,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_PSRDIFF', index=21, number=54,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_RTKFLOAT', index=22, number=55,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_RTKFIXED', index=23, number=56,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_OMNISTAR', index=24, number=57,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_OMNISTAR_HP', index=25, number=58,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_OMNISTAR_XP', index=26, number=59,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OMNISTAR_HP', index=27, number=64,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='OMNISTAR_XP', index=28, number=65,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PPP_CONVERGING', index=29, number=68,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PPP', index=30, number=69,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_PPP_CONVERGING', index=31, number=73,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INS_PPP', index=32, number=74,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=51,
  serialized_end=591,
)
_sym_db.RegisterEnumDescriptor(_POSITIONTYPE)

PositionType = enum_type_wrapper.EnumTypeWrapper(_POSITIONTYPE)
NONE = 0
FIXED = 1
FIXEDHEIGHT = 2
FLOATCONV = 4
WIDELANE = 5
NARROWLANE = 6
DOPPLER_VELOCITY = 8
SINGLE = 16
PSRDIFF = 17
WAAS = 18
PROPAGATED = 19
OMNISTAR = 20
L1_FLOAT = 32
IONOFREE_FLOAT = 33
NARROW_FLOAT = 34
L1_INT = 48
WIDE_INT = 49
NARROW_INT = 50
RTK_DIRECT_INS = 51
INS_SBAS = 52
INS_PSRSP = 53
INS_PSRDIFF = 54
INS_RTKFLOAT = 55
INS_RTKFIXED = 56
INS_OMNISTAR = 57
INS_OMNISTAR_HP = 58
INS_OMNISTAR_XP = 59
OMNISTAR_HP = 64
OMNISTAR_XP = 65
PPP_CONVERGING = 68
PPP = 69
INS_PPP_CONVERGING = 73
INS_PPP = 74


DESCRIPTOR.enum_types_by_name['PositionType'] = _POSITIONTYPE


# @@protoc_insertion_point(module_scope)
