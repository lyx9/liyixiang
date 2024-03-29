# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: simulation_local_player.proto

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
  name='simulation_local_player.proto',
  package='adu.simulation',
  serialized_pb=_b('\n\x1dsimulation_local_player.proto\x12\x0e\x61\x64u.simulation\"\x99\x01\n\nPlayerInfo\x12\x0b\n\x03\x62\x61g\x18\x01 \x01(\t\x12\x0c\n\x04rate\x18\x02 \x01(\x01\x12\x11\n\tstart_sec\x18\x03 \x01(\x01\x12\x14\n\x0c\x64uration_sec\x18\x04 \x01(\x01\x12\x13\n\x0b\x64\x65\x62ug_topic\x18\x05 \x01(\t\x12\x0c\n\x04loop\x18\x06 \x01(\x08\x12\x12\n\nbegin_time\x18\x07 \x01(\x01\x12\x10\n\x08\x65nd_time\x18\x08 \x01(\x01\"\x90\x01\n\tLPControl\x12\x30\n\x06signal\x18\x01 \x01(\x0e\x32 .adu.simulation.LPControl.LPType\x12(\n\x04info\x18\x02 \x01(\x0b\x32\x1a.adu.simulation.PlayerInfo\"\'\n\x06LPType\x12\x08\n\x04PLAY\x10\x00\x12\t\n\x05PAUSE\x10\x01\x12\x08\n\x04INFO\x10\x02\"4\n\x08LPStatus\x12(\n\x04info\x18\x01 \x03(\x0b\x32\x1a.adu.simulation.PlayerInfo')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_LPCONTROL_LPTYPE = _descriptor.EnumDescriptor(
  name='LPType',
  full_name='adu.simulation.LPControl.LPType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='PLAY', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PAUSE', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='INFO', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=311,
  serialized_end=350,
)
_sym_db.RegisterEnumDescriptor(_LPCONTROL_LPTYPE)


_PLAYERINFO = _descriptor.Descriptor(
  name='PlayerInfo',
  full_name='adu.simulation.PlayerInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='bag', full_name='adu.simulation.PlayerInfo.bag', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='rate', full_name='adu.simulation.PlayerInfo.rate', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start_sec', full_name='adu.simulation.PlayerInfo.start_sec', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='duration_sec', full_name='adu.simulation.PlayerInfo.duration_sec', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='debug_topic', full_name='adu.simulation.PlayerInfo.debug_topic', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='loop', full_name='adu.simulation.PlayerInfo.loop', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='begin_time', full_name='adu.simulation.PlayerInfo.begin_time', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end_time', full_name='adu.simulation.PlayerInfo.end_time', index=7,
      number=8, type=1, cpp_type=5, label=1,
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
  serialized_start=50,
  serialized_end=203,
)


_LPCONTROL = _descriptor.Descriptor(
  name='LPControl',
  full_name='adu.simulation.LPControl',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='signal', full_name='adu.simulation.LPControl.signal', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='info', full_name='adu.simulation.LPControl.info', index=1,
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
    _LPCONTROL_LPTYPE,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=206,
  serialized_end=350,
)


_LPSTATUS = _descriptor.Descriptor(
  name='LPStatus',
  full_name='adu.simulation.LPStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='info', full_name='adu.simulation.LPStatus.info', index=0,
      number=1, type=11, cpp_type=10, label=3,
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
  serialized_start=352,
  serialized_end=404,
)

_LPCONTROL.fields_by_name['signal'].enum_type = _LPCONTROL_LPTYPE
_LPCONTROL.fields_by_name['info'].message_type = _PLAYERINFO
_LPCONTROL_LPTYPE.containing_type = _LPCONTROL
_LPSTATUS.fields_by_name['info'].message_type = _PLAYERINFO
DESCRIPTOR.message_types_by_name['PlayerInfo'] = _PLAYERINFO
DESCRIPTOR.message_types_by_name['LPControl'] = _LPCONTROL
DESCRIPTOR.message_types_by_name['LPStatus'] = _LPSTATUS

PlayerInfo = _reflection.GeneratedProtocolMessageType('PlayerInfo', (_message.Message,), dict(
  DESCRIPTOR = _PLAYERINFO,
  __module__ = 'simulation_local_player_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.PlayerInfo)
  ))
_sym_db.RegisterMessage(PlayerInfo)

LPControl = _reflection.GeneratedProtocolMessageType('LPControl', (_message.Message,), dict(
  DESCRIPTOR = _LPCONTROL,
  __module__ = 'simulation_local_player_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.LPControl)
  ))
_sym_db.RegisterMessage(LPControl)

LPStatus = _reflection.GeneratedProtocolMessageType('LPStatus', (_message.Message,), dict(
  DESCRIPTOR = _LPSTATUS,
  __module__ = 'simulation_local_player_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.LPStatus)
  ))
_sym_db.RegisterMessage(LPStatus)


# @@protoc_insertion_point(module_scope)
