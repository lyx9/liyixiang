# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: car_sound.proto

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


DESCRIPTOR = _descriptor.FileDescriptor(
  name='car_sound.proto',
  package='adu.common.sound',
  serialized_pb=_b('\n\x0f\x63\x61r_sound.proto\x12\x10\x61\x64u.common.sound\x1a\x0cheader.proto\"\x86\x02\n\x0cSoundRequest\x12)\n\x06header\x18\x01 \x01(\x0b\x32\x19.adu.common.header.Header\x12>\n\x08priority\x18\x02 \x01(\x0e\x32,.adu.common.sound.SoundRequest.PriorityLevel\x12\x31\n\x04mode\x18\x03 \x01(\x0e\x32#.adu.common.sound.SoundRequest.Mode\x12\r\n\x05words\x18\x04 \x01(\t\".\n\rPriorityLevel\x12\x07\n\x03LOW\x10\x00\x12\n\n\x06MEDIUM\x10\x01\x12\x08\n\x04HIGH\x10\x02\"\x19\n\x04Mode\x12\x07\n\x03SAY\x10\x00\x12\x08\n\x04\x42\x45\x45P\x10\x01')
  ,
  dependencies=[header_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_SOUNDREQUEST_PRIORITYLEVEL = _descriptor.EnumDescriptor(
  name='PriorityLevel',
  full_name='adu.common.sound.SoundRequest.PriorityLevel',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='LOW', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MEDIUM', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HIGH', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=241,
  serialized_end=287,
)
_sym_db.RegisterEnumDescriptor(_SOUNDREQUEST_PRIORITYLEVEL)

_SOUNDREQUEST_MODE = _descriptor.EnumDescriptor(
  name='Mode',
  full_name='adu.common.sound.SoundRequest.Mode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='SAY', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='BEEP', index=1, number=1,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=289,
  serialized_end=314,
)
_sym_db.RegisterEnumDescriptor(_SOUNDREQUEST_MODE)


_SOUNDREQUEST = _descriptor.Descriptor(
  name='SoundRequest',
  full_name='adu.common.sound.SoundRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='adu.common.sound.SoundRequest.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='priority', full_name='adu.common.sound.SoundRequest.priority', index=1,
      number=2, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='mode', full_name='adu.common.sound.SoundRequest.mode', index=2,
      number=3, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='words', full_name='adu.common.sound.SoundRequest.words', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _SOUNDREQUEST_PRIORITYLEVEL,
    _SOUNDREQUEST_MODE,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=52,
  serialized_end=314,
)

_SOUNDREQUEST.fields_by_name['header'].message_type = header_pb2._HEADER
_SOUNDREQUEST.fields_by_name['priority'].enum_type = _SOUNDREQUEST_PRIORITYLEVEL
_SOUNDREQUEST.fields_by_name['mode'].enum_type = _SOUNDREQUEST_MODE
_SOUNDREQUEST_PRIORITYLEVEL.containing_type = _SOUNDREQUEST
_SOUNDREQUEST_MODE.containing_type = _SOUNDREQUEST
DESCRIPTOR.message_types_by_name['SoundRequest'] = _SOUNDREQUEST

SoundRequest = _reflection.GeneratedProtocolMessageType('SoundRequest', (_message.Message,), dict(
  DESCRIPTOR = _SOUNDREQUEST,
  __module__ = 'car_sound_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.sound.SoundRequest)
  ))
_sym_db.RegisterMessage(SoundRequest)


# @@protoc_insertion_point(module_scope)