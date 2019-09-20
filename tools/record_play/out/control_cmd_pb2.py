# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: control_cmd.proto

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
import global_adc_status_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='control_cmd.proto',
  package='adu.common.control',
  serialized_pb=_b('\n\x11\x63ontrol_cmd.proto\x12\x12\x61\x64u.common.control\x1a\x0cheader.proto\x1a\x17global_adc_status.proto\"\xbb\x06\n\x05\x44\x65\x62ug\x12;\n\nerror_code\x18\x01 \x01(\x0e\x32#.adu.common.control.Debug.ErrorCode:\x02OK\x12\x44\n\x11lon_debug_message\x18\x02 \x01(\x0b\x32).adu.common.control.Debug.LonDebugMessage\x12\x44\n\x11lat_debug_message\x18\x03 \x01(\x0b\x32).adu.common.control.Debug.LatDebugMessage\x1a\xb1\x02\n\x0fLonDebugMessage\x12\x15\n\rstation_error\x18\x01 \x01(\x01\x12\x13\n\x0bspeed_error\x18\x02 \x01(\x01\x12\x11\n\tspeed_cur\x18\x03 \x01(\x01\x12\x11\n\tspeed_ref\x18\x04 \x01(\x01\x12\x11\n\tacc_error\x18\x05 \x01(\x01\x12\x0f\n\x07\x61\x63\x63_cur\x18\x06 \x01(\x01\x12\x0f\n\x07\x61\x63\x63_ref\x18\x07 \x01(\x01\x12\x0f\n\x07\x61\x63\x63_mpc\x18\x08 \x01(\x01\x12\x0f\n\x07\x61\x63\x63_out\x18\t \x01(\x01\x12\x18\n\x10throttle_control\x18\n \x01(\x01\x12\x18\n\x10throttle_chassis\x18\x0b \x01(\x01\x12\x15\n\rbrake_control\x18\x0c \x01(\x01\x12\x15\n\rbrake_chassis\x18\r \x01(\x01\x12\x13\n\x0bpath_remain\x18\x0e \x01(\x01\x1a\xea\x01\n\x0fLatDebugMessage\x12\x15\n\rlateral_error\x18\x01 \x01(\x01\x12\x15\n\rheading_error\x18\x02 \x01(\x01\x12\x13\n\x0bheading_cur\x18\x03 \x01(\x01\x12\x13\n\x0bheading_ref\x18\x04 \x01(\x01\x12\x15\n\rcurvature_cur\x18\x05 \x01(\x01\x12\x15\n\rcurvature_ref\x18\x06 \x01(\x01\x12\x10\n\x08steer_ff\x18\x07 \x01(\x01\x12\x11\n\tsteer_mpc\x18\x08 \x01(\x01\x12\x15\n\rsteer_control\x18\t \x01(\x01\x12\x15\n\rsteer_chassis\x18\n \x01(\x01\"H\n\tErrorCode\x12\x06\n\x02OK\x10\x00\x12\x19\n\x15\x45RR_SECOND_EXCEPTIONS\x10\x01\x12\x18\n\x14\x45RR_THIRD_EXCEPTIONS\x10\x02\"\x83\x04\n\x0e\x43ontrolCommand\x12)\n\x06header\x18\x01 \x01(\x0b\x32\x19.adu.common.header.Header\x12\x10\n\x08throttle\x18\x03 \x01(\x01\x12\r\n\x05\x62rake\x18\x04 \x01(\x01\x12\x15\n\rsteering_rate\x18\x06 \x01(\x01\x12\x17\n\x0fsteering_target\x18\x07 \x01(\x01\x12\x15\n\rparking_brake\x18\x08 \x01(\x08\x12\r\n\x05speed\x18\t \x01(\x01\x12\x14\n\x0c\x61\x63\x63\x65leration\x18\n \x01(\x01\x12\x11\n\thigh_beam\x18\x0b \x01(\x08\x12\x10\n\x08low_beam\x18\x0c \x01(\x08\x12\x11\n\tleft_turn\x18\r \x01(\x08\x12\x12\n\nright_turn\x18\x0e \x01(\x08\x12\x0c\n\x04horn\x18\x0f \x01(\x08\x12\x13\n\x0breset_model\x18\x10 \x01(\x08\x12\x15\n\rengine_on_off\x18\x11 \x01(\x08\x12\x1b\n\x13trajectory_fraction\x18\x12 \x01(\x01\x12\x34\n\x0c\x64riving_mode\x18\x13 \x01(\x0e\x32\x1e.adu.common.status.DrivingMode\x12\x36\n\rgear_location\x18\x14 \x01(\x0e\x32\x1f.adu.common.status.GearPosition\x12(\n\x05\x64\x65\x62ug\x18\x15 \x01(\x0b\x32\x19.adu.common.control.Debug')
  ,
  dependencies=[header_pb2.DESCRIPTOR,global_adc_status_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_DEBUG_ERRORCODE = _descriptor.EnumDescriptor(
  name='ErrorCode',
  full_name='adu.common.control.Debug.ErrorCode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OK', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERR_SECOND_EXCEPTIONS', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERR_THIRD_EXCEPTIONS', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=836,
  serialized_end=908,
)
_sym_db.RegisterEnumDescriptor(_DEBUG_ERRORCODE)


_DEBUG_LONDEBUGMESSAGE = _descriptor.Descriptor(
  name='LonDebugMessage',
  full_name='adu.common.control.Debug.LonDebugMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='station_error', full_name='adu.common.control.Debug.LonDebugMessage.station_error', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='speed_error', full_name='adu.common.control.Debug.LonDebugMessage.speed_error', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='speed_cur', full_name='adu.common.control.Debug.LonDebugMessage.speed_cur', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='speed_ref', full_name='adu.common.control.Debug.LonDebugMessage.speed_ref', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='acc_error', full_name='adu.common.control.Debug.LonDebugMessage.acc_error', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='acc_cur', full_name='adu.common.control.Debug.LonDebugMessage.acc_cur', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='acc_ref', full_name='adu.common.control.Debug.LonDebugMessage.acc_ref', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='acc_mpc', full_name='adu.common.control.Debug.LonDebugMessage.acc_mpc', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='acc_out', full_name='adu.common.control.Debug.LonDebugMessage.acc_out', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='throttle_control', full_name='adu.common.control.Debug.LonDebugMessage.throttle_control', index=9,
      number=10, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='throttle_chassis', full_name='adu.common.control.Debug.LonDebugMessage.throttle_chassis', index=10,
      number=11, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='brake_control', full_name='adu.common.control.Debug.LonDebugMessage.brake_control', index=11,
      number=12, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='brake_chassis', full_name='adu.common.control.Debug.LonDebugMessage.brake_chassis', index=12,
      number=13, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='path_remain', full_name='adu.common.control.Debug.LonDebugMessage.path_remain', index=13,
      number=14, type=1, cpp_type=5, label=1,
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
  serialized_start=292,
  serialized_end=597,
)

_DEBUG_LATDEBUGMESSAGE = _descriptor.Descriptor(
  name='LatDebugMessage',
  full_name='adu.common.control.Debug.LatDebugMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='lateral_error', full_name='adu.common.control.Debug.LatDebugMessage.lateral_error', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='heading_error', full_name='adu.common.control.Debug.LatDebugMessage.heading_error', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='heading_cur', full_name='adu.common.control.Debug.LatDebugMessage.heading_cur', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='heading_ref', full_name='adu.common.control.Debug.LatDebugMessage.heading_ref', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='curvature_cur', full_name='adu.common.control.Debug.LatDebugMessage.curvature_cur', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='curvature_ref', full_name='adu.common.control.Debug.LatDebugMessage.curvature_ref', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='steer_ff', full_name='adu.common.control.Debug.LatDebugMessage.steer_ff', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='steer_mpc', full_name='adu.common.control.Debug.LatDebugMessage.steer_mpc', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='steer_control', full_name='adu.common.control.Debug.LatDebugMessage.steer_control', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='steer_chassis', full_name='adu.common.control.Debug.LatDebugMessage.steer_chassis', index=9,
      number=10, type=1, cpp_type=5, label=1,
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
  serialized_start=600,
  serialized_end=834,
)

_DEBUG = _descriptor.Descriptor(
  name='Debug',
  full_name='adu.common.control.Debug',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='error_code', full_name='adu.common.control.Debug.error_code', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lon_debug_message', full_name='adu.common.control.Debug.lon_debug_message', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lat_debug_message', full_name='adu.common.control.Debug.lat_debug_message', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_DEBUG_LONDEBUGMESSAGE, _DEBUG_LATDEBUGMESSAGE, ],
  enum_types=[
    _DEBUG_ERRORCODE,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=81,
  serialized_end=908,
)


_CONTROLCOMMAND = _descriptor.Descriptor(
  name='ControlCommand',
  full_name='adu.common.control.ControlCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='adu.common.control.ControlCommand.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='throttle', full_name='adu.common.control.ControlCommand.throttle', index=1,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='brake', full_name='adu.common.control.ControlCommand.brake', index=2,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='steering_rate', full_name='adu.common.control.ControlCommand.steering_rate', index=3,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='steering_target', full_name='adu.common.control.ControlCommand.steering_target', index=4,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='parking_brake', full_name='adu.common.control.ControlCommand.parking_brake', index=5,
      number=8, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='speed', full_name='adu.common.control.ControlCommand.speed', index=6,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='acceleration', full_name='adu.common.control.ControlCommand.acceleration', index=7,
      number=10, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='high_beam', full_name='adu.common.control.ControlCommand.high_beam', index=8,
      number=11, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='low_beam', full_name='adu.common.control.ControlCommand.low_beam', index=9,
      number=12, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='left_turn', full_name='adu.common.control.ControlCommand.left_turn', index=10,
      number=13, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_turn', full_name='adu.common.control.ControlCommand.right_turn', index=11,
      number=14, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='horn', full_name='adu.common.control.ControlCommand.horn', index=12,
      number=15, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='reset_model', full_name='adu.common.control.ControlCommand.reset_model', index=13,
      number=16, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='engine_on_off', full_name='adu.common.control.ControlCommand.engine_on_off', index=14,
      number=17, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='trajectory_fraction', full_name='adu.common.control.ControlCommand.trajectory_fraction', index=15,
      number=18, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='driving_mode', full_name='adu.common.control.ControlCommand.driving_mode', index=16,
      number=19, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='gear_location', full_name='adu.common.control.ControlCommand.gear_location', index=17,
      number=20, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='debug', full_name='adu.common.control.ControlCommand.debug', index=18,
      number=21, type=11, cpp_type=10, label=1,
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
  serialized_start=911,
  serialized_end=1426,
)

_DEBUG_LONDEBUGMESSAGE.containing_type = _DEBUG
_DEBUG_LATDEBUGMESSAGE.containing_type = _DEBUG
_DEBUG.fields_by_name['error_code'].enum_type = _DEBUG_ERRORCODE
_DEBUG.fields_by_name['lon_debug_message'].message_type = _DEBUG_LONDEBUGMESSAGE
_DEBUG.fields_by_name['lat_debug_message'].message_type = _DEBUG_LATDEBUGMESSAGE
_DEBUG_ERRORCODE.containing_type = _DEBUG
_CONTROLCOMMAND.fields_by_name['header'].message_type = header_pb2._HEADER
_CONTROLCOMMAND.fields_by_name['driving_mode'].enum_type = global_adc_status_pb2._DRIVINGMODE
_CONTROLCOMMAND.fields_by_name['gear_location'].enum_type = global_adc_status_pb2._GEARPOSITION
_CONTROLCOMMAND.fields_by_name['debug'].message_type = _DEBUG
DESCRIPTOR.message_types_by_name['Debug'] = _DEBUG
DESCRIPTOR.message_types_by_name['ControlCommand'] = _CONTROLCOMMAND

Debug = _reflection.GeneratedProtocolMessageType('Debug', (_message.Message,), dict(

  LonDebugMessage = _reflection.GeneratedProtocolMessageType('LonDebugMessage', (_message.Message,), dict(
    DESCRIPTOR = _DEBUG_LONDEBUGMESSAGE,
    __module__ = 'control_cmd_pb2'
    # @@protoc_insertion_point(class_scope:adu.common.control.Debug.LonDebugMessage)
    ))
  ,

  LatDebugMessage = _reflection.GeneratedProtocolMessageType('LatDebugMessage', (_message.Message,), dict(
    DESCRIPTOR = _DEBUG_LATDEBUGMESSAGE,
    __module__ = 'control_cmd_pb2'
    # @@protoc_insertion_point(class_scope:adu.common.control.Debug.LatDebugMessage)
    ))
  ,
  DESCRIPTOR = _DEBUG,
  __module__ = 'control_cmd_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.control.Debug)
  ))
_sym_db.RegisterMessage(Debug)
_sym_db.RegisterMessage(Debug.LonDebugMessage)
_sym_db.RegisterMessage(Debug.LatDebugMessage)

ControlCommand = _reflection.GeneratedProtocolMessageType('ControlCommand', (_message.Message,), dict(
  DESCRIPTOR = _CONTROLCOMMAND,
  __module__ = 'control_cmd_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.control.ControlCommand)
  ))
_sym_db.RegisterMessage(ControlCommand)


# @@protoc_insertion_point(module_scope)