# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: planning_internal.proto

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
import decision_pb2
import global_adc_status_pb2
import perception_obstacle_pb2
import prediction_obstacle_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='planning_internal.proto',
  package='adu.common.planning_internal',
  serialized_pb=_b('\n\x17planning_internal.proto\x12\x1c\x61\x64u.common.planning_internal\x1a\x0cheader.proto\x1a\x0e\x64\x65\x63ision.proto\x1a\x17global_adc_status.proto\x1a\x19perception_obstacle.proto\x1a\x19prediction_obstacle.proto\"\xe4\x05\n\x10PlanningObstacle\x12\x15\n\rperception_id\x18\x01 \x01(\x05\x12\x13\n\x0b\x64\x65\x63ision_id\x18\x02 \x01(\t\x12.\n\x08position\x18\x03 \x01(\x0b\x32\x1c.adu.common.perception.Point\x12\r\n\x05theta\x18\x04 \x01(\x01\x12.\n\x08velocity\x18\x05 \x01(\x0b\x32\x1c.adu.common.perception.Point\x12\r\n\x05speed\x18\x06 \x01(\x01\x12\x0e\n\x06length\x18\x07 \x01(\x01\x12\r\n\x05width\x18\x08 \x01(\x01\x12\x0e\n\x06height\x18\t \x01(\x01\x12\x33\n\rpolygon_point\x18\n \x03(\x0b\x32\x1c.adu.common.perception.Point\x12\x15\n\rtracking_time\x18\x0b \x01(\x01\x12\x1c\n\x14perception_timestamp\x18\x0c \x01(\x01\x12N\n\x16perception_object_type\x18\r \x01(\x0e\x32..adu.common.perception.PerceptionObstacle.Type\x12\x1c\n\x14prediction_timestamp\x18\x0e \x01(\x01\x12@\n\x15prediction_trajectory\x18\x0f \x03(\x0b\x32!.adu.common.prediction.Trajectory\x12L\n\x14\x64\x65\x63ision_object_type\x18\x10 \x01(\x0e\x32..adu.common.decision.ObjectDecision.ObjectType\x12\x44\n\x0fobject_decision\x18\x11 \x01(\x0b\x32\'.adu.common.decision.ObjectDecisionTypeB\x02\x18\x01\x12I\n\x18planning_object_decision\x18\x12 \x03(\x0b\x32\'.adu.common.decision.ObjectDecisionType\"\xb2\x04\n\x05\x44\x65\x62ug\x12\x45\n\nerror_code\x18\x01 \x01(\x0e\x32-.adu.common.planning_internal.Debug.ErrorCode:\x02OK\x12\x41\n\rplanning_data\x18\x02 \x01(\x0b\x32*.adu.common.planning_internal.PlanningData\x12G\n\rdebug_message\x18\x03 \x03(\x0b\x32\x30.adu.common.planning_internal.Debug.DebugMessage\x1a\xc4\x01\n\x0c\x44\x65\x62ugMessage\x12\x45\n\nerror_code\x18\x01 \x01(\x0e\x32-.adu.common.planning_internal.Debug.ErrorCode:\x02OK\x12\n\n\x02id\x18\x02 \x01(\x05\x12\x0f\n\x05trace\x18\x03 \x01(\tH\x00\x12\x0e\n\x04info\x18\x04 \x01(\tH\x00\x12\x0e\n\x04warn\x18\x05 \x01(\tH\x00\x12\x0f\n\x05\x65rror\x18\x06 \x01(\tH\x00\x12\x0f\n\x05\x66\x61tal\x18\x07 \x01(\tH\x00\x42\x0e\n\x0c\x64\x65\x62ug_string\"\x8e\x01\n\tErrorCode\x12\x06\n\x02OK\x10\x00\x12\x11\n\rERR_NOT_READY\x10\x01\x12\r\n\tERR_ESTOP\x10\x02\x12\x16\n\x12\x45RR_PATH_OPTIMIZER\x10\x03\x12\x17\n\x13\x45RR_SPEED_OPTIMIZER\x10\x04\x12\x10\n\x0c\x45RR_ST_GRAPH\x10\x05\x12\x14\n\x10\x45RR_SANITY_CHECK\x10\x06\"\xa6\x02\n\x0cPlanningData\x12)\n\x06header\x18\x01 \x01(\x0b\x32\x19.adu.common.header.Header\x12.\n\x0binit_status\x18\x02 \x01(\x0b\x32\x19.adu.common.status.Status\x12\x38\n\rmain_decision\x18\x03 \x01(\x0b\x32!.adu.common.decision.MainDecision\x12I\n\x11planning_obstacle\x18\x04 \x03(\x0b\x32..adu.common.planning_internal.PlanningObstacle\x12\x36\n\x0clight_signal\x18\x05 \x01(\x0b\x32 .adu.common.decision.LightSignal')
  ,
  dependencies=[header_pb2.DESCRIPTOR,decision_pb2.DESCRIPTOR,global_adc_status_pb2.DESCRIPTOR,perception_obstacle_pb2.DESCRIPTOR,prediction_obstacle_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_DEBUG_ERRORCODE = _descriptor.EnumDescriptor(
  name='ErrorCode',
  full_name='adu.common.planning_internal.Debug.ErrorCode',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OK', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERR_NOT_READY', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERR_ESTOP', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERR_PATH_OPTIMIZER', index=3, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERR_SPEED_OPTIMIZER', index=4, number=4,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERR_ST_GRAPH', index=5, number=5,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='ERR_SANITY_CHECK', index=6, number=6,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1330,
  serialized_end=1472,
)
_sym_db.RegisterEnumDescriptor(_DEBUG_ERRORCODE)


_PLANNINGOBSTACLE = _descriptor.Descriptor(
  name='PlanningObstacle',
  full_name='adu.common.planning_internal.PlanningObstacle',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='perception_id', full_name='adu.common.planning_internal.PlanningObstacle.perception_id', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='decision_id', full_name='adu.common.planning_internal.PlanningObstacle.decision_id', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='position', full_name='adu.common.planning_internal.PlanningObstacle.position', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='theta', full_name='adu.common.planning_internal.PlanningObstacle.theta', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='velocity', full_name='adu.common.planning_internal.PlanningObstacle.velocity', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='speed', full_name='adu.common.planning_internal.PlanningObstacle.speed', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='length', full_name='adu.common.planning_internal.PlanningObstacle.length', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='width', full_name='adu.common.planning_internal.PlanningObstacle.width', index=7,
      number=8, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='height', full_name='adu.common.planning_internal.PlanningObstacle.height', index=8,
      number=9, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='polygon_point', full_name='adu.common.planning_internal.PlanningObstacle.polygon_point', index=9,
      number=10, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='tracking_time', full_name='adu.common.planning_internal.PlanningObstacle.tracking_time', index=10,
      number=11, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='perception_timestamp', full_name='adu.common.planning_internal.PlanningObstacle.perception_timestamp', index=11,
      number=12, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='perception_object_type', full_name='adu.common.planning_internal.PlanningObstacle.perception_object_type', index=12,
      number=13, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='prediction_timestamp', full_name='adu.common.planning_internal.PlanningObstacle.prediction_timestamp', index=13,
      number=14, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='prediction_trajectory', full_name='adu.common.planning_internal.PlanningObstacle.prediction_trajectory', index=14,
      number=15, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='decision_object_type', full_name='adu.common.planning_internal.PlanningObstacle.decision_object_type', index=15,
      number=16, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='object_decision', full_name='adu.common.planning_internal.PlanningObstacle.object_decision', index=16,
      number=17, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=_descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\030\001'))),
    _descriptor.FieldDescriptor(
      name='planning_object_decision', full_name='adu.common.planning_internal.PlanningObstacle.planning_object_decision', index=17,
      number=18, type=11, cpp_type=10, label=3,
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
  serialized_start=167,
  serialized_end=907,
)


_DEBUG_DEBUGMESSAGE = _descriptor.Descriptor(
  name='DebugMessage',
  full_name='adu.common.planning_internal.Debug.DebugMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='error_code', full_name='adu.common.planning_internal.Debug.DebugMessage.error_code', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.common.planning_internal.Debug.DebugMessage.id', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='trace', full_name='adu.common.planning_internal.Debug.DebugMessage.trace', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='info', full_name='adu.common.planning_internal.Debug.DebugMessage.info', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='warn', full_name='adu.common.planning_internal.Debug.DebugMessage.warn', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='error', full_name='adu.common.planning_internal.Debug.DebugMessage.error', index=5,
      number=6, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='fatal', full_name='adu.common.planning_internal.Debug.DebugMessage.fatal', index=6,
      number=7, type=9, cpp_type=9, label=1,
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
    _descriptor.OneofDescriptor(
      name='debug_string', full_name='adu.common.planning_internal.Debug.DebugMessage.debug_string',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=1131,
  serialized_end=1327,
)

_DEBUG = _descriptor.Descriptor(
  name='Debug',
  full_name='adu.common.planning_internal.Debug',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='error_code', full_name='adu.common.planning_internal.Debug.error_code', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='planning_data', full_name='adu.common.planning_internal.Debug.planning_data', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='debug_message', full_name='adu.common.planning_internal.Debug.debug_message', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[_DEBUG_DEBUGMESSAGE, ],
  enum_types=[
    _DEBUG_ERRORCODE,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=910,
  serialized_end=1472,
)


_PLANNINGDATA = _descriptor.Descriptor(
  name='PlanningData',
  full_name='adu.common.planning_internal.PlanningData',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='adu.common.planning_internal.PlanningData.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='init_status', full_name='adu.common.planning_internal.PlanningData.init_status', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='main_decision', full_name='adu.common.planning_internal.PlanningData.main_decision', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='planning_obstacle', full_name='adu.common.planning_internal.PlanningData.planning_obstacle', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='light_signal', full_name='adu.common.planning_internal.PlanningData.light_signal', index=4,
      number=5, type=11, cpp_type=10, label=1,
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
  serialized_start=1475,
  serialized_end=1769,
)

_PLANNINGOBSTACLE.fields_by_name['position'].message_type = perception_obstacle_pb2._POINT
_PLANNINGOBSTACLE.fields_by_name['velocity'].message_type = perception_obstacle_pb2._POINT
_PLANNINGOBSTACLE.fields_by_name['polygon_point'].message_type = perception_obstacle_pb2._POINT
_PLANNINGOBSTACLE.fields_by_name['perception_object_type'].enum_type = perception_obstacle_pb2._PERCEPTIONOBSTACLE_TYPE
_PLANNINGOBSTACLE.fields_by_name['prediction_trajectory'].message_type = prediction_obstacle_pb2._TRAJECTORY
_PLANNINGOBSTACLE.fields_by_name['decision_object_type'].enum_type = decision_pb2._OBJECTDECISION_OBJECTTYPE
_PLANNINGOBSTACLE.fields_by_name['object_decision'].message_type = decision_pb2._OBJECTDECISIONTYPE
_PLANNINGOBSTACLE.fields_by_name['planning_object_decision'].message_type = decision_pb2._OBJECTDECISIONTYPE
_DEBUG_DEBUGMESSAGE.fields_by_name['error_code'].enum_type = _DEBUG_ERRORCODE
_DEBUG_DEBUGMESSAGE.containing_type = _DEBUG
_DEBUG_DEBUGMESSAGE.oneofs_by_name['debug_string'].fields.append(
  _DEBUG_DEBUGMESSAGE.fields_by_name['trace'])
_DEBUG_DEBUGMESSAGE.fields_by_name['trace'].containing_oneof = _DEBUG_DEBUGMESSAGE.oneofs_by_name['debug_string']
_DEBUG_DEBUGMESSAGE.oneofs_by_name['debug_string'].fields.append(
  _DEBUG_DEBUGMESSAGE.fields_by_name['info'])
_DEBUG_DEBUGMESSAGE.fields_by_name['info'].containing_oneof = _DEBUG_DEBUGMESSAGE.oneofs_by_name['debug_string']
_DEBUG_DEBUGMESSAGE.oneofs_by_name['debug_string'].fields.append(
  _DEBUG_DEBUGMESSAGE.fields_by_name['warn'])
_DEBUG_DEBUGMESSAGE.fields_by_name['warn'].containing_oneof = _DEBUG_DEBUGMESSAGE.oneofs_by_name['debug_string']
_DEBUG_DEBUGMESSAGE.oneofs_by_name['debug_string'].fields.append(
  _DEBUG_DEBUGMESSAGE.fields_by_name['error'])
_DEBUG_DEBUGMESSAGE.fields_by_name['error'].containing_oneof = _DEBUG_DEBUGMESSAGE.oneofs_by_name['debug_string']
_DEBUG_DEBUGMESSAGE.oneofs_by_name['debug_string'].fields.append(
  _DEBUG_DEBUGMESSAGE.fields_by_name['fatal'])
_DEBUG_DEBUGMESSAGE.fields_by_name['fatal'].containing_oneof = _DEBUG_DEBUGMESSAGE.oneofs_by_name['debug_string']
_DEBUG.fields_by_name['error_code'].enum_type = _DEBUG_ERRORCODE
_DEBUG.fields_by_name['planning_data'].message_type = _PLANNINGDATA
_DEBUG.fields_by_name['debug_message'].message_type = _DEBUG_DEBUGMESSAGE
_DEBUG_ERRORCODE.containing_type = _DEBUG
_PLANNINGDATA.fields_by_name['header'].message_type = header_pb2._HEADER
_PLANNINGDATA.fields_by_name['init_status'].message_type = global_adc_status_pb2._STATUS
_PLANNINGDATA.fields_by_name['main_decision'].message_type = decision_pb2._MAINDECISION
_PLANNINGDATA.fields_by_name['planning_obstacle'].message_type = _PLANNINGOBSTACLE
_PLANNINGDATA.fields_by_name['light_signal'].message_type = decision_pb2._LIGHTSIGNAL
DESCRIPTOR.message_types_by_name['PlanningObstacle'] = _PLANNINGOBSTACLE
DESCRIPTOR.message_types_by_name['Debug'] = _DEBUG
DESCRIPTOR.message_types_by_name['PlanningData'] = _PLANNINGDATA

PlanningObstacle = _reflection.GeneratedProtocolMessageType('PlanningObstacle', (_message.Message,), dict(
  DESCRIPTOR = _PLANNINGOBSTACLE,
  __module__ = 'planning_internal_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.planning_internal.PlanningObstacle)
  ))
_sym_db.RegisterMessage(PlanningObstacle)

Debug = _reflection.GeneratedProtocolMessageType('Debug', (_message.Message,), dict(

  DebugMessage = _reflection.GeneratedProtocolMessageType('DebugMessage', (_message.Message,), dict(
    DESCRIPTOR = _DEBUG_DEBUGMESSAGE,
    __module__ = 'planning_internal_pb2'
    # @@protoc_insertion_point(class_scope:adu.common.planning_internal.Debug.DebugMessage)
    ))
  ,
  DESCRIPTOR = _DEBUG,
  __module__ = 'planning_internal_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.planning_internal.Debug)
  ))
_sym_db.RegisterMessage(Debug)
_sym_db.RegisterMessage(Debug.DebugMessage)

PlanningData = _reflection.GeneratedProtocolMessageType('PlanningData', (_message.Message,), dict(
  DESCRIPTOR = _PLANNINGDATA,
  __module__ = 'planning_internal_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.planning_internal.PlanningData)
  ))
_sym_db.RegisterMessage(PlanningData)


_PLANNINGOBSTACLE.fields_by_name['object_decision'].has_options = True
_PLANNINGOBSTACLE.fields_by_name['object_decision']._options = _descriptor._ParseOptions(descriptor_pb2.FieldOptions(), _b('\030\001'))
# @@protoc_insertion_point(module_scope)
