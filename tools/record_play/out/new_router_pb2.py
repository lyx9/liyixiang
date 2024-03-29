# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: new_router.proto

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


import header_pb2
import common_geometry_pb2
import error_code_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='new_router.proto',
  package='adu.common.new_router',
  serialized_pb=_b('\n\x10new_router.proto\x12\x15\x61\x64u.common.new_router\x1a\x0cheader.proto\x1a\x15\x63ommon_geometry.proto\x1a\x10\x65rror_code.proto\"\x85\x01\n\x0cLaneWaypoint\x12\n\n\x02id\x18\x01 \x01(\t\x12\t\n\x01s\x18\x02 \x01(\x01\x12\"\n\x04pose\x18\x03 \x01(\x0b\x32\x14.adu.common.PointENU\x12:\n\x04type\x18\x04 \x01(\x0e\x32#.adu.common.new_router.WaypointType:\x07PASS_BY\"9\n\x0bLaneSegment\x12\n\n\x02id\x18\x01 \x01(\t\x12\x0f\n\x07start_s\x18\x02 \x01(\x01\x12\r\n\x05\x65nd_s\x18\x03 \x01(\x01\"\xe3\x01\n\x0eRoutingRequest\x12)\n\x06header\x18\x01 \x01(\x0b\x32\x19.adu.common.header.Header\x12\x35\n\x08waypoint\x18\x02 \x03(\x0b\x32#.adu.common.new_router.LaneWaypoint\x12<\n\x10\x62lacklisted_lane\x18\x03 \x03(\x0b\x32\".adu.common.new_router.LaneSegment\x12\x18\n\x10\x62lacklisted_road\x18\x04 \x03(\t\x12\x17\n\tbroadcast\x18\x05 \x01(\x08:\x04true\"\x1f\n\x0bMeasurement\x12\x10\n\x08\x64istance\x18\x01 \x01(\x01\"\xc6\x01\n\x07Passage\x12\n\n\x02id\x18\x01 \x01(\t\x12\x33\n\x07segment\x18\x02 \x03(\x0b\x32\".adu.common.new_router.LaneSegment\x12\x17\n\x08\x63\x61n_exit\x18\x03 \x01(\x08:\x05\x66\x61lse\x12H\n\x10\x63hange_lane_type\x18\x04 \x01(\x0e\x32%.adu.common.new_router.ChangeLaneType:\x07\x46ORWARD\x12\x17\n\x0fnext_passage_id\x18\x05 \x03(\t\"\x8c\x01\n\x11PassageChangeInfo\x12?\n\x10\x63hange_lane_type\x18\x01 \x01(\x0e\x32%.adu.common.new_router.ChangeLaneType\x12\x1b\n\x13start_passage_index\x18\x02 \x01(\x05\x12\x19\n\x11\x65nd_passage_index\x18\x03 \x01(\x05\"\xc8\x01\n\x0bRoadSegment\x12\n\n\x02id\x18\x01 \x01(\t\x12/\n\x07passage\x18\x02 \x03(\x0b\x32\x1e.adu.common.new_router.Passage\x12\x45\n\x13passage_change_info\x18\x03 \x03(\x0b\x32(.adu.common.new_router.PassageChangeInfo\x12\x35\n\x08waypoint\x18\x04 \x03(\x0b\x32#.adu.common.new_router.LaneWaypoint\"\xa2\x02\n\x0fRoutingResponse\x12)\n\x06header\x18\x01 \x01(\x0b\x32\x19.adu.common.header.Header\x12\x30\n\x04road\x18\x02 \x03(\x0b\x32\".adu.common.new_router.RoadSegment\x12\x37\n\x0bmeasurement\x18\x03 \x01(\x0b\x32\".adu.common.new_router.Measurement\x12>\n\x0frouting_request\x18\x04 \x01(\x0b\x32%.adu.common.new_router.RoutingRequest\x12\x13\n\x0bmap_version\x18\x05 \x01(\x0c\x12$\n\x06status\x18\x06 \x01(\x0b\x32\x14.adu.common.StatusPb*(\n\x0cWaypointType\x12\x0b\n\x07PICK_UP\x10\x00\x12\x0b\n\x07PASS_BY\x10\x01*2\n\x0e\x43hangeLaneType\x12\x0b\n\x07\x46ORWARD\x10\x00\x12\x08\n\x04LEFT\x10\x01\x12\t\n\x05RIGHT\x10\x02')
  ,
  dependencies=[header_pb2.DESCRIPTOR,common_geometry_pb2.DESCRIPTOR,error_code_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

_WAYPOINTTYPE = _descriptor.EnumDescriptor(
  name='WaypointType',
  full_name='adu.common.new_router.WaypointType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='PICK_UP', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='PASS_BY', index=1, number=1,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1396,
  serialized_end=1436,
)
_sym_db.RegisterEnumDescriptor(_WAYPOINTTYPE)

WaypointType = enum_type_wrapper.EnumTypeWrapper(_WAYPOINTTYPE)
_CHANGELANETYPE = _descriptor.EnumDescriptor(
  name='ChangeLaneType',
  full_name='adu.common.new_router.ChangeLaneType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='FORWARD', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LEFT', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RIGHT', index=2, number=2,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=1438,
  serialized_end=1488,
)
_sym_db.RegisterEnumDescriptor(_CHANGELANETYPE)

ChangeLaneType = enum_type_wrapper.EnumTypeWrapper(_CHANGELANETYPE)
PICK_UP = 0
PASS_BY = 1
FORWARD = 0
LEFT = 1
RIGHT = 2



_LANEWAYPOINT = _descriptor.Descriptor(
  name='LaneWaypoint',
  full_name='adu.common.new_router.LaneWaypoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.common.new_router.LaneWaypoint.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='s', full_name='adu.common.new_router.LaneWaypoint.s', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pose', full_name='adu.common.new_router.LaneWaypoint.pose', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='type', full_name='adu.common.new_router.LaneWaypoint.type', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=1,
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
  serialized_start=99,
  serialized_end=232,
)


_LANESEGMENT = _descriptor.Descriptor(
  name='LaneSegment',
  full_name='adu.common.new_router.LaneSegment',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.common.new_router.LaneSegment.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start_s', full_name='adu.common.new_router.LaneSegment.start_s', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end_s', full_name='adu.common.new_router.LaneSegment.end_s', index=2,
      number=3, type=1, cpp_type=5, label=1,
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
  serialized_start=234,
  serialized_end=291,
)


_ROUTINGREQUEST = _descriptor.Descriptor(
  name='RoutingRequest',
  full_name='adu.common.new_router.RoutingRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='adu.common.new_router.RoutingRequest.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='waypoint', full_name='adu.common.new_router.RoutingRequest.waypoint', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='blacklisted_lane', full_name='adu.common.new_router.RoutingRequest.blacklisted_lane', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='blacklisted_road', full_name='adu.common.new_router.RoutingRequest.blacklisted_road', index=3,
      number=4, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='broadcast', full_name='adu.common.new_router.RoutingRequest.broadcast', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=True,
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
  serialized_start=294,
  serialized_end=521,
)


_MEASUREMENT = _descriptor.Descriptor(
  name='Measurement',
  full_name='adu.common.new_router.Measurement',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='distance', full_name='adu.common.new_router.Measurement.distance', index=0,
      number=1, type=1, cpp_type=5, label=1,
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
  serialized_start=523,
  serialized_end=554,
)


_PASSAGE = _descriptor.Descriptor(
  name='Passage',
  full_name='adu.common.new_router.Passage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.common.new_router.Passage.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='segment', full_name='adu.common.new_router.Passage.segment', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='can_exit', full_name='adu.common.new_router.Passage.can_exit', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='change_lane_type', full_name='adu.common.new_router.Passage.change_lane_type', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='next_passage_id', full_name='adu.common.new_router.Passage.next_passage_id', index=4,
      number=5, type=9, cpp_type=9, label=3,
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
  serialized_start=557,
  serialized_end=755,
)


_PASSAGECHANGEINFO = _descriptor.Descriptor(
  name='PassageChangeInfo',
  full_name='adu.common.new_router.PassageChangeInfo',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='change_lane_type', full_name='adu.common.new_router.PassageChangeInfo.change_lane_type', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start_passage_index', full_name='adu.common.new_router.PassageChangeInfo.start_passage_index', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end_passage_index', full_name='adu.common.new_router.PassageChangeInfo.end_passage_index', index=2,
      number=3, type=5, cpp_type=1, label=1,
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
  serialized_start=758,
  serialized_end=898,
)


_ROADSEGMENT = _descriptor.Descriptor(
  name='RoadSegment',
  full_name='adu.common.new_router.RoadSegment',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.common.new_router.RoadSegment.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='passage', full_name='adu.common.new_router.RoadSegment.passage', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='passage_change_info', full_name='adu.common.new_router.RoadSegment.passage_change_info', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='waypoint', full_name='adu.common.new_router.RoadSegment.waypoint', index=3,
      number=4, type=11, cpp_type=10, label=3,
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
  serialized_start=901,
  serialized_end=1101,
)


_ROUTINGRESPONSE = _descriptor.Descriptor(
  name='RoutingResponse',
  full_name='adu.common.new_router.RoutingResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='adu.common.new_router.RoutingResponse.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='road', full_name='adu.common.new_router.RoutingResponse.road', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='measurement', full_name='adu.common.new_router.RoutingResponse.measurement', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='routing_request', full_name='adu.common.new_router.RoutingResponse.routing_request', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='map_version', full_name='adu.common.new_router.RoutingResponse.map_version', index=4,
      number=5, type=12, cpp_type=9, label=1,
      has_default_value=False, default_value=_b(""),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='adu.common.new_router.RoutingResponse.status', index=5,
      number=6, type=11, cpp_type=10, label=1,
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
  serialized_start=1104,
  serialized_end=1394,
)

_LANEWAYPOINT.fields_by_name['pose'].message_type = common_geometry_pb2._POINTENU
_LANEWAYPOINT.fields_by_name['type'].enum_type = _WAYPOINTTYPE
_ROUTINGREQUEST.fields_by_name['header'].message_type = header_pb2._HEADER
_ROUTINGREQUEST.fields_by_name['waypoint'].message_type = _LANEWAYPOINT
_ROUTINGREQUEST.fields_by_name['blacklisted_lane'].message_type = _LANESEGMENT
_PASSAGE.fields_by_name['segment'].message_type = _LANESEGMENT
_PASSAGE.fields_by_name['change_lane_type'].enum_type = _CHANGELANETYPE
_PASSAGECHANGEINFO.fields_by_name['change_lane_type'].enum_type = _CHANGELANETYPE
_ROADSEGMENT.fields_by_name['passage'].message_type = _PASSAGE
_ROADSEGMENT.fields_by_name['passage_change_info'].message_type = _PASSAGECHANGEINFO
_ROADSEGMENT.fields_by_name['waypoint'].message_type = _LANEWAYPOINT
_ROUTINGRESPONSE.fields_by_name['header'].message_type = header_pb2._HEADER
_ROUTINGRESPONSE.fields_by_name['road'].message_type = _ROADSEGMENT
_ROUTINGRESPONSE.fields_by_name['measurement'].message_type = _MEASUREMENT
_ROUTINGRESPONSE.fields_by_name['routing_request'].message_type = _ROUTINGREQUEST
_ROUTINGRESPONSE.fields_by_name['status'].message_type = error_code_pb2._STATUSPB
DESCRIPTOR.message_types_by_name['LaneWaypoint'] = _LANEWAYPOINT
DESCRIPTOR.message_types_by_name['LaneSegment'] = _LANESEGMENT
DESCRIPTOR.message_types_by_name['RoutingRequest'] = _ROUTINGREQUEST
DESCRIPTOR.message_types_by_name['Measurement'] = _MEASUREMENT
DESCRIPTOR.message_types_by_name['Passage'] = _PASSAGE
DESCRIPTOR.message_types_by_name['PassageChangeInfo'] = _PASSAGECHANGEINFO
DESCRIPTOR.message_types_by_name['RoadSegment'] = _ROADSEGMENT
DESCRIPTOR.message_types_by_name['RoutingResponse'] = _ROUTINGRESPONSE
DESCRIPTOR.enum_types_by_name['WaypointType'] = _WAYPOINTTYPE
DESCRIPTOR.enum_types_by_name['ChangeLaneType'] = _CHANGELANETYPE

LaneWaypoint = _reflection.GeneratedProtocolMessageType('LaneWaypoint', (_message.Message,), dict(
  DESCRIPTOR = _LANEWAYPOINT,
  __module__ = 'new_router_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.new_router.LaneWaypoint)
  ))
_sym_db.RegisterMessage(LaneWaypoint)

LaneSegment = _reflection.GeneratedProtocolMessageType('LaneSegment', (_message.Message,), dict(
  DESCRIPTOR = _LANESEGMENT,
  __module__ = 'new_router_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.new_router.LaneSegment)
  ))
_sym_db.RegisterMessage(LaneSegment)

RoutingRequest = _reflection.GeneratedProtocolMessageType('RoutingRequest', (_message.Message,), dict(
  DESCRIPTOR = _ROUTINGREQUEST,
  __module__ = 'new_router_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.new_router.RoutingRequest)
  ))
_sym_db.RegisterMessage(RoutingRequest)

Measurement = _reflection.GeneratedProtocolMessageType('Measurement', (_message.Message,), dict(
  DESCRIPTOR = _MEASUREMENT,
  __module__ = 'new_router_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.new_router.Measurement)
  ))
_sym_db.RegisterMessage(Measurement)

Passage = _reflection.GeneratedProtocolMessageType('Passage', (_message.Message,), dict(
  DESCRIPTOR = _PASSAGE,
  __module__ = 'new_router_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.new_router.Passage)
  ))
_sym_db.RegisterMessage(Passage)

PassageChangeInfo = _reflection.GeneratedProtocolMessageType('PassageChangeInfo', (_message.Message,), dict(
  DESCRIPTOR = _PASSAGECHANGEINFO,
  __module__ = 'new_router_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.new_router.PassageChangeInfo)
  ))
_sym_db.RegisterMessage(PassageChangeInfo)

RoadSegment = _reflection.GeneratedProtocolMessageType('RoadSegment', (_message.Message,), dict(
  DESCRIPTOR = _ROADSEGMENT,
  __module__ = 'new_router_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.new_router.RoadSegment)
  ))
_sym_db.RegisterMessage(RoadSegment)

RoutingResponse = _reflection.GeneratedProtocolMessageType('RoutingResponse', (_message.Message,), dict(
  DESCRIPTOR = _ROUTINGRESPONSE,
  __module__ = 'new_router_pb2'
  # @@protoc_insertion_point(class_scope:adu.common.new_router.RoutingResponse)
  ))
_sym_db.RegisterMessage(RoutingResponse)


# @@protoc_insertion_point(module_scope)
