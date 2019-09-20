# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sim_map.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import map_lane_pb2
import map_geometry_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='sim_map.proto',
  package='adu.simulation',
  serialized_pb=_b('\n\rsim_map.proto\x12\x0e\x61\x64u.simulation\x1a\x0emap_lane.proto\x1a\x12map_geometry.proto\"V\n\x0fSimCurveSegment\x12\x35\n\x0cline_segment\x18\x01 \x01(\x0b\x32\x1d.adu.common.hdmap.LineSegmentH\x00\x42\x0c\n\ncurve_type\"<\n\x08SimCurve\x12\x30\n\x07segment\x18\x01 \x03(\x0b\x32\x1f.adu.simulation.SimCurveSegment\"F\n\x0cSimCrosswalk\x12\n\n\x02id\x18\x01 \x01(\t\x12*\n\x07polygon\x18\x02 \x01(\x0b\x32\x19.adu.common.hdmap.Polygon\"9\n\x0cSimSubsignal\x12)\n\x08location\x18\x01 \x01(\x0b\x32\x17.adu.common.hdmap.Point\"\xa2\x01\n\tSimSignal\x12\n\n\x02id\x18\x01 \x01(\t\x12+\n\x08\x62oundary\x18\x02 \x01(\x0b\x32\x19.adu.common.hdmap.Polygon\x12/\n\tsubsignal\x18\x03 \x03(\x0b\x32\x1c.adu.simulation.SimSubsignal\x12+\n\tstop_line\x18\x04 \x03(\x0b\x32\x18.adu.simulation.SimCurve\"\x86\x01\n\x0fSimLaneBoundary\x12\'\n\x05\x63urve\x18\x01 \x01(\x0b\x32\x18.adu.simulation.SimCurve\x12\x0f\n\x07virtual\x18\x02 \x01(\x08\x12\x39\n\rboundary_type\x18\x03 \x03(\x0b\x32\".adu.common.hdmap.LaneBoundaryType\"\xb7\x01\n\x07SimLane\x12\n\n\x02id\x18\x01 \x01(\t\x12/\n\rcentral_curve\x18\x02 \x01(\x0b\x32\x18.adu.simulation.SimCurve\x12\x36\n\rleft_boundary\x18\x03 \x01(\x0b\x32\x1f.adu.simulation.SimLaneBoundary\x12\x37\n\x0eright_boundary\x18\x04 \x01(\x0b\x32\x1f.adu.simulation.SimLaneBoundary\"\x8b\x01\n\x06SimMap\x12/\n\tcrosswalk\x18\x01 \x03(\x0b\x32\x1c.adu.simulation.SimCrosswalk\x12%\n\x04lane\x18\x02 \x03(\x0b\x32\x17.adu.simulation.SimLane\x12)\n\x06signal\x18\x03 \x03(\x0b\x32\x19.adu.simulation.SimSignal')
  ,
  dependencies=[map_lane_pb2.DESCRIPTOR,map_geometry_pb2.DESCRIPTOR,])
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_SIMCURVESEGMENT = _descriptor.Descriptor(
  name='SimCurveSegment',
  full_name='adu.simulation.SimCurveSegment',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='line_segment', full_name='adu.simulation.SimCurveSegment.line_segment', index=0,
      number=1, type=11, cpp_type=10, label=1,
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
    _descriptor.OneofDescriptor(
      name='curve_type', full_name='adu.simulation.SimCurveSegment.curve_type',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=69,
  serialized_end=155,
)


_SIMCURVE = _descriptor.Descriptor(
  name='SimCurve',
  full_name='adu.simulation.SimCurve',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='segment', full_name='adu.simulation.SimCurve.segment', index=0,
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
  serialized_start=157,
  serialized_end=217,
)


_SIMCROSSWALK = _descriptor.Descriptor(
  name='SimCrosswalk',
  full_name='adu.simulation.SimCrosswalk',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.simulation.SimCrosswalk.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='polygon', full_name='adu.simulation.SimCrosswalk.polygon', index=1,
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
  serialized_start=219,
  serialized_end=289,
)


_SIMSUBSIGNAL = _descriptor.Descriptor(
  name='SimSubsignal',
  full_name='adu.simulation.SimSubsignal',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='location', full_name='adu.simulation.SimSubsignal.location', index=0,
      number=1, type=11, cpp_type=10, label=1,
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
  serialized_start=291,
  serialized_end=348,
)


_SIMSIGNAL = _descriptor.Descriptor(
  name='SimSignal',
  full_name='adu.simulation.SimSignal',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.simulation.SimSignal.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='boundary', full_name='adu.simulation.SimSignal.boundary', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='subsignal', full_name='adu.simulation.SimSignal.subsignal', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='stop_line', full_name='adu.simulation.SimSignal.stop_line', index=3,
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
  serialized_start=351,
  serialized_end=513,
)


_SIMLANEBOUNDARY = _descriptor.Descriptor(
  name='SimLaneBoundary',
  full_name='adu.simulation.SimLaneBoundary',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='curve', full_name='adu.simulation.SimLaneBoundary.curve', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='virtual', full_name='adu.simulation.SimLaneBoundary.virtual', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='boundary_type', full_name='adu.simulation.SimLaneBoundary.boundary_type', index=2,
      number=3, type=11, cpp_type=10, label=3,
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
  serialized_start=516,
  serialized_end=650,
)


_SIMLANE = _descriptor.Descriptor(
  name='SimLane',
  full_name='adu.simulation.SimLane',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='adu.simulation.SimLane.id', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='central_curve', full_name='adu.simulation.SimLane.central_curve', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='left_boundary', full_name='adu.simulation.SimLane.left_boundary', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='right_boundary', full_name='adu.simulation.SimLane.right_boundary', index=3,
      number=4, type=11, cpp_type=10, label=1,
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
  serialized_start=653,
  serialized_end=836,
)


_SIMMAP = _descriptor.Descriptor(
  name='SimMap',
  full_name='adu.simulation.SimMap',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='crosswalk', full_name='adu.simulation.SimMap.crosswalk', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='lane', full_name='adu.simulation.SimMap.lane', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='signal', full_name='adu.simulation.SimMap.signal', index=2,
      number=3, type=11, cpp_type=10, label=3,
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
  serialized_start=839,
  serialized_end=978,
)

_SIMCURVESEGMENT.fields_by_name['line_segment'].message_type = map_geometry_pb2._LINESEGMENT
_SIMCURVESEGMENT.oneofs_by_name['curve_type'].fields.append(
  _SIMCURVESEGMENT.fields_by_name['line_segment'])
_SIMCURVESEGMENT.fields_by_name['line_segment'].containing_oneof = _SIMCURVESEGMENT.oneofs_by_name['curve_type']
_SIMCURVE.fields_by_name['segment'].message_type = _SIMCURVESEGMENT
_SIMCROSSWALK.fields_by_name['polygon'].message_type = map_geometry_pb2._POLYGON
_SIMSUBSIGNAL.fields_by_name['location'].message_type = map_geometry_pb2._POINT
_SIMSIGNAL.fields_by_name['boundary'].message_type = map_geometry_pb2._POLYGON
_SIMSIGNAL.fields_by_name['subsignal'].message_type = _SIMSUBSIGNAL
_SIMSIGNAL.fields_by_name['stop_line'].message_type = _SIMCURVE
_SIMLANEBOUNDARY.fields_by_name['curve'].message_type = _SIMCURVE
_SIMLANEBOUNDARY.fields_by_name['boundary_type'].message_type = map_lane_pb2._LANEBOUNDARYTYPE
_SIMLANE.fields_by_name['central_curve'].message_type = _SIMCURVE
_SIMLANE.fields_by_name['left_boundary'].message_type = _SIMLANEBOUNDARY
_SIMLANE.fields_by_name['right_boundary'].message_type = _SIMLANEBOUNDARY
_SIMMAP.fields_by_name['crosswalk'].message_type = _SIMCROSSWALK
_SIMMAP.fields_by_name['lane'].message_type = _SIMLANE
_SIMMAP.fields_by_name['signal'].message_type = _SIMSIGNAL
DESCRIPTOR.message_types_by_name['SimCurveSegment'] = _SIMCURVESEGMENT
DESCRIPTOR.message_types_by_name['SimCurve'] = _SIMCURVE
DESCRIPTOR.message_types_by_name['SimCrosswalk'] = _SIMCROSSWALK
DESCRIPTOR.message_types_by_name['SimSubsignal'] = _SIMSUBSIGNAL
DESCRIPTOR.message_types_by_name['SimSignal'] = _SIMSIGNAL
DESCRIPTOR.message_types_by_name['SimLaneBoundary'] = _SIMLANEBOUNDARY
DESCRIPTOR.message_types_by_name['SimLane'] = _SIMLANE
DESCRIPTOR.message_types_by_name['SimMap'] = _SIMMAP

SimCurveSegment = _reflection.GeneratedProtocolMessageType('SimCurveSegment', (_message.Message,), dict(
  DESCRIPTOR = _SIMCURVESEGMENT,
  __module__ = 'sim_map_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.SimCurveSegment)
  ))
_sym_db.RegisterMessage(SimCurveSegment)

SimCurve = _reflection.GeneratedProtocolMessageType('SimCurve', (_message.Message,), dict(
  DESCRIPTOR = _SIMCURVE,
  __module__ = 'sim_map_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.SimCurve)
  ))
_sym_db.RegisterMessage(SimCurve)

SimCrosswalk = _reflection.GeneratedProtocolMessageType('SimCrosswalk', (_message.Message,), dict(
  DESCRIPTOR = _SIMCROSSWALK,
  __module__ = 'sim_map_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.SimCrosswalk)
  ))
_sym_db.RegisterMessage(SimCrosswalk)

SimSubsignal = _reflection.GeneratedProtocolMessageType('SimSubsignal', (_message.Message,), dict(
  DESCRIPTOR = _SIMSUBSIGNAL,
  __module__ = 'sim_map_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.SimSubsignal)
  ))
_sym_db.RegisterMessage(SimSubsignal)

SimSignal = _reflection.GeneratedProtocolMessageType('SimSignal', (_message.Message,), dict(
  DESCRIPTOR = _SIMSIGNAL,
  __module__ = 'sim_map_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.SimSignal)
  ))
_sym_db.RegisterMessage(SimSignal)

SimLaneBoundary = _reflection.GeneratedProtocolMessageType('SimLaneBoundary', (_message.Message,), dict(
  DESCRIPTOR = _SIMLANEBOUNDARY,
  __module__ = 'sim_map_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.SimLaneBoundary)
  ))
_sym_db.RegisterMessage(SimLaneBoundary)

SimLane = _reflection.GeneratedProtocolMessageType('SimLane', (_message.Message,), dict(
  DESCRIPTOR = _SIMLANE,
  __module__ = 'sim_map_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.SimLane)
  ))
_sym_db.RegisterMessage(SimLane)

SimMap = _reflection.GeneratedProtocolMessageType('SimMap', (_message.Message,), dict(
  DESCRIPTOR = _SIMMAP,
  __module__ = 'sim_map_pb2'
  # @@protoc_insertion_point(class_scope:adu.simulation.SimMap)
  ))
_sym_db.RegisterMessage(SimMap)


# @@protoc_insertion_point(module_scope)