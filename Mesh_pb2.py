# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: Mesh.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\nMesh.proto\"9\n\x0b\x46loat4Proto\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\x12\t\n\x01w\x18\x04 \x01(\x02\"x\n\x0eTransformProto\x12\x18\n\x02\x63\x31\x18\x01 \x01(\x0b\x32\x0c.Float4Proto\x12\x18\n\x02\x63\x32\x18\x02 \x01(\x0b\x32\x0c.Float4Proto\x12\x18\n\x02\x63\x33\x18\x03 \x01(\x0b\x32\x0c.Float4Proto\x12\x18\n\x02\x63\x34\x18\x04 \x01(\x0b\x32\x0c.Float4Proto\"O\n\x0bVertexProto\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\x12\t\n\x01u\x18\x04 \x01(\x02\x12\t\n\x01v\x18\x05 \x01(\x02\x12\t\n\x01w\x18\x06 \x01(\x02\"[\n\tMeshProto\x12\x1e\n\x08vertices\x18\x01 \x03(\x0b\x32\x0c.VertexProto\x12\"\n\ttransform\x18\x02 \x01(\x0b\x32\x0f.TransformProto\x12\n\n\x02id\x18\x03 \x01(\t\")\n\x0bMeshesProto\x12\x1a\n\x06meshes\x18\x01 \x03(\x0b\x32\n.MeshProto\"?\n\x11\x44irectionAndDepth\x12\t\n\x01u\x18\x01 \x01(\x02\x12\t\n\x01v\x18\x02 \x01(\x02\x12\t\n\x01w\x18\x03 \x01(\x02\x12\t\n\x01\x64\x18\x04 \x01(\x02\"A\n\x06Points\x12\"\n\x06points\x18\x01 \x03(\x0b\x32\x12.DirectionAndDepth\x12\x13\n\x0b\x63onfidences\x18\x02 \x03(\rb\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'Mesh_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _FLOAT4PROTO._serialized_start=14
  _FLOAT4PROTO._serialized_end=71
  _TRANSFORMPROTO._serialized_start=73
  _TRANSFORMPROTO._serialized_end=193
  _VERTEXPROTO._serialized_start=195
  _VERTEXPROTO._serialized_end=274
  _MESHPROTO._serialized_start=276
  _MESHPROTO._serialized_end=367
  _MESHESPROTO._serialized_start=369
  _MESHESPROTO._serialized_end=410
  _DIRECTIONANDDEPTH._serialized_start=412
  _DIRECTIONANDDEPTH._serialized_end=475
  _POINTS._serialized_start=477
  _POINTS._serialized_end=542
# @@protoc_insertion_point(module_scope)
