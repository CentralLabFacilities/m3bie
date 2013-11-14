# Generated by the protocol buffer compiler.  DO NOT EDIT!

from google.protobuf import descriptor
from google.protobuf import message
from google.protobuf import reflection
from google.protobuf import service
from google.protobuf import service_reflection
from google.protobuf import descriptor_pb2



_M3EXAMPLESTATUS = descriptor.Descriptor(
  name='M3ExampleStatus',
  full_name='M3ExampleStatus',
  filename='example.proto',
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='base', full_name='M3ExampleStatus.base', index=0,
      number=1, type=11, cpp_type=10, label=1,
      default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='foo', full_name='M3ExampleStatus.foo', index=1,
      number=2, type=1, cpp_type=5, label=1,
      default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],  # TODO(robinson): Implement.
  enum_types=[
  ],
  options=None)


_M3EXAMPLEPARAM = descriptor.Descriptor(
  name='M3ExampleParam',
  full_name='M3ExampleParam',
  filename='example.proto',
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='max_fx', full_name='M3ExampleParam.max_fx', index=0,
      number=1, type=1, cpp_type=5, label=1,
      default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='max_fy', full_name='M3ExampleParam.max_fy', index=1,
      number=2, type=1, cpp_type=5, label=1,
      default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='max_fz', full_name='M3ExampleParam.max_fz', index=2,
      number=3, type=1, cpp_type=5, label=1,
      default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],  # TODO(robinson): Implement.
  enum_types=[
  ],
  options=None)


_M3EXAMPLECOMMAND = descriptor.Descriptor(
  name='M3ExampleCommand',
  full_name='M3ExampleCommand',
  filename='example.proto',
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='enable', full_name='M3ExampleCommand.enable', index=0,
      number=1, type=8, cpp_type=7, label=1,
      default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='fx', full_name='M3ExampleCommand.fx', index=1,
      number=2, type=1, cpp_type=5, label=1,
      default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='fy', full_name='M3ExampleCommand.fy', index=2,
      number=3, type=1, cpp_type=5, label=1,
      default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='fz', full_name='M3ExampleCommand.fz', index=3,
      number=4, type=1, cpp_type=5, label=1,
      default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],  # TODO(robinson): Implement.
  enum_types=[
  ],
  options=None)

import component_base_pb2

_M3EXAMPLESTATUS.fields_by_name['base'].message_type = component_base_pb2._M3BASESTATUS

class M3ExampleStatus(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3EXAMPLESTATUS

class M3ExampleParam(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3EXAMPLEPARAM

class M3ExampleCommand(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3EXAMPLECOMMAND
