clear all; clear java

%% Add the java classes
PWD = pwd;
javaaddpath([PWD '/rsb-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rsb-matlab-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/protobuf-java-2.5.0.jar'])
javaaddpath([PWD '/rst-fleximon-0.11-SNAPSHOT.jar'])
javaaddpath([PWD '/rst-sandbox-fleximon-0.11-SNAPSHOT.jar'])

%% Do the RPC

rsb.matlab.ConverterRegistration.register('rst.generic.MethodCallType', 'MethodCall')

b = rsb.matlab.ProtobufUtils.getBuilder('rst.generic.MethodCallType', 'MethodCall');
b.setName(com.google.protobuf.ByteString.copyFromUtf8('doFoo'));
a1 = b.addArgumentsBuilder();
valueClass = rsb.matlab.ProtobufUtils.getInnerClass('rst.generic.ValueType', 'Value');
typeClass = rsb.matlab.ProtobufUtils.getInnerClass(valueClass, 'Type');
d = rsb.matlab.ProtobufUtils.getEnumConstant(typeClass, 'DOUBLE');
a1.setType(d);
a1.setDouble(12.34)

factory = rsb.Factory.getInstance();
srv = factory.createRemoteServer('/example/server');
srv.activate();
reply = srv.call('echo', b.build());
