#!/bin/bash
echo "Running protoc for every *.proto file to create the *pb.cc and *pb.h files:"
RST_V=${1}
if [[ ${RST_V} = "" ]]; then
  echo "No RST version specified: exiting"
  exit 1
fi

echo "Using RST version ${RST_V}"
# To be more explicit, we run every file to get propter output
for list in $(ls *.proto)
do
  echo "protoc ${list} --cpp_out=."
  protoc ${list} --cpp_out=. --proto_path=. --proto_path=`pkg-config --variable=prefix rst${RST_V}`/share/rst${RST_V}/proto/sandbox/ --proto_path=`pkg-config --variable=prefix rst${RST_V}`/share/rst${RST_V}/proto/stable/
done
