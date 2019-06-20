#include "OcTreeConverter.h"

namespace converter_OcTree {

// We have to pass two arguments to the base-class constructor:
// 1. The data-type
// 2. The wire-schema
//
// Note: this could also be written as
// Converter<string>("octal-tree", RSB_TYPE_TAG(OcTree))
// to infer the "string" name of the data-type using RTTI.
OcTreeConverter::OcTreeConverter() :
    Converter<string> ("octomap::OcTree", "octal-tree", true) {
}

string OcTreeConverter::serialize(const AnnotatedData& data, string& wire) {
    // Ensure that DATA actually holds a datum of the data-type we
    // expect.
    assert(data.first == getDataType()); // this->getDataType() == "converter_OcTree::OcTree"

    // Force conversion to the expected data-type.
    //
    boost::shared_ptr<const OcTree> tree =
            static_pointer_cast<const OcTree> (data.second);

    // Prepare the sending data
    //
    // Get the binary data of the octree and write it to a char array
    std::ostringstream stream;                  // Define a outputstream
    tree->writeBinaryData(stream);              // Write binary data to outputstream
    std::string   str = stream.str();           // Copy the stream to a string
    const char*   chr = str.c_str();            // Copy the string to a char
    // Get the resolution of the leafs
    const  double leaf_res = tree->getResolution();     // Get the resolution
    // Get the amount of leafs
    const  uint   str_size = str.size();        // Get the amount of binary data


    // Store the content of TREE in WIRE according to the selected
    // binary layout.
    // Define the amount of sending data
    // Allocate amount of leafs (int -> 4), the resolution (double -> 8) and the data (char[] -> str.size())
    wire.resize( sizeof(uint) + sizeof(double) + str.size() );
    // Write the amount of leafs
    copy(((char*) &str_size), ((char*) &str_size) + sizeof(uint), wire.begin());
    // Write the resolution of leafs
    copy(((char*) &leaf_res), ((char*) &leaf_res) + sizeof(double), wire.begin() + sizeof(uint));
    // Write the data
    copy(chr, chr + str.size(), wire.begin() + sizeof(uint) + sizeof(double) );

    // Return the wire-schema of the serialized representation in
    // WIRE.
    return getWireSchema(); // this->getWireSchema() == "octal-tree"
}

AnnotatedData OcTreeConverter::deserialize(const string& wireSchema,
        const string& wire) {
    // Ensure that WIRE uses the expected wire-schema.
    assert(wireSchema == getWireSchema()); // this->getWireSchema() == "octal-tree"

    // Allocate a new SimpleImage object and set its data members from
    // the content of WIRE.
    //

    // Get the amount of leafs
    const uint   str_size = *((uint*)   &*wire.begin());
    // Get the resolution
    const double leaf_res = *((double*) &*(wire.begin() + sizeof(uint)));


    // store the binary data to a stream
    std::istringstream stream(std::string(&*(wire.begin() + sizeof(uint) + sizeof(double)), str_size));

    // Save the stream to the octree
    OcTree * tree = new OcTree(leaf_res);
    tree->readBinaryData(stream);

    // Return (a shared_ptr to) the constructed object along with its
    // data-type.
    return make_pair(getDataType(), boost::shared_ptr<OcTree> (tree));
}

}
