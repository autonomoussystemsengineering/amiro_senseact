#include "main.hpp"

namespace muroxConverter {
// We have to pass two arguments to the base-class constructor:
// 1. The data-type
// 2. The wire-schema
//
// Note: this could also be written as
// Converter<string>("vector of TYPE_PAYLOAD_", RSB_TYPE_TAG())
// to infer the "string" name of the data-type using RTTI.
CLASS_NAME_::CLASS_NAME_() :
    Converter<string> (rsc::runtime::typeName<vector<TYPE_PAYLOAD_> >(), INFO_PAYLOAD_STRING_, true) {
}

string CLASS_NAME_::serialize(const AnnotatedData& data, string& wire) {
    // Ensure that DATA actually holds a datum of the data-type we
    // expect.
    assert(data.first == getDataType());

    // Force conversion to the expected data-type.
    //
    // NOTE: a dynamic_pointer_cast cannot be used from void*
    boost::shared_ptr<const std::vector<TYPE_PAYLOAD_> > vector =
            static_pointer_cast<const std::vector<TYPE_PAYLOAD_> > (data.second);

    // Prepare the sending data
    //
    // Get the information of the matrix and store it
	    
    // Get number of values in the vector
    TYPE_UINT_ uiVecSize = (TYPE_UINT_) vector->size();
    
    // Resize the wire
    wire.resize( SIZE_UINT_ + SIZE_PAYLOAD_ * uiVecSize );
    
    // Copy the data
    // Copy the size of the vector
    copy(((char*) &uiVecSize), ((char*) &uiVecSize) + SIZE_UINT_, wire.begin());
    // Copy the vector values
    copy( (char*) &((*vector)[0]), ((char*) &((*vector)[uiVecSize])) + SIZE_PAYLOAD_, wire.begin() + SIZE_UINT_ );
    
    // Return the wire-schema of the serialized representation in
    // WIRE.
    return getWireSchema(); // this->getWireSchema() == "vector of TYPE_PAYLOAD_"
}

AnnotatedData CLASS_NAME_::deserialize(const string& wireSchema,
        const string& wire) {
    // Ensure that WIRE uses the expected wire-schema.

    assert(wireSchema == getWireSchema()); // this->getWireSchema() == "vector of TYPE_PAYLOAD_"

    // Allocate a new SimpleImage object and set its data members from
    // the content of WIRE.
    //

    // Get number of values in the vector
    const TYPE_UINT_ uiVecSize = *(TYPE_UINT_*)   &*wire.begin();

    // Create and copy the vector
    const std::vector<TYPE_PAYLOAD_>* vector = new std::vector<TYPE_PAYLOAD_>( (TYPE_PAYLOAD_*) &*(wire.begin() + SIZE_PAYLOAD_), (TYPE_PAYLOAD_*) &*(wire.begin() + SIZE_UINT_ + SIZE_PAYLOAD_ * uiVecSize) );

    
      return make_pair(getDataType(), boost::shared_ptr< std::vector<TYPE_PAYLOAD_> > (vector));
}
}