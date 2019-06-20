namespace muroxConverter {
// We have to pass two arguments to the base-class constructor:
// 1. The data-type
// 2. The wire-schema
//
// Note: this could also be written as
// Converter<string>("vector of T", RSB_TYPE_TAG())
// to infer the "string" name of the data-type using RTTI.
template<class T>
vecConverter<T>::vecConverter() :
    Converter<string> (rsc::runtime::typeName<vector<T> >(), "vector", true) {
}

template<class T>
string vecConverter<T>::serialize(const AnnotatedData& data, string& wire) {
    // Ensure that DATA actually holds a datum of the data-type we
    // expect.
    assert(data.first == getDataType());

    // Force conversion to the expected data-type.
    //
    // NOTE: a dynamic_pointer_cast cannot be used from void*
    boost::shared_ptr<const std::vector<T> > vector =
            static_pointer_cast<const std::vector<T> > (data.second);

    // Prepare the sending data
    //
    // Get the information of the vector and store it
	    
    // Get number of values in the vector
    const unsigned int uiVecSize = vector->size();
    
    // Resize the wire
    wire.resize( sizeof(unsigned int) + sizeof(T) * uiVecSize );
    
    // Copy the data
    // Copy the size of the vector
    copy(((char*) &uiVecSize), ((char*) &uiVecSize) + sizeof(unsigned int), wire.begin());
    // Copy the vector values
    copy( (char*) &((*vector)[0]), ((char*) &((*vector)[uiVecSize])) + sizeof(T), wire.begin() + sizeof(unsigned int) );
    
    // Return the wire-schema of the serialized representation in
    // WIRE.
    return getWireSchema(); // this->getWireSchema() == "vector of T"
}

template<class T>
AnnotatedData vecConverter<T>::deserialize(const string& wireSchema,
        const string& wire) {
    // Ensure that WIRE uses the expected wire-schema.

    assert(wireSchema == getWireSchema()); // this->getWireSchema() == "vector of T"

    // Get number of values in the vector
    const unsigned int uiVecSize = *(unsigned int*)   &*wire.begin();

    // Create and copy the vector
    std::vector<T>* vector = new std::vector<T>( (T*) &*(wire.begin() + sizeof(T)), (T*) &*(wire.begin() + sizeof(unsigned int) + sizeof(T) * uiVecSize) );

    
      return make_pair(getDataType(), boost::shared_ptr< std::vector<T> > (vector));
}
}
