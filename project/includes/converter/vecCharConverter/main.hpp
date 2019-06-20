//============================================================================
// Name        : vec*Converter.cpp
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Converter for vectors of *
// Usage       :
//       // Prepare RSB informer
//       shared_ptr<vec*Converter> converter(new vec*Converter());
//       converterRepository<std::string>()->registerConverter(converter);
//       rsb::Factory& factory = rsb::Factory::getInstance();
//       rsb::Informer< std::vector<*> >::Ptr informer_vec = factory.createInformer< std::vector<*> > ("/scope");
// 
//       // Allocate a vector for sending with 10 values initalized with 0
//       shared_ptr< std::vector<*> > vecData( new std::vector<*>(10,0) );
//       
//       for(;;) {
// 	      // Send data
// 	      informer_vec->publish(vecData);
//       }
//============================================================================

#pragma once

#include <rsb/converter/Converter.h>

#ifdef SIZE_UINT_
#undef SIZE_UINT_
#endif
#ifdef TYPE_UINT_
#undef TYPE_UINT_
#endif

#define SIZE_UINT_ sizeof(unsigned int)
#define TYPE_UINT_ unsigned int

// Definition of the payload
// If you want another vector converter, then replace every datatype (e.g. int)
// in the following defines

#ifdef SIZE_PAYLOAD_
#undef SIZE_PAYLOAD_
#endif
#ifdef TYPE_PAYLOAD_
#undef TYPE_PAYLOAD_
#endif
#ifdef TYPE_PAYLOAD_STRING_
#undef TYPE_PAYLOAD_STRING_
#endif
#ifdef INFO_PAYLOAD_STRING_
#undef INFO_PAYLOAD_STRING_
#endif
#ifdef CLASS_NAME_
#undef CLASS_NAME_
#endif

#define SIZE_PAYLOAD_ sizeof(char)
#define TYPE_PAYLOAD_ char
#define TYPE_PAYLOAD_STRING_ "std::vector<char, std::allocator<char> >"
#define INFO_PAYLOAD_STRING_ "vector"
#define CLASS_NAME_ vecCharConverter

using namespace std;
using namespace boost;
using namespace rsb;
using namespace rsb::converter;

namespace muroxConverter {

/**
 * A simple converter for the vec object.
 */
class CLASS_NAME_: public rsb::converter::Converter<std::string> {
public:
    CLASS_NAME_();

    std::string serialize(const rsb::AnnotatedData& data,
            std::string& wire);

    rsb::AnnotatedData deserialize(const std::string& wireSchema,
            const std::string& wire);
};

}
