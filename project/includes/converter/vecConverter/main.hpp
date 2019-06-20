//============================================================================
// Name        : vecConverter.cpp
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Converter for vectors of template type T
// Usage       :
//       // Prepare RSB informer
//       #define MYTYPE int
//       shared_ptr<vecConverter<MYTYPE> > converter(new vecConverter<MYTYPE>());
//       converterRepository<std::string>()->registerConverter(converter);
//       rsb::Factory& factory = rsb::Factory::getInstance();
//       rsb::Informer< std::vector<MYTYPE> >::Ptr informer_vec = factory.createInformer< std::vector<MYTYPE> > ("/scope");
// 
//       // Allocate a vector for sending with 10 values initalized with 0
//       shared_ptr< std::vector<MYTYPE> > vecData( new std::vector<MYTYPE>(10,0) );
//       
//       for(;;) {
// 	      // Send data
// 	      informer_vec->publish(vecData);
//       }
//============================================================================

#pragma once

#include <rsb/converter/Converter.h>


using namespace std;
using namespace boost;
using namespace rsb;
using namespace rsb::converter;

namespace muroxConverter {

/**
 * A simple converter for the vec object.
 */
template<class T>
class vecConverter: public rsb::converter::Converter<std::string> {
public:
    vecConverter();

    std::string serialize(const rsb::AnnotatedData& data,
            std::string& wire);

    rsb::AnnotatedData deserialize(const std::string& wireSchema,
            const std::string& wire);
};

}

#include "main.tpp"
