/*
 * extspread.cxx
 *
 *  Created on: Jan 22, 2015
 *      Author: nneumann <nneumann@techfak.uni-bielefeld.de>
 */

#include "extspread.hpp"
#include <iostream>
#include <boost/program_options.hpp>
#include <rsb/Factory.h>
#include <rsb/Handler.h>

using namespace std;

rsb::ParticipantConfig getextspreadconfig(rsb::Factory& factory, std::string host, std::string port)
{
      // Get the global participant config as a template
      rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
            {
              // Get the options for spread transport
              rsc::runtime::Properties tmpPropSocket  = tmpPartConf.mutableTransport("spread").getOptions();

              // Change the config
              tmpPropSocket["host"] = boost::any(host);

              // Change the Port
              tmpPropSocket["port"] = boost::any(port);

              // Write the spread transport options back
              tmpPartConf.mutableTransport("spread").setOptions(tmpPropSocket);
            }
	//return the new Spreadconfig
	return tmpPartConf;
}
