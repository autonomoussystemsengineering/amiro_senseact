/*
 * extspread.hpp
 *
 *  Created on: Jan 22, 2015
 *      Author: nneumann <nneumann@techfak.uni-bielefeld.de>
 */

#ifndef EXTSPREAD_HPP_
#define EXTSPREAD_HPP_

#include <rsb/Factory.h>
#include <iostream>

rsb::ParticipantConfig getextspreadconfig(rsb::Factory& factory, std::string host, std::string port);

#endif /* EXTSPREAD_HPP_ */
