/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "serialport.hpp"
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>

#include <thread>
#include <cmath>
#include <ctime>
#include <chrono>

using std::string;
using std::stringstream;
using std::cout;
using std::cerr;
using std::endl;

int main(int argc, char **argv) {
		int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("freq")) || (0 == commandlineArguments.count("cid"))) {
    	td::cerr << argv[0] << " : Module handling communication between Lynx and STM32F4" << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--id=<Identifier in case of multiple STM32F4 units>] [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " ADD EXAMPLE HERE" << std::endl;
        retCode = 1;
    } else {
    
    const uint32_t ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const float FREQ{std::stof(commandlineArguments["freq"])};
        std::cout << "Micro-Service ID:" << ID << std::endl;

        // Interface to a running OpenDaVINCI session.
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        
        //Test receiving of incoming requests
        auto onSwitchStateReading{[](cluon::data::Envelope &&envelope)
            {
                cout << "Something\n";
            }};
            od4.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), onSwitchStateReading);

			/*
			This example sends strings "<read/write>.<pin>.<value>" to STM32F4
			Upon receiving, STM32F4 would send the received messages back
			*/
    	//Serial port of STM32F4
    	string port("/dev/ttyACM0");
    	serial::Port myPort(port, 38400);
			cout << "Serial port created\n";

    	auto readingCallback = [](const string data){
        cout << "Received: " << data << std::endl;
   	 	};

    	myPort.read(readingCallback);
			/*
			Send read request to pin 12, with value as count (until 100000). then exit
			*/
    	int count = 0;
    	while(count++ < 100000) {
        stringstream sstr;
        sstr << "r.p12." << count << std::endl;
        const string testData = sstr.str();
        myPort.write(testData);
        usleep(5*1000);
     	}
    }
		cout << "Exit service\n";
    return 0;
}
