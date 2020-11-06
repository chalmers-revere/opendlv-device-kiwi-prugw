/*
 * Copyright (C) 2018 Björnborg Nguyen
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

#include <ncurses.h>
#include <poll.h>
#include <thread>
#include <sys/stat.h> // checking if file/dir exist
#include <fstream>
#include <string>
#include <math.h>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "PwmMotors.h"

void write2file(std::string const &a_path, std::string const &a_str)
{
  std::ofstream file(a_path, std::ofstream::out);
  if (file.is_open()) {
    file << a_str;
  } else {
    std::cerr << " Could not open " << a_path << "." << std::endl;
    exit(1);
  }    
  file.flush();
  file.close();
}


void LedController(std::shared_ptr<std::mutex> mtx, std::shared_ptr<bool> isActive, std::shared_ptr<bool> programIsRunning)
{
  // std::ofstream brightness("/sys/devices/platform/leds/leds/wifi/brightness", std::ofstream::out);
  std::string const redPath = "/sys/class/leds/red/brightness";
  std::string const greenPath = "/sys/class/leds/green/brightness";
  while (*programIsRunning){
    {
      std::lock_guard<std::mutex> lock(*mtx);
      if(*isActive) {
        write2file(greenPath, "1");
        write2file(redPath, "0");
      } else {
        write2file(greenPath, "0");
        write2file(redPath, "1");
      }
    }
    sleep(1);
  }
  write2file(greenPath, "0");
  write2file(redPath, "0");
}

void ButtonListener(std::shared_ptr<std::mutex> mtx, std::shared_ptr<bool> isActive, std::shared_ptr<PwmMotors> pwmMotors, std::shared_ptr<bool> programIsRunning)
{
  struct stat sb;
  if (stat("/sys/class/gpio/gpio68", &sb) != 0) {
    write2file("/sys/class/gpio/export", "68");
  }
  if (stat("/sys/class/gpio/gpio69", &sb) != 0) {
    write2file("/sys/class/gpio/export", "69");
  }
  write2file("/sys/class/gpio/gpio68/direction", "in");
  write2file("/sys/class/gpio/gpio69/direction", "in");
  write2file("/sys/class/gpio/gpio68/edge", "falling");
  write2file("/sys/class/gpio/gpio69/edge", "falling");
  int32_t const gpio_mod_fd = open("/sys/class/gpio/gpio68/value", O_RDONLY | O_NONBLOCK );
  int32_t const gpio_pause_fd = open("/sys/class/gpio/gpio69/value", O_RDONLY | O_NONBLOCK );
  struct pollfd fdset[2];
  int32_t const nfds = 2;
  char buf[1];

  {
    std::lock_guard<std::mutex> lock(*mtx);
    pwmMotors->terminatePru();
    pwmMotors->initialisePru();
    pwmMotors->terminatePru();
  }

  while (*programIsRunning) {
    memset(&fdset[0], 0, sizeof(fdset));
    fdset[0].fd = gpio_mod_fd;
    fdset[0].events = POLLPRI;
    fdset[1].fd = gpio_pause_fd;
    fdset[1].events = POLLPRI;

    if (poll(fdset, nfds, 1000) < 0) {
      std::cout << "poll() failed!" << std::endl;
    }

    if (fdset[0].revents & POLLPRI) {
      // cluon::data::TimeStamp pressTimestamp = cluon::time::now();
      lseek(fdset[0].fd, 0, SEEK_SET);
      int32_t len = read(fdset[0].fd, buf, 1);
      if (len == 1 && atoi(buf) == 0) {
        // std::cout << "Mod pressed..." << std::endl;
        std::lock_guard<std::mutex> lock(*mtx);
        *isActive = true;
        pwmMotors->initialisePru();
        // std::cout << "PRU started!" << std::endl;
    }
    } else if (fdset[1].revents & POLLPRI) {
      cluon::data::TimeStamp pressTimestamp = cluon::time::now();
      lseek(fdset[1].fd, 0, SEEK_SET);
      int32_t len = read(fdset[1].fd, buf, 1);
      if (len == 1 && atoi(buf) == 0) {
        // std::cout << "Pause pressed..." << std::endl;
        std::lock_guard<std::mutex> lock(*mtx);
        *isActive = false;
        pwmMotors->terminatePru();
        std::cout << "PRU terminated!" << std::endl;
      }
    }
  }
  write2file("/sys/class/gpio/unexport", "68");
  write2file("/sys/class/gpio/unexport", "69");
}


int32_t main(int32_t argc, char **argv) {
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") ||
      0 == commandlineArguments.count("names") ||
      0 == commandlineArguments.count("types") ||
      0 == commandlineArguments.count("channels") ||
      0 == commandlineArguments.count("offsets") ||
      0 == commandlineArguments.count("maxvals") ||
      0 == commandlineArguments.count("angleconversion")) {
    std::cerr << argv[0] << " interfaces to the motors of the Kiwi platform." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --names=<Strings> --types=<esc or servo> --channels=<1...8>  --offsets<-1...1> --maxvals=<floats> --angleconversion=<const> [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --names=steering,propulsion --types=servo,esc --channels=1,2 --offsets=0,0 --maxvals=0.5,0 --angleconversion=1" << std::endl;
    return 1;
  } else {

    // Setup
    int32_t VERBOSE{commandlineArguments.count("verbose") != 0};
    if (VERBOSE) {
      VERBOSE = std::stoi(commandlineArguments["verbose"]);
    }
    float const FREQ = 100;

    std::vector<std::string> names = stringtoolbox::split(commandlineArguments["names"],',');
    std::vector<std::string> types = stringtoolbox::split(commandlineArguments["types"],',');
    std::vector<std::string> channels = stringtoolbox::split(commandlineArguments["channels"],',');
    std::vector<std::string> offsets = stringtoolbox::split(commandlineArguments["offsets"],',');
    std::vector<std::string> maxvals = stringtoolbox::split(commandlineArguments["maxvals"],',');
    float const angleConversion = std::stof(commandlineArguments["angleconversion"]);

    if (names.empty())
    {
      names.push_back(commandlineArguments["names"]);
      types.push_back(commandlineArguments["types"]);
      channels.push_back(commandlineArguments["channels"]);
      offsets.push_back(commandlineArguments["offsets"]);
      maxvals.push_back(commandlineArguments["maxvals"]);
    }

    if (names.size() != types.size() ||
        names.size() != types.size() ||
        names.size() != channels.size() ||
        names.size() != offsets.size() ||
        names.size() != maxvals.size()) {
      std::cerr << "Number of arguments do not match, use ',' as delimiter." << std::endl;
      return 1;
    }

    // PwmMotors pwmMotors(names, types, channels, offsets, maxvals);
    std::shared_ptr<PwmMotors> pwmMotors = std::make_shared<PwmMotors>(names, types, channels, offsets, maxvals);

    auto onGroundSteeringRequest{[&pwmMotors, &angleConversion](cluon::data::Envelope &&envelope)
    {
      if (envelope.senderStamp() == 0){
        auto const gst = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(envelope));
        float const groundSteering = gst.groundSteering()/angleConversion;

	/*float thrustersteer = 0.5f;
	if (groundSteering > 0.2f){
	  thrustersteer = 0.3f;
	} else if (groundSteering < -0.2f) {
	  thrustersteer = 0.7f;
	}*/

	pwmMotors->setMotorPower(3, groundSteering);
	pwmMotors->setMotorPower(4, groundSteering);
	//pwmMotors->setMotorPower(5, thrustersteer);
	//std::cout << "groundSteering: " << groundSteering << std::endl;

	} else if (envelope.senderStamp() == 9999) {
        auto const gst = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(envelope));
        float const groundSteering = gst.groundSteering();
        pwmMotors->setMotorOffset(3, groundSteering);
	pwmMotors->setMotorOffset(4, groundSteering);
      }
    }};
    auto onPedalPositionRequest{[&pwmMotors](cluon::data::Envelope &&envelope)
    {
	opendlv::proxy::PedalPositionRequest const ppr = cluon::extractMessage<opendlv::proxy::PedalPositionRequest>(std::move(envelope));
	float val = (ppr.position()+1)/2.0f;
	if (val > 1.0f) {
	  val = 1.0f;
	} else if (val < 0.1f){
	  val = 0.1f;
	}

	//std::cout << "Motor val: " << val << std::endl;

	pwmMotors->setMotorPower(2, val);
	pwmMotors->setMotorPower(1, val);
    }};

    auto onActuationRequest{[&pwmMotors, &angleConversion](cluon::data::Envelope &&envelope)
    {
	opendlv::proxy::ActuationRequest const actdata = cluon::extractMessage<opendlv::proxy::ActuationRequest>(std::move(envelope));

	//float actval = (actdata.acceleration()+1)/2.0f;

	float actval = 0.5f;
	if (actdata.acceleration() > 2.5f) {
	  actval = actval + actdata.acceleration()/100.0f;
	} else if (actdata.acceleration() < -0.5f){
	  actval = actval + actdata.acceleration()/20.0f;
	}

	float const actsteering = actdata.steering() / 3*(angleConversion);
	/*float actthruster = 0.5f;
	if (actsteering > 0.3f){
	  actthruster = 0.7f;
	} else if (actsteering < -0.3f) {
	  actthruster = 0.3f;
	}*/


	if (actval > 0.9f){
	   actval = 0.8f;
	} else if (actval < 0.1f){
	    actval = 0.2f;
	}

	//std::cout << "Gamepad spd value: " << actval << std::endl;
	//std::cout << "Gamepad Steering: " << actsteering << std::endl;

	pwmMotors->setMotorPower(1, actval);
	pwmMotors->setMotorPower(2, actval);
	pwmMotors->setMotorPower(3, actsteering);
	pwmMotors->setMotorPower(4, actsteering);
	//pwmMotors->setMotorPower(5, actthruster);
    }};

    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);
    od4.dataTrigger(opendlv::proxy::PedalPositionRequest::ID(), onPedalPositionRequest);
    od4.dataTrigger(opendlv::proxy::ActuationRequest::ID(), onActuationRequest);

    if (VERBOSE == 2) {
      initscr();
    }
    std::shared_ptr<std::mutex> mtx = std::make_shared<std::mutex>();
    std::shared_ptr<bool> isActive = std::make_shared<bool>(false);
    std::shared_ptr<bool> programIsRunning = std::make_shared<bool>(true);

    auto atFrequency{[&pwmMotors, &VERBOSE, &mtx, &isActive]() -> bool
    {
      std::lock_guard<std::mutex> lock(*mtx);
      // This must be called regularly (>40hz) to keep servos or ESCs awake.
      if (*isActive) {
        pwmMotors->actuate();
      }
      if (VERBOSE == 1) {
        std::cout << pwmMotors->toString() << std::endl;
      }
      if (VERBOSE == 2) {
        mvprintw(1,1,(pwmMotors->toString()).c_str()); 
        refresh();      /* Print it on to the real screen */
      }
      return true;
    }};
    std::thread ledThread(LedController, mtx, isActive, programIsRunning);
    std::thread buttonThread(ButtonListener, mtx, isActive, pwmMotors, programIsRunning);

    od4.timeTrigger(FREQ, atFrequency);

    if (VERBOSE == 2) {
      endwin();     /* End curses mode      */
    }
    *programIsRunning = false;
    ledThread.join();
    buttonThread.join();
  }
  return 0;
}
