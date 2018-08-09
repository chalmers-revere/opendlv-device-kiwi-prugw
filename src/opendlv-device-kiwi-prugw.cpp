/*
 * Copyright (C) 2018 Ola Benderius
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

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "PwmMotors.h"


[[noreturn]] void LedController(std::mutex *mtx, bool *isActive)
{
  std::ofstream brightness("/sys/devices/platform/leds/leds/wifi/brightness", std::ofstream::out);
  if (brightness.is_open()) {
    bool isLit = false; 
    while (1){
      {
        std::lock_guard<std::mutex> lock(*mtx);
        if(isActive) {
          brightness << '1';
        } else {
          brightness << std::to_string(isLit);
          isLit = !isLit;
        }
      }
      sleep(1);
    }
  } else {
    std::cerr << "Could not open led node." << std::endl;
    exit(1);
  }    
  brightness.flush();
  brightness.close();

}

[[noreturn]] void ButtonListener(std::mutex *mtx, bool *isActive, PwmMotors *pwmMotors)
{
  int32_t const gpio_mod_fd = open("/sys/class/gpio/gpio68/val", O_RDONLY | O_NONBLOCK );
  int32_t const gpio_pause_fd = open("/sys/class/gpio/gpio69/val", O_RDONLY | O_NONBLOCK );
  struct pollfd fdset[2];
  int32_t const nfds = 2;
  // int gpio_fd, rc;
  char buf[1];

  while (1) {
    memset(&fdset[0], 0, sizeof(fdset));

    fdset[0].fd = gpio_mod_fd;
    fdset[0].events = POLLPRI;
    fdset[1].fd = gpio_pause_fd;
    fdset[1].events = POLLPRI;

    if (poll(fdset, nfds, -1) < 0) {
      std::cout << "poll() failed!" << std::endl;
      exit(1);
    }

    if (fdset[0].revents & POLLPRI) {
      cluon::data::TimeStamp pressTimestamp = cluon::time::now();
      lseek(fdset[0].fd, 0, SEEK_SET);
      int len = read(fdset[0].fd, buf, 1);
      if (len == 1 && atoi(buf) == '1') {
        std::cout << "Mod pressed..." << std::endl;
        if (poll(&fdset[0], nfds-1, -1) < 0) {
          std::cout << "poll() failed!" << std::endl;
          exit(1);
        }
        if (fdset[0].revents & POLLPRI) {
          std::cout << "Mod released...." << std::endl;
          cluon::data::TimeStamp releaseTimestamp = cluon::time::now();
          double ref = (double) releaseTimestamp.seconds() + (double) releaseTimestamp.microseconds();
          ref -= ((double) pressTimestamp.seconds() + (double) pressTimestamp.microseconds());
          std::cout << "Mod held for " << ref << "seconds." << std::endl;
          {
            std::lock_guard<std::mutex> lock(*mtx);
            if (ref < 1.0) {
              *isActive = true;
              pwmMotors->initialisePru();
            } else {
              *isActive = false;
              pwmMotors->terminatePru();
            }
          }
        }
      }
    } else if (fdset[1].revents & POLLPRI) {
      cluon::data::TimeStamp pressTimestamp = cluon::time::now();
      lseek(fdset[1].fd, 0, SEEK_SET);
      int len = read(fdset[1].fd, buf, 1);
      if (len == 1 && atoi(buf) == '1') {
        std::cout << "Pause pressed..." << std::endl;
        if (poll(&fdset[1], nfds-1, -1) < 0) {
          std::cout << "poll() failed!" << std::endl;
          exit(1);
        }
        if (fdset[0].revents & POLLPRI) {
          std::cout << "Pause released...." << std::endl;
          cluon::data::TimeStamp releaseTimestamp = cluon::time::now();
          double ref = (double) releaseTimestamp.seconds() + (double) releaseTimestamp.microseconds();
          ref -= ((double) pressTimestamp.seconds() + (double) pressTimestamp.microseconds());
          std::cout << "Pause held for " << ref << "seconds." << std::endl;
        }
      }
    }
  }
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
    float const FREQ = 20;

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

    PwmMotors pwmMotors(names, types, channels, offsets, maxvals);

    auto onGroundSteeringRequest{[&pwmMotors, &angleConversion](cluon::data::Envelope &&envelope)
    {
      auto const gst = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(envelope));
      float const groundSteering = gst.groundSteering() / angleConversion;
      pwmMotors.setMotorPower(1, groundSteering);
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
      pwmMotors.setMotorPower(2, val);
    }};

    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);
    od4.dataTrigger(opendlv::proxy::PedalPositionRequest::ID(), onPedalPositionRequest);

    if (VERBOSE == 2) {
      initscr();
    }
    std::mutex mtx;
    bool isActive;
  
    auto atFrequency{[&pwmMotors, &VERBOSE, &mtx, &isActive]() -> bool
    {
      {
        std::lock_guard<std::mutex> lock(mtx);
        // This must be called regularly (>40hz) to keep servos or ESCs awake.
        if (isActive) {
          pwmMotors.actuate();
        }
        if (VERBOSE == 1) {
          std::cout << pwmMotors.toString() << std::endl;
        }
        if (VERBOSE == 2) {
          mvprintw(1,1,(pwmMotors.toString()).c_str()); 
          refresh();      /* Print it on to the real screen */
        }
      }
      return true;
    }};
    std::thread ledThread(LedController, &mtx, &isActive);
    std::thread buttonThread(ButtonListener, &mtx, &isActive, &pwmMotors);

    od4.timeTrigger(FREQ, atFrequency);

    if (VERBOSE == 2) {
      endwin();     /* End curses mode      */
    }
    ledThread.join();
    buttonThread.join();
  }
  return 0;
}
