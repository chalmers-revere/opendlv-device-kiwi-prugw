/**
 * Copyright (C) 2020 Björnborg Nguyen
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#include <algorithm>
#include <chrono>  //milliseconds
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>  //thread sleep

#include <fcntl.h>     // for open
#include <sys/mman.h>  // mmap
#include <unistd.h>    // for close

#include <sys/stat.h>  // checking if file/dir exist

#include "PwmMotors.h"

PwmMotors::PwmMotors(std::vector<std::string> a_names,
                     std::vector<std::string> a_types,
                     std::vector<std::string> a_channels,
                     std::vector<std::string> a_offets,
                     std::vector<std::string> a_maxvals)
    : m_motors(),
      m_mutex(),
      m_prusharedMemInt32_ptr{nullptr},
      m_SERVO_PRU_CH{1}  // PRU1
      ,
      m_SERVO_PRU_FW{"am335x-pru1-rc-servo-fw"}  //"am335x-pru1-rc-servo-fw"
      ,
      m_PRU0_STATE{"/dev/remoteproc/pruss-core0/state"},
      m_PRU1_STATE{"/dev/remoteproc/pruss-core1/state"},
      m_PRU0_FW{"/dev/remoteproc/pruss-core0/firmware"},
      m_PRU1_FW{"/dev/remoteproc/pruss-core1/firmware"},

      m_calFile(".env"),
      m_lastUpdate(cluon::time::now()),
      m_active{false},
      m_idle{true} {
  if (a_names.size() == a_channels.size() && a_names.size() == a_types.size() &&
      a_channels.size() <= NUM_SERVO_CHANNELS && a_channels.size() > 0 &&
      a_names.size() == a_offets.size() && a_names.size() == a_maxvals.size()) {
    for (uint8_t i = 0; i < a_names.size(); ++i) {
      std::string name = a_names.at(i);
      std::string type = a_types.at(i);
      uint8_t channel = std::stoi(a_channels.at(i));
      Motor::MotorType motorType;
      std::transform(type.begin(), type.end(), type.begin(), ::toupper);
      float offset = std::stof(a_offets.at(i));
      float maxval = std::stof(a_maxvals.at(i));
      if (type.compare("SERVO") == 0) {
        motorType = Motor::MotorType::Servo;
        m_motors.push_back(Motor(name, motorType, channel, offset, maxval));
      } else if (type.compare("ESC") == 0) {
        motorType = Motor::MotorType::Esc;
        m_motors.push_back(Motor(name, motorType, channel, offset, maxval));
      } else {
        std::cerr << " Incorrect configuration for motor type.\n";
        exit(1);
      }
    }
    terminatePru();
    m_prusharedMemInt32_ptr = nullptr;
    int32_t fileDescriptor = 0;

    // start mmaping shared memory
    fileDescriptor = open("/dev/mem", O_RDWR | O_SYNC);
    if (fileDescriptor == -1) {
      std::cerr << "ERROR: could not open /dev/mem." << std::endl;
    }

    volatile uint32_t *pru;  // Points to start of PRU memory.
    pru = static_cast<volatile uint32_t *>(
        mmap(0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fileDescriptor,
             PRU_ADDR));
    if (pru == MAP_FAILED) {
      std::cerr << " ERROR: could not map memory." << std::endl;
    }
    close(fileDescriptor);

    m_prusharedMemInt32_ptr =
        pru + PRU_SHAREDMEM / 4;  // Points to start of shared memory
    if (m_prusharedMemInt32_ptr == nullptr) {
      std::cerr << " ERROR: pru shared mem is nullptr." << std::endl;
    }
    // std::memset(m_prusharedMemInt32_ptr, 0, SERVO_CHANNELS * 4);
    for (uint8_t i = 0;
         i < NUM_SERVO_CHANNELS && m_prusharedMemInt32_ptr != nullptr; i++) {
      m_prusharedMemInt32_ptr[i] = 42;
    }
  } else {
    std::cerr << " Invalid number of configurations for pwm motors.\n";
  }
}

void PwmMotors::initialisePru() {
  std::lock_guard<std::mutex> l(m_mutex);
  struct stat sb;

  // check if firmware exists
  if (stat(("/lib/firmware/" + m_SERVO_PRU_FW).c_str(), &sb) != 0 ||
      !S_ISREG(sb.st_mode)) {
    std::cerr << " ERROR: missing am335x pru firmware" << std::endl;
    exit(1);
  }

  write2file(m_PRU1_STATE, "stop");
  write2file(m_PRU1_FW, m_SERVO_PRU_FW);
  write2file(m_PRU1_STATE, "start");
  // std::this_thread::sleep_for(std::chrono::milliseconds(250));

  std::ifstream fileStatus(m_PRU1_STATE);
  std::string strStatus;
  std::getline(fileStatus, strStatus);
  if (!strStatus.compare("running\n")) {
    std::cerr << " ERROR code: " << strStatus << std::endl;
    exit(1);
  }
  for (uint8_t i = 0;
       i < NUM_SERVO_CHANNELS && m_prusharedMemInt32_ptr != nullptr; i++) {
    m_prusharedMemInt32_ptr[i] = 42;
  }

  while (m_prusharedMemInt32_ptr[0] != 0) {
    std::clog << "The shared memory with PRU didn't get zeroed out..."
              << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::clog << "Successfully initialized servo/esc PRU" << std::endl;
  m_active = true;
  // powerServoRail(true);
}

void PwmMotors::terminatePru() {
  for (auto motor : m_motors) {
    // setMotorPower(motor.getChannel(), 0.0f);
    if (motor.getType() == Motor::MotorType::Servo) {
      setMotorPower(motor.getChannel(), 0.0f);
    } else {
      setMotorPower(motor.getChannel(), 0.5f);
    }
  }
  powerServoRail(false);
  std::lock_guard<std::mutex> l(m_mutex);
  write2file(m_PRU1_STATE, "stop");
  for (uint8_t i = 0;
       i < NUM_SERVO_CHANNELS && m_prusharedMemInt32_ptr != nullptr; i++) {
    m_prusharedMemInt32_ptr[i] = 0;
  }
  m_active = false;
  m_idle = true;
}

PwmMotors::~PwmMotors() {
  saveCalibration();
  terminatePru();
  m_prusharedMemInt32_ptr = nullptr;
}

void PwmMotors::setMotorPower(uint8_t const &a_ch, float const &a_power) {
  for (uint8_t i = 0; i < m_motors.size(); ++i) {
    if (m_motors.at(i).getChannel() == a_ch) {
      std::lock_guard<std::mutex> l(m_mutex);
      m_motors.at(i).setPower(a_power);
      break;
    }
  }
}

void PwmMotors::setMotorOffset(uint8_t const &a_ch, float const &a_offset) {
  for (uint8_t i = 0; i < m_motors.size(); ++i) {
    if (m_motors.at(i).getChannel() == a_ch) {
      std::lock_guard<std::mutex> l(m_mutex);
      m_motors.at(i).setOffset(a_offset);
      break;
    }
  }
  saveCalibration();
}

std::string PwmMotors::toString() {
  std::stringstream ss;
  for (uint8_t i = 0; i < m_motors.size(); ++i) {
    ss << "\t" << m_motors[i].toString() << std::endl;
  }
  return ss.str();
}

void PwmMotors::actuate() {
  std::lock_guard<std::mutex> l(m_mutex);
  for (uint8_t i = 0; i < m_motors.size(); ++i) {
    switch (m_motors.at(i).getType()) {
      case Motor::MotorType::Esc:
        setEscNormalized(m_motors.at(i).getChannel(),
                         m_motors.at(i).getPower());
        break;
      case Motor::MotorType::Servo:
        setServoNormalized(m_motors.at(i).getChannel(),
                           m_motors.at(i).getPower());
        break;
      default:
        break;
    }
  }
}

void PwmMotors::powerServoRail(bool const &a_val) {
  struct stat sb;
  if (a_val) {
    if (stat("/sys/class/gpio/gpio80", &sb) != 0) {
      write2file("/sys/class/gpio/export", "80");
    }
    write2file("/sys/class/gpio/gpio80/direction", "out");
    write2file("/sys/class/gpio/gpio80/value", "1");
  } else {
    if (stat("/sys/class/gpio/gpio80", &sb) == 0) {
      write2file("/sys/class/gpio/gpio80/value", "0");
      write2file("/sys/class/gpio/gpio80/direction", "in");
      write2file("/sys/class/gpio/unexport", "80");
    }
  }
}

void PwmMotors::write2file(std::string const &a_path,
                           std::string const &a_str) {
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

void PwmMotors::saveCalibration() {
  if (m_motors.size() < 1) {
    return;
  }

  std::stringstream ss;
  std::string line;

  std::ifstream filein(m_calFile);  // File to read from

  if (filein.is_open()) {
    while (getline(filein, line)) {
      if (!(line.find("OFFSETS") == std::string::npos)) {
        ss << "OFFSETS=";
        ss << std::to_string(m_motors.at(0).getOffset());
        for (uint8_t i = 1; i < m_motors.size(); i++) {
          ss << ",";
          ss << std::to_string(m_motors.at(i).getOffset());
        }
        ss << "\n";
      } else {
        ss << line;
        ss << "\n";
      }
    }
  }
  filein.close();
  std::ofstream fileout(m_calFile, std::ofstream::trunc);

  if (fileout.is_open()) {
    std::string buffer = ss.str();
    buffer.erase(
        std::unique(buffer.begin(), buffer.end(),
                    [](char a, char b) { return a == '\n' && b == '\n'; }),
        buffer.end());
    fileout << buffer;
  }
  fileout.close();
}

/*******************************************************************************
 * Sends a single pulse of duration us (microseconds) to a channels.
 * This must be called regularly (>40hz) to keep servos or ESCs awake.
 *******************************************************************************/
int8_t PwmMotors::setPwmMicroSeconds(uint8_t const &a_ch,
                                     uint32_t const &a_us) {
  // Sanity Checks
  if (a_ch < 1 || a_ch > NUM_SERVO_CHANNELS) {
    std::clog << " ERROR: Channel must be between 1 and " << NUM_SERVO_CHANNELS
              << ". \n";
    return -2;
  }
  if (m_prusharedMemInt32_ptr == nullptr) {
    std::clog << " ERROR: PRU servo Controller not initialized.\n";
    return -2;
  }

  // first check to make sure no pulse is currently being sent
  if (m_prusharedMemInt32_ptr[a_ch - 1] != 0) {
    std::clog << " WARNING: Tried to start a new pulse amidst another.\n";
    return -1;
  }

  // PRU runs at 200Mhz. find #loops needed
  uint32_t numLoops = (a_us * 200) / PRU_SERVO_LOOP_INSTRUCTIONS;
  // write to PRU shared memory
  m_prusharedMemInt32_ptr[a_ch - 1] = numLoops;
  return 0;
}

/*******************************************************************************
 * Sends a single pulse of duration us (microseconds) to all channels.
 * This must be called regularly (>40hz) to keep servos or ESCs awake.
 *******************************************************************************/
int8_t PwmMotors::setPwmMicroSecondsAll(uint32_t const &a_us) {
  int ret = 0;
  for (uint8_t i = 1; i <= NUM_SERVO_CHANNELS; i++) {
    int8_t retCh = setPwmMicroSeconds(i, a_us);
    if (retCh == -2) {
      return -2;
    } else if (retCh == -1) {
      ret = -1;
    }
  }
  return ret;
}

int8_t PwmMotors::setServoNormalized(uint8_t const &a_ch,
                                     float const &a_input) {
  if (a_ch < 1 || a_ch > NUM_SERVO_CHANNELS) {
    std::clog << " ERROR: Channel must be between 1 and " << NUM_SERVO_CHANNELS
              << ". \n";
    return -1;
  }
  if (a_input < -1.5f || a_input > 1.5f) {
    std::clog
        << " ERROR: Servo normalized input must be between -1.5 and 1.5\n";
    return -1;
  }
  uint32_t us = static_cast<uint32_t>(SERVO_MID_US +
                                      (a_input * (SERVO_NORMAL_RANGE / 2)));
  return setPwmMicroSeconds(a_ch, us);
}

int8_t PwmMotors::setServoNormalizedAll(float const &a_input) {
  int ret = 0;
  for (uint8_t i = 1; i <= NUM_SERVO_CHANNELS; i++) {
    int8_t retCh = setServoNormalized(i, a_input);
    if (retCh == -2) {
      return -2;
    } else if (retCh == -1) {
      ret = -1;
    }
  }
  return ret;
}

/*******************************************************************************
 * normalized input of 0-1 corresponds to output pulse from 1000-2000 us
 * input is allowed to go down to -0.1 so ESC can be armed below minimum
 *throttle
 *******************************************************************************/
int8_t PwmMotors::setEscNormalized(uint8_t const &a_ch, float const &a_input) {
  if (a_ch < 1 || a_ch > NUM_SERVO_CHANNELS) {
    std::clog << " ERROR: Channel must be between 1 and " << NUM_SERVO_CHANNELS
              << ". \n";
    return -1;
  }
  if (a_input < -0.1f || a_input > 1.0f) {
    std::clog << " ERROR: ESC normalized input must be between -0.1 & 1\n";
    return -1;
  }
  uint32_t micros = static_cast<uint32_t>(1000 + (a_input * 1000.0f));
  return setPwmMicroSeconds(a_ch, micros);
}

int8_t PwmMotors::setEscNormalizedAll(float const &a_input) {
  int ret = 0;
  for (uint8_t i = 1; i <= NUM_SERVO_CHANNELS; i++) {
    int8_t retCh = setEscNormalized(i, a_input);
    if (retCh == -2) {
      return -2;
    } else if (retCh == -1) {
      ret = -1;
    }
  }
  return ret;
}

/*******************************************************************************
 * normalized input of 0-1 corresponds to output pulse from 125-250 us
 * input is allowed to go down to -0.1 so ESC can be armed below minimum
 *throttle
 *******************************************************************************/
int8_t PwmMotors::setEscOneshotNormalized(uint8_t const &a_ch,
                                          float const &a_input) {
  if (a_ch < 1 || a_ch > NUM_SERVO_CHANNELS) {
    std::clog << " ERROR: Channel must be between 1 and " << NUM_SERVO_CHANNELS
              << ". \n";
    return -1;
  }
  if (a_input < -0.1f || a_input > 1.0f) {
    std::clog << " ERROR: ESC normalized input must be between -0.1 & 1\n";
    return -1;
  }
  uint32_t micros = static_cast<uint32_t>(125.0f + (a_input * 125.0f));
  return setPwmMicroSeconds(a_ch, micros);
}

bool PwmMotors::isIdle() noexcept {
  // std::clog << " Checking idle\n";
  // cluon::data::TimeStamp now = cluon::time::now();
  // micro = 10^-6
  if (m_idle) {
    return true;
  } else {
    int64_t const timeThresholdinMicroseconds = 500000;
    {
      std::lock_guard<std::mutex> l(m_mutex);
      int64_t timeDiffInMicroseconds =
          cluon::time::deltaInMicroseconds(cluon::time::now(), m_lastUpdate);
      m_idle = timeDiffInMicroseconds > timeThresholdinMicroseconds;
      // std::clog << " Time diff: " << timeDiffInMicroseconds << "  Idle?"
      //        << m_idle;
    }
    if (m_idle) {
      for (auto motor : m_motors) {
        // setMotorPower(motor.getChannel(), 0.0f);
        if (motor.getType() == Motor::MotorType::Servo) {
          setMotorPower(motor.getChannel(), 0.0f);
        } else {
          setMotorPower(motor.getChannel(), 0.5f);
        }
      }
      powerServoRail(false);
      std::clog << " Started to idle\n";
    }
    return m_idle;
  }
}

void PwmMotors::updateTimestamp() {
  std::lock_guard<std::mutex> l(m_mutex);
  m_lastUpdate = cluon::time::now();
  if (m_idle && m_active) {
    std::clog << " Stopped being idle\n";
    powerServoRail(true);
    m_idle = false;
  }
}
