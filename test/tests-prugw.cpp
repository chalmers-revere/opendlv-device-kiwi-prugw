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

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "PwmMotors.h"

#include <string>
#include <vector>
#include <cmath>

TEST_CASE("Test servo motor") {
  std::string name = "servo";
  Motor::MotorType type = Motor::MotorType::Servo;
  uint8_t channel = 0;
  float offset = -0.12f;
  float maxval = 1.5f;
  Motor servo(name, type, channel, offset, maxval);
  REQUIRE(Motor::MotorType::Servo == servo.getType());
  REQUIRE(channel == servo.getChannel());
  REQUIRE(servo.getPower() == Approx(-0.12f));
  servo.setPower(2.0f);
  REQUIRE(servo.getPower()== Approx(1.5f + offset));
  servo.setPower(-2.0f);
  REQUIRE(servo.getPower() == Approx(-1.5f));
  servo.setPower(-1.0f);
  REQUIRE(servo.getPower() == Approx(-1.0f + offset));
  servo.setPower(1.0f);
  REQUIRE(servo.getPower() == Approx(1.0f + offset));
  
}


TEST_CASE("Test esc motor") {
  std::string name = "esc";
  Motor::MotorType type = Motor::MotorType::Esc;
  uint8_t channel = 0;
  float offset = 0.5;
  float maxval = 0.5;
  Motor esc(name, type, channel, offset, maxval);
  REQUIRE(Motor::MotorType::Esc == esc.getType());
  REQUIRE(channel == esc.getChannel());
  REQUIRE(esc.getPower() == Approx(0.5f)) ;
  esc.setPower(2.0f);
  REQUIRE(esc.getPower() == Approx(1.0f));
  esc.setPower(-2.0f);
  REQUIRE(esc.getPower() == Approx(0.0f));
  esc.setPower(0.2f);
  REQUIRE(esc.getPower() == Approx(0.2f));
  esc.setPower(0.8f);
  REQUIRE(esc.getPower() == Approx(0.8f));

}

