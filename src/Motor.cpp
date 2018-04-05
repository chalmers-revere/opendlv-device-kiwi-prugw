/**
 * Copyright (C) 2018 Chalmers Revere
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "Motor.h"


Motor::Motor(std::string const a_name, MotorType const a_type, 
    uint8_t const a_ch, float const a_offset, float const a_maxval)
    : m_name(a_name)
    , m_type(a_type)
    , m_channel(a_ch)
    , m_power()
    , m_offset(a_offset)
    , m_maxval(a_maxval)
{
  switch (m_type) {
    case MotorType::Esc :
      m_power = 0.5f;
      break;
    case MotorType::Servo :
      m_power = 0.0f;
      break;
    default:
      break;
  }
}
Motor::~Motor()
{}

std::string Motor::getName()
{
  return m_name;
}

Motor::MotorType Motor::getType()
{
  return m_type;
}

uint8_t Motor::getChannel()
{
  return m_channel;
}

float Motor::getPower()
{
  if (m_power > 1.5f) {
    return 1.4999f;
  } else if (m_power < -1.5f) {
    return -1.4999f;
  }
  return m_power;
}

void Motor::setPower(float const a_val)
{
  if(m_type ==  MotorType::Esc) {
    float val = a_val;
    if (val < (- m_maxval * 2.0f) + m_offset) {
      val = (- m_maxval * 2.0f) + m_offset;
    } else if (val > m_maxval + m_offset) {
      val = m_maxval + m_offset;
    }
    m_power = val;
  } else {
    float val{a_val + m_offset};
    if (val < -m_maxval + m_offset) {
      val = -m_maxval + m_offset;
    } else if (val > m_maxval + m_offset) {
      val = m_maxval + m_offset;
    }
    m_power = val;
  }
}

std::string Motor::toString() 
{
  std::string type;
  switch (m_type) {
    case MotorType::Esc :
      type = "esc";
      break;
    case MotorType::Servo :
      type = "servo";
      break;
    default:
      break;
  }
  return " Name: " + m_name + "   \tType: " + type + "\tChannel: " + std::to_string(m_channel) + "\tPower: " + std::to_string(getPower()) + ".";
}

