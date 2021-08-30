#include <iostream>
#include <thread>
#include <chrono> 

#include "gripper.h"

int main(int argc, char **argv)
{
  Gripper* gripper = new Gripper();

  std::this_thread::sleep_for (std::chrono::seconds(2));

  GripperStatus s;
  GripperControl msgs;
  msgs.activate = true;
  msgs.go = true;
  msgs.mode = 0;
  msgs.autorelease = false;
  msgs.glove = false;
  msgs.individualfinger = false;
  msgs.individualscissor = false;
  for(int i = 0; i < 3; i++) {
    msgs.finger[i].position = 100;
    msgs.finger[i].speed = 0;
    msgs.finger[i].force = 0;
  }
  msgs.scissor.position = 128;
  msgs.scissor.speed = 0;
  msgs.scissor.force = 0;

  gripper->sendCommand(msgs);

  std::this_thread::sleep_for (std::chrono::seconds(10));

  // for (int i = 0; i < 20; i++) {
  //   std::cout << 
  // }

  s = gripper->readStatus();
  gripper->printStatus(s);

  delete gripper;
  return 0;
}
