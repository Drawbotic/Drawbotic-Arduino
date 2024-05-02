#ifndef DRAWBOTIC_NAVIGATION_H
#define DRAWBOTIC_NAVIGATION_H

#include <Wire.h>

#include "db1/inc/drawbotic_db1_drv.h"
#include "db1/inc/drawbotic_nav.h"

/*!
 * \brief The Drawbotic_Navigation class contains all of the functionality needed to create a queue of navigation actions that can be performed sequentially.
 * 
 * \note Multiple instances of Drawbotic_Navigation can be created to implement multiple queues to be run at different times.
 */
class Drawbotic_Navigation {
public:
  Drawbotic_Navigation(float updateRate_ms = 1.0f, float speed = 0.1f, float correctionPower = 0.015f);
  void clearAllActions();
  void addForwardAction(float distance_mm, bool front=false);
  void addTurnAction(float radius_mm, float angle_deg, bool front=false);
  void addRotateAction(float angle_deg, bool front=false);
  void addStopAction(float time_ms, bool front=false);
  void addPenAction(bool down, bool front=false);
  /*!
   * \brief The current size of the navigation queue
   * \return The current size of the navigation queue 
   */
  int  getQueueSize() { return m_queueSize; }
  void update(float deltaTime_ms);
  void setSpeed(float speed);
  void setCorrectionPower(float cp);
  void printQueue();

private:

  //Internal private struct to represent a nav action
  struct NavigationAction {
    db1_nav_action_t action;
    NavigationAction* prev;
    NavigationAction* next;
  };

  void addActionFront(NavigationAction *action);
  void addActionBack(NavigationAction *action);

  NavigationAction* m_queueHead;
  NavigationAction* m_queueTail;
  int   m_queueSize;
  float m_timeBank;
  float m_updateRate_ms;
};

#endif
