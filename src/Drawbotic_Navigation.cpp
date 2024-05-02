#include "Drawbotic_Navigation.h"

#include <stddef.h>

/*!
 * \brief Construct a new Drawbotic_Navigation object with it's own Navigation queue
 * 
 * \param updateRate_ms (Default: 1.0 ms) The rate in milliseconds that navigation changes should be calculated. The update method will check to see if at least this much has passed since last update before changing the navigation output
 * \param speed (Default: 0.1) The maximum speed at which the motors can run, slower speeds with increase accuracy
 * \param correctionPower (Default: 0.015) The stength of the correction factor uses to keep the two motors in sync
 */
Drawbotic_Navigation::Drawbotic_Navigation(float updateRate_ms, float speed, float correctionPower) {
  m_timeBank = 0;
  m_updateRate_ms = updateRate_ms;
  m_queueSize = 0;
  db1_nav_set_cp(correctionPower);
  db1_nav_set_speed(speed);
}

/*!
 * \brief Clears all actions from the navigation queue
 */
void Drawbotic_Navigation::clearAllActions() {
  // Iterate through the queue until there is nothing next
  while (m_queueHead != NULL)
  {
    NavigationAction *a = m_queueHead;
    m_queueHead = a->next;
    delete a; // Delete each action object
  }
  // Reset the queue state
  m_queueSize = 0;
  m_queueHead = NULL;
  m_queueTail = NULL;
}

/*!
 * \brief Add a new forward action to the navigation queue. The action will cause DB-1 to travel forward a set distance
 * 
 * \param distance_mm The distance in millimetres to travel
 * \param front (Default: false) If true this action will be inserted at the front of the queue, before all existing queued actions
 */
void Drawbotic_Navigation::addForwardAction(float distance_mm, bool front) {
  NavigationAction* a = new NavigationAction();
  db1_nav_build_forward_action(distance_mm, &a->action);

  if(front) {
    addActionFront(a);
  }
  else {
    addActionBack(a);
  }
}

/*!
 * \brief Add a new turn action to the navigation queue. The action will cause DB-1 to turn following an arc of a certain radius and angle
 * 
 * \param radius_mm The radius of the circle that defines the arc for DB-1 to follow
 * \param angle_deg The number of degrees around that circle that DB-1 should go. A positive angle moves DB-1 in a Counter-Clockwise direction, a negative angle moves DB-1 in a Clockwise direction.
 * \param front (Default: false) If true this action will be inserted at the front of the queue, before all existing queued actions
 */
void Drawbotic_Navigation::addTurnAction(float radius_mm, float angle_deg, bool front) {
  NavigationAction* a = new NavigationAction();
  db1_nav_build_turn_action(radius_mm, angle_deg, &a->action);

  if(front) {
    addActionFront(a);
  }
  else {
    addActionBack(a);
  }
}

/*!
 * \brief Add a new rotate action to the navigation queue. The actoin will cause DB-1 to rotate a specific amount on the spot.
 * 
 * \param angle_deg The number of degrees to rotate. A positive angle moves DB-1 in a Counter-Clockwise direction, a negative angle moves DB-1 in a Clockwise direction.
 * \param front (Default: false) If true this action will be inserted at the front of the queue, before all existing queued actions
 */
void Drawbotic_Navigation::addRotateAction(float angle_deg, bool front) {
  NavigationAction* a = new NavigationAction();
  db1_nav_build_rotate_action(angle_deg, &a->action);

  if(front) {
    addActionFront(a);
  }
  else {
    addActionBack(a);
  }
}

/*!
 * \brief Add a new stop action to the navigation queue. The action will cause DB-1 to stop in place for a set amount of time.
 * 
 * \param time_ms The time (in milliseconds) to stop for
 * \param front (Default: false) If true this action will be inserted at the front of the queue, before all existing queued actions
 */
void Drawbotic_Navigation::addStopAction(float time_ms, bool front) {
  NavigationAction* a = new NavigationAction();
  db1_nav_build_stop_action(time_ms, &a->action);

  if(front) {
    addActionFront(a);
  }
  else {
    addActionBack(a);
  }
}

/*!
 * \brief Add a new pen action to the navigation queue. The action will cause DB-1 to raise or lower the pen
 * 
 * \param down If true the action will cause DB-1 to lower the pen when processed. If false the action will raise the pen
 * \param front (Default: false) If true this action will be inserted at the front of the queue, before all existing queued actions
 */
void Drawbotic_Navigation::addPenAction(bool down, bool front) {
  NavigationAction* a = new NavigationAction();
  db1_nav_build_pen_action(down, &a->action);

  if(front) {
    addActionFront(a);
  }
  else {
    addActionBack(a);
  }
}

/*!
 * \brief Update the navigation system. This method will recalculate the motor speeds based on the current action to perform. If an action is complete calling update will cause the queue to move on to the next action. Calling update while the queue is empty will have no effect.
 * 
 * \param deltaTime_ms The number of milliseconds that have past since update was last called. This is required to ensure that timing based actions are performed correctly. It is also used to determine if the next step of the navigation logic should be performed. See the updateRate_ms paramter in the Constructor
 * 
 * \note Update must be called peroidically within the main execution loop. The delta time parameter must be correctly provided by the calling code.
 */
void Drawbotic_Navigation::update(float deltaTime_ms) {
  m_timeBank += deltaTime_ms;

  // if enough time has passed for another update
  if (m_timeBank >= m_updateRate_ms) {
    // assign the action at the head of the queue as the current action
    NavigationAction *currentAction = m_queueHead;
    if (currentAction != NULL) {
      // perform a step of the current action and check if the action has finished
      bool finished = db1_nav_perform_action(&currentAction->action, false);

      if (finished) {
        // Move to the next item in the queue
        m_queueHead = currentAction->next;
        delete currentAction;
        m_queueSize--;

        // if the queue is empty, stop the robot
        if (m_queueSize == 0) {
          db1_set_motor_speed(DB1_M1, 0);
          db1_set_motor_speed(DB1_M2, 0);
          m_queueHead = NULL;
          m_queueTail = NULL;
        }
      }
    }
    m_timeBank = 0; // reset the time bank for the next update
  }
}

/*!
 * \brief Sets the maximum speed that DB-1 will use during navigation movements
 * 
 * \param speed A vaule between 0.0-1.0 where 0.0 is no power and 1.0 is full power
 */
void Drawbotic_Navigation::setSpeed(float speed) {
    db1_nav_set_speed(speed);
}

/*!
 * \brief Sets the strength of the error correction that DB-1 will use during navigation movements
 * 
 * \param cp The desired correction power, typical values are very small, e.g. 0.015
 */
void Drawbotic_Navigation::setCorrectionPower(float cp) {
  db1_nav_set_cp(cp);
}

void Drawbotic_Navigation::printQueue() {
  NavigationAction *a = m_queueHead;

  printf("Current Navigation Queue:\n");
  while(a != NULL) {
    switch(a->action.type) {
    case NAV_FORWARD:
      printf("FORWARD: %fmm\n", a->action.desired_distance);
      break;
    case NAV_TURN:
      printf("TURN: %fdeg, %fmm\n", a->action.desired_angle, a->action.desired_radius);
      break;
    case NAV_ROTATE:
      printf("ROTATE: %fdeg\n", a->action.desired_angle);
      break;
    case NAV_STOP:
      printf("STOP: %fms\n", a->action.desired_time);
      break;
    case NAV_PEN_UP:
      printf("PEN UP\n");
      break;
    case NAV_PEN_DOWN:
      printf("PEN DOWN\n");
      break;
    }
    a = a->next;
  }

}

void Drawbotic_Navigation::addActionFront(NavigationAction *action) {
  // The previous node of the current head is the new node
  m_queueHead->prev = action;

  // The previous node of the new node is nothing (it's at the head of the queue)
  action->prev = NULL;
  // The next node of the new node is the current head node
  action->next = m_queueHead;
  // Update the head node to point to the new node
  m_queueHead = action;

  // If the queue is empty the new node is also the current tail node
  if (m_queueSize == 0)
    m_queueTail = action;

  // Update the size of the queue
  m_queueSize++;
}

void Drawbotic_Navigation::addActionBack(NavigationAction *action) {
  // The next node of the current tail is the new node
  m_queueTail->next = action;

  // The previous node of the new node is the current tail
  action->prev = m_queueTail;
  // The next node of the new node is nothing (it's at the end of the queue)
  action->next = NULL;
  // Update the tail node to point to the new node
  m_queueTail = action;

  // If the queue is empty the new node is also the current head node
  if (m_queueSize == 0)
    m_queueHead = action;

  // Update the size of the queue
  m_queueSize++;
}