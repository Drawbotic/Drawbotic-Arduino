/*!
 * \file Drawbotic_StateMachine.h
 * \author Elliott Wilson (elliott.wilson@monash.edu)
 * \brief A simple state machine class
 * \version 1.0
 * \date 2023-03-23
 */
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

//! The maximum number of possible states (10)
#define MAX_STATES 10

//! The State Switch (on enter / on exit) callback method type
typedef void (*Drawbotic_SwitchMethod)();
//! The State Update callback method type, allows for the inclusion of a delta time parameter
typedef void (*Drawbotic_UpdateMethod)(int deltaTime);

/*!
 * \brief Class implementing the logic for registering, switching between and updating a finite set of states
 */
class Drawbotic_StateMachine 
{
private:
    int m_currentState;
    int m_lastState;
    Drawbotic_SwitchMethod m_onEnterMethods[MAX_STATES];
    Drawbotic_SwitchMethod m_onExitMethods[MAX_STATES];
    Drawbotic_UpdateMethod m_updateMethods[MAX_STATES];
    
public:
    Drawbotic_StateMachine();
    void addState(int state, Drawbotic_SwitchMethod on_enter, Drawbotic_SwitchMethod on_exit, Drawbotic_UpdateMethod update);
    void switchState(int state);
    void update(int deltaTime);
    /*!
     * \brief The id of the current state the state machine is in
     * \return The id of the current state the state machine is in
     */
    int getCurrentState() { return m_currentState; }
    /*!
     * \brief The id of the last state the state machine was in
     * \return The id of the last state the state machine was in
     */
    int getLastState() { return m_lastState; }
};

#endif