#include "Drawbotic_StateMachine.h"

/*!
 * \brief Construct a new Drawbotic_StateMachine object
 */
Drawbotic_StateMachine::Drawbotic_StateMachine()
{
    m_currentState = -1;
}

/*!
 * \brief Register a new state with the state machine. Must be done before switching to that state or updating the state machine
 * 
 * \param state An unique number representing that state, all added states should be sequentially identified (i.e the first state should be 0, the next 1 and so forth). Typically the different states are represented by an enum 
 * \param on_enter A pointer to the fuction that is run when this state is entered
 * \param on_exit A pointer to the function that is run when this state is exited
 * \param update A pointer to the function that is run when this state is updated
 */
void Drawbotic_StateMachine::AddState(int state, Drawbotic_SwitchMethod on_enter, Drawbotic_SwitchMethod on_exit, Drawbotic_UpdateMethod update)
{
    m_onEnterMethods[state] = on_enter;
    m_onExitMethods[state] = on_exit;
    m_updateMethods[state] = update;
}

/*!
 * \brief Switches the State Machine from the current state to a desired state, the on exit function of the current state will be called, followed by the on enter function of the desired state
 * 
 * \param state The unique identifier of the desired state
 */
void Drawbotic_StateMachine::SwitchState(int state)
{
    m_lastState = m_currentState;
    
    if(m_onExitMethods[m_currentState] != 0)
        m_onExitMethods[m_currentState]();
    
    m_currentState = state;
    
    if(m_onEnterMethods[m_currentState] != 0)
        m_onEnterMethods[m_currentState]();
}

/*!
 * \brief Updates the state machine, typically this is called repeatedly in loop(). The update method of the current state is called
 * 
 * \param deltaTime The elapsed time since the last time the state machine was updated. This parameter is passed on to the current state's update function
 */
void Drawbotic_StateMachine::Update(int deltaTime)
{
    if(m_updateMethods[m_currentState] != 0)
        m_updateMethods[m_currentState](deltaTime);
}