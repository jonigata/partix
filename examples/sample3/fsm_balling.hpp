#ifndef FSM_BALLING_HPP
#define FSM_BALLING_HPP

#include <cassert>

namespace fsm_balling {

enum state_type {
    state_initial,
    state_caption,
    state_throwing,
    state_rolling,
    state_result,
    state_before_caption,
};

template < class ActionHandler >
class StateMachine {
private:
    typedef void (StateMachine::*event_type)();

public:
    StateMachine( ActionHandler& ah ) : event_( 0 ), ah_( ah )
    {
        state_ = state_initial;
    }
    ~StateMachine(){}

    state_type state() { return state_; }

    bool step()
    {
        if( event_ ) {
            event_type e = event_; event_ = 0;
            (this->*e)();
            return true;
        } else {
            return false;
        }
    }

public:
    void Render()
    {
        assert( !event_ );
        event_ = &StateMachine::exec_Render;
    }

    void Restart()
    {
        assert( !event_ );
        event_ = &StateMachine::exec_Restart;
    }

    void Result()
    {
        assert( !event_ );
        event_ = &StateMachine::exec_Result;
    }

    void Rolling()
    {
        assert( !event_ );
        event_ = &StateMachine::exec_Rolling;
    }

    void Start()
    {
        assert( !event_ );
        event_ = &StateMachine::exec_Start;
    }

    void Throwing()
    {
        assert( !event_ );
        event_ = &StateMachine::exec_Throwing;
    }

    void Update()
    {
        assert( !event_ );
        event_ = &StateMachine::exec_Update;
    }

private:
    void exec_Render()
    {
        switch( state_ ) {
        case state_caption:
            state_ = state_caption;
            ah_.RenderCaption();
            break;
        case state_throwing:
            state_ = state_throwing;
            ah_.RenderThrowing();
            break;
        case state_result:
            state_ = state_result;
            ah_.RenderResult();
            break;
        default: ah_.unexpected_token( state_, "Render" );
        }
    }

    void exec_Restart()
    {
        switch( state_ ) {
        case state_result:
            state_ = state_before_caption;
            event_ = &StateMachine::autotrans_before_caption_to_caption;
            break;
        default: ah_.unexpected_token( state_, "Restart" );
        }
    }

    void exec_Result()
    {
        switch( state_ ) {
        case state_rolling:
            state_ = state_result;
            ah_.EntryResult();
            break;
        default: ah_.unexpected_token( state_, "Result" );
        }
    }

    void exec_Rolling()
    {
        switch( state_ ) {
        case state_throwing:
            state_ = state_rolling;
            ah_.EntryRolling();
            break;
        default: ah_.unexpected_token( state_, "Rolling" );
        }
    }

    void exec_Start()
    {
        switch( state_ ) {
        case state_initial:
            state_ = state_before_caption;
            event_ = &StateMachine::autotrans_before_caption_to_caption;
            ah_.Initialize();
            break;
        default: ah_.unexpected_token( state_, "Start" );
        }
    }

    void exec_Throwing()
    {
        switch( state_ ) {
        case state_caption:
            state_ = state_throwing;
            ah_.EntryThrowing();
            break;
        default: ah_.unexpected_token( state_, "Throwing" );
        }
    }

    void exec_Update()
    {
        switch( state_ ) {
        case state_caption:
            state_ = state_caption;
            ah_.UpdateCaption();
            break;
        case state_rolling:
            state_ = state_rolling;
            ah_.UpdateRolling();
            break;
        case state_throwing:
            state_ = state_throwing;
            ah_.UpdateThrowing();
            break;
        case state_result:
            state_ = state_result;
            ah_.UpdateResult();
            break;
        default: ah_.unexpected_token( state_, "Update" );
        }
    }

    void autotrans_before_caption_to_caption()
    {
        state_ = state_caption;
        ah_.EntryCaption();
    }

private:
    event_type event_;
    state_type state_;
    ActionHandler& ah_;

};

}

#endif
