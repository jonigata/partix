/*!
  @file     event.hpp
  @brief    <ŠT—v>

  <à–¾>
  $Id: event.hpp 16 2008-03-28 14:58:11Z Naoyuki.Hirayama $
*/
#ifndef EVENT_HPP
#define EVENT_HPP

#include "window_base.hpp"

namespace zw {

namespace win {

namespace event {

struct create {
        window_handle_type      handle;
        extent_type             extent;
};

struct destroy {
};

struct close {
        bool ok;
};

struct size {
        enum {
                sized,
                minimized,
                maximized,
        }               type;
        extent_type     extent;
};

struct move {
        offset_type     position;
};

struct erase_bg {
        dc_type         dc;
        bool            result_erased;
};

struct paint {
        dc_type         dc;
        bool            result_painted;
};

struct mouse {
        offset_type     position;
        bool            double_click;
        bool            ctrl;
        bool            shift;
        bool            alt;
        bool            lbutton;
        bool            rbutton;
        bool            mbutton;
        int             wheel_delta;
};

struct keychar {
        boost::uint32_t code;
        bool            ctrl;
        bool            shift;
        bool            alt;
};

struct keystate {
        bool            down;
        boost::uint32_t keycode;
        int             repeat;
        bool            prevdown;
        bool            enhanced;
        boost::uint32_t scancode;
        bool            contextcode;
        bool            transstate;
        bool            ctrl;
        bool            shift;
        bool            alt;

};

struct ime {
        enum {
                command_text,
                command_setopenstatus,
                command_closestatuswindow,
        }               command;
        std::string     text;
};

struct clipboard {
        bool            dummy;
};

struct filedrop {
        std::vector< std::string >     filenames;
};

} // namespace event

} // namespace win

} // namespace zw

#endif // EVENT_HPP
