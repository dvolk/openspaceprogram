// flightlog.h -- the per-vessel flight journal (header-only pure C++).
//
// One Vehicle owns one FlightLog. It records when the flight began and
// every SoI enter/leave, so the Flight Summary window (W_FlightSummary,
// opened on recover) can list the mission. Not save-persisted: a load
// starts a fresh journal at the load instant (v1).
//
// `observe` is the only writer, called with the vessel's current SoI body
// name ("" if none): the first call begins the journal, later calls emit
// left/entered when the body changes. The single writer of SoI events in
// the game is Vehicle::setSoi -- the one SoI re-home site -- so a journal
// begins when the vessel is placed (creation, load, split) and events are
// stamped where the switch actually happens, never polled. Repeat observes
// of an unchanged body are free no-ops, so the odd extra call (recover's
// belt-and-braces snapshot) is harmless. Timestamps are sim-clock seconds
// (Game::time).
//
// Pure containers + logic -- no game types -- so tests can pin the
// enter/leave pairing without linking Vehicle.

#pragma once

#include <string>
#include <string_view>
#include <vector>

struct FlightEvent {
    double t = 0.0;
    bool enter = true;   // false = left
    std::string body;
};

struct FlightLog {
    bool started = false;
    double start_t = 0.0;
    std::string start_body;   // SoI at begin ("" = none)
    std::string last_body;    // SoI after the most recent observe
    std::vector<FlightEvent> events;

    /* Record the ship's SoI body at sim time `t`. The first call starts
       the journal (and logs an "entered" for the initial body, if any);
       a body change logs "left <old>" then "entered <new>"; either side
       may be empty (a ship with no SoI body). Repeated observes of the
       same body are free. */
    void observe(double t, std::string_view body) {
        if(!started) {
            started = true;
            start_t = t;
            start_body = body;
            last_body = body;
            if(!body.empty()) {
                events.push_back(FlightEvent{start_t, true, std::string(body)});
            }
            return;
        }
        if(body == last_body) { return; }
        if(!last_body.empty()) {
            events.push_back(FlightEvent{t, false, last_body});
        }
        if(!body.empty()) {
            events.push_back(FlightEvent{t, true, std::string(body)});
        }
        last_body = body;
    }
};
