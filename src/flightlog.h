// flightlog.h -- the per-vessel flight journal (header-only pure C++).
// `observe` is the only writer (Vehicle::setSoi, the one SoI re-home site). On
// load it is restored BEFORE the placement's setSoi, so the mission history
// continues across save/load. Timestamps are sim-clock seconds (Game::time).

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

    // First call starts the journal; a body change logs left/entered.
    // Same-body observes are free.
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
