#pragma once

#include <string>
#include <vector>

/* Fleet data model: the JSON-backed description of which ships exist in the
   game. GL-free (no rendering, no Bullet), so the parse path can be
   unit-tested headless (tests/test_fleet). Semantic resolution -- that a
   body exists in the loaded system, that a scenario is known -- happens in
   main.cpp, which owns the System and the scenario table.

   fleet file (see res/fleet.json):
     {
       "ships": [
         { "ship": "res/ships/basic.json",   // ship def; default: res/ships/basic.json
           "name": "Alpha",                 // optional; default: the def's name
           "body": "Eerbon",               // optional; default: --body / the system home
           "scenario": "rot-orbit" },      // optional; default: --scenario / pad
         ...
       ]
     }

   Entries sharing a (body, scenario) pair are slotted automatically by
   main.cpp: pad ships 20 m apart along the pad, orbit ships 100 m apart
   along the orbit binormal.
*/

struct FleetEntry {
    std::string ship;      // ship def path (load_fleet fills the default)
    std::string name;      // empty = use the ship def's name
    std::string body;      // empty = fall back to --body / the system home
    std::string scenario;  // empty = fall back to --scenario / "pad"
};

struct Fleet {
    std::vector<FleetEntry> ships;
};

/* Parse + validate, in the load_system() style: throws std::runtime_error
   naming the file and the offending entry on any bad/missing data. */
Fleet load_fleet(const char *path);
