// question.h -- the question card's pure rules, free of Game.
//
// A card is answered on a LATER frame, so onAnswer must capture IDENTITY (a
// name, an id), never a pointer -- the subject can be gone by then. Two rules
// make that safe, and both used to be reachable only through a test hook that
// acted while a card was modal (every key is swallowed, so no click can):
//
//   drop    -- a world teardown drops the card (issue #304). The card cannot
//              outlive the world its subject lived in (case 258).
//   answer  -- the card closes BEFORE onAnswer runs, so the callback may open
//              the next one. A callback that re-finds its subject by name then
//              finds nobody rather than dereferencing a freed pointer
//              (case 265).
#pragma once

#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

struct Question {
    std::string text;
    std::vector<std::string> labels;   // one button each, in order
    std::function<void(int choice)> onAnswer;
};

/* Collapse newlines so one card is one log line (the e2e EXPECT is a
   substring of the whole output, not a regex over lines). */
inline std::string questionOneLine(const std::string &s) {
    std::string out;
    out.reserve(s.size());
    for(char c : s) {
        if(c == '\n' || c == '\r') {
            if(!out.empty() && out.back() != ' ') { out.push_back(' '); }
        } else {
            out.push_back(c);
        }
    }
    while(!out.empty() && out.back() == ' ') { out.pop_back(); }
    return out;
}

/* Drop a pending card, if any. Every teardown runs this: a card cannot outlive
   the world its subject lived in. */
inline void dropPendingQuestion(std::unique_ptr<Question> &q) {
    if(q == nullptr) { return; }
    printf("[game] reset: dropped pending question: %s\n",
           questionOneLine(q->text).c_str());
    fflush(stdout);
    q.reset();
}

/* Close the card first, then fire the callback: an answer may open the next
   question. No-op with none up. */
inline void answerPendingQuestion(std::unique_ptr<Question> &q, int choice) {
    if(!q) { return; }
    printf("[ask] answered %d\n", choice);
    fflush(stdout);
    auto fn = std::move(q->onAnswer);
    q.reset();
    if(fn) { fn(choice); }
}
