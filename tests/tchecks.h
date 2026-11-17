#pragma once
/* One dialect of check for the unit tests (#954).

   Each test file used to carry its own CHECK / CHECK_TRUE / CHECK_NEAR: 31 files
   defined CHECK_NEAR, in shapes that looked identical at the call site and did not
   mean the same thing. The third argument is a metre in test_orbit.cpp and a ratio
   of the expected value in test_fuel.cpp; the message is required in one file and
   does not exist in the next. Copy a check sideways and it either stops compiling
   at the macro -- which at least says where -- or compiles and quietly means
   something else.

   Here: CHECK_NEAR is ABSOLUTE (|a-e| <= tol) and CHECK_REL is RELATIVE
   (|a-e| <= rel*|e| + 1e-12), which is the whole vocabulary. The message is
   optional on every one of them; without it the failure prints the expressions as
   written, which is what the message-less CHECK family did by hand. CHECK and
   CHECK_TRUE are one thing under two names, because half the suite used each.

   Every numeric check asks std::isfinite on both sides first (#604): a bare
   "diff > tol" comparison cannot see a NaN, so the files that lacked the guard
   passed on one. */

#include <cmath>
#include <cstdio>

inline int g_checks = 0;
inline int g_failures = 0;

inline void tcTrueReport(const char *file, int line, const char *expr, bool cond,
                         const char *msg) {
    g_checks++;
    if(cond) { return; }
    g_failures++;
    if(msg != nullptr) {
        printf("FAIL: %s:%d: %s [%s]\n", file, line, msg, expr);
    } else {
        printf("FAIL: %s:%d: %s\n", file, line, expr);
    }
}

inline void tcNearReport(const char *file, int line, const char *ea, const char *ee,
                         double a, double e, double tol, bool rel, const char *msg) {
    g_checks++;
    const double limit = rel ? tol * std::fabs(e) + 1e-12 : tol;
    if(std::isfinite(a) && std::isfinite(e) && std::fabs(a - e) <= limit) { return; }
    g_failures++;
    if(msg != nullptr) {
        printf("FAIL: %s:%d: %s -- got %.9g, want %.9g, %s %.3g\n",
               file, line, msg, a, e, rel ? "rel_tol" : "tol", tol);
    } else {
        printf("FAIL: %s:%d: %s ~ %s -- got %.9g, want %.9g, %s %.3g\n",
               file, line, ea, ee, a, e, rel ? "rel_tol" : "tol", tol);
    }
}

/* The message is the optional last argument. The picker is given the call's
   arguments plus the two wrapper names (with the arity-matching one first) and
   answers the name to use; each wrapper stringifies what it was handed, which a
   function cannot do for itself. */
#define TC_PICK4(_1, _2, _3, _4, NAME, ...) NAME
#define TC_PICK2(_1, _2, NAME, ...) NAME

#define tcNearExpr(a_, e_, tol_) \
    tcNearReport(__FILE__, __LINE__, #a_, #e_, (a_), (e_), (tol_), false, nullptr)
#define tcNearMsg(a_, e_, tol_, m_) \
    tcNearReport(__FILE__, __LINE__, #a_, #e_, (a_), (e_), (tol_), false, (m_))
#define tcRelExpr(a_, e_, rel_) \
    tcNearReport(__FILE__, __LINE__, #a_, #e_, (a_), (e_), (rel_), true, nullptr)
#define tcRelMsg(a_, e_, rel_, m_) \
    tcNearReport(__FILE__, __LINE__, #a_, #e_, (a_), (e_), (rel_), true, (m_))
#define tcTrueExpr(c_) tcTrueReport(__FILE__, __LINE__, #c_, (c_), nullptr)
#define tcTrueMsg(c_, m_) tcTrueReport(__FILE__, __LINE__, #c_, (c_), (m_))

#define CHECK_NEAR(...) TC_PICK4(__VA_ARGS__, tcNearMsg, tcNearExpr)(__VA_ARGS__)
#define CHECK_REL(...) TC_PICK4(__VA_ARGS__, tcRelMsg, tcRelExpr)(__VA_ARGS__)
#define CHECK_TRUE(...) TC_PICK2(__VA_ARGS__, tcTrueMsg, tcTrueExpr)(__VA_ARGS__)
#define CHECK(...) CHECK_TRUE(__VA_ARGS__)
