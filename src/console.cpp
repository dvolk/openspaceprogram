#include "console.h"

#include <stdio.h>

/* The one TU that includes windows.h. It leaks TEXT/ERROR/IN/OUT/OPTIONAL/
   TRUE/far/near/INFINITE into whatever else is in the file, so it stays here
   rather than in main.cpp. WIN32_LEAN_AND_MEAN trims the GDI/winsock half;
   NOMINMAX already comes from libstdc++'s mingw os_defines.h. */
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

/* Windows documents that a GUI-subsystem process has invalid std handles "on
   startup until AttachConsole is called", the exception being a parent that
   passed handles to inherit -- redirection and pipes, which must be left
   alone: reopening over a redirected handle sends the log to the screen
   instead of the file. Started from Explorer there is no parent console,
   AttachConsole fails, and no console is what we asked for.
   The e2e harness runs the .exe under wine with piped handles, so the handles
   are already there and this returns at the first check. */
#ifdef _WIN32
void reattach_console() {
    const HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
    const HANDLE err = GetStdHandle(STD_ERROR_HANDLE);
    const bool out_gone = (out == nullptr || out == INVALID_HANDLE_VALUE);
    const bool err_gone = (err == nullptr || err == INVALID_HANDLE_VALUE);
    if(!out_gone && !err_gone) { return; }

    /* A launcher that attached a console but handed over NULL handles gets
       ERROR_ACCESS_DENIED here -- the console exists, so CONOUT$ is still
       openable. GetConsoleWindow() separates that from the no-console case. */
    if(!AttachConsole(ATTACH_PARENT_PROCESS) && GetConsoleWindow() == nullptr) {
        return;
    }
    if(out_gone) { freopen("CONOUT$", "w", stdout); }
    if(err_gone) { freopen("CONOUT$", "w", stderr); }
}
#else
void reattach_console() {}
#endif
