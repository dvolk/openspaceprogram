#pragma once

// console.h -- puts a shell's console back on the windows exe.
// osp.exe links -mwindows so double-clicking it does not open a console
// window; the price is dead std handles when it is started from a shell
// instead. Called first thing in main(); no-op on other OSes.

void reattach_console();
