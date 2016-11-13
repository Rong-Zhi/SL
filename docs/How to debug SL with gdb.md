How to debug SL with gdb
------------------------

Because SL is a multiprocess program, simply launching gdb on the main
process will not be useful for debugging as crashes usually happen on the
task process. Instead, we need to launch SL normally, find out the PID of
the process to be debugged and attach it to gdb.

### 1. Build with debugging symbols
* In the CMake interface activate the debug build mode
  `CMAKE_BUILD_TYPE DEBUG`
* Configure and generate (press `c` then `g`)
* Build (`make install`)

### 2. Attach the process to gdb
* Launch SL normally
* Attach the task process to gdb (the same can be done with servos such as
  vision, motor, etc.)
  `sudo gdb -p $(ps aux | grep xtask | grep -v xterm | grep -v grep | awk '{print $2;}')`

This is it. As soon as the process is attached, gdb will stop it and give
you the hand. You can set breakpoints, inspect variables, etc... or continue
the execution with the continue command.

Remember that gdb requires special permission to attach to other processes
memory. For this reason, run gdb with sudo rights, such that then you do
not have to modify the default behavior for all the users. More info
[here](https://wiki.ubuntu.com/SecurityTeam/Roadmap/KernelHardening#ptrace).
