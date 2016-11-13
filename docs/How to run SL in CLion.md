How to run SL in CLion
----------------------

CLion is a cross-platform IDE for C/C++ development with integrated
CMake build toolchain and GDB (or LLDB on OS X) debugger. You can use it
on Linux and OS X for building and debugging your tasks in SL.

Here is an example showing how you can run and debug the *ball_following* task.
Make sure you have installed all the dependencies for your system.

* Open SL's root folder in CLion as a new project.
* To configure CMake
    * Click **CMake** on the bottom panel and switch
      to the **Cache** tab.
    * Set `BUILD_barrett` to `ON` and click the save icon
      on the left (it is equivalent to pressing `c` when using `ccmake`).
* To build and run the project
    * Go to the top right corner of the screen,
      click on the selector-list that currently shows *Build All* and
      choose *install_barrett*.
    * Click the run icon. A dialog shows up. Set *Executable* to *xbarrett*
      and click **Run**.
    * Run the *ball_following* task by executing `st` in the blue terminal.
    * Close SL.
* To debug the *ball_following* task
    * Open `sl/barrett/src/ball_following/ball_following_task.c` and set a
      breakpoint somewhere in the `run_ball_following_task` function.
    * Click the bug icon in the top right corner of the screen. The debugger will
      start on the main thread.
      If you get the `ptrace: Operation not permitted.` error,
      [fix it](https://www.jetbrains.com/help/clion/2016.1/attaching-to-local-process.html)
      and start the debugger again.
    * You need to attach the debugger to the `barrett_xtask` process.
      In CLion, go to Run -> Attach to Local Process... and pick
      the desired process.
    * Now you may start the task in SL and enjoy debugging.

If SL crashes, there is high probability that it will leave shared memory
blocks and semaphores hanging somewhere in your system. To clean-up, use
the provided `remove_semaphore.sh` and `mac_remove_semaphore.sh` shell scripts.

### Links
* [CMake and CLion](https://www.jetbrains.com/help/clion/2016.1/quick-cmake-tutorial.html?origin=old_help)
* [GDB and CLion](http://blog.jetbrains.com/clion/2015/05/debug-clion/)
