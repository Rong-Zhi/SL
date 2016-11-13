# How to write a new task

To get started, copy the [barebone](docs/barebone) task into the `src` folder of your robot. 
This guide will tell you how to modify the files within as needed for your task.


1 Organization
--------------

Let's say you want to write a task named `EXAMPLE`.
To do a nice and clean job you should create the following files and put them in the `src/EXAMPLE` folder:

- `EXAMPLE_task.c`, the main file with the task,
- `EXAMPLE_sim.c`, the simulator of the physics of the problem (if needed),
- `EXAMPLE_graphics.c`, the simulator for the animations (if needed),

and the following headers:

- `EXAMPLE.h`, the header with common variables and functions shared by the files above,
- `EXAMPLE_env.h`, the header with the environment structure.

### 1.1 EXAMPLE_graphics.c

Have a main function called `void display_EXAMPLE(EXAMPLE_environmentVar *env)` that takes care of the drawing,
and a function called `void add_EXAMPLE_graphics()` that adds your display handler to SL.
You can call some basic functions already provided by `initUserGraphics.c`, or add additional ones if needed.
Also, include both `EXAMPLE.h` and `EXAMPLE_env.h`.

### 1.2 EXAMPLE_sim.c

Here you take care of the physics simulation. You should write the following functions and call them from `EXAMPLE_task.c` when needed:

- `void reset_EXAMPLE_simulation()`
- `int sim_EXAMPLE_task()`
- `int add_EXAMPLE_vars()`
- `void send_EXAMPLE_graphics()`

The last one should end with `sendUserGraphics("KEYWORD",&(env), sizeof(EXAMPLE_environmentVar))`.
`env` is a struct of type `EXAMPLE_environmentVar` declared in `EXAMPLE_env.h`. This means that you have to include both `EXAMPLE.h` and `EXAMPLE_env.h`.
Please see the examples provided to have a better idea. See also `EXAMPLE.h` for more details.

### 1.3 EXAMPLE_task.c

Write the following mandatory functions:

- `void add_EXAMPLE_task(void)`, that adds your task to the selection menu and tells SL what to do when it is chosen;
- `static int init_EXAMPLE_task(void)`, is be called before the task is run;
- `static int run_EXAMPLE_task(void)`, is called once during every time step;
- `static int change_EXAMPLE_task(void)`, allows for user input during execution.

Here you should need to include only `EXAMPLE.h`.
See the examples provided for more details.

### 1.4 EXAMPLE.h

Create it only if needed, i.e. if you have more than one `.c` file and you need to share functions or variables. About variables declaration, you should declare only the following types of variables:

- `static const`: variables that are the same for all files that include the header (one copy of the variable will be created, you have to initialize them),
- `extern`: variables that are not constant but shared between files (one copy of the variable will be created but ONLY ONE file that includes the header have to redeclare it without `extern`).

Of course, you can also declare `static` variables, but this doesn't make much sense because each file will have its own copy of the variable (there will be no sharing).
Please also note that you can share variables only between `EXAMPLE_sim.c` and `EXAMPLE_task.c`, since the graphic simulator (`EXAMPLE_graphics.c`) is run on a different program.

Finally, use *header guards* in order to avoid problems of double inclusion:

```
#ifndef EXAMPLE_H
#define EXAMPLE_H

// body of the header

#endif
```

Please refer to the examples to better understand the writing of the header.
Also have a look at http://stackoverflow.com/questions/1433204/how-do-i-use-extern-to-share-variables-between-source-files-in-c

### 1.5 EXAMPLE_env.h

In this header you have to declare a struct called `EXAMPLE_environmentVar` that contains all the variables of the environment of the task (e.g. the position of a ball, of the table, etc...). An instance of this struct will be instantiated by `EXAMPLE_sim.c` and passed to `EXAMPLE_graphics.c` in order to draw the environment objects. Again, use header guards.


2 Compilation settings
----------------------

Finally, you need to create a `CMakeLists.txt` file in the EXAMPLE source folder, i.e., `src/EXAMPLE/CMakeLists.txt`. The file should look like this:

```
cmake_minimum_required(VERSION 2.8)

set(TASKNAME EXAMPLE)

project(${TASKNAME})

message( STATUS "Adding ${TASKNAME} task for ${NAME}" )

# add the following line if the task requires to communicate with Matlab
required_matlab_shared_mem()

include_directories(
    ${CMAKE_SOURCE_DIR}/IAS/include
)

# for executables
set(ADD_SRCS_TASK
    EXAMPLE_sim.c
    EXAMPLE_task.c
    # any other additional file needed
)

# for graphics
set(ADD_SRCS_OPENGL
    EXAMPLE_graphics.c
    # any other additional file needed
)

finalize_auto_add_task()
```

Be sure that in `set(TASKNAME EXAMPLE)` you set the name correctly! It needs to be the same that you are using for all the other files (`EXAMPLE_sim.c`, `EXAMPLE_task.c`, ...).


3 Recap
-------

1. Write the headers `EXAMPLE.h` and `EXAMPLE_env.h`. Put the constant variables, the shared variables, the environment struct and the common functions declarations.
2. Write `EXAMPLE_task.c`, `EXAMPLE_sim.c`, `EXAMPLE_graphics.c`. The task executes the controller and calls the physics simulator; the physics updates the environment according to the controller, instantiate a struct with the environment variables and calls the graphics; the graphics draws the 3D animation.
3. Write the `CMakeLists.txt` file.


