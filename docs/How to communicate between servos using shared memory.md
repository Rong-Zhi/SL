How to communicate between servos using shared memory
-----------------------------------------------------

This document describes how to define a custom data type and 
share information of that type between servos using shared memory.
As an example, `int` and `char[10]` are sent from the ROS-servo to the Task-servo.
Note that this mechanism is different from sending messages between servos,
which can be done using built-in functions (e.g. `sendMessageTaskServo`).
If you don't know how to run the ROS servo, read
[How to use the ROS servo](docs/How%20to%20use%20the%20ROS%20servo.md).


### 1. Define your custom message type

You will need to create two files: `shared_memory.h` and `shared_memory.c`
(you can name them whatever you like). In the header file, define the
structure type you are going to share between servos.

```C
#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "SL_system_headers.h"


typedef struct {
  SEM_ID          sm_sem;
  int             age;
  char            name[10];
} smPlim;


#ifdef __cplusplus
extern "C" {
#endif


extern smPlim*    sm_plim;
extern SEM_ID     sm_plim_sem;
int init_shmem();


#ifdef __cplusplus
}
#endif

#endif //SHARED_MEMORY_H
```

Here `smPlim` is the data structure, `sm_plim` is the pointer used to access
it from different servos, and `sm_plim_sem` is the semaphore used to
synchronize access to the shared data. Next we will define
function `init_shmem()` that will reserve shared memory and the semaphore.
Its definition goes into `shared_memory.c`.

```C
#include "SL_system_headers.h"
#include "SL.h"

#include "shared_memory.h"

smPlim* sm_plim;
SEM_ID sm_plim_sem;

int init_shmem() {
  if (!init_sm_object("smPlim", sizeof(smPlim), 0, &sm_plim_sem,
                      (void**) &sm_plim)) {
    return FALSE;
  }
  return TRUE;
}
```

`init_shmem()` must be called exactly once
from each servo using `sm_plim` and `sm_plim_sem`.


#### 1.1 Initialize shared memory and semaphores

`init_sm_object()` provides a platform-independent way to acquire
shared memory, that guarantees compatibility with real-time operating systems.
Add the following code directly after includes in `shared_memory.c`.

```C
/*****************************************************************************
\param[in]  smname     :  name of the shared memory object
\param[in]  structsize :  size of base structure
\param[in]  datasize   :  size of data structure
\param[out] semptr     :  ptr to object's semaphore
\param[out] structptr  :  handle to object's base structure
*****************************************************************************/
static int init_sm_object(char* smname, size_t structsize, size_t datasize,
                          SEM_ID* semptr, void** structptr) {
  int mtype;
  SEM_ID* semptr_sm;
  STATUS error;
  char smn[100];
  char semn[100];

    // build the full name from both robot name and desired name
#ifdef VX  /* vxworks has a 20 character limit */
  sprintf(smn,"%s",smname);
#else
  sprintf(smn, "%s.%s", robot_name, smname);
#endif

  if (smNameFind(smn, structptr, &mtype, NO_WAIT) == ERROR) {

    // get the object's base structure and add datasize to memory chunk
#ifdef VX
    *structptr = (void *)smMemCalloc(1,structsize+datasize);
#else
    *structptr = (void*) smMemCalloc(smn, parent_process_id, 1,
                                     structsize + datasize);
#endif
    if (*structptr == NULL) {
      printf("Couldn't create shared memory object %s\n", smn);
      return FALSE;
    }

    // get the object's semaphore
    semptr_sm = (SEM_ID*) *structptr;
#ifdef VX
    *semptr = semBSmCreate(SEM_Q_FIFO, SEM_FULL);
    if (*semptr == NULL) {
#else
    sprintf(semn, "%s.%s_sem", robot_name, smname);
    *semptr = semBSmCreate(semn, parent_process_id, SEM_Q_FIFO, SEM_FULL);
    if (*semptr == (SEM_ID) (-1)) {
#endif
      printf("Couldn't create shared semaphore for object %s -- sem_id=%ld\n",
             semn, (long) *semptr);
      return FALSE;
    }

#ifdef __XENO__
    // xenomai does not have a global identifier for semaphores
    *semptr_sm = NULL;
#else
    // vxWorks and Unix use global identifiers for semaphores which
    // we can keep in shared memory
    *semptr_sm = *semptr;
#endif

#ifdef VX
    // add the object to the name database
    error = smNameAdd(smn,(void*)smObjLocalToGlobal(*structptr),T_SM_PART_ID);
    if (error == ERROR)
      return FALSE;
#endif

  } else {
    *structptr = smObjGlobalToLocal(*structptr);
  }

  return TRUE;
}
```

Now we need to make good use of `sm_plim`, `sm_plim_sem` and `init_shmem()`
in our servos.


### 2. Send data from ROS-servo

In `SL_user_ros.cpp`, add the following code

```C
static const long TIME_OUT_NS = 1000000;
static void send_data_to_task_servo() {

  static int i = 0;

  if (semTake(sm_plim_sem, ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++ros_servo_errors;
    printf("Couldn't take the plim semaphore\n");
  } else {
    sm_plim->age = i;
    strcpy(sm_plim->name, "santa");
    printf("%s: %d\n", sm_plim->name, sm_plim->age);
    semGive(sm_plim_sem);
  }

  ++i;

}
```

The above code takes a semaphore, fills in some data, and returns the semaphore.
Add `init_shmem()` call to `init_user_ros()`, and
`send_data_to_task_servo()` to `run_user_ros()`. Also, do not forget to include
`shared_memory.h` in `SL_user_ros.cpp`.


### 3. Receive data in Task-servo

In `TASKNAME_task.c`, add

```C
static void receive_data_from_ros_servo() {

  if (semTake(sm_plim_sem, ns2ticks(TIME_OUT_NS)) == ERROR) {
    ++task_servo_errors;
    printf("Couldn't take the plim semaphore\n");
  } else {
    printf("%s: %d\n", sm_plim->name, sm_plim->age);
    semGive(sm_plim_sem);
  }

}
```

Make sure to call `init_shmem()` in `init_TASKNAME_task()` and
`receive_data_from_ros_servo()` from `run_TASKNAME_task()`.
Finally, add

    #include "SL_system_headers.h"
    #include "SL.h"
    
    #include "shared_memory.h"

to the list of includes in `TASKNAME_task.c` and add the line

    static const long TIME_OUT_NS = 1000000;
    
after all includes.


### 4. Adjust CMakeLists.txt files

Our shared memory code will be accessed from C and C++ files, therefore we
will build a static library and link the servos' executables against it.

#### 4.1 Build shared_memory library

Open `CMakeLists.txt` in `YOUR_ROBOT` folder (e.g. in `barrett` or `darias`).
Add the following code before the ROS-specific part

```CMake
# Create shared_memory library for communication
add_library(${NAME}_shmem
		src/shared_memory.c
		${SRCS_COMMON}
)
```

#### 4.2 Link ROS-servo against shared_memory library

Add `${NAME}_shmem` to the list of `target_link_libraries` for the target
`${NAME}_xros`.

#### 4.3 Link Task-servo against shared_memory library

Close `CMakeLists.txt` and open `YOUR_ROBOT/src/YOUR_TASK/CMakeLists.txt`.
Before `finalize_auto_add_task()`, add

```CMake
set(LIBS_XTASK ${NAME}_shmem)
```


### 5. Test run

Start SL (e.g. `./xdarias -ros`) and observe that ROS-servo is producing
output. Start a task and observe that Task-servo is repeating the output
of the ROS-servo.
