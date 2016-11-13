#include "SL.h"
#include "SL_tasks.h"

#include "barebone.h"

static int init_barebone_task() {
  // If you use static global variables, init them here
  return TRUE;
}

static int run_barebone_task() {
  // That's how your run task should be organized
  sim_barebone_task(); // First run a sim step
  send_barebone_graphics(); // Then update the graphics
  return TRUE;
}

// Just a default function, no need to change it
static int change_barebone_task() {
  return TRUE;
}

// Just a default function, used to display the task name in the task servo
void add_barebone_task() {
  addTask("Barebone", init_barebone_task, run_barebone_task, change_barebone_task);
}
