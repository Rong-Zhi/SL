#include "SL_system_headers.h"
#include "SL_userGraphics.h"

#include "barebone_env.h"

static void display_barebone(void *env_pointer) {
  Env* env = env_pointer; // We read the struct that we receive
  printf("Receiving %d\n", env->x); // This is just an example (the struct has an integer field x)
}

void add_barebone_graphics() {
  addToUserGraphics("Barebone", "Display graphics", &(display_barebone), sizeof(Env));
}
