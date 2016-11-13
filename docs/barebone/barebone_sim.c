#include "SL_system_headers.h"
#include "SL.h"

#include "barebone_env.h"

static Env env;

int sim_barebone_task() {
  return TRUE;
}

void send_barebone_graphics() {
  env.x = 42; // This is just an example
  printf("Sending %d\n", env.x);
  sendUserGraphics("Barebone", &(env), sizeof(Env)); // This is the important part: we are sending the WHOLE env struct
}
