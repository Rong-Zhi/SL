#ifndef QUAT_TOOLS_H_
#define QUAT_TOOLS_H_

#include "SL_system_headers.h"

#include "SL.h"
#include "SL_common.h"
#include "utility.h"

void rotate_quat(SL_quat q, Vector v, Vector v_new);
void rotate_negative_quat(SL_quat q, Vector v, Vector v_new);
void mult_quat(SL_quat *q1, SL_quat *q2, SL_quat *q_new);
void quat_vec_mult(SL_quat q, Vector in, Vector out);
void rotate_quat_inv(SL_quat *q, Vector v, Vector v_new);

#endif // QUAT_TOOLS_H_
