const float b0 = 0.08636403;
const float b1 = 0.08636403;
const float a1 = -0.82727195;

void derive(float dt, float x, float &vel, float &acc)
{
  static float x1=0, y1=0, dx_dt_1=0, y_acc1 = 0, dv_dt = 0, dv_dt_1=0;
  static float y, y_acc, dx_dt;
  dx_dt = (x - x1)/dt;
  y = b0 * dx_dt + b1 * dx_dt_1 - a1 * y1;
  dv_dt = (y - y1)/dt;
  y_acc = b0* dv_dt + b1 * dv_dt_1 - a1 * y_acc1;

  y_acc1 = y_acc;
  dv_dt_1 = dv_dt;

  x1 = x;
  dx_dt_1 = dx_dt;

  y1 = y;
  vel = y;
  acc = y_acc;

}