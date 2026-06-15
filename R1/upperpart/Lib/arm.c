#include "arm.h"
#include "cmsis_os.h"

float fb_des = 0.0f, lr_des =0.0f, ud_des = 150.0f, end_des = -1.57f;
float yaw, pitch, elbow, terminal;

// ================= 常量定义 =================
#define FORBIDDEN_RADIUS 10.0f
#define Z_MIN -50.0f
#define Z_MAX 600.0f

// ================= 区域判断 =================
static inline bool is_in_cylinder_forbidden(float x, float y, float z)
{
	float r_sq = x*x + y*y;

	return (r_sq < FORBIDDEN_RADIUS * FORBIDDEN_RADIUS &&
			z > Z_MIN &&
			z < Z_MAX);
}

// ================= 腕部解算 =================
static inline void calc_wrist_point(
	float x, float y, float z, float phi,
	float *x_w, float *y_w, float *z_w, float *r_w)
{
	float r = sqrtf(x*x + y*y);

	*r_w = r - L4 * cosf(phi);
	*z_w = z - L4 * sinf(phi);

	float scale = (r > 1e-6f) ? (*r_w / r) : 0.0f;

	*x_w = x * scale;
	*y_w = y * scale;
}

// ================= 使能/失能 =================
void arm_enable(void)
{
	dm_enable(&ARM_CAN,0x01); HAL_Delay(1);
	dm_enable(&ARM_CAN,0x02); HAL_Delay(1);
	dm_enable(&ARM_CAN,0x03); HAL_Delay(1);
	dm_enable(&ARM_CAN,0x04); HAL_Delay(1);
}

void arm_disable(void)
{
	dm_disable(&ARM_CAN,0x01); HAL_Delay(1);
	dm_disable(&ARM_CAN,0x02); HAL_Delay(1);
	dm_disable(&ARM_CAN,0x03); HAL_Delay(1);
	dm_disable(&ARM_CAN,0x04); HAL_Delay(1);
}

// ================= 关节控制 =================

void arm_ctrl(float terminal,float elbow,float pitch,float yaw)
{
	// if(pitch>1.57f)pitch = 1.57f;
	// if(pitch<-1.0f)pitch = -1.0f;

	float terminal_motor = terminal * (80.0f / M_PI);
	if (terminal_motor > 40.0f)  terminal_motor = 40.0f;
	if (terminal_motor < -50.0f) terminal_motor = -50.0f;

	pos_ctrl(&ARM_CAN,0x01,yaw,1.0f);  HAL_Delay(1);
	pos_ctrl(&ARM_CAN,0x02,pitch,1.0f); HAL_Delay(1);
	pos_ctrl(&ARM_CAN,0x03,elbow,1.0f); HAL_Delay(1);
	pos_ctrl(&ARM_CAN,0x04,terminal_motor, 0.5f); HAL_Delay(1);
}

// ================= 3D IK 逆解(完整) =================
bool drive_arm_to_ik_3d(float x, float y, float z, float phi)
{
	float yaw, pitch, elbow, terminal;

	// ====== 1. 腕部解算 ======
	float x_w, y_w, z_w, r_w;
	calc_wrist_point(x, y, z, phi, &x_w, &y_w, &z_w, &r_w);

	// ====== 2. 禁区检测(碰撞) ======
	if (is_in_cylinder_forbidden(x_w, y_w, z_w)) {
		return false;
	}

	// ====== 3. 偏航角 ======
	yaw = atan2f(y, x);

	// ====== 4. IK目标点 ======
	float r_target = r_w;
	float z_target = z_w - L1;

	float d_sq = r_target * r_target + z_target * z_target;
	float d = sqrtf(d_sq);

	if (d > (L2 + L3 - 0.1f) || d < (fabsf(L2 - L3) + 0.1f)) {
		return false;
	}

	// ====== 5. 余弦定理 ======
	float cos_beta = (L2 * L2 + d_sq - L3 * L3) / (2.0f * L2 * d);
	cos_beta = fminf(fmaxf(cos_beta, -1.0f), 1.0f);
	float beta = acosf(cos_beta);

	float cos_alpha = (L2 * L2 + L3 * L3 - d_sq) / (2.0f * L2 * L3);
	cos_alpha = fminf(fmaxf(cos_alpha, -1.0f), 1.0f);
	float alpha = acosf(cos_alpha);

	float gamma = atan2f(z_target, r_target);

	float theta_2 = gamma + beta;
	float theta_3 = theta_2 - M_PI + alpha;
	float theta_4 = phi;

	// ====== 6. 角度映射 ======
	pitch    = M_PI / 2.0f - theta_2;
	elbow    = theta_3 - theta_2 + M_PI / 2.0f;
	terminal = theta_4 - theta_3;

	// ====== 7. 执行 ======
	arm_ctrl(terminal, elbow, pitch, yaw);

	return true;
}

// ================= 2D IK 逆解(平面) =================
bool drive_arm_to_ik_2d(float x, float y, float phi)
{
	float x_w = x - L4 * cosf(phi);
	float y_w = y - L4 * sinf(phi);

	float x_target = x_w;
	float y_target = y_w - L1;

	float d_sq = x_target * x_target + y_target * y_target;
	float d = sqrtf(d_sq);

	if (d > (L2 + L3 - 0.1f) || d < (fabsf(L2 - L3) + 0.1f)) {
		return false;
	}

	float cos_beta = (L2 * L2 + d_sq - L3 * L3) / (2.0f * L2 * d);
	cos_beta = fminf(fmaxf(cos_beta, -1.0f), 1.0f);
	float beta = acosf(cos_beta);

	float cos_alpha = (L2 * L2 + L3 * L3 - d_sq) / (2.0f * L2 * L3);
	cos_alpha = fminf(fmaxf(cos_alpha, -1.0f), 1.0f);
	float alpha = acosf(cos_alpha);

	float gamma = atan2f(y_target, x_target);

	float theta_2 = gamma + beta;
	float theta_3 = theta_2 - M_PI + alpha;
	float theta_4 = phi;

	pitch    = M_PI / 2.0f - theta_2;
	elbow    = theta_3 - theta_2 + M_PI / 2.0f;
	terminal = theta_4 - theta_3;
	yaw      = 0.0f;

	arm_ctrl(terminal, elbow, pitch, yaw);

	return true;
}

// ================= 预置动作 =================

void arm_back_zero(void)
{
    arm_ctrl(0.0f, 0.0f, 0.0f, 0.0f);
}

