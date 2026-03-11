# task_state_estimator.py
# Discrete-time observer / state estimator for the Romi (low-allocation).

import micropython
import math

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)


class task_state_estimator:
    def __init__(self,
                 enable_est,
                 # measurements
                 posL_counts, posR_counts,
                 effortL, effortR,
                 imu_heading_deg, imu_yawrate_dps,
                 # required outputs
                 xhat_s, xhat_psi,
                 # optional outputs
                 x_pos=None, y_pos=None, dist_traveled=None,
                 xhat_omegaL=None, xhat_omegaR=None,
                 yhat_sL=None, yhat_sR=None, yhat_psi=None, yhat_psidot=None,
                 # constants
                 wheel_radius_mm=35.0,
                 track_width_mm=141.0,
                 counts_per_rad=229.1831,
                 v_batt=7.4,
                 unwrap_heading=True,
                 pose_heading_sign=-1.0):

        # shares
        self._en = enable_est
        self._posL = posL_counts
        self._posR = posR_counts
        self._effL = effortL
        self._effR = effortR
        self._hdg_deg = imu_heading_deg
        self._wz_dps  = imu_yawrate_dps

        self._xhat_s   = xhat_s
        self._xhat_psi = xhat_psi

        self._xhat_omegaL = xhat_omegaL
        self._xhat_omegaR = xhat_omegaR

        self._yhat_sL     = yhat_sL
        self._yhat_sR     = yhat_sR
        self._yhat_psi    = yhat_psi
        self._yhat_psidot = yhat_psidot

        self._x_pos = x_pos
        self._y_pos = y_pos
        self._dist  = dist_traveled

        # constants
        self._r = float(wheel_radius_mm)
        self._w = float(track_width_mm)
        self._cpr = float(counts_per_rad)
        self._w2 = 0.5 * self._w
        self._r_over_w = (self._r / self._w) if self._w != 0.0 else 0.0

        self._u_scale = float(v_batt) / 100.0  # effort [-100..100] -> volts
        self._unwrap = bool(unwrap_heading)
        self._psi_prev = 0.0
        self._deg2rad = math.pi / 180.0
        self._pose_hdg_sign = float(pose_heading_sign)

        # observer matrices (already include injection)
        self._A = (
            (0.6523, 0.2845, 0.1625, -0.0000),
            (0.2845, 0.6523, 0.1625, -0.0000),
            (0.1106, 0.1106, 0.4311,  0.0000),
            (0.0000, -0.0000, 0.0000, 1.0000),
        )
        self._B = (
            (0.2261, 0.0469, -0.0813, -0.0813, -0.0000, -1.1459),
            (0.0469, 0.2261, -0.0813, -0.0813,  0.0000,  1.1459),
            (0.0183, 0.0183,  0.2845,  0.2845, -0.0000,  0.0000),
            (0.0000, -0.0000, -0.0000, -0.0000, 0.0000,  0.0100),
        )

        # state: xhat = [OmegaL, OmegaR, s, psi]
        self.xhat = [0.0, 0.0, 0.0, 0.0]
        self._gx = 0.0
        self._gy = 0.0
        self._s_prev = 0.0

        self._state = S0_INIT

        # publish yhat only if any wired
        self._has_yhat = (yhat_sL is not None) or (yhat_sR is not None) or (yhat_psi is not None) or (yhat_psidot is not None)

    def _deg_to_rad(self, deg):
        return float(deg) * self._deg2rad

    def _counts_to_mm(self, counts):
        return self._r * (float(counts) / self._cpr)

    def _unwrap_psi(self, psi):
        if not self._unwrap:
            return psi
        d = psi - self._psi_prev
        if d > math.pi:
            psi -= 2.0 * math.pi
        elif d < -math.pi:
            psi += 2.0 * math.pi
        return psi

    def _update_observer(self, uL, uR, sL, sR, psi, psidot):
        x0, x1, x2, x3 = self.xhat
        A = self._A
        B = self._B

        xn0 = (A[0][0]*x0 + A[0][1]*x1 + A[0][2]*x2 + A[0][3]*x3) + (B[0][0]*uL + B[0][1]*uR + B[0][2]*sL + B[0][3]*sR + B[0][4]*psi + B[0][5]*psidot)
        xn1 = (A[1][0]*x0 + A[1][1]*x1 + A[1][2]*x2 + A[1][3]*x3) + (B[1][0]*uL + B[1][1]*uR + B[1][2]*sL + B[1][3]*sR + B[1][4]*psi + B[1][5]*psidot)
        xn2 = (A[2][0]*x0 + A[2][1]*x1 + A[2][2]*x2 + A[2][3]*x3) + (B[2][0]*uL + B[2][1]*uR + B[2][2]*sL + B[2][3]*sR + B[2][4]*psi + B[2][5]*psidot)
        xn3 = (A[3][0]*x0 + A[3][1]*x1 + A[3][2]*x2 + A[3][3]*x3) + (B[3][0]*uL + B[3][1]*uR + B[3][2]*sL + B[3][3]*sR + B[3][4]*psi + B[3][5]*psidot)

        self.xhat[0] = float(xn0)
        self.xhat[1] = float(xn1)
        self.xhat[2] = float(xn2)
        self.xhat[3] = float(xn3)

    def _compute_yhat(self):
        x0, x1, x2, x3 = self.xhat
        y0 = x2 - self._w2 * x3
        y1 = x2 + self._w2 * x3
        y2 = x3
        y3 = (-self._r_over_w) * x0 + (self._r_over_w) * x1
        return y0, y1, y2, y3

    def run(self):
        while True:
            if self._state == S0_INIT:
                try:
                    psi0 = self._deg_to_rad(self._hdg_deg.get())
                except Exception:
                    psi0 = 0.0

                psi0 = self._unwrap_psi(psi0)
                self._psi_prev = psi0

                self.xhat[0] = 0.0
                self.xhat[1] = 0.0
                self.xhat[2] = 0.0
                self.xhat[3] = float(psi0)

                self._s_prev = 0.0
                self._gx = 0.0
                self._gy = 0.0

                # publish
                self._xhat_s.put(0.0)
                self._xhat_psi.put(float(psi0))
                if self._xhat_omegaL is not None:
                    self._xhat_omegaL.put(0.0)
                if self._xhat_omegaR is not None:
                    self._xhat_omegaR.put(0.0)

                if self._has_yhat:
                    y0, y1, y2, y3 = self._compute_yhat()
                    if self._yhat_sL is not None:
                        self._yhat_sL.put(y0)
                    if self._yhat_sR is not None:
                        self._yhat_sR.put(y1)
                    if self._yhat_psi is not None:
                        self._yhat_psi.put(y2)
                    if self._yhat_psidot is not None:
                        self._yhat_psidot.put(y3)

                if self._x_pos is not None:
                    self._x_pos.put(0.0)
                if self._y_pos is not None:
                    self._y_pos.put(0.0)
                if self._dist is not None:
                    self._dist.put(0.0)

                self._state = S1_RUN

            elif self._state == S1_RUN:
                en = True
                try:
                    en = bool(self._en.get())
                except Exception:
                    en = True

                # keep unwrap synced
                try:
                    psi_meas = self._deg_to_rad(self._hdg_deg.get())
                    psi_meas = self._unwrap_psi(psi_meas)
                    self._psi_prev = psi_meas
                except Exception:
                    psi_meas = self._psi_prev

                if not en:
                    self._s_prev = float(self.xhat[2])
                    yield self._state
                    continue

                # wheel displacements
                try:
                    sL = self._counts_to_mm(self._posL.get())
                    sR = self._counts_to_mm(self._posR.get())
                except Exception:
                    sL = 0.0
                    sR = 0.0

                psi = psi_meas

                try:
                    psidot = self._deg_to_rad(self._wz_dps.get())
                except Exception:
                    psidot = 0.0

                # efforts -> volts
                try:
                    uL = float(self._effL.get()) * self._u_scale
                    uR = float(self._effR.get()) * self._u_scale
                except Exception:
                    uL = 0.0
                    uR = 0.0

                self._update_observer(uL, uR, sL, sR, psi, psidot)

                # publish xhat
                if self._xhat_omegaL is not None:
                    self._xhat_omegaL.put(self.xhat[0])
                if self._xhat_omegaR is not None:
                    self._xhat_omegaR.put(self.xhat[1])
                self._xhat_s.put(self.xhat[2])
                self._xhat_psi.put(self.xhat[3])

                # publish yhat (optional)
                if self._has_yhat:
                    y0, y1, y2, y3 = self._compute_yhat()
                    if self._yhat_sL is not None:
                        self._yhat_sL.put(y0)
                    if self._yhat_sR is not None:
                        self._yhat_sR.put(y1)
                    if self._yhat_psi is not None:
                        self._yhat_psi.put(y2)
                    if self._yhat_psidot is not None:
                        self._yhat_psidot.put(y3)

                # pose integrate
                if (self._x_pos is not None) or (self._y_pos is not None) or (self._dist is not None):
                    s_now = float(self.xhat[2])
                    ds = s_now - self._s_prev
                    self._s_prev = s_now

                    psi_pose = self._pose_hdg_sign * psi
                    self._gx += ds * math.cos(psi_pose)
                    self._gy += ds * math.sin(psi_pose)

                    if self._x_pos is not None:
                        self._x_pos.put(self._gx)
                    if self._y_pos is not None:
                        self._y_pos.put(self._gy)
                    if self._dist is not None:
                        self._dist.put(s_now)

            yield self._state
