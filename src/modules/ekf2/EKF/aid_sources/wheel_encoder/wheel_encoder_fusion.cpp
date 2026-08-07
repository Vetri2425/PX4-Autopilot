/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file wheel_encoder_fusion.cpp
 * Fuses wheel-encoder velocity as a body-frame velocity observation.
 *
 * A differential-drive ground rover provides a body-X (forward) velocity from
 * its wheel speeds. This is fused via the existing body-frame velocity fuser,
 * which is rotation-aware (its Jacobian accounts for attitude states), so the
 * measurement is compared correctly against the NED state velocity regardless
 * of heading.
 *
 * In addition to the forward speed, a non-holonomic side-slip constraint is
 * applied by observing the body-lateral (body-Y) velocity as zero. The body-Z
 * axis is given a large variance so it does not meaningfully update the state.
 *
 * IMU lever arm: the fused state velocity (_state.vel) is referenced to the
 * IMU, not the body/axle origin the wheel encoders describe (see
 * output_predictor.h's getVelocity(), which subtracts this same offset to
 * report body-origin velocity). GNSS velocity, optical flow, and baro dynamic
 * pressure all correct their sensor's measurement for this same lever arm
 * before fusion (updateGnssVel() in gps_control.cpp, optical_flow_fusion.cpp,
 * baro_height_control.cpp). Without the equivalent correction here, a nonzero
 * EKF2_IMU_POS_X/Y and a real yaw rate during an in-place pivot produce a
 * genuine IMU-frame lateral velocity of omega x imu_pos_body that is nonzero
 * even though the vehicle isn't translating -- and the fixed zero-side-slip
 * constraint two lines below then fights that real term on every fusion
 * cycle for the whole pivot, biasing the velocity (and, through
 * velocity-position cross-covariance, the position) state. Confirmed against
 * field logs: the resulting position walk traces a circle of radius
 * ~= EKF2_IMU_POS_X as heading sweeps, not a random drift, and collapses when
 * EKF2_WENC_CTRL=0 or EKF2_IMU_POS_X=0.
 *
 * This is a secondary aid: it is only fused while another source is already
 * providing horizontal aiding (isHorizontalAidingActive()), so wheel encoders
 * never initialise the horizontal solution on their own.
 *
 * The rad/s -> m/s conversion (wheel radius) and the enable gating are handled
 * upstream in EKF2.cpp, which only pushes samples when fusion is enabled and a
 * valid wheel radius is configured. The buffer therefore only exists when the
 * feature is active.
 */

#include "ekf.h"

void Ekf::controlWheelEncoderFusion(const imuSample &imu_sample)
{
	if (_wheel_encoder_buffer) {
		wheelEncoderSample sample;

		if (_wheel_encoder_buffer->pop_first_older_than(imu_sample.time_us, &sample)) {

			// Secondary aid only: require an existing horizontal aiding source.
			if (isHorizontalAidingActive()) {

				// Correct for the lever arm between the IMU and the wheel encoders'
				// reference point (the body/axle origin, i.e. zero offset by
				// convention -- same pattern as updateGnssVel() in gps_control.cpp,
				// just with the axle taken as the zero-offset reference instead of
				// the GNSS antenna). Entirely in body frame: unlike GNSS velocity,
				// this measurement is fused via fuseBodyFrameVelocity() and never
				// leaves the body frame, so no _R_to_earth rotation is needed here.
				const Vector3f pos_offset_body = -_params.imu_pos_body;
				const Vector3f angular_velocity = imu_sample.delta_ang / imu_sample.delta_ang_dt - _state.gyro_bias;
				const Vector3f vel_offset_body = angular_velocity % pos_offset_body;

				// Body-frame measurement: forward speed, zero side-slip, unconstrained
				// vertical, corrected above for the IMU lever arm.
				const Vector3f measurement = Vector3f(sample.vel_body_fwd, 0.f, 0.f) - vel_offset_body;

				// Per-axis observation variance:
				//  - body-X: reported forward-velocity variance
				//  - body-Y: side-slip constraint noise (small -> enforces no lateral motion)
				//  - body-Z: large -> near-zero Kalman gain, effectively unconstrained
				const Vector3f measurement_var(math::max(sample.vel_fwd_var, sq(0.01f)),
							       sq(_params.wenc_lat_noise),
							       sq(1000.f));

				fuseBodyFrameVelocity(_aid_src_wheel_encoder, sample.time_us, measurement, measurement_var,
						      math::max(_params.wenc_gate, 1.f));
			}
		}
	}
}

void Ekf::stopWheelEncoderFusion()
{
	ECL_INFO("stopping wheel encoder fusion");
}
