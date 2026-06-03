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
				// Body-frame measurement: forward speed, zero side-slip, unconstrained vertical.
				const Vector3f measurement(sample.vel_body_fwd, 0.f, 0.f);

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
