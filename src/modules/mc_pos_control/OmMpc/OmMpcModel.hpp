// Copyright 2026 Chaoheng Meng. SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "OmQpSolver.hpp"
#include <matrix/matrix/math.hpp>
#include <cmath>
#include <cstring>

// Nominal on-manifold / ROS error model in float32.
// Reference discretization mismatch is neglected, as in the ROS/MATLAB OM-MPC path.
// NED + FRD, right SO(3) error, no disturbance model in this first integration.
namespace om_mpc
{
using matrix::Vector3f;
using matrix::Matrix3f;
using matrix::Dcmf;
struct Path {
	Vector3f center{}, amplitude{}, harmonic{}, offset{};
	float phase{}, omega{}, rotation{}, yaw{};
};

struct Knot {
	Vector3f position{}, velocity{}, rate{};
	Dcmf rotation{};
	float thrust{};
};

inline Matrix3f hat(const Vector3f &v)
{
	Matrix3f m;
	m(0, 1) = -v(2);
	m(0, 2) = v(1);
	m(1, 0) = v(2);
	m(1, 2) = -v(0);
	m(2, 0) = -v(1);
	m(2, 1) = v(0);
	return m;
}

inline bool normalized(const Vector3f &v, const Vector3f &d, Vector3f &n, Vector3f &dn, float *magnitude = nullptr)
{
	const float norm = v.norm();

	if (!std::isfinite(norm) || norm < 1e-5f) {
		return false;
	}

	if (magnitude) {
		*magnitude = norm;
	}

	n = v / norm;
	dn = (d - n * n.dot(d)) / norm;
	return n.isAllFinite() && dn.isAllFinite();
}

// All horizon knots share these derivative amplitudes. Preserve the original
// multiplication order; only phase and its sine/cosine vary within one build.
struct ReferenceScales {
	Vector3f position, velocity, acceleration, jerk;
	explicit ReferenceScales(const Path &p) : position(p.amplitude)
	{
		for (int i = 0; i < 3; i++) {
			const float w = p.harmonic(i) * p.omega;
			velocity(i) = p.amplitude(i) * w;
			acceleration(i) = -p.amplitude(i) * w * w;
			jerk(i) = -p.amplitude(i) * w * w * w;
		}
	}
};

inline bool reference(const Path &p, float t, Knot &out, const Dcmf &heading, const Vector3f &x_heading,
		      const ReferenceScales &scales)
{
	Vector3f derivatives[4];

	for (int i = 0; i < 3; i++) {
		const float theta = p.harmonic(i) * (p.phase + p.omega * t) + p.offset(i);
		const float sine = sinf(theta), cosine = cosf(theta);
		derivatives[0](i) = scales.position(i) * sine;
		derivatives[1](i) = scales.velocity(i) * cosine;
		derivatives[2](i) = scales.acceleration(i) * sine;
		derivatives[3](i) = scales.jerk(i) * cosine;
	}

	for (auto &v : derivatives) {
		v = heading * v;
	}

	out.position = p.center + derivatives[0];
	out.velocity = derivatives[1];
	const Vector3f thrust_axis = Vector3f{0.f, 0.f, 9.80665f} -derivatives[2];
	Vector3f z, dz, y, dy;

	if (!normalized(thrust_axis, -derivatives[3], z, dz, &out.thrust)) {
		return false;
	}

	if (!normalized(z.cross(x_heading), dz.cross(x_heading), y, dy)) {
		return false;
	}

	const Vector3f x = y.cross(z), dx = dy.cross(z) + y.cross(dz);

	for (int i = 0; i < 3; i++) {
		out.rotation(i, 0) = x(i);
		out.rotation(i, 1) = y(i);
		out.rotation(i, 2) = z(i);
	}

	// Only the three vee entries of R^T*dR are needed.
	out.rate = {z.dot(dy), x.dot(dz), y.dot(dx)};
	return out.position.isAllFinite() && out.velocity.isAllFinite() && out.rotation.isAllFinite() && out.rate.isAllFinite();
}

inline bool reference(const Path &p, float t, Knot &out, const Dcmf &heading, const Vector3f &x_heading)
{
	return reference(p, t, out, heading, x_heading, ReferenceScales(p));
}

inline bool reference(const Path &p, float t, Knot &out)
{
	const Dcmf heading(matrix::Eulerf(0.f, 0.f, p.rotation));
	const Vector3f x_heading{cosf(p.yaw), sinf(p.yaw), 0.f};
	return reference(p, t, out, heading, x_heading);
}

inline bool build(const Path &path, float elapsed, const Vector3f &position, const Vector3f &velocity,
		  const Dcmf &rotation, float max_thrust, const Vector3f &rate_limit, OmFloatProblem &p,
		  bool reuse_structure = false, float prediction_dt = .04f, int horizon = 15)
{
	if (horizon < 1 || horizon > OmMaxHorizon) { return false; }

	if (!std::isfinite(prediction_dt) || prediction_dt < .005f || prediction_dt > .1f) { return false; }

	// Reuse only a problem initialized by this builder. All trajectory-dependent
	// entries and bounds are overwritten below; costs and structural zeros are fixed.
	// A grid change clears the maximum-sized storage, including inactive tail slots.
	if (!reuse_structure || p.horizon != horizon || memcmp(&p.dt, &prediction_dt, sizeof(prediction_dt)) != 0) {
		memset(&p, 0, sizeof(p));
		p.horizon = horizon;
		p.dt = prediction_dt;
		const float q[] {324.f, 324.f, 324.f, 10.f, 10.f, 10.f, 4.32f, 4.32f, 4.32f};
		const float r[] {.25f, .8f, .8f, .8f};

		for (int i = 0; i < 9; i++) {
			p.q[i] = p.p[i] = q[i];
		}

		for (int i = 0; i < 4; i++) {
			p.r[i] = r[i];
		}

		for (int k = 0; k < horizon; k++) {
			for (int i = 0; i < 9; i++) {
				p.a[k][i * 9 + i] = 1.f;
			}

			for (int i = 0; i < 3; i++) {
				p.a[k][i * 9 + i + 3] = p.dt;
			}
		}
	}

	// Constant path/yaw frames are shared by all horizon knots.
	const Dcmf heading(matrix::Eulerf(0.f, 0.f, path.rotation));
	const Vector3f x_heading{cosf(path.yaw), sinf(path.yaw), 0.f};
	const ReferenceScales scales(path);
	Knot ref;

	if (!reference(path, elapsed, ref, heading, x_heading, scales)) { return false; }

	for (int k = 0; k < horizon; k++) {
		if (k && !reference(path, elapsed + k * p.dt, ref, heading, x_heading, scales)) { return false; }

		if (!k) {
			const Vector3f ep = position - ref.position, ev = velocity - ref.velocity;
			matrix::Quatf error_q(Dcmf(ref.rotation.transpose()*rotation));
			error_q.canonicalize();
			const Vector3f er = matrix::AxisAnglef(error_q);

			for (int i = 0; i < 3; i++) {
				p.x0[i] = ep(i);
				p.x0[3 + i] = ev(i);
				p.x0[6 + i] = er(i);
				p.reference_input[1 + i] = ref.rate(i);
			}

			p.reference_input[0] = ref.thrust;
		}

		// e = [position error, velocity error, Log(R_ref^T R)], u = [thrust acceleration, body rates].
		const Vector3f phi = ref.rate * p.dt;
		const float t2 = phi.dot(phi);
		const Matrix3f K = hat(phi);
		Matrix3f identity;
		identity.setIdentity();
		// hat(phi)^2 = phi*phi^T - |phi|^2 I. Share this and the SO(3)
		// coefficients between Exp(-phi) and the right Jacobian J(phi)^T.
		Matrix3f K2;

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) {
				K2(i, j) = phi(i) * phi(j) - (i == j ? t2 : 0.f);
			}

		float sinc, cosc, jc;

		if (t2 < .01f) {
			const float t4 = t2 * t2, t6 = t4 * t2;
			sinc = 1.f - t2 / 6.f + t4 / 120.f - t6 / 5040.f;
			cosc = .5f - t2 / 24.f + t4 / 720.f - t6 / 40320.f;
			jc = 1.f / 6.f - t2 / 120.f; // Preserve the previous small-angle Jacobian.

		} else {
			const float theta = sqrtf(t2), sine = sinf(theta), cosine = cosf(theta);
			sinc = sine / theta;
			cosc = (1.f - cosine) / t2;
			jc = (theta - sine) / (t2 * theta);
		}

		const Matrix3f gx = identity - K * sinc + K2 * cosc;
		const Matrix3f gf = identity - K * (t2 < .01f ? (.5f - t2 / 24.f) : cosc) + K2 * jc;
		// Nominal on-manifold model: c=0; retain Exp(-h*w_d) and the SO(3) right Jacobian.
		const float thrust_step = p.dt * ref.thrust;

		for (int i = 0; i < 3; i++) {
			p.b[k][(i + 3) * 4] = -p.dt * ref.rotation(i, 2);

			for (int j = 0; j < 3; j++) {
				// R*hat(e3) has columns [R_y, -R_x, 0].
				p.a[k][(i + 3) * 9 + j + 6] = j == 0 ? ref.rotation(i, 1) * thrust_step : j == 1 ? -ref.rotation(i, 0) * thrust_step : 0.f;
				p.a[k][(i + 6) * 9 + j + 6] = gx(i, j);
				p.b[k][(i + 6) * 4 + j + 1] = p.dt * gf(i, j);
			}

			p.lower[k][i + 1] = -rate_limit(i) - ref.rate(i);
			p.upper[k][i + 1] = rate_limit(i) - ref.rate(i);
		}

		p.lower[k][0] = -ref.thrust;
		p.upper[k][0] = max_thrust - ref.thrust;
	}

	return true;
}
}
