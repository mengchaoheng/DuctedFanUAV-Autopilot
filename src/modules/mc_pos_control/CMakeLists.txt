############################################################################
#
#   Copyright (c) 2015-2020 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(GotoControl)
add_subdirectory(PositionControl)
add_subdirectory(Takeoff)

set(om_sources)
set(om_depends)
set(om_flags)

if(CONFIG_MC_POS_CONTROL_OM_MPC)
	add_subdirectory(OmMpc)
	list(APPEND om_depends om_qpdunes)

	list(APPEND om_sources OmMpc/OmMpc.cpp OmMpc/OmSolverRuntime.cpp)
	set_source_files_properties(OmMpc/OmMpc.cpp PROPERTIES COMPILE_OPTIONS "-O3")
	list(APPEND om_flags -DMC_POS_OM_MPC)
endif()

px4_add_module(
	MODULE modules__mc_pos_control
	MAIN mc_pos_control
	COMPILE_FLAGS ${om_flags}
	SRCS ${om_sources}
		MulticopterPositionControl.cpp
		MulticopterPositionControl.hpp
	MODULE_CONFIG
		multicopter_om_mpc_params.yaml
		multicopter_altitude_mode_params.yaml
		multicopter_autonomous_params.yaml
		multicopter_nudging_params.yaml
		multicopter_position_control_gain_params.yaml
		multicopter_position_control_limits_params.yaml
		multicopter_position_control_params.yaml
		multicopter_position_mode_params.yaml
		multicopter_responsiveness_params.yaml
		multicopter_stabilized_mode_params.yaml
		multicopter_takeoff_land_params.yaml
		multicopter_acceleration_indi_params.yaml
	DEPENDS ${om_depends}
		GotoControl
		PositionControl
		Takeoff
		controllib
		geo
		SlewRate
	)
