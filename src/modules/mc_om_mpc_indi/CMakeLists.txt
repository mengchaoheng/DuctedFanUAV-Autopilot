############################################################################
# Copyright 2026 Chaoheng Meng
# SPDX-License-Identifier: BSD-3-Clause
############################################################################

px4_add_module(
	MODULE modules__mc_om_mpc_indi
	MAIN mc_om_mpc_indi
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		McOmMpcIndi.cpp
		McOmMpcIndi.hpp
		OmMpcIndiControl.cpp
		OmMpcIndiControl.hpp
	MODULE_CONFIG
		mc_om_mpc_indi_params.yaml
	DEPENDS
		mathlib
		px4_work_queue
	)
