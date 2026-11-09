############################################################################
# Copyright 2026 Chaoheng Meng
# SPDX-License-Identifier: BSD-3-Clause
############################################################################

px4_add_module(
	MODULE modules__mc_om_mpc_acc_indi
	MAIN mc_om_mpc_acc_indi
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		McOmMpcAccIndi.cpp
		McOmMpcAccIndi.hpp
		OmMpcAccIndiControl.cpp
		OmMpcAccIndiControl.hpp
	MODULE_CONFIG
		mc_om_mpc_acc_indi_params.yaml
	DEPENDS
		mathlib
		px4_work_queue
	)

px4_add_unit_gtest(
	SRC OmMpcAccIndiControlTest.cpp
	EXTRA_SRCS OmMpcAccIndiControl.cpp
)
