/* Include files */

#include <stddef.h>
#include "blas.h"
#include "kalman_verifier_sfun.h"
#include "c5_kalman_verifier.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "kalman_verifier_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c5_debug_family_names[18] = { "u", "A", "B", "P", "Q", "R",
  "x_next", "cov_next", "K", "nargin", "nargout", "in", "z", "confidence", "x",
  "cov", "x_corred", "cov_corred" };

/* Function Declarations */
static void initialize_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance);
static void initialize_params_c5_kalman_verifier
  (SFc5_kalman_verifierInstanceStruct *chartInstance);
static void enable_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance);
static void disable_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance);
static void c5_update_debugger_state_c5_kalman_verifier
  (SFc5_kalman_verifierInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_kalman_verifier
  (SFc5_kalman_verifierInstanceStruct *chartInstance);
static void set_sim_state_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct *
  chartInstance, const mxArray *c5_st);
static void finalize_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance);
static void sf_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance);
static void c5_chartstep_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance);
static void initSimStructsc5_kalman_verifier(SFc5_kalman_verifierInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static void c5_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_cov_corred, const char_T *c5_identifier,
  real_T c5_y[25]);
static void c5_b_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[25]);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_c_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_x_corred, const char_T *c5_identifier,
  real_T c5_y[5]);
static void c5_d_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[5]);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static real_T c5_e_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_e_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_info_helper(const mxArray **c5_info);
static const mxArray *c5_emlrt_marshallOut(char * c5_u);
static const mxArray *c5_b_emlrt_marshallOut(uint32_T c5_u);
static void c5_eml_scalar_eg(SFc5_kalman_verifierInstanceStruct *chartInstance);
static void c5_b_eml_scalar_eg(SFc5_kalman_verifierInstanceStruct *chartInstance);
static void c5_c_eml_scalar_eg(SFc5_kalman_verifierInstanceStruct *chartInstance);
static void c5_d_eml_scalar_eg(SFc5_kalman_verifierInstanceStruct *chartInstance);
static const mxArray *c5_f_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_f_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static uint8_T c5_g_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_kalman_verifier, const char_T
  *c5_identifier);
static uint8_T c5_h_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void init_dsm_address_info(SFc5_kalman_verifierInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c5_is_active_c5_kalman_verifier = 0U;
}

static void initialize_params_c5_kalman_verifier
  (SFc5_kalman_verifierInstanceStruct *chartInstance)
{
}

static void enable_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c5_update_debugger_state_c5_kalman_verifier
  (SFc5_kalman_verifierInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c5_kalman_verifier
  (SFc5_kalman_verifierInstanceStruct *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  int32_T c5_i0;
  real_T c5_u[25];
  const mxArray *c5_b_y = NULL;
  int32_T c5_i1;
  real_T c5_b_u[5];
  const mxArray *c5_c_y = NULL;
  uint8_T c5_hoistedGlobal;
  uint8_T c5_c_u;
  const mxArray *c5_d_y = NULL;
  real_T (*c5_x_corred)[5];
  real_T (*c5_cov_corred)[25];
  c5_cov_corred = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 2);
  c5_x_corred = (real_T (*)[5])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellarray(3), FALSE);
  for (c5_i0 = 0; c5_i0 < 25; c5_i0++) {
    c5_u[c5_i0] = (*c5_cov_corred)[c5_i0];
  }

  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 5, 5), FALSE);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  for (c5_i1 = 0; c5_i1 < 5; c5_i1++) {
    c5_b_u[c5_i1] = (*c5_x_corred)[c5_i1];
  }

  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", c5_b_u, 0, 0U, 1U, 0U, 1, 5), FALSE);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  c5_hoistedGlobal = chartInstance->c5_is_active_c5_kalman_verifier;
  c5_c_u = c5_hoistedGlobal;
  c5_d_y = NULL;
  sf_mex_assign(&c5_d_y, sf_mex_create("y", &c5_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c5_y, 2, c5_d_y);
  sf_mex_assign(&c5_st, c5_y, FALSE);
  return c5_st;
}

static void set_sim_state_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct *
  chartInstance, const mxArray *c5_st)
{
  const mxArray *c5_u;
  real_T c5_dv0[25];
  int32_T c5_i2;
  real_T c5_dv1[5];
  int32_T c5_i3;
  real_T (*c5_cov_corred)[25];
  real_T (*c5_x_corred)[5];
  c5_cov_corred = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 2);
  c5_x_corred = (real_T (*)[5])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c5_doneDoubleBufferReInit = TRUE;
  c5_u = sf_mex_dup(c5_st);
  c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 0)),
                      "cov_corred", c5_dv0);
  for (c5_i2 = 0; c5_i2 < 25; c5_i2++) {
    (*c5_cov_corred)[c5_i2] = c5_dv0[c5_i2];
  }

  c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 1)),
                        "x_corred", c5_dv1);
  for (c5_i3 = 0; c5_i3 < 5; c5_i3++) {
    (*c5_x_corred)[c5_i3] = c5_dv1[c5_i3];
  }

  chartInstance->c5_is_active_c5_kalman_verifier = c5_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 2)),
     "is_active_c5_kalman_verifier");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_kalman_verifier(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance)
{
}

static void sf_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance)
{
  int32_T c5_i4;
  int32_T c5_i5;
  int32_T c5_i6;
  int32_T c5_i7;
  int32_T c5_i8;
  real_T *c5_z;
  real_T *c5_confidence;
  real_T (*c5_cov)[25];
  real_T (*c5_x)[5];
  real_T (*c5_cov_corred)[25];
  real_T (*c5_x_corred)[5];
  real_T (*c5_in)[3];
  c5_cov = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 4);
  c5_x = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 3);
  c5_cov_corred = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 2);
  c5_x_corred = (real_T (*)[5])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_confidence = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c5_z = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_in = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  for (c5_i4 = 0; c5_i4 < 3; c5_i4++) {
    _SFD_DATA_RANGE_CHECK((*c5_in)[c5_i4], 0U);
  }

  _SFD_DATA_RANGE_CHECK(*c5_z, 1U);
  _SFD_DATA_RANGE_CHECK(*c5_confidence, 2U);
  for (c5_i5 = 0; c5_i5 < 5; c5_i5++) {
    _SFD_DATA_RANGE_CHECK((*c5_x_corred)[c5_i5], 3U);
  }

  for (c5_i6 = 0; c5_i6 < 25; c5_i6++) {
    _SFD_DATA_RANGE_CHECK((*c5_cov_corred)[c5_i6], 4U);
  }

  for (c5_i7 = 0; c5_i7 < 5; c5_i7++) {
    _SFD_DATA_RANGE_CHECK((*c5_x)[c5_i7], 5U);
  }

  for (c5_i8 = 0; c5_i8 < 25; c5_i8++) {
    _SFD_DATA_RANGE_CHECK((*c5_cov)[c5_i8], 6U);
  }

  chartInstance->c5_sfEvent = CALL_EVENT;
  c5_chartstep_c5_kalman_verifier(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_kalman_verifierMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c5_chartstep_c5_kalman_verifier(SFc5_kalman_verifierInstanceStruct
  *chartInstance)
{
  real_T c5_hoistedGlobal;
  real_T c5_b_hoistedGlobal;
  int32_T c5_i9;
  real_T c5_in[3];
  real_T c5_z;
  real_T c5_confidence;
  int32_T c5_i10;
  real_T c5_x[5];
  int32_T c5_i11;
  real_T c5_cov[25];
  uint32_T c5_debug_family_var_map[18];
  real_T c5_u;
  real_T c5_A[25];
  real_T c5_B[5];
  real_T c5_P[5];
  real_T c5_Q;
  real_T c5_R[25];
  real_T c5_x_next[5];
  real_T c5_cov_next[25];
  real_T c5_K[5];
  real_T c5_nargin = 5.0;
  real_T c5_nargout = 2.0;
  real_T c5_x_corred[5];
  real_T c5_cov_corred[25];
  real_T c5_b_A;
  real_T c5_b_x;
  real_T c5_c_x;
  real_T c5_y;
  real_T c5_a;
  real_T c5_b_y;
  int32_T c5_i12;
  static real_T c5_b_a[25] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.05, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.05, 0.0, -0.74291, 0.0, 0.0, 0.0, 1.0, 1.67972, 0.0, 0.0, 0.0, 1.0,
    0.0, 1.0 };

  int32_T c5_i13;
  static real_T c5_c_a[5] = { 0.0, 0.0, 0.10454, 0.11146, 0.0 };

  int32_T c5_i14;
  static real_T c5_d_a[5] = { 1.0, 0.0, 0.0, 0.0, 0.0 };

  real_T c5_b_B;
  real_T c5_c_y;
  real_T c5_d_y;
  int32_T c5_i15;
  static real_T c5_dv2[25] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c5_i16;
  real_T c5_b[5];
  int32_T c5_i17;
  real_T c5_e_y[5];
  int32_T c5_i18;
  int32_T c5_i19;
  real_T c5_b_b;
  int32_T c5_i20;
  int32_T c5_i21;
  int32_T c5_i22;
  real_T c5_c_b[25];
  int32_T c5_i23;
  int32_T c5_i24;
  int32_T c5_i25;
  real_T c5_f_y[25];
  int32_T c5_i26;
  int32_T c5_i27;
  int32_T c5_i28;
  int32_T c5_i29;
  int32_T c5_i30;
  real_T c5_g_y[25];
  int32_T c5_i31;
  int32_T c5_i32;
  static real_T c5_d_b[25] = { 1.0, 0.05, 0.0, 0.0, 0.0, 0.0, 1.0, 0.05, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, -0.74291, 1.67972, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0 };

  int32_T c5_i33;
  int32_T c5_i34;
  int32_T c5_i35;
  int32_T c5_i36;
  int32_T c5_i37;
  static real_T c5_e_b[5] = { 1.0, 0.0, 0.0, 0.0, 0.0 };

  int32_T c5_i38;
  int32_T c5_i39;
  int32_T c5_i40;
  real_T c5_h_y[5];
  int32_T c5_i41;
  real_T c5_i_y;
  int32_T c5_k;
  int32_T c5_b_k;
  real_T c5_c_B;
  real_T c5_j_y;
  real_T c5_k_y;
  int32_T c5_i42;
  int32_T c5_i43;
  real_T c5_l_y;
  int32_T c5_c_k;
  int32_T c5_d_k;
  int32_T c5_i44;
  real_T c5_f_b;
  int32_T c5_i45;
  int32_T c5_i46;
  int32_T c5_i47;
  real_T c5_I[25];
  int32_T c5_e_k;
  int32_T c5_f_k;
  int32_T c5_i48;
  int32_T c5_i49;
  int32_T c5_i50;
  int32_T c5_i51;
  int32_T c5_i52;
  int32_T c5_i53;
  int32_T c5_i54;
  int32_T c5_i55;
  int32_T c5_i56;
  int32_T c5_i57;
  int32_T c5_i58;
  int32_T c5_i59;
  int32_T c5_i60;
  int32_T c5_i61;
  int32_T c5_i62;
  int32_T c5_i63;
  int32_T c5_i64;
  int32_T c5_i65;
  int32_T c5_i66;
  int32_T c5_i67;
  int32_T c5_i68;
  real_T *c5_b_z;
  real_T *c5_b_confidence;
  real_T (*c5_b_x_corred)[5];
  real_T (*c5_b_cov_corred)[25];
  real_T (*c5_b_cov)[25];
  real_T (*c5_d_x)[5];
  real_T (*c5_b_in)[3];
  c5_b_cov = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 4);
  c5_d_x = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 3);
  c5_b_cov_corred = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S, 2);
  c5_b_x_corred = (real_T (*)[5])ssGetOutputPortSignal(chartInstance->S, 1);
  c5_b_confidence = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c5_b_z = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_b_in = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  c5_hoistedGlobal = *c5_b_z;
  c5_b_hoistedGlobal = *c5_b_confidence;
  for (c5_i9 = 0; c5_i9 < 3; c5_i9++) {
    c5_in[c5_i9] = (*c5_b_in)[c5_i9];
  }

  c5_z = c5_hoistedGlobal;
  c5_confidence = c5_b_hoistedGlobal;
  for (c5_i10 = 0; c5_i10 < 5; c5_i10++) {
    c5_x[c5_i10] = (*c5_d_x)[c5_i10];
  }

  for (c5_i11 = 0; c5_i11 < 25; c5_i11++) {
    c5_cov[c5_i11] = (*c5_b_cov)[c5_i11];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 18U, 18U, c5_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_u, 0U, c5_c_sf_marshallOut,
    c5_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_A, 1U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_B, 2U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_P, 3U, c5_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_Q, 4U, c5_c_sf_marshallOut,
    c5_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_R, 5U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_x_next, 6U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_cov_next, 7U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_K, 8U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 9U, c5_c_sf_marshallOut,
    c5_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 10U, c5_c_sf_marshallOut,
    c5_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_in, 11U, c5_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_z, 12U, c5_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_confidence, 13U, c5_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_x, 14U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_cov, 15U, c5_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_x_corred, 16U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_cov_corred, 17U, c5_sf_marshallOut,
    c5_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 2);
  c5_b_A = c5_in[0];
  c5_b_x = c5_b_A;
  c5_c_x = c5_b_x;
  c5_y = c5_c_x / 100.0;
  c5_a = c5_in[1];
  c5_b_y = c5_a * 2.3;
  c5_u = c5_y + c5_b_y;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 3);
  for (c5_i12 = 0; c5_i12 < 25; c5_i12++) {
    c5_A[c5_i12] = c5_b_a[c5_i12];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 4);
  for (c5_i13 = 0; c5_i13 < 5; c5_i13++) {
    c5_B[c5_i13] = c5_c_a[c5_i13];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 5);
  for (c5_i14 = 0; c5_i14 < 5; c5_i14++) {
    c5_P[c5_i14] = c5_d_a[c5_i14];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 6);
  c5_b_B = c5_confidence;
  c5_c_y = c5_b_B;
  c5_d_y = c5_c_y;
  c5_Q = 100.0 / c5_d_y;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 7);
  for (c5_i15 = 0; c5_i15 < 25; c5_i15++) {
    c5_R[c5_i15] = c5_dv2[c5_i15];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 10);
  for (c5_i16 = 0; c5_i16 < 5; c5_i16++) {
    c5_b[c5_i16] = c5_x[c5_i16];
  }

  c5_eml_scalar_eg(chartInstance);
  c5_eml_scalar_eg(chartInstance);
  for (c5_i17 = 0; c5_i17 < 5; c5_i17++) {
    c5_e_y[c5_i17] = 0.0;
    c5_i18 = 0;
    for (c5_i19 = 0; c5_i19 < 5; c5_i19++) {
      c5_e_y[c5_i17] += c5_b_a[c5_i18 + c5_i17] * c5_b[c5_i19];
      c5_i18 += 5;
    }
  }

  c5_b_b = c5_u;
  for (c5_i20 = 0; c5_i20 < 5; c5_i20++) {
    c5_b[c5_i20] = c5_c_a[c5_i20] * c5_b_b;
  }

  for (c5_i21 = 0; c5_i21 < 5; c5_i21++) {
    c5_x_next[c5_i21] = c5_e_y[c5_i21] + c5_b[c5_i21];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 11);
  for (c5_i22 = 0; c5_i22 < 25; c5_i22++) {
    c5_c_b[c5_i22] = c5_cov[c5_i22];
  }

  c5_b_eml_scalar_eg(chartInstance);
  c5_b_eml_scalar_eg(chartInstance);
  for (c5_i23 = 0; c5_i23 < 5; c5_i23++) {
    c5_i24 = 0;
    for (c5_i25 = 0; c5_i25 < 5; c5_i25++) {
      c5_f_y[c5_i24 + c5_i23] = 0.0;
      c5_i26 = 0;
      for (c5_i27 = 0; c5_i27 < 5; c5_i27++) {
        c5_f_y[c5_i24 + c5_i23] += c5_b_a[c5_i26 + c5_i23] * c5_c_b[c5_i27 +
          c5_i24];
        c5_i26 += 5;
      }

      c5_i24 += 5;
    }
  }

  c5_b_eml_scalar_eg(chartInstance);
  c5_b_eml_scalar_eg(chartInstance);
  for (c5_i28 = 0; c5_i28 < 5; c5_i28++) {
    c5_i29 = 0;
    for (c5_i30 = 0; c5_i30 < 5; c5_i30++) {
      c5_g_y[c5_i29 + c5_i28] = 0.0;
      c5_i31 = 0;
      for (c5_i32 = 0; c5_i32 < 5; c5_i32++) {
        c5_g_y[c5_i29 + c5_i28] += c5_f_y[c5_i31 + c5_i28] * c5_d_b[c5_i32 +
          c5_i29];
        c5_i31 += 5;
      }

      c5_i29 += 5;
    }
  }

  for (c5_i33 = 0; c5_i33 < 25; c5_i33++) {
    c5_cov_next[c5_i33] = c5_g_y[c5_i33] + c5_R[c5_i33];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 14);
  if (CV_EML_IF(0, 1, 0, c5_confidence > 0.001)) {
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 15);
    for (c5_i34 = 0; c5_i34 < 25; c5_i34++) {
      c5_g_y[c5_i34] = c5_cov_next[c5_i34];
    }

    c5_eml_scalar_eg(chartInstance);
    c5_eml_scalar_eg(chartInstance);
    for (c5_i35 = 0; c5_i35 < 5; c5_i35++) {
      c5_e_y[c5_i35] = 0.0;
      c5_i36 = 0;
      for (c5_i37 = 0; c5_i37 < 5; c5_i37++) {
        c5_e_y[c5_i35] += c5_g_y[c5_i36 + c5_i35] * c5_e_b[c5_i37];
        c5_i36 += 5;
      }
    }

    for (c5_i38 = 0; c5_i38 < 25; c5_i38++) {
      c5_c_b[c5_i38] = c5_cov_next[c5_i38];
    }

    c5_c_eml_scalar_eg(chartInstance);
    c5_c_eml_scalar_eg(chartInstance);
    c5_i39 = 0;
    for (c5_i40 = 0; c5_i40 < 5; c5_i40++) {
      c5_h_y[c5_i40] = 0.0;
      for (c5_i41 = 0; c5_i41 < 5; c5_i41++) {
        c5_h_y[c5_i40] += c5_d_a[c5_i41] * c5_c_b[c5_i41 + c5_i39];
      }

      c5_i39 += 5;
    }

    c5_d_eml_scalar_eg(chartInstance);
    c5_d_eml_scalar_eg(chartInstance);
    c5_i_y = 0.0;
    for (c5_k = 1; c5_k < 6; c5_k++) {
      c5_b_k = c5_k;
      c5_i_y += c5_h_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c5_b_k), 1, 5, 1, 0) - 1] *
        c5_e_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c5_b_k), 1, 5, 1, 0) - 1];
    }

    c5_c_B = c5_i_y + c5_Q;
    c5_j_y = c5_c_B;
    c5_k_y = c5_j_y;
    for (c5_i42 = 0; c5_i42 < 5; c5_i42++) {
      c5_K[c5_i42] = c5_e_y[c5_i42] / c5_k_y;
    }

    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 16);
    for (c5_i43 = 0; c5_i43 < 5; c5_i43++) {
      c5_b[c5_i43] = c5_x_next[c5_i43];
    }

    c5_d_eml_scalar_eg(chartInstance);
    c5_d_eml_scalar_eg(chartInstance);
    c5_l_y = 0.0;
    for (c5_c_k = 1; c5_c_k < 6; c5_c_k++) {
      c5_d_k = c5_c_k;
      c5_l_y += c5_e_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c5_d_k), 1, 5, 1, 0) - 1] *
        c5_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c5_d_k), 1, 5, 1, 0) - 1];
    }

    for (c5_i44 = 0; c5_i44 < 5; c5_i44++) {
      c5_b[c5_i44] = c5_K[c5_i44];
    }

    c5_f_b = c5_z - c5_l_y;
    for (c5_i45 = 0; c5_i45 < 5; c5_i45++) {
      c5_b[c5_i45] *= c5_f_b;
    }

    for (c5_i46 = 0; c5_i46 < 5; c5_i46++) {
      c5_x_corred[c5_i46] = c5_x_next[c5_i46] + c5_b[c5_i46];
    }

    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 17);
    for (c5_i47 = 0; c5_i47 < 25; c5_i47++) {
      c5_I[c5_i47] = 0.0;
    }

    for (c5_e_k = 1; c5_e_k < 6; c5_e_k++) {
      c5_f_k = c5_e_k;
      c5_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c5_f_k), 1, 5, 1, 0) + 5 * (_SFD_EML_ARRAY_BOUNDS_CHECK("",
              (int32_T)_SFD_INTEGER_CHECK("", (real_T)c5_f_k), 1, 5, 2, 0) - 1))
        - 1] = 1.0;
    }

    for (c5_i48 = 0; c5_i48 < 5; c5_i48++) {
      c5_b[c5_i48] = c5_K[c5_i48];
    }

    for (c5_i49 = 0; c5_i49 < 5; c5_i49++) {
      c5_i50 = 0;
      for (c5_i51 = 0; c5_i51 < 5; c5_i51++) {
        c5_f_y[c5_i50 + c5_i49] = c5_b[c5_i49] * c5_d_a[c5_i51];
        c5_i50 += 5;
      }
    }

    for (c5_i52 = 0; c5_i52 < 25; c5_i52++) {
      c5_I[c5_i52] -= c5_f_y[c5_i52];
    }

    for (c5_i53 = 0; c5_i53 < 25; c5_i53++) {
      c5_c_b[c5_i53] = c5_cov_next[c5_i53];
    }

    c5_b_eml_scalar_eg(chartInstance);
    c5_b_eml_scalar_eg(chartInstance);
    for (c5_i54 = 0; c5_i54 < 25; c5_i54++) {
      c5_cov_corred[c5_i54] = 0.0;
    }

    for (c5_i55 = 0; c5_i55 < 25; c5_i55++) {
      c5_cov_corred[c5_i55] = 0.0;
    }

    for (c5_i56 = 0; c5_i56 < 25; c5_i56++) {
      c5_g_y[c5_i56] = c5_cov_corred[c5_i56];
    }

    for (c5_i57 = 0; c5_i57 < 25; c5_i57++) {
      c5_cov_corred[c5_i57] = c5_g_y[c5_i57];
    }

    for (c5_i58 = 0; c5_i58 < 25; c5_i58++) {
      c5_g_y[c5_i58] = c5_cov_corred[c5_i58];
    }

    for (c5_i59 = 0; c5_i59 < 25; c5_i59++) {
      c5_cov_corred[c5_i59] = c5_g_y[c5_i59];
    }

    for (c5_i60 = 0; c5_i60 < 5; c5_i60++) {
      c5_i61 = 0;
      for (c5_i62 = 0; c5_i62 < 5; c5_i62++) {
        c5_cov_corred[c5_i61 + c5_i60] = 0.0;
        c5_i63 = 0;
        for (c5_i64 = 0; c5_i64 < 5; c5_i64++) {
          c5_cov_corred[c5_i61 + c5_i60] += c5_I[c5_i63 + c5_i60] *
            c5_c_b[c5_i64 + c5_i61];
          c5_i63 += 5;
        }

        c5_i61 += 5;
      }
    }
  } else {
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 19);
    for (c5_i65 = 0; c5_i65 < 5; c5_i65++) {
      c5_x_corred[c5_i65] = c5_x_next[c5_i65];
    }

    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 20);
    for (c5_i66 = 0; c5_i66 < 25; c5_i66++) {
      c5_cov_corred[c5_i66] = c5_cov_next[c5_i66];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -20);
  _SFD_SYMBOL_SCOPE_POP();
  for (c5_i67 = 0; c5_i67 < 5; c5_i67++) {
    (*c5_b_x_corred)[c5_i67] = c5_x_corred[c5_i67];
  }

  for (c5_i68 = 0; c5_i68 < 25; c5_i68++) {
    (*c5_b_cov_corred)[c5_i68] = c5_cov_corred[c5_i68];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
}

static void initSimStructsc5_kalman_verifier(SFc5_kalman_verifierInstanceStruct *
  chartInstance)
{
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber)
{
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i69;
  int32_T c5_i70;
  int32_T c5_i71;
  real_T c5_b_inData[25];
  int32_T c5_i72;
  int32_T c5_i73;
  int32_T c5_i74;
  real_T c5_u[25];
  const mxArray *c5_y = NULL;
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i69 = 0;
  for (c5_i70 = 0; c5_i70 < 5; c5_i70++) {
    for (c5_i71 = 0; c5_i71 < 5; c5_i71++) {
      c5_b_inData[c5_i71 + c5_i69] = (*(real_T (*)[25])c5_inData)[c5_i71 +
        c5_i69];
    }

    c5_i69 += 5;
  }

  c5_i72 = 0;
  for (c5_i73 = 0; c5_i73 < 5; c5_i73++) {
    for (c5_i74 = 0; c5_i74 < 5; c5_i74++) {
      c5_u[c5_i74 + c5_i72] = c5_b_inData[c5_i74 + c5_i72];
    }

    c5_i72 += 5;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 5, 5), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static void c5_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_cov_corred, const char_T *c5_identifier,
  real_T c5_y[25])
{
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_cov_corred), &c5_thisId,
                        c5_y);
  sf_mex_destroy(&c5_cov_corred);
}

static void c5_b_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[25])
{
  real_T c5_dv3[25];
  int32_T c5_i75;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv3, 1, 0, 0U, 1, 0U, 2, 5, 5);
  for (c5_i75 = 0; c5_i75 < 25; c5_i75++) {
    c5_y[c5_i75] = c5_dv3[c5_i75];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_cov_corred;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[25];
  int32_T c5_i76;
  int32_T c5_i77;
  int32_T c5_i78;
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c5_cov_corred = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_cov_corred), &c5_thisId,
                        c5_y);
  sf_mex_destroy(&c5_cov_corred);
  c5_i76 = 0;
  for (c5_i77 = 0; c5_i77 < 5; c5_i77++) {
    for (c5_i78 = 0; c5_i78 < 5; c5_i78++) {
      (*(real_T (*)[25])c5_outData)[c5_i78 + c5_i76] = c5_y[c5_i78 + c5_i76];
    }

    c5_i76 += 5;
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i79;
  real_T c5_b_inData[5];
  int32_T c5_i80;
  real_T c5_u[5];
  const mxArray *c5_y = NULL;
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i79 = 0; c5_i79 < 5; c5_i79++) {
    c5_b_inData[c5_i79] = (*(real_T (*)[5])c5_inData)[c5_i79];
  }

  for (c5_i80 = 0; c5_i80 < 5; c5_i80++) {
    c5_u[c5_i80] = c5_b_inData[c5_i80];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 5), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static void c5_c_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_x_corred, const char_T *c5_identifier,
  real_T c5_y[5])
{
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_x_corred), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_x_corred);
}

static void c5_d_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[5])
{
  real_T c5_dv4[5];
  int32_T c5_i81;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv4, 1, 0, 0U, 1, 0U, 1, 5);
  for (c5_i81 = 0; c5_i81 < 5; c5_i81++) {
    c5_y[c5_i81] = c5_dv4[c5_i81];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_x_corred;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[5];
  int32_T c5_i82;
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c5_x_corred = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_x_corred), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_x_corred);
  for (c5_i82 = 0; c5_i82 < 5; c5_i82++) {
    (*(real_T (*)[5])c5_outData)[c5_i82] = c5_y[c5_i82];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i83;
  real_T c5_b_inData[3];
  int32_T c5_i84;
  real_T c5_u[3];
  const mxArray *c5_y = NULL;
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i83 = 0; c5_i83 < 3; c5_i83++) {
    c5_b_inData[c5_i83] = (*(real_T (*)[3])c5_inData)[c5_i83];
  }

  for (c5_i84 = 0; c5_i84 < 3; c5_i84++) {
    c5_u[c5_i84] = c5_b_inData[c5_i84];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static real_T c5_e_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d0;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d0, 1, 0, 0U, 0, 0U, 0);
  c5_y = c5_d0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_nargout;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c5_nargout = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_nargout), &c5_thisId);
  sf_mex_destroy(&c5_nargout);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_e_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i85;
  real_T c5_b_inData[5];
  int32_T c5_i86;
  real_T c5_u[5];
  const mxArray *c5_y = NULL;
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i85 = 0; c5_i85 < 5; c5_i85++) {
    c5_b_inData[c5_i85] = (*(real_T (*)[5])c5_inData)[c5_i85];
  }

  for (c5_i86 = 0; c5_i86 < 5; c5_i86++) {
    c5_u[c5_i86] = c5_b_inData[c5_i86];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 1, 5), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

const mxArray *sf_c5_kalman_verifier_get_eml_resolved_functions_info(void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  sf_mex_assign(&c5_nameCaptureInfo, sf_mex_createstruct("structure", 2, 47, 1),
                FALSE);
  c5_info_helper(&c5_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs0 = NULL;
  const mxArray *c5_lhs0 = NULL;
  const mxArray *c5_rhs1 = NULL;
  const mxArray *c5_lhs1 = NULL;
  const mxArray *c5_rhs2 = NULL;
  const mxArray *c5_lhs2 = NULL;
  const mxArray *c5_rhs3 = NULL;
  const mxArray *c5_lhs3 = NULL;
  const mxArray *c5_rhs4 = NULL;
  const mxArray *c5_lhs4 = NULL;
  const mxArray *c5_rhs5 = NULL;
  const mxArray *c5_lhs5 = NULL;
  const mxArray *c5_rhs6 = NULL;
  const mxArray *c5_lhs6 = NULL;
  const mxArray *c5_rhs7 = NULL;
  const mxArray *c5_lhs7 = NULL;
  const mxArray *c5_rhs8 = NULL;
  const mxArray *c5_lhs8 = NULL;
  const mxArray *c5_rhs9 = NULL;
  const mxArray *c5_lhs9 = NULL;
  const mxArray *c5_rhs10 = NULL;
  const mxArray *c5_lhs10 = NULL;
  const mxArray *c5_rhs11 = NULL;
  const mxArray *c5_lhs11 = NULL;
  const mxArray *c5_rhs12 = NULL;
  const mxArray *c5_lhs12 = NULL;
  const mxArray *c5_rhs13 = NULL;
  const mxArray *c5_lhs13 = NULL;
  const mxArray *c5_rhs14 = NULL;
  const mxArray *c5_lhs14 = NULL;
  const mxArray *c5_rhs15 = NULL;
  const mxArray *c5_lhs15 = NULL;
  const mxArray *c5_rhs16 = NULL;
  const mxArray *c5_lhs16 = NULL;
  const mxArray *c5_rhs17 = NULL;
  const mxArray *c5_lhs17 = NULL;
  const mxArray *c5_rhs18 = NULL;
  const mxArray *c5_lhs18 = NULL;
  const mxArray *c5_rhs19 = NULL;
  const mxArray *c5_lhs19 = NULL;
  const mxArray *c5_rhs20 = NULL;
  const mxArray *c5_lhs20 = NULL;
  const mxArray *c5_rhs21 = NULL;
  const mxArray *c5_lhs21 = NULL;
  const mxArray *c5_rhs22 = NULL;
  const mxArray *c5_lhs22 = NULL;
  const mxArray *c5_rhs23 = NULL;
  const mxArray *c5_lhs23 = NULL;
  const mxArray *c5_rhs24 = NULL;
  const mxArray *c5_lhs24 = NULL;
  const mxArray *c5_rhs25 = NULL;
  const mxArray *c5_lhs25 = NULL;
  const mxArray *c5_rhs26 = NULL;
  const mxArray *c5_lhs26 = NULL;
  const mxArray *c5_rhs27 = NULL;
  const mxArray *c5_lhs27 = NULL;
  const mxArray *c5_rhs28 = NULL;
  const mxArray *c5_lhs28 = NULL;
  const mxArray *c5_rhs29 = NULL;
  const mxArray *c5_lhs29 = NULL;
  const mxArray *c5_rhs30 = NULL;
  const mxArray *c5_lhs30 = NULL;
  const mxArray *c5_rhs31 = NULL;
  const mxArray *c5_lhs31 = NULL;
  const mxArray *c5_rhs32 = NULL;
  const mxArray *c5_lhs32 = NULL;
  const mxArray *c5_rhs33 = NULL;
  const mxArray *c5_lhs33 = NULL;
  const mxArray *c5_rhs34 = NULL;
  const mxArray *c5_lhs34 = NULL;
  const mxArray *c5_rhs35 = NULL;
  const mxArray *c5_lhs35 = NULL;
  const mxArray *c5_rhs36 = NULL;
  const mxArray *c5_lhs36 = NULL;
  const mxArray *c5_rhs37 = NULL;
  const mxArray *c5_lhs37 = NULL;
  const mxArray *c5_rhs38 = NULL;
  const mxArray *c5_lhs38 = NULL;
  const mxArray *c5_rhs39 = NULL;
  const mxArray *c5_lhs39 = NULL;
  const mxArray *c5_rhs40 = NULL;
  const mxArray *c5_lhs40 = NULL;
  const mxArray *c5_rhs41 = NULL;
  const mxArray *c5_lhs41 = NULL;
  const mxArray *c5_rhs42 = NULL;
  const mxArray *c5_lhs42 = NULL;
  const mxArray *c5_rhs43 = NULL;
  const mxArray *c5_lhs43 = NULL;
  const mxArray *c5_rhs44 = NULL;
  const mxArray *c5_lhs44 = NULL;
  const mxArray *c5_rhs45 = NULL;
  const mxArray *c5_lhs45 = NULL;
  const mxArray *c5_rhs46 = NULL;
  const mxArray *c5_lhs46 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1373306508U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1319729966U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c5_rhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("rdivide"), "name", "name", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c5_rhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c5_rhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c5_rhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_div"), "name", "name", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713866U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c5_rhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("mtimes"), "name", "name", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c5_rhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m!common_checks"),
                  "context", "context", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c5_rhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c5_rhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c5_rhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xgemm"), "name", "name", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713870U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c5_rhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_blas_inline"), "name",
                  "name", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1299076768U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c5_rhs10, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs10, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold"),
                  "context", "context", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("mtimes"), "name", "name", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c5_rhs11, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs11, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m"),
                  "context", "context", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c5_rhs12, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs12, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m"),
                  "context", "context", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c5_rhs13, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs13, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_refblas_xgemm"), "name",
                  "name", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360282350U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c5_rhs14, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs14, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xdotu"), "name", "name",
                  15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713870U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c5_rhs15, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs15, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_blas_inline"), "name",
                  "name", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1299076768U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c5_rhs16, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs16, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xdot"), "name", "name", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713868U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c5_rhs17, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs17, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_blas_inline"), "name",
                  "name", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1299076768U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c5_rhs18, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs18, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m"),
                  "context", "context", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c5_rhs19, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs19, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m"),
                  "context", "context", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_refblas_xdot"), "name",
                  "name", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1299076772U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c5_rhs20, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs20, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m"),
                  "context", "context", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_refblas_xdotx"), "name",
                  "name", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360282350U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c5_rhs21, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs21, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c5_rhs22, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs22, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c5_rhs23, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs23, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818778U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c5_rhs24, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs24, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c5_rhs25, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs25, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818780U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c5_rhs26, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs26, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c5_rhs27, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs27, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818778U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c5_rhs28, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs28, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c5_rhs29, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs29, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1346510340U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c5_rhs30, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs30, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmax"), "name", "name", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c5_rhs31, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs31, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eye"), "name", "name", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1368183030U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c5_rhs32, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs32, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1368183030U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c5_rhs33, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs33, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c5_rhs34, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs34, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral"),
                  "context", "context", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("isinf"), "name", "name", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c5_rhs35, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs35, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c5_rhs36, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs36, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_is_integer_class"), "name",
                  "name", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_integer_class.m"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286818782U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c5_rhs37, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs37, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmax"), "name", "name", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c5_rhs38, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs38, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmin"), "name", "name", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c5_rhs39, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs39, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326728322U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c5_rhs40, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs40, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c5_rhs41, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs41, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmin"), "name", "name", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c5_rhs42, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs42, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size"),
                  "context", "context", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("mtimes"), "name", "name", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c5_rhs43, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs43, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c5_rhs44, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs44, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmax"), "name", "name", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c5_rhs45, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs45, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1346510340U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c5_rhs46, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c5_lhs46, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs46), "lhs", "lhs",
                  46);
  sf_mex_destroy(&c5_rhs0);
  sf_mex_destroy(&c5_lhs0);
  sf_mex_destroy(&c5_rhs1);
  sf_mex_destroy(&c5_lhs1);
  sf_mex_destroy(&c5_rhs2);
  sf_mex_destroy(&c5_lhs2);
  sf_mex_destroy(&c5_rhs3);
  sf_mex_destroy(&c5_lhs3);
  sf_mex_destroy(&c5_rhs4);
  sf_mex_destroy(&c5_lhs4);
  sf_mex_destroy(&c5_rhs5);
  sf_mex_destroy(&c5_lhs5);
  sf_mex_destroy(&c5_rhs6);
  sf_mex_destroy(&c5_lhs6);
  sf_mex_destroy(&c5_rhs7);
  sf_mex_destroy(&c5_lhs7);
  sf_mex_destroy(&c5_rhs8);
  sf_mex_destroy(&c5_lhs8);
  sf_mex_destroy(&c5_rhs9);
  sf_mex_destroy(&c5_lhs9);
  sf_mex_destroy(&c5_rhs10);
  sf_mex_destroy(&c5_lhs10);
  sf_mex_destroy(&c5_rhs11);
  sf_mex_destroy(&c5_lhs11);
  sf_mex_destroy(&c5_rhs12);
  sf_mex_destroy(&c5_lhs12);
  sf_mex_destroy(&c5_rhs13);
  sf_mex_destroy(&c5_lhs13);
  sf_mex_destroy(&c5_rhs14);
  sf_mex_destroy(&c5_lhs14);
  sf_mex_destroy(&c5_rhs15);
  sf_mex_destroy(&c5_lhs15);
  sf_mex_destroy(&c5_rhs16);
  sf_mex_destroy(&c5_lhs16);
  sf_mex_destroy(&c5_rhs17);
  sf_mex_destroy(&c5_lhs17);
  sf_mex_destroy(&c5_rhs18);
  sf_mex_destroy(&c5_lhs18);
  sf_mex_destroy(&c5_rhs19);
  sf_mex_destroy(&c5_lhs19);
  sf_mex_destroy(&c5_rhs20);
  sf_mex_destroy(&c5_lhs20);
  sf_mex_destroy(&c5_rhs21);
  sf_mex_destroy(&c5_lhs21);
  sf_mex_destroy(&c5_rhs22);
  sf_mex_destroy(&c5_lhs22);
  sf_mex_destroy(&c5_rhs23);
  sf_mex_destroy(&c5_lhs23);
  sf_mex_destroy(&c5_rhs24);
  sf_mex_destroy(&c5_lhs24);
  sf_mex_destroy(&c5_rhs25);
  sf_mex_destroy(&c5_lhs25);
  sf_mex_destroy(&c5_rhs26);
  sf_mex_destroy(&c5_lhs26);
  sf_mex_destroy(&c5_rhs27);
  sf_mex_destroy(&c5_lhs27);
  sf_mex_destroy(&c5_rhs28);
  sf_mex_destroy(&c5_lhs28);
  sf_mex_destroy(&c5_rhs29);
  sf_mex_destroy(&c5_lhs29);
  sf_mex_destroy(&c5_rhs30);
  sf_mex_destroy(&c5_lhs30);
  sf_mex_destroy(&c5_rhs31);
  sf_mex_destroy(&c5_lhs31);
  sf_mex_destroy(&c5_rhs32);
  sf_mex_destroy(&c5_lhs32);
  sf_mex_destroy(&c5_rhs33);
  sf_mex_destroy(&c5_lhs33);
  sf_mex_destroy(&c5_rhs34);
  sf_mex_destroy(&c5_lhs34);
  sf_mex_destroy(&c5_rhs35);
  sf_mex_destroy(&c5_lhs35);
  sf_mex_destroy(&c5_rhs36);
  sf_mex_destroy(&c5_lhs36);
  sf_mex_destroy(&c5_rhs37);
  sf_mex_destroy(&c5_lhs37);
  sf_mex_destroy(&c5_rhs38);
  sf_mex_destroy(&c5_lhs38);
  sf_mex_destroy(&c5_rhs39);
  sf_mex_destroy(&c5_lhs39);
  sf_mex_destroy(&c5_rhs40);
  sf_mex_destroy(&c5_lhs40);
  sf_mex_destroy(&c5_rhs41);
  sf_mex_destroy(&c5_lhs41);
  sf_mex_destroy(&c5_rhs42);
  sf_mex_destroy(&c5_lhs42);
  sf_mex_destroy(&c5_rhs43);
  sf_mex_destroy(&c5_lhs43);
  sf_mex_destroy(&c5_rhs44);
  sf_mex_destroy(&c5_lhs44);
  sf_mex_destroy(&c5_rhs45);
  sf_mex_destroy(&c5_lhs45);
  sf_mex_destroy(&c5_rhs46);
  sf_mex_destroy(&c5_lhs46);
}

static const mxArray *c5_emlrt_marshallOut(char * c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c5_u)), FALSE);
  return c5_y;
}

static const mxArray *c5_b_emlrt_marshallOut(uint32_T c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 7, 0U, 0U, 0U, 0), FALSE);
  return c5_y;
}

static void c5_eml_scalar_eg(SFc5_kalman_verifierInstanceStruct *chartInstance)
{
}

static void c5_b_eml_scalar_eg(SFc5_kalman_verifierInstanceStruct *chartInstance)
{
}

static void c5_c_eml_scalar_eg(SFc5_kalman_verifierInstanceStruct *chartInstance)
{
}

static void c5_d_eml_scalar_eg(SFc5_kalman_verifierInstanceStruct *chartInstance)
{
}

static const mxArray *c5_f_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, FALSE);
  return c5_mxArrayOutData;
}

static int32_T c5_f_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i87;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i87, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i87;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static uint8_T c5_g_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_kalman_verifier, const char_T
  *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_kalman_verifier), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_kalman_verifier);
  return c5_y;
}

static uint8_T c5_h_emlrt_marshallIn(SFc5_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void init_dsm_address_info(SFc5_kalman_verifierInstanceStruct
  *chartInstance)
{
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c5_kalman_verifier_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4016515356U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(721490894U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3729220575U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1782515516U);
}

mxArray *sf_c5_kalman_verifier_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("ZWGHeTnJwql4DYU3mfYCxD");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(5);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(5);
      pr[1] = (double)(5);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(5);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(5);
      pr[1] = (double)(5);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c5_kalman_verifier_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c5_kalman_verifier_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c5_kalman_verifier(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[12],T\"cov_corred\",},{M[1],M[9],T\"x_corred\",},{M[8],M[0],T\"is_active_c5_kalman_verifier\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_kalman_verifier_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_kalman_verifierInstanceStruct *chartInstance;
    chartInstance = (SFc5_kalman_verifierInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _kalman_verifierMachineNumber_,
           5,
           1,
           1,
           7,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation(_kalman_verifierMachineNumber_,
            chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_kalman_verifierMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _kalman_verifierMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"in");
          _SFD_SET_DATA_PROPS(1,1,1,0,"z");
          _SFD_SET_DATA_PROPS(2,1,1,0,"confidence");
          _SFD_SET_DATA_PROPS(3,2,0,1,"x_corred");
          _SFD_SET_DATA_PROPS(4,2,0,1,"cov_corred");
          _SFD_SET_DATA_PROPS(5,1,1,0,"x");
          _SFD_SET_DATA_PROPS(6,1,1,0,"cov");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,1,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,596);
        _SFD_CV_INIT_EML_IF(0,1,0,385,408,538,596);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)
            c5_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 5;
          dimVector[1]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)
            c5_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 5;
          dimVector[1]= 5;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T *c5_z;
          real_T *c5_confidence;
          real_T (*c5_in)[3];
          real_T (*c5_x_corred)[5];
          real_T (*c5_cov_corred)[25];
          real_T (*c5_x)[5];
          real_T (*c5_cov)[25];
          c5_cov = (real_T (*)[25])ssGetInputPortSignal(chartInstance->S, 4);
          c5_x = (real_T (*)[5])ssGetInputPortSignal(chartInstance->S, 3);
          c5_cov_corred = (real_T (*)[25])ssGetOutputPortSignal(chartInstance->S,
            2);
          c5_x_corred = (real_T (*)[5])ssGetOutputPortSignal(chartInstance->S, 1);
          c5_confidence = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c5_z = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c5_in = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c5_in);
          _SFD_SET_DATA_VALUE_PTR(1U, c5_z);
          _SFD_SET_DATA_VALUE_PTR(2U, c5_confidence);
          _SFD_SET_DATA_VALUE_PTR(3U, *c5_x_corred);
          _SFD_SET_DATA_VALUE_PTR(4U, *c5_cov_corred);
          _SFD_SET_DATA_VALUE_PTR(5U, *c5_x);
          _SFD_SET_DATA_VALUE_PTR(6U, *c5_cov);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _kalman_verifierMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "0HfCEgdVO0X3NOXVQDEWZH";
}

static void sf_opaque_initialize_c5_kalman_verifier(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_kalman_verifierInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c5_kalman_verifier((SFc5_kalman_verifierInstanceStruct*)
    chartInstanceVar);
  initialize_c5_kalman_verifier((SFc5_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c5_kalman_verifier(void *chartInstanceVar)
{
  enable_c5_kalman_verifier((SFc5_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c5_kalman_verifier(void *chartInstanceVar)
{
  disable_c5_kalman_verifier((SFc5_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c5_kalman_verifier(void *chartInstanceVar)
{
  sf_c5_kalman_verifier((SFc5_kalman_verifierInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c5_kalman_verifier(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c5_kalman_verifier
    ((SFc5_kalman_verifierInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_kalman_verifier();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c5_kalman_verifier(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_kalman_verifier();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_kalman_verifier((SFc5_kalman_verifierInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c5_kalman_verifier(SimStruct* S)
{
  return sf_internal_get_sim_state_c5_kalman_verifier(S);
}

static void sf_opaque_set_sim_state_c5_kalman_verifier(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c5_kalman_verifier(S, st);
}

static void sf_opaque_terminate_c5_kalman_verifier(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_kalman_verifierInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_kalman_verifier_optimization_info();
    }

    finalize_c5_kalman_verifier((SFc5_kalman_verifierInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc5_kalman_verifier((SFc5_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_kalman_verifier(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c5_kalman_verifier((SFc5_kalman_verifierInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_kalman_verifier(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_kalman_verifier_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,5,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,5);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,5,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3469144829U));
  ssSetChecksum1(S,(953045445U));
  ssSetChecksum2(S,(40067085U));
  ssSetChecksum3(S,(4041880676U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c5_kalman_verifier(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_kalman_verifier(SimStruct *S)
{
  SFc5_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc5_kalman_verifierInstanceStruct *)utMalloc(sizeof
    (SFc5_kalman_verifierInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc5_kalman_verifierInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_kalman_verifier;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_kalman_verifier;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c5_kalman_verifier;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c5_kalman_verifier;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c5_kalman_verifier;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_kalman_verifier;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_kalman_verifier;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_kalman_verifier;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_kalman_verifier;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_kalman_verifier;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_kalman_verifier;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c5_kalman_verifier_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_kalman_verifier(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_kalman_verifier(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_kalman_verifier(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_kalman_verifier_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
