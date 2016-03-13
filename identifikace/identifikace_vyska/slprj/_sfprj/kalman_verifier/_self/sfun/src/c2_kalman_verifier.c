/* Include files */

#include <stddef.h>
#include "blas.h"
#include "kalman_verifier_sfun.h"
#include "c2_kalman_verifier.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "kalman_verifier_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[18] = { "u", "A", "B", "P", "Q", "R",
  "x_next", "cov_next", "K", "nargin", "nargout", "in", "z", "confidence", "x",
  "cov", "x_corred", "cov_corred" };

/* Function Declarations */
static void initialize_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance);
static void initialize_params_c2_kalman_verifier
  (SFc2_kalman_verifierInstanceStruct *chartInstance);
static void enable_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance);
static void disable_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance);
static void c2_update_debugger_state_c2_kalman_verifier
  (SFc2_kalman_verifierInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_kalman_verifier
  (SFc2_kalman_verifierInstanceStruct *chartInstance);
static void set_sim_state_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct *
  chartInstance, const mxArray *c2_st);
static void finalize_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance);
static void sf_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance);
static void c2_chartstep_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance);
static void initSimStructsc2_kalman_verifier(SFc2_kalman_verifierInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static void c2_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_cov_corred, const char_T *c2_identifier,
  real_T c2_y[16]);
static void c2_b_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16]);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_x_corred, const char_T *c2_identifier,
  real_T c2_y[4]);
static void c2_d_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static real_T c2_e_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_info_helper(const mxArray **c2_info);
static const mxArray *c2_emlrt_marshallOut(char * c2_u);
static const mxArray *c2_b_emlrt_marshallOut(uint32_T c2_u);
static void c2_eml_scalar_eg(SFc2_kalman_verifierInstanceStruct *chartInstance);
static void c2_b_eml_scalar_eg(SFc2_kalman_verifierInstanceStruct *chartInstance);
static void c2_c_eml_scalar_eg(SFc2_kalman_verifierInstanceStruct *chartInstance);
static void c2_d_eml_scalar_eg(SFc2_kalman_verifierInstanceStruct *chartInstance);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_f_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_g_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_kalman_verifier, const char_T
  *c2_identifier);
static uint8_T c2_h_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info(SFc2_kalman_verifierInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_kalman_verifier = 0U;
}

static void initialize_params_c2_kalman_verifier
  (SFc2_kalman_verifierInstanceStruct *chartInstance)
{
}

static void enable_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_kalman_verifier
  (SFc2_kalman_verifierInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_kalman_verifier
  (SFc2_kalman_verifierInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[16];
  const mxArray *c2_b_y = NULL;
  int32_T c2_i1;
  real_T c2_b_u[4];
  const mxArray *c2_c_y = NULL;
  uint8_T c2_hoistedGlobal;
  uint8_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T (*c2_x_corred)[4];
  real_T (*c2_cov_corred)[16];
  c2_cov_corred = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_x_corred = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(3), FALSE);
  for (c2_i0 = 0; c2_i0 < 16; c2_i0++) {
    c2_u[c2_i0] = (*c2_cov_corred)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  for (c2_i1 = 0; c2_i1 < 4; c2_i1++) {
    c2_b_u[c2_i1] = (*c2_x_corred)[c2_i1];
  }

  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", c2_b_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_hoistedGlobal = chartInstance->c2_is_active_c2_kalman_verifier;
  c2_c_u = c2_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct *
  chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[16];
  int32_T c2_i2;
  real_T c2_dv1[4];
  int32_T c2_i3;
  real_T (*c2_cov_corred)[16];
  real_T (*c2_x_corred)[4];
  c2_cov_corred = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_x_corred = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
                      "cov_corred", c2_dv0);
  for (c2_i2 = 0; c2_i2 < 16; c2_i2++) {
    (*c2_cov_corred)[c2_i2] = c2_dv0[c2_i2];
  }

  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
                        "x_corred", c2_dv1);
  for (c2_i3 = 0; c2_i3 < 4; c2_i3++) {
    (*c2_x_corred)[c2_i3] = c2_dv1[c2_i3];
  }

  chartInstance->c2_is_active_c2_kalman_verifier = c2_g_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
     "is_active_c2_kalman_verifier");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_kalman_verifier(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance)
{
}

static void sf_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance)
{
  int32_T c2_i4;
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  int32_T c2_i8;
  real_T *c2_z;
  real_T *c2_confidence;
  real_T (*c2_cov)[16];
  real_T (*c2_x)[4];
  real_T (*c2_cov_corred)[16];
  real_T (*c2_x_corred)[4];
  real_T (*c2_in)[3];
  c2_cov = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 4);
  c2_x = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 3);
  c2_cov_corred = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_x_corred = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_confidence = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_z = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_in = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i4 = 0; c2_i4 < 3; c2_i4++) {
    _SFD_DATA_RANGE_CHECK((*c2_in)[c2_i4], 0U);
  }

  _SFD_DATA_RANGE_CHECK(*c2_z, 1U);
  _SFD_DATA_RANGE_CHECK(*c2_confidence, 2U);
  for (c2_i5 = 0; c2_i5 < 4; c2_i5++) {
    _SFD_DATA_RANGE_CHECK((*c2_x_corred)[c2_i5], 3U);
  }

  for (c2_i6 = 0; c2_i6 < 16; c2_i6++) {
    _SFD_DATA_RANGE_CHECK((*c2_cov_corred)[c2_i6], 4U);
  }

  for (c2_i7 = 0; c2_i7 < 4; c2_i7++) {
    _SFD_DATA_RANGE_CHECK((*c2_x)[c2_i7], 5U);
  }

  for (c2_i8 = 0; c2_i8 < 16; c2_i8++) {
    _SFD_DATA_RANGE_CHECK((*c2_cov)[c2_i8], 6U);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_kalman_verifier(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_kalman_verifierMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_kalman_verifier(SFc2_kalman_verifierInstanceStruct
  *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  int32_T c2_i9;
  real_T c2_in[3];
  real_T c2_z;
  real_T c2_confidence;
  int32_T c2_i10;
  real_T c2_x[4];
  int32_T c2_i11;
  real_T c2_cov[16];
  uint32_T c2_debug_family_var_map[18];
  real_T c2_u;
  real_T c2_A[16];
  real_T c2_B[4];
  real_T c2_P[4];
  real_T c2_Q;
  real_T c2_R[16];
  real_T c2_x_next[4];
  real_T c2_cov_next[16];
  real_T c2_K[4];
  real_T c2_nargin = 5.0;
  real_T c2_nargout = 2.0;
  real_T c2_x_corred[4];
  real_T c2_cov_corred[16];
  real_T c2_b_A;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_y;
  real_T c2_a;
  real_T c2_b_y;
  int32_T c2_i12;
  static real_T c2_b_a[16] = { 1.0, 0.0, 0.0, 0.0, 0.05, 1.0, 0.0, 0.0, 0.0,
    0.05, 0.0, -0.742912189150098, 0.0, 0.0, 1.0, 1.6797231283611489 };

  int32_T c2_i13;
  static real_T c2_c_a[4] = { 0.0, 0.0, 0.104540214230006, 0.11145552610717 };

  int32_T c2_i14;
  static real_T c2_d_a[4] = { 1.0, 0.0, 0.0, 0.0 };

  real_T c2_b_B;
  real_T c2_c_y;
  real_T c2_d_y;
  int32_T c2_i15;
  static real_T c2_dv2[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c2_i16;
  real_T c2_b[4];
  int32_T c2_i17;
  real_T c2_e_y[4];
  int32_T c2_i18;
  int32_T c2_i19;
  real_T c2_b_b;
  int32_T c2_i20;
  int32_T c2_i21;
  int32_T c2_i22;
  real_T c2_c_b[16];
  int32_T c2_i23;
  int32_T c2_i24;
  int32_T c2_i25;
  real_T c2_f_y[16];
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  int32_T c2_i29;
  int32_T c2_i30;
  real_T c2_g_y[16];
  int32_T c2_i31;
  int32_T c2_i32;
  static real_T c2_d_b[16] = { 1.0, 0.05, 0.0, 0.0, 0.0, 1.0, 0.05, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, -0.742912189150098, 1.6797231283611489 };

  int32_T c2_i33;
  int32_T c2_i34;
  int32_T c2_i35;
  int32_T c2_i36;
  int32_T c2_i37;
  static real_T c2_e_b[4] = { 1.0, 0.0, 0.0, 0.0 };

  int32_T c2_i38;
  int32_T c2_i39;
  int32_T c2_i40;
  real_T c2_h_y[4];
  int32_T c2_i41;
  real_T c2_i_y;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_c_B;
  real_T c2_j_y;
  real_T c2_k_y;
  int32_T c2_i42;
  int32_T c2_i43;
  real_T c2_l_y;
  int32_T c2_c_k;
  int32_T c2_d_k;
  int32_T c2_i44;
  real_T c2_f_b;
  int32_T c2_i45;
  int32_T c2_i46;
  int32_T c2_i47;
  real_T c2_I[16];
  int32_T c2_e_k;
  int32_T c2_f_k;
  int32_T c2_i48;
  int32_T c2_i49;
  int32_T c2_i50;
  int32_T c2_i51;
  int32_T c2_i52;
  int32_T c2_i53;
  int32_T c2_i54;
  int32_T c2_i55;
  int32_T c2_i56;
  int32_T c2_i57;
  int32_T c2_i58;
  int32_T c2_i59;
  int32_T c2_i60;
  int32_T c2_i61;
  int32_T c2_i62;
  int32_T c2_i63;
  int32_T c2_i64;
  int32_T c2_i65;
  int32_T c2_i66;
  int32_T c2_i67;
  int32_T c2_i68;
  real_T *c2_b_z;
  real_T *c2_b_confidence;
  real_T (*c2_b_x_corred)[4];
  real_T (*c2_b_cov_corred)[16];
  real_T (*c2_b_cov)[16];
  real_T (*c2_d_x)[4];
  real_T (*c2_b_in)[3];
  c2_b_cov = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 4);
  c2_d_x = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 3);
  c2_b_cov_corred = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S, 2);
  c2_b_x_corred = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_b_confidence = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_b_z = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c2_b_in = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_z;
  c2_b_hoistedGlobal = *c2_b_confidence;
  for (c2_i9 = 0; c2_i9 < 3; c2_i9++) {
    c2_in[c2_i9] = (*c2_b_in)[c2_i9];
  }

  c2_z = c2_hoistedGlobal;
  c2_confidence = c2_b_hoistedGlobal;
  for (c2_i10 = 0; c2_i10 < 4; c2_i10++) {
    c2_x[c2_i10] = (*c2_d_x)[c2_i10];
  }

  for (c2_i11 = 0; c2_i11 < 16; c2_i11++) {
    c2_cov[c2_i11] = (*c2_b_cov)[c2_i11];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 18U, 18U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_u, 0U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_A, 1U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_B, 2U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_P, 3U, c2_e_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Q, 4U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_R, 5U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_x_next, 6U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_cov_next, 7U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_K, 8U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 9U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 10U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_in, 11U, c2_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_z, 12U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_confidence, 13U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_x, 14U, c2_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_cov, 15U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_x_corred, 16U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_cov_corred, 17U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 2);
  c2_b_A = c2_in[0];
  c2_b_x = c2_b_A;
  c2_c_x = c2_b_x;
  c2_y = c2_c_x / 100.0;
  c2_a = c2_in[1];
  c2_b_y = c2_a * 2.3;
  c2_u = (c2_y + c2_b_y) - 91.65;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 3);
  for (c2_i12 = 0; c2_i12 < 16; c2_i12++) {
    c2_A[c2_i12] = c2_b_a[c2_i12];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  for (c2_i13 = 0; c2_i13 < 4; c2_i13++) {
    c2_B[c2_i13] = c2_c_a[c2_i13];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  for (c2_i14 = 0; c2_i14 < 4; c2_i14++) {
    c2_P[c2_i14] = c2_d_a[c2_i14];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  c2_b_B = c2_confidence;
  c2_c_y = c2_b_B;
  c2_d_y = c2_c_y;
  c2_Q = 100.0 / c2_d_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  for (c2_i15 = 0; c2_i15 < 16; c2_i15++) {
    c2_R[c2_i15] = c2_dv2[c2_i15];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  for (c2_i16 = 0; c2_i16 < 4; c2_i16++) {
    c2_b[c2_i16] = c2_x[c2_i16];
  }

  c2_eml_scalar_eg(chartInstance);
  c2_eml_scalar_eg(chartInstance);
  for (c2_i17 = 0; c2_i17 < 4; c2_i17++) {
    c2_e_y[c2_i17] = 0.0;
    c2_i18 = 0;
    for (c2_i19 = 0; c2_i19 < 4; c2_i19++) {
      c2_e_y[c2_i17] += c2_b_a[c2_i18 + c2_i17] * c2_b[c2_i19];
      c2_i18 += 4;
    }
  }

  c2_b_b = c2_u;
  for (c2_i20 = 0; c2_i20 < 4; c2_i20++) {
    c2_b[c2_i20] = c2_c_a[c2_i20] * c2_b_b;
  }

  for (c2_i21 = 0; c2_i21 < 4; c2_i21++) {
    c2_x_next[c2_i21] = c2_e_y[c2_i21] + c2_b[c2_i21];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  for (c2_i22 = 0; c2_i22 < 16; c2_i22++) {
    c2_c_b[c2_i22] = c2_cov[c2_i22];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i23 = 0; c2_i23 < 4; c2_i23++) {
    c2_i24 = 0;
    for (c2_i25 = 0; c2_i25 < 4; c2_i25++) {
      c2_f_y[c2_i24 + c2_i23] = 0.0;
      c2_i26 = 0;
      for (c2_i27 = 0; c2_i27 < 4; c2_i27++) {
        c2_f_y[c2_i24 + c2_i23] += c2_b_a[c2_i26 + c2_i23] * c2_c_b[c2_i27 +
          c2_i24];
        c2_i26 += 4;
      }

      c2_i24 += 4;
    }
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i28 = 0; c2_i28 < 4; c2_i28++) {
    c2_i29 = 0;
    for (c2_i30 = 0; c2_i30 < 4; c2_i30++) {
      c2_g_y[c2_i29 + c2_i28] = 0.0;
      c2_i31 = 0;
      for (c2_i32 = 0; c2_i32 < 4; c2_i32++) {
        c2_g_y[c2_i29 + c2_i28] += c2_f_y[c2_i31 + c2_i28] * c2_d_b[c2_i32 +
          c2_i29];
        c2_i31 += 4;
      }

      c2_i29 += 4;
    }
  }

  for (c2_i33 = 0; c2_i33 < 16; c2_i33++) {
    c2_cov_next[c2_i33] = c2_g_y[c2_i33] + c2_R[c2_i33];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  if (CV_EML_IF(0, 1, 0, c2_confidence > 0.001)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
    for (c2_i34 = 0; c2_i34 < 16; c2_i34++) {
      c2_g_y[c2_i34] = c2_cov_next[c2_i34];
    }

    c2_eml_scalar_eg(chartInstance);
    c2_eml_scalar_eg(chartInstance);
    for (c2_i35 = 0; c2_i35 < 4; c2_i35++) {
      c2_e_y[c2_i35] = 0.0;
      c2_i36 = 0;
      for (c2_i37 = 0; c2_i37 < 4; c2_i37++) {
        c2_e_y[c2_i35] += c2_g_y[c2_i36 + c2_i35] * c2_e_b[c2_i37];
        c2_i36 += 4;
      }
    }

    for (c2_i38 = 0; c2_i38 < 16; c2_i38++) {
      c2_c_b[c2_i38] = c2_cov_next[c2_i38];
    }

    c2_c_eml_scalar_eg(chartInstance);
    c2_c_eml_scalar_eg(chartInstance);
    c2_i39 = 0;
    for (c2_i40 = 0; c2_i40 < 4; c2_i40++) {
      c2_h_y[c2_i40] = 0.0;
      for (c2_i41 = 0; c2_i41 < 4; c2_i41++) {
        c2_h_y[c2_i40] += c2_d_a[c2_i41] * c2_c_b[c2_i41 + c2_i39];
      }

      c2_i39 += 4;
    }

    c2_d_eml_scalar_eg(chartInstance);
    c2_d_eml_scalar_eg(chartInstance);
    c2_i_y = 0.0;
    for (c2_k = 1; c2_k < 5; c2_k++) {
      c2_b_k = c2_k;
      c2_i_y += c2_h_y[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_b_k), 1, 4, 1, 0) - 1] *
        c2_e_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_b_k), 1, 4, 1, 0) - 1];
    }

    c2_c_B = c2_i_y + c2_Q;
    c2_j_y = c2_c_B;
    c2_k_y = c2_j_y;
    for (c2_i42 = 0; c2_i42 < 4; c2_i42++) {
      c2_K[c2_i42] = c2_e_y[c2_i42] / c2_k_y;
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
    for (c2_i43 = 0; c2_i43 < 4; c2_i43++) {
      c2_b[c2_i43] = c2_x_next[c2_i43];
    }

    c2_d_eml_scalar_eg(chartInstance);
    c2_d_eml_scalar_eg(chartInstance);
    c2_l_y = 0.0;
    for (c2_c_k = 1; c2_c_k < 5; c2_c_k++) {
      c2_d_k = c2_c_k;
      c2_l_y += c2_e_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
        _SFD_INTEGER_CHECK("", (real_T)c2_d_k), 1, 4, 1, 0) - 1] *
        c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
        (real_T)c2_d_k), 1, 4, 1, 0) - 1];
    }

    for (c2_i44 = 0; c2_i44 < 4; c2_i44++) {
      c2_b[c2_i44] = c2_K[c2_i44];
    }

    c2_f_b = c2_z - c2_l_y;
    for (c2_i45 = 0; c2_i45 < 4; c2_i45++) {
      c2_b[c2_i45] *= c2_f_b;
    }

    for (c2_i46 = 0; c2_i46 < 4; c2_i46++) {
      c2_x_corred[c2_i46] = c2_x_next[c2_i46] + c2_b[c2_i46];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
    for (c2_i47 = 0; c2_i47 < 16; c2_i47++) {
      c2_I[c2_i47] = 0.0;
    }

    for (c2_e_k = 1; c2_e_k < 5; c2_e_k++) {
      c2_f_k = c2_e_k;
      c2_I[(_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)_SFD_INTEGER_CHECK("",
              (real_T)c2_f_k), 1, 4, 1, 0) + ((_SFD_EML_ARRAY_BOUNDS_CHECK("",
               (int32_T)_SFD_INTEGER_CHECK("", (real_T)c2_f_k), 1, 4, 2, 0) - 1)
             << 2)) - 1] = 1.0;
    }

    for (c2_i48 = 0; c2_i48 < 4; c2_i48++) {
      c2_b[c2_i48] = c2_K[c2_i48];
    }

    for (c2_i49 = 0; c2_i49 < 4; c2_i49++) {
      c2_i50 = 0;
      for (c2_i51 = 0; c2_i51 < 4; c2_i51++) {
        c2_f_y[c2_i50 + c2_i49] = c2_b[c2_i49] * c2_d_a[c2_i51];
        c2_i50 += 4;
      }
    }

    for (c2_i52 = 0; c2_i52 < 16; c2_i52++) {
      c2_I[c2_i52] -= c2_f_y[c2_i52];
    }

    for (c2_i53 = 0; c2_i53 < 16; c2_i53++) {
      c2_c_b[c2_i53] = c2_cov_next[c2_i53];
    }

    c2_b_eml_scalar_eg(chartInstance);
    c2_b_eml_scalar_eg(chartInstance);
    for (c2_i54 = 0; c2_i54 < 16; c2_i54++) {
      c2_cov_corred[c2_i54] = 0.0;
    }

    for (c2_i55 = 0; c2_i55 < 16; c2_i55++) {
      c2_cov_corred[c2_i55] = 0.0;
    }

    for (c2_i56 = 0; c2_i56 < 16; c2_i56++) {
      c2_g_y[c2_i56] = c2_cov_corred[c2_i56];
    }

    for (c2_i57 = 0; c2_i57 < 16; c2_i57++) {
      c2_cov_corred[c2_i57] = c2_g_y[c2_i57];
    }

    for (c2_i58 = 0; c2_i58 < 16; c2_i58++) {
      c2_g_y[c2_i58] = c2_cov_corred[c2_i58];
    }

    for (c2_i59 = 0; c2_i59 < 16; c2_i59++) {
      c2_cov_corred[c2_i59] = c2_g_y[c2_i59];
    }

    for (c2_i60 = 0; c2_i60 < 4; c2_i60++) {
      c2_i61 = 0;
      for (c2_i62 = 0; c2_i62 < 4; c2_i62++) {
        c2_cov_corred[c2_i61 + c2_i60] = 0.0;
        c2_i63 = 0;
        for (c2_i64 = 0; c2_i64 < 4; c2_i64++) {
          c2_cov_corred[c2_i61 + c2_i60] += c2_I[c2_i63 + c2_i60] *
            c2_c_b[c2_i64 + c2_i61];
          c2_i63 += 4;
        }

        c2_i61 += 4;
      }
    }
  } else {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
    for (c2_i65 = 0; c2_i65 < 4; c2_i65++) {
      c2_x_corred[c2_i65] = c2_x_next[c2_i65];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 20);
    for (c2_i66 = 0; c2_i66 < 16; c2_i66++) {
      c2_cov_corred[c2_i66] = c2_cov_next[c2_i66];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -20);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i67 = 0; c2_i67 < 4; c2_i67++) {
    (*c2_b_x_corred)[c2_i67] = c2_x_corred[c2_i67];
  }

  for (c2_i68 = 0; c2_i68 < 16; c2_i68++) {
    (*c2_b_cov_corred)[c2_i68] = c2_cov_corred[c2_i68];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_kalman_verifier(SFc2_kalman_verifierInstanceStruct *
  chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i69;
  int32_T c2_i70;
  int32_T c2_i71;
  real_T c2_b_inData[16];
  int32_T c2_i72;
  int32_T c2_i73;
  int32_T c2_i74;
  real_T c2_u[16];
  const mxArray *c2_y = NULL;
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i69 = 0;
  for (c2_i70 = 0; c2_i70 < 4; c2_i70++) {
    for (c2_i71 = 0; c2_i71 < 4; c2_i71++) {
      c2_b_inData[c2_i71 + c2_i69] = (*(real_T (*)[16])c2_inData)[c2_i71 +
        c2_i69];
    }

    c2_i69 += 4;
  }

  c2_i72 = 0;
  for (c2_i73 = 0; c2_i73 < 4; c2_i73++) {
    for (c2_i74 = 0; c2_i74 < 4; c2_i74++) {
      c2_u[c2_i74 + c2_i72] = c2_b_inData[c2_i74 + c2_i72];
    }

    c2_i72 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_cov_corred, const char_T *c2_identifier,
  real_T c2_y[16])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_cov_corred), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_cov_corred);
}

static void c2_b_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[16])
{
  real_T c2_dv3[16];
  int32_T c2_i75;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv3, 1, 0, 0U, 1, 0U, 2, 4, 4);
  for (c2_i75 = 0; c2_i75 < 16; c2_i75++) {
    c2_y[c2_i75] = c2_dv3[c2_i75];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_cov_corred;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[16];
  int32_T c2_i76;
  int32_T c2_i77;
  int32_T c2_i78;
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c2_cov_corred = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_cov_corred), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_cov_corred);
  c2_i76 = 0;
  for (c2_i77 = 0; c2_i77 < 4; c2_i77++) {
    for (c2_i78 = 0; c2_i78 < 4; c2_i78++) {
      (*(real_T (*)[16])c2_outData)[c2_i78 + c2_i76] = c2_y[c2_i78 + c2_i76];
    }

    c2_i76 += 4;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i79;
  real_T c2_b_inData[4];
  int32_T c2_i80;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i79 = 0; c2_i79 < 4; c2_i79++) {
    c2_b_inData[c2_i79] = (*(real_T (*)[4])c2_inData)[c2_i79];
  }

  for (c2_i80 = 0; c2_i80 < 4; c2_i80++) {
    c2_u[c2_i80] = c2_b_inData[c2_i80];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_x_corred, const char_T *c2_identifier,
  real_T c2_y[4])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_x_corred), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_x_corred);
}

static void c2_d_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv4[4];
  int32_T c2_i81;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv4, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i81 = 0; c2_i81 < 4; c2_i81++) {
    c2_y[c2_i81] = c2_dv4[c2_i81];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_x_corred;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i82;
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c2_x_corred = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_x_corred), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_x_corred);
  for (c2_i82 = 0; c2_i82 < 4; c2_i82++) {
    (*(real_T (*)[4])c2_outData)[c2_i82] = c2_y[c2_i82];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i83;
  real_T c2_b_inData[3];
  int32_T c2_i84;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i83 = 0; c2_i83 < 3; c2_i83++) {
    c2_b_inData[c2_i83] = (*(real_T (*)[3])c2_inData)[c2_i83];
  }

  for (c2_i84 = 0; c2_i84 < 3; c2_i84++) {
    c2_u[c2_i84] = c2_b_inData[c2_i84];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_e_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_nargout;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c2_nargout = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_nargout), &c2_thisId);
  sf_mex_destroy(&c2_nargout);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i85;
  real_T c2_b_inData[4];
  int32_T c2_i86;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i85 = 0; c2_i85 < 4; c2_i85++) {
    c2_b_inData[c2_i85] = (*(real_T (*)[4])c2_inData)[c2_i85];
  }

  for (c2_i86 = 0; c2_i86 < 4; c2_i86++) {
    c2_u[c2_i86] = c2_b_inData[c2_i86];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

const mxArray *sf_c2_kalman_verifier_get_eml_resolved_functions_info(void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_createstruct("structure", 2, 47, 1),
                FALSE);
  c2_info_helper(&c2_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs0 = NULL;
  const mxArray *c2_lhs0 = NULL;
  const mxArray *c2_rhs1 = NULL;
  const mxArray *c2_lhs1 = NULL;
  const mxArray *c2_rhs2 = NULL;
  const mxArray *c2_lhs2 = NULL;
  const mxArray *c2_rhs3 = NULL;
  const mxArray *c2_lhs3 = NULL;
  const mxArray *c2_rhs4 = NULL;
  const mxArray *c2_lhs4 = NULL;
  const mxArray *c2_rhs5 = NULL;
  const mxArray *c2_lhs5 = NULL;
  const mxArray *c2_rhs6 = NULL;
  const mxArray *c2_lhs6 = NULL;
  const mxArray *c2_rhs7 = NULL;
  const mxArray *c2_lhs7 = NULL;
  const mxArray *c2_rhs8 = NULL;
  const mxArray *c2_lhs8 = NULL;
  const mxArray *c2_rhs9 = NULL;
  const mxArray *c2_lhs9 = NULL;
  const mxArray *c2_rhs10 = NULL;
  const mxArray *c2_lhs10 = NULL;
  const mxArray *c2_rhs11 = NULL;
  const mxArray *c2_lhs11 = NULL;
  const mxArray *c2_rhs12 = NULL;
  const mxArray *c2_lhs12 = NULL;
  const mxArray *c2_rhs13 = NULL;
  const mxArray *c2_lhs13 = NULL;
  const mxArray *c2_rhs14 = NULL;
  const mxArray *c2_lhs14 = NULL;
  const mxArray *c2_rhs15 = NULL;
  const mxArray *c2_lhs15 = NULL;
  const mxArray *c2_rhs16 = NULL;
  const mxArray *c2_lhs16 = NULL;
  const mxArray *c2_rhs17 = NULL;
  const mxArray *c2_lhs17 = NULL;
  const mxArray *c2_rhs18 = NULL;
  const mxArray *c2_lhs18 = NULL;
  const mxArray *c2_rhs19 = NULL;
  const mxArray *c2_lhs19 = NULL;
  const mxArray *c2_rhs20 = NULL;
  const mxArray *c2_lhs20 = NULL;
  const mxArray *c2_rhs21 = NULL;
  const mxArray *c2_lhs21 = NULL;
  const mxArray *c2_rhs22 = NULL;
  const mxArray *c2_lhs22 = NULL;
  const mxArray *c2_rhs23 = NULL;
  const mxArray *c2_lhs23 = NULL;
  const mxArray *c2_rhs24 = NULL;
  const mxArray *c2_lhs24 = NULL;
  const mxArray *c2_rhs25 = NULL;
  const mxArray *c2_lhs25 = NULL;
  const mxArray *c2_rhs26 = NULL;
  const mxArray *c2_lhs26 = NULL;
  const mxArray *c2_rhs27 = NULL;
  const mxArray *c2_lhs27 = NULL;
  const mxArray *c2_rhs28 = NULL;
  const mxArray *c2_lhs28 = NULL;
  const mxArray *c2_rhs29 = NULL;
  const mxArray *c2_lhs29 = NULL;
  const mxArray *c2_rhs30 = NULL;
  const mxArray *c2_lhs30 = NULL;
  const mxArray *c2_rhs31 = NULL;
  const mxArray *c2_lhs31 = NULL;
  const mxArray *c2_rhs32 = NULL;
  const mxArray *c2_lhs32 = NULL;
  const mxArray *c2_rhs33 = NULL;
  const mxArray *c2_lhs33 = NULL;
  const mxArray *c2_rhs34 = NULL;
  const mxArray *c2_lhs34 = NULL;
  const mxArray *c2_rhs35 = NULL;
  const mxArray *c2_lhs35 = NULL;
  const mxArray *c2_rhs36 = NULL;
  const mxArray *c2_lhs36 = NULL;
  const mxArray *c2_rhs37 = NULL;
  const mxArray *c2_lhs37 = NULL;
  const mxArray *c2_rhs38 = NULL;
  const mxArray *c2_lhs38 = NULL;
  const mxArray *c2_rhs39 = NULL;
  const mxArray *c2_lhs39 = NULL;
  const mxArray *c2_rhs40 = NULL;
  const mxArray *c2_lhs40 = NULL;
  const mxArray *c2_rhs41 = NULL;
  const mxArray *c2_lhs41 = NULL;
  const mxArray *c2_rhs42 = NULL;
  const mxArray *c2_lhs42 = NULL;
  const mxArray *c2_rhs43 = NULL;
  const mxArray *c2_lhs43 = NULL;
  const mxArray *c2_rhs44 = NULL;
  const mxArray *c2_lhs44 = NULL;
  const mxArray *c2_rhs45 = NULL;
  const mxArray *c2_lhs45 = NULL;
  const mxArray *c2_rhs46 = NULL;
  const mxArray *c2_lhs46 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1373306508U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1319729966U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c2_rhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("rdivide"), "name", "name", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c2_rhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c2_rhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c2_rhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713866U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c2_rhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c2_rhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m!common_checks"),
                  "context", "context", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c2_rhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c2_rhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c2_rhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xgemm"), "name", "name", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713870U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c2_rhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_blas_inline"), "name",
                  "name", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1299076768U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c2_rhs10, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs10, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold"),
                  "context", "context", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c2_rhs11, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs11, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m"),
                  "context", "context", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c2_rhs12, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs12, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m"),
                  "context", "context", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c2_rhs13, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs13, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m"),
                  "context", "context", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_refblas_xgemm"), "name",
                  "name", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360282350U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c2_rhs14, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs14, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xdotu"), "name", "name",
                  15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"),
                  "resolved", "resolved", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713870U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c2_rhs15, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs15, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_blas_inline"), "name",
                  "name", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1299076768U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c2_rhs16, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs16, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xdot"), "name", "name", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713868U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c2_rhs17, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs17, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_blas_inline"), "name",
                  "name", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m"),
                  "resolved", "resolved", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1299076768U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c2_rhs18, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs18, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m"),
                  "context", "context", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c2_rhs19, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs19, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m"),
                  "context", "context", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_refblas_xdot"), "name",
                  "name", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1299076772U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c2_rhs20, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs20, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m"),
                  "context", "context", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_refblas_xdotx"), "name",
                  "name", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360282350U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c2_rhs21, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs21, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c2_rhs22, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs22, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c2_rhs23, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs23, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818778U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c2_rhs24, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs24, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c2_rhs25, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs25, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818780U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c2_rhs26, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs26, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c2_rhs27, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs27, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818778U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c2_rhs28, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs28, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c2_rhs29, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs29, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346510340U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c2_rhs30, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs30, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c2_rhs31, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs31, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eye"), "name", "name", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "resolved",
                  "resolved", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1368183030U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c2_rhs32, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs32, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1368183030U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c2_rhs33, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs33, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c2_rhs34, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs34, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral"),
                  "context", "context", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isinf"), "name", "name", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713856U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c2_rhs35, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs35, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c2_rhs36, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs36, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_is_integer_class"), "name",
                  "name", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_integer_class.m"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286818782U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c2_rhs37, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs37, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c2_rhs38, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs38, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmin"), "name", "name", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c2_rhs39, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs39, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326728322U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c2_rhs40, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs40, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326727996U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c2_rhs41, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs41, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmin"), "name", "name", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c2_rhs42, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs42, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size"),
                  "context", "context", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c2_rhs43, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs43, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323170578U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c2_rhs44, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs44, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362261882U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c2_rhs45, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs45, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eye.m"), "context",
                  "context", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346510340U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c2_rhs46, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs46, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs46), "lhs", "lhs",
                  46);
  sf_mex_destroy(&c2_rhs0);
  sf_mex_destroy(&c2_lhs0);
  sf_mex_destroy(&c2_rhs1);
  sf_mex_destroy(&c2_lhs1);
  sf_mex_destroy(&c2_rhs2);
  sf_mex_destroy(&c2_lhs2);
  sf_mex_destroy(&c2_rhs3);
  sf_mex_destroy(&c2_lhs3);
  sf_mex_destroy(&c2_rhs4);
  sf_mex_destroy(&c2_lhs4);
  sf_mex_destroy(&c2_rhs5);
  sf_mex_destroy(&c2_lhs5);
  sf_mex_destroy(&c2_rhs6);
  sf_mex_destroy(&c2_lhs6);
  sf_mex_destroy(&c2_rhs7);
  sf_mex_destroy(&c2_lhs7);
  sf_mex_destroy(&c2_rhs8);
  sf_mex_destroy(&c2_lhs8);
  sf_mex_destroy(&c2_rhs9);
  sf_mex_destroy(&c2_lhs9);
  sf_mex_destroy(&c2_rhs10);
  sf_mex_destroy(&c2_lhs10);
  sf_mex_destroy(&c2_rhs11);
  sf_mex_destroy(&c2_lhs11);
  sf_mex_destroy(&c2_rhs12);
  sf_mex_destroy(&c2_lhs12);
  sf_mex_destroy(&c2_rhs13);
  sf_mex_destroy(&c2_lhs13);
  sf_mex_destroy(&c2_rhs14);
  sf_mex_destroy(&c2_lhs14);
  sf_mex_destroy(&c2_rhs15);
  sf_mex_destroy(&c2_lhs15);
  sf_mex_destroy(&c2_rhs16);
  sf_mex_destroy(&c2_lhs16);
  sf_mex_destroy(&c2_rhs17);
  sf_mex_destroy(&c2_lhs17);
  sf_mex_destroy(&c2_rhs18);
  sf_mex_destroy(&c2_lhs18);
  sf_mex_destroy(&c2_rhs19);
  sf_mex_destroy(&c2_lhs19);
  sf_mex_destroy(&c2_rhs20);
  sf_mex_destroy(&c2_lhs20);
  sf_mex_destroy(&c2_rhs21);
  sf_mex_destroy(&c2_lhs21);
  sf_mex_destroy(&c2_rhs22);
  sf_mex_destroy(&c2_lhs22);
  sf_mex_destroy(&c2_rhs23);
  sf_mex_destroy(&c2_lhs23);
  sf_mex_destroy(&c2_rhs24);
  sf_mex_destroy(&c2_lhs24);
  sf_mex_destroy(&c2_rhs25);
  sf_mex_destroy(&c2_lhs25);
  sf_mex_destroy(&c2_rhs26);
  sf_mex_destroy(&c2_lhs26);
  sf_mex_destroy(&c2_rhs27);
  sf_mex_destroy(&c2_lhs27);
  sf_mex_destroy(&c2_rhs28);
  sf_mex_destroy(&c2_lhs28);
  sf_mex_destroy(&c2_rhs29);
  sf_mex_destroy(&c2_lhs29);
  sf_mex_destroy(&c2_rhs30);
  sf_mex_destroy(&c2_lhs30);
  sf_mex_destroy(&c2_rhs31);
  sf_mex_destroy(&c2_lhs31);
  sf_mex_destroy(&c2_rhs32);
  sf_mex_destroy(&c2_lhs32);
  sf_mex_destroy(&c2_rhs33);
  sf_mex_destroy(&c2_lhs33);
  sf_mex_destroy(&c2_rhs34);
  sf_mex_destroy(&c2_lhs34);
  sf_mex_destroy(&c2_rhs35);
  sf_mex_destroy(&c2_lhs35);
  sf_mex_destroy(&c2_rhs36);
  sf_mex_destroy(&c2_lhs36);
  sf_mex_destroy(&c2_rhs37);
  sf_mex_destroy(&c2_lhs37);
  sf_mex_destroy(&c2_rhs38);
  sf_mex_destroy(&c2_lhs38);
  sf_mex_destroy(&c2_rhs39);
  sf_mex_destroy(&c2_lhs39);
  sf_mex_destroy(&c2_rhs40);
  sf_mex_destroy(&c2_lhs40);
  sf_mex_destroy(&c2_rhs41);
  sf_mex_destroy(&c2_lhs41);
  sf_mex_destroy(&c2_rhs42);
  sf_mex_destroy(&c2_lhs42);
  sf_mex_destroy(&c2_rhs43);
  sf_mex_destroy(&c2_lhs43);
  sf_mex_destroy(&c2_rhs44);
  sf_mex_destroy(&c2_lhs44);
  sf_mex_destroy(&c2_rhs45);
  sf_mex_destroy(&c2_lhs45);
  sf_mex_destroy(&c2_rhs46);
  sf_mex_destroy(&c2_lhs46);
}

static const mxArray *c2_emlrt_marshallOut(char * c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c2_u)), FALSE);
  return c2_y;
}

static const mxArray *c2_b_emlrt_marshallOut(uint32_T c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 7, 0U, 0U, 0U, 0), FALSE);
  return c2_y;
}

static void c2_eml_scalar_eg(SFc2_kalman_verifierInstanceStruct *chartInstance)
{
}

static void c2_b_eml_scalar_eg(SFc2_kalman_verifierInstanceStruct *chartInstance)
{
}

static void c2_c_eml_scalar_eg(SFc2_kalman_verifierInstanceStruct *chartInstance)
{
}

static void c2_d_eml_scalar_eg(SFc2_kalman_verifierInstanceStruct *chartInstance)
{
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_f_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i87;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i87, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i87;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_g_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_kalman_verifier, const char_T
  *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_kalman_verifier), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_kalman_verifier);
  return c2_y;
}

static uint8_T c2_h_emlrt_marshallIn(SFc2_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info(SFc2_kalman_verifierInstanceStruct
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

void sf_c2_kalman_verifier_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2613617682U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2642698607U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1008557598U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1034904586U);
}

mxArray *sf_c2_kalman_verifier_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("MBHZ6gaNTwAwpAbPJRJWRG");
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
      pr[0] = (double)(4);
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
      pr[0] = (double)(4);
      pr[1] = (double)(4);
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
      pr[0] = (double)(4);
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
      pr[0] = (double)(4);
      pr[1] = (double)(4);
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

mxArray *sf_c2_kalman_verifier_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_kalman_verifier_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_kalman_verifier(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[12],T\"cov_corred\",},{M[1],M[9],T\"x_corred\",},{M[8],M[0],T\"is_active_c2_kalman_verifier\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_kalman_verifier_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_kalman_verifierInstanceStruct *chartInstance;
    chartInstance = (SFc2_kalman_verifierInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _kalman_verifierMachineNumber_,
           2,
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
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,600);
        _SFD_CV_INIT_EML_IF(0,1,0,389,412,542,600);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)
            c2_b_sf_marshallIn);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)
            c2_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[2];
          dimVector[0]= 4;
          dimVector[1]= 4;
          _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,2,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T *c2_z;
          real_T *c2_confidence;
          real_T (*c2_in)[3];
          real_T (*c2_x_corred)[4];
          real_T (*c2_cov_corred)[16];
          real_T (*c2_x)[4];
          real_T (*c2_cov)[16];
          c2_cov = (real_T (*)[16])ssGetInputPortSignal(chartInstance->S, 4);
          c2_x = (real_T (*)[4])ssGetInputPortSignal(chartInstance->S, 3);
          c2_cov_corred = (real_T (*)[16])ssGetOutputPortSignal(chartInstance->S,
            2);
          c2_x_corred = (real_T (*)[4])ssGetOutputPortSignal(chartInstance->S, 1);
          c2_confidence = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c2_z = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c2_in = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_in);
          _SFD_SET_DATA_VALUE_PTR(1U, c2_z);
          _SFD_SET_DATA_VALUE_PTR(2U, c2_confidence);
          _SFD_SET_DATA_VALUE_PTR(3U, *c2_x_corred);
          _SFD_SET_DATA_VALUE_PTR(4U, *c2_cov_corred);
          _SFD_SET_DATA_VALUE_PTR(5U, *c2_x);
          _SFD_SET_DATA_VALUE_PTR(6U, *c2_cov);
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
  return "OEHXCDQoCWMpWxhmAj69LB";
}

static void sf_opaque_initialize_c2_kalman_verifier(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_kalman_verifierInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_kalman_verifier((SFc2_kalman_verifierInstanceStruct*)
    chartInstanceVar);
  initialize_c2_kalman_verifier((SFc2_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_kalman_verifier(void *chartInstanceVar)
{
  enable_c2_kalman_verifier((SFc2_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c2_kalman_verifier(void *chartInstanceVar)
{
  disable_c2_kalman_verifier((SFc2_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c2_kalman_verifier(void *chartInstanceVar)
{
  sf_c2_kalman_verifier((SFc2_kalman_verifierInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_kalman_verifier(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_kalman_verifier
    ((SFc2_kalman_verifierInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_kalman_verifier();/* state var info */
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

extern void sf_internal_set_sim_state_c2_kalman_verifier(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_kalman_verifier();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_kalman_verifier((SFc2_kalman_verifierInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_kalman_verifier(SimStruct* S)
{
  return sf_internal_get_sim_state_c2_kalman_verifier(S);
}

static void sf_opaque_set_sim_state_c2_kalman_verifier(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c2_kalman_verifier(S, st);
}

static void sf_opaque_terminate_c2_kalman_verifier(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_kalman_verifierInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_kalman_verifier_optimization_info();
    }

    finalize_c2_kalman_verifier((SFc2_kalman_verifierInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_kalman_verifier((SFc2_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_kalman_verifier(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_kalman_verifier((SFc2_kalman_verifierInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_kalman_verifier(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_kalman_verifier_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
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

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(3170693301U));
  ssSetChecksum1(S,(3136529689U));
  ssSetChecksum2(S,(4234071980U));
  ssSetChecksum3(S,(1743682152U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_kalman_verifier(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_kalman_verifier(SimStruct *S)
{
  SFc2_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc2_kalman_verifierInstanceStruct *)utMalloc(sizeof
    (SFc2_kalman_verifierInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_kalman_verifierInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_kalman_verifier;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_kalman_verifier;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_kalman_verifier;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_kalman_verifier;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_kalman_verifier;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_kalman_verifier;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_kalman_verifier;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_kalman_verifier;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_kalman_verifier;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_kalman_verifier;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_kalman_verifier;
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

void c2_kalman_verifier_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_kalman_verifier(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_kalman_verifier(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_kalman_verifier(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_kalman_verifier_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
