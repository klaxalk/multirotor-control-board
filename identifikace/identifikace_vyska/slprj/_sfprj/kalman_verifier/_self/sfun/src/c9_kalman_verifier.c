/* Include files */

#include <stddef.h>
#include "blas.h"
#include "kalman_verifier_sfun.h"
#include "c9_kalman_verifier.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "kalman_verifier_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c9_debug_family_names[11] = { "nargin", "nargout", "sensor",
  "kalman", "start", "threshold", "lastmemory", "lastsum", "override", "memory",
  "sum" };

/* Function Declarations */
static void initialize_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct
  *chartInstance);
static void initialize_params_c9_kalman_verifier
  (SFc9_kalman_verifierInstanceStruct *chartInstance);
static void enable_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct
  *chartInstance);
static void disable_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct
  *chartInstance);
static void c9_update_debugger_state_c9_kalman_verifier
  (SFc9_kalman_verifierInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c9_kalman_verifier
  (SFc9_kalman_verifierInstanceStruct *chartInstance);
static void set_sim_state_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct *
  chartInstance, const mxArray *c9_st);
static void finalize_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct
  *chartInstance);
static void sf_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct
  *chartInstance);
static void initSimStructsc9_kalman_verifier(SFc9_kalman_verifierInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c9_machineNumber, uint32_T
  c9_chartNumber);
static const mxArray *c9_sf_marshallOut(void *chartInstanceVoid, void *c9_inData);
static real_T c9_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_sum, const char_T *c9_identifier);
static real_T c9_b_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static const mxArray *c9_b_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static void c9_c_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_memory, const char_T *c9_identifier, real_T
  c9_y[20]);
static void c9_d_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[20]);
static void c9_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static void c9_info_helper(const mxArray **c9_info);
static const mxArray *c9_emlrt_marshallOut(char * c9_u);
static const mxArray *c9_b_emlrt_marshallOut(uint32_T c9_u);
static const mxArray *c9_c_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData);
static int32_T c9_e_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void c9_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData);
static uint8_T c9_f_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_b_is_active_c9_kalman_verifier, const char_T
  *c9_identifier);
static uint8_T c9_g_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId);
static void init_dsm_address_info(SFc9_kalman_verifierInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct
  *chartInstance)
{
  chartInstance->c9_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c9_is_active_c9_kalman_verifier = 0U;
}

static void initialize_params_c9_kalman_verifier
  (SFc9_kalman_verifierInstanceStruct *chartInstance)
{
}

static void enable_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c9_update_debugger_state_c9_kalman_verifier
  (SFc9_kalman_verifierInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c9_kalman_verifier
  (SFc9_kalman_verifierInstanceStruct *chartInstance)
{
  const mxArray *c9_st;
  const mxArray *c9_y = NULL;
  int32_T c9_i0;
  real_T c9_u[20];
  const mxArray *c9_b_y = NULL;
  real_T c9_hoistedGlobal;
  real_T c9_b_u;
  const mxArray *c9_c_y = NULL;
  real_T c9_b_hoistedGlobal;
  real_T c9_c_u;
  const mxArray *c9_d_y = NULL;
  uint8_T c9_c_hoistedGlobal;
  uint8_T c9_d_u;
  const mxArray *c9_e_y = NULL;
  real_T *c9_override;
  real_T *c9_sum;
  real_T (*c9_memory)[20];
  c9_sum = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c9_memory = (real_T (*)[20])ssGetOutputPortSignal(chartInstance->S, 2);
  c9_override = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c9_st = NULL;
  c9_st = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_createcellarray(4), FALSE);
  for (c9_i0 = 0; c9_i0 < 20; c9_i0++) {
    c9_u[c9_i0] = (*c9_memory)[c9_i0];
  }

  c9_b_y = NULL;
  sf_mex_assign(&c9_b_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 1, 20), FALSE);
  sf_mex_setcell(c9_y, 0, c9_b_y);
  c9_hoistedGlobal = *c9_override;
  c9_b_u = c9_hoistedGlobal;
  c9_c_y = NULL;
  sf_mex_assign(&c9_c_y, sf_mex_create("y", &c9_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c9_y, 1, c9_c_y);
  c9_b_hoistedGlobal = *c9_sum;
  c9_c_u = c9_b_hoistedGlobal;
  c9_d_y = NULL;
  sf_mex_assign(&c9_d_y, sf_mex_create("y", &c9_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c9_y, 2, c9_d_y);
  c9_c_hoistedGlobal = chartInstance->c9_is_active_c9_kalman_verifier;
  c9_d_u = c9_c_hoistedGlobal;
  c9_e_y = NULL;
  sf_mex_assign(&c9_e_y, sf_mex_create("y", &c9_d_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c9_y, 3, c9_e_y);
  sf_mex_assign(&c9_st, c9_y, FALSE);
  return c9_st;
}

static void set_sim_state_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct *
  chartInstance, const mxArray *c9_st)
{
  const mxArray *c9_u;
  real_T c9_dv0[20];
  int32_T c9_i1;
  real_T *c9_override;
  real_T *c9_sum;
  real_T (*c9_memory)[20];
  c9_sum = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c9_memory = (real_T (*)[20])ssGetOutputPortSignal(chartInstance->S, 2);
  c9_override = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c9_doneDoubleBufferReInit = TRUE;
  c9_u = sf_mex_dup(c9_st);
  c9_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c9_u, 0)),
                        "memory", c9_dv0);
  for (c9_i1 = 0; c9_i1 < 20; c9_i1++) {
    (*c9_memory)[c9_i1] = c9_dv0[c9_i1];
  }

  *c9_override = c9_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c9_u, 1)), "override");
  *c9_sum = c9_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c9_u, 2)),
    "sum");
  chartInstance->c9_is_active_c9_kalman_verifier = c9_f_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c9_u, 3)),
     "is_active_c9_kalman_verifier");
  sf_mex_destroy(&c9_u);
  c9_update_debugger_state_c9_kalman_verifier(chartInstance);
  sf_mex_destroy(&c9_st);
}

static void finalize_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct
  *chartInstance)
{
}

static void sf_c9_kalman_verifier(SFc9_kalman_verifierInstanceStruct
  *chartInstance)
{
  int32_T c9_i2;
  int32_T c9_i3;
  real_T c9_hoistedGlobal;
  real_T c9_b_hoistedGlobal;
  real_T c9_c_hoistedGlobal;
  real_T c9_d_hoistedGlobal;
  real_T c9_e_hoistedGlobal;
  real_T c9_sensor;
  real_T c9_kalman;
  real_T c9_start;
  real_T c9_threshold;
  int32_T c9_i4;
  real_T c9_lastmemory[20];
  real_T c9_lastsum;
  uint32_T c9_debug_family_var_map[11];
  real_T c9_nargin = 6.0;
  real_T c9_nargout = 3.0;
  real_T c9_override;
  real_T c9_memory[20];
  real_T c9_sum;
  int32_T c9_i5;
  real_T c9_x;
  real_T c9_b_x;
  real_T c9_y;
  int32_T c9_i6;
  int32_T c9_i7;
  int32_T c9_i8;
  real_T *c9_b_override;
  real_T *c9_b_sensor;
  real_T *c9_b_kalman;
  real_T *c9_b_start;
  real_T *c9_b_threshold;
  real_T *c9_b_lastsum;
  real_T *c9_b_sum;
  real_T (*c9_b_memory)[20];
  real_T (*c9_b_lastmemory)[20];
  c9_b_sum = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c9_b_memory = (real_T (*)[20])ssGetOutputPortSignal(chartInstance->S, 2);
  c9_b_lastsum = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
  c9_b_lastmemory = (real_T (*)[20])ssGetInputPortSignal(chartInstance->S, 4);
  c9_b_threshold = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c9_b_start = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c9_b_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c9_b_sensor = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  c9_b_override = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 8U, chartInstance->c9_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c9_b_override, 0U);
  _SFD_DATA_RANGE_CHECK(*c9_b_sensor, 1U);
  _SFD_DATA_RANGE_CHECK(*c9_b_kalman, 2U);
  _SFD_DATA_RANGE_CHECK(*c9_b_start, 3U);
  _SFD_DATA_RANGE_CHECK(*c9_b_threshold, 4U);
  for (c9_i2 = 0; c9_i2 < 20; c9_i2++) {
    _SFD_DATA_RANGE_CHECK((*c9_b_lastmemory)[c9_i2], 5U);
  }

  _SFD_DATA_RANGE_CHECK(*c9_b_lastsum, 6U);
  for (c9_i3 = 0; c9_i3 < 20; c9_i3++) {
    _SFD_DATA_RANGE_CHECK((*c9_b_memory)[c9_i3], 7U);
  }

  _SFD_DATA_RANGE_CHECK(*c9_b_sum, 8U);
  chartInstance->c9_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 8U, chartInstance->c9_sfEvent);
  c9_hoistedGlobal = *c9_b_sensor;
  c9_b_hoistedGlobal = *c9_b_kalman;
  c9_c_hoistedGlobal = *c9_b_start;
  c9_d_hoistedGlobal = *c9_b_threshold;
  c9_e_hoistedGlobal = *c9_b_lastsum;
  c9_sensor = c9_hoistedGlobal;
  c9_kalman = c9_b_hoistedGlobal;
  c9_start = c9_c_hoistedGlobal;
  c9_threshold = c9_d_hoistedGlobal;
  for (c9_i4 = 0; c9_i4 < 20; c9_i4++) {
    c9_lastmemory[c9_i4] = (*c9_b_lastmemory)[c9_i4];
  }

  c9_lastsum = c9_e_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 11U, 11U, c9_debug_family_names,
    c9_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargin, 0U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_nargout, 1U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_sensor, 2U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_kalman, 3U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_start, 4U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_threshold, 5U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c9_lastmemory, 6U, c9_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c9_lastsum, 7U, c9_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_override, 8U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c9_memory, 9U, c9_b_sf_marshallOut,
    c9_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c9_sum, 10U, c9_sf_marshallOut,
    c9_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 3);
  if (CV_EML_IF(0, 1, 0, c9_start > 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 4);
    for (c9_i5 = 0; c9_i5 < 20; c9_i5++) {
      c9_memory[c9_i5] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 5);
    c9_x = c9_sensor - c9_kalman;
    c9_b_x = c9_x;
    c9_y = muDoubleScalarAbs(c9_b_x);
    c9_memory[0] = c9_y;
    _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 6);
    for (c9_i6 = 0; c9_i6 < 19; c9_i6++) {
      c9_memory[c9_i6 + 1] = c9_lastmemory[c9_i6];
    }

    _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 7);
    if (CV_EML_IF(0, 1, 1, c9_lastsum > c9_threshold)) {
      _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 8);
      c9_sum = (c9_lastsum - c9_lastmemory[19]) + c9_memory[0];
      _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 9);
      c9_override = 1.0;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 11);
      c9_sum = 0.0;
      _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 12);
      c9_override = 0.0;
    }
  } else {
    _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 15);
    c9_sum = 4.0;
    _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 16);
    for (c9_i7 = 0; c9_i7 < 20; c9_i7++) {
      c9_memory[c9_i7] = 0.2;
    }

    _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, 17);
    c9_override = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c9_sfEvent, -17);
  _SFD_SYMBOL_SCOPE_POP();
  *c9_b_override = c9_override;
  for (c9_i8 = 0; c9_i8 < 20; c9_i8++) {
    (*c9_b_memory)[c9_i8] = c9_memory[c9_i8];
  }

  *c9_b_sum = c9_sum;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 8U, chartInstance->c9_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_kalman_verifierMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void initSimStructsc9_kalman_verifier(SFc9_kalman_verifierInstanceStruct *
  chartInstance)
{
}

static void init_script_number_translation(uint32_T c9_machineNumber, uint32_T
  c9_chartNumber)
{
}

static const mxArray *c9_sf_marshallOut(void *chartInstanceVoid, void *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  real_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc9_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_u = *(real_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, FALSE);
  return c9_mxArrayOutData;
}

static real_T c9_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_sum, const char_T *c9_identifier)
{
  real_T c9_y;
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_sum), &c9_thisId);
  sf_mex_destroy(&c9_sum);
  return c9_y;
}

static real_T c9_b_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  real_T c9_y;
  real_T c9_d0;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_d0, 1, 0, 0U, 0, 0U, 0);
  c9_y = c9_d0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_sum;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y;
  SFc9_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc9_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c9_sum = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_sum), &c9_thisId);
  sf_mex_destroy(&c9_sum);
  *(real_T *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static const mxArray *c9_b_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_i9;
  real_T c9_b_inData[20];
  int32_T c9_i10;
  real_T c9_u[20];
  const mxArray *c9_y = NULL;
  SFc9_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc9_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  for (c9_i9 = 0; c9_i9 < 20; c9_i9++) {
    c9_b_inData[c9_i9] = (*(real_T (*)[20])c9_inData)[c9_i9];
  }

  for (c9_i10 = 0; c9_i10 < 20; c9_i10++) {
    c9_u[c9_i10] = c9_b_inData[c9_i10];
  }

  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 0, 0U, 1U, 0U, 1, 20), FALSE);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, FALSE);
  return c9_mxArrayOutData;
}

static void c9_c_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_memory, const char_T *c9_identifier, real_T
  c9_y[20])
{
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_memory), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_memory);
}

static void c9_d_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId,
  real_T c9_y[20])
{
  real_T c9_dv1[20];
  int32_T c9_i11;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), c9_dv1, 1, 0, 0U, 1, 0U, 1, 20);
  for (c9_i11 = 0; c9_i11 < 20; c9_i11++) {
    c9_y[c9_i11] = c9_dv1[c9_i11];
  }

  sf_mex_destroy(&c9_u);
}

static void c9_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_memory;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  real_T c9_y[20];
  int32_T c9_i12;
  SFc9_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc9_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c9_memory = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_memory), &c9_thisId, c9_y);
  sf_mex_destroy(&c9_memory);
  for (c9_i12 = 0; c9_i12 < 20; c9_i12++) {
    (*(real_T (*)[20])c9_outData)[c9_i12] = c9_y[c9_i12];
  }

  sf_mex_destroy(&c9_mxArrayInData);
}

const mxArray *sf_c9_kalman_verifier_get_eml_resolved_functions_info(void)
{
  const mxArray *c9_nameCaptureInfo = NULL;
  c9_nameCaptureInfo = NULL;
  sf_mex_assign(&c9_nameCaptureInfo, sf_mex_createstruct("structure", 2, 5, 1),
                FALSE);
  c9_info_helper(&c9_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c9_nameCaptureInfo);
  return c9_nameCaptureInfo;
}

static void c9_info_helper(const mxArray **c9_info)
{
  const mxArray *c9_rhs0 = NULL;
  const mxArray *c9_lhs0 = NULL;
  const mxArray *c9_rhs1 = NULL;
  const mxArray *c9_lhs1 = NULL;
  const mxArray *c9_rhs2 = NULL;
  const mxArray *c9_lhs2 = NULL;
  const mxArray *c9_rhs3 = NULL;
  const mxArray *c9_lhs3 = NULL;
  const mxArray *c9_rhs4 = NULL;
  const mxArray *c9_lhs4 = NULL;
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("abs"), "name", "name", 0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c9_rhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c9_lhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 1);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 1);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c9_rhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c9_lhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 2);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 2);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c9_rhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c9_lhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(""), "context", "context", 3);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("mtimes"), "name", "name", 3);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 3);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c9_rhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c9_lhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m!common_checks"),
                  "context", "context", 4);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 4);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c9_info, c9_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c9_info, c9_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c9_rhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c9_lhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c9_info, sf_mex_duplicatearraysafe(&c9_lhs4), "lhs", "lhs", 4);
  sf_mex_destroy(&c9_rhs0);
  sf_mex_destroy(&c9_lhs0);
  sf_mex_destroy(&c9_rhs1);
  sf_mex_destroy(&c9_lhs1);
  sf_mex_destroy(&c9_rhs2);
  sf_mex_destroy(&c9_lhs2);
  sf_mex_destroy(&c9_rhs3);
  sf_mex_destroy(&c9_lhs3);
  sf_mex_destroy(&c9_rhs4);
  sf_mex_destroy(&c9_lhs4);
}

static const mxArray *c9_emlrt_marshallOut(char * c9_u)
{
  const mxArray *c9_y = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", c9_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c9_u)), FALSE);
  return c9_y;
}

static const mxArray *c9_b_emlrt_marshallOut(uint32_T c9_u)
{
  const mxArray *c9_y = NULL;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 7, 0U, 0U, 0U, 0), FALSE);
  return c9_y;
}

static const mxArray *c9_c_sf_marshallOut(void *chartInstanceVoid, void
  *c9_inData)
{
  const mxArray *c9_mxArrayOutData = NULL;
  int32_T c9_u;
  const mxArray *c9_y = NULL;
  SFc9_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc9_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c9_mxArrayOutData = NULL;
  c9_u = *(int32_T *)c9_inData;
  c9_y = NULL;
  sf_mex_assign(&c9_y, sf_mex_create("y", &c9_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c9_mxArrayOutData, c9_y, FALSE);
  return c9_mxArrayOutData;
}

static int32_T c9_e_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  int32_T c9_y;
  int32_T c9_i13;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_i13, 1, 6, 0U, 0, 0U, 0);
  c9_y = c9_i13;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void c9_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c9_mxArrayInData, const char_T *c9_varName, void *c9_outData)
{
  const mxArray *c9_b_sfEvent;
  const char_T *c9_identifier;
  emlrtMsgIdentifier c9_thisId;
  int32_T c9_y;
  SFc9_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc9_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c9_b_sfEvent = sf_mex_dup(c9_mxArrayInData);
  c9_identifier = c9_varName;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c9_b_sfEvent),
    &c9_thisId);
  sf_mex_destroy(&c9_b_sfEvent);
  *(int32_T *)c9_outData = c9_y;
  sf_mex_destroy(&c9_mxArrayInData);
}

static uint8_T c9_f_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_b_is_active_c9_kalman_verifier, const char_T
  *c9_identifier)
{
  uint8_T c9_y;
  emlrtMsgIdentifier c9_thisId;
  c9_thisId.fIdentifier = c9_identifier;
  c9_thisId.fParent = NULL;
  c9_y = c9_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c9_b_is_active_c9_kalman_verifier), &c9_thisId);
  sf_mex_destroy(&c9_b_is_active_c9_kalman_verifier);
  return c9_y;
}

static uint8_T c9_g_emlrt_marshallIn(SFc9_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c9_u, const emlrtMsgIdentifier *c9_parentId)
{
  uint8_T c9_y;
  uint8_T c9_u0;
  sf_mex_import(c9_parentId, sf_mex_dup(c9_u), &c9_u0, 1, 3, 0U, 0, 0U, 0);
  c9_y = c9_u0;
  sf_mex_destroy(&c9_u);
  return c9_y;
}

static void init_dsm_address_info(SFc9_kalman_verifierInstanceStruct
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

void sf_c9_kalman_verifier_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2882168718U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1506948995U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(226329006U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3644841998U);
}

mxArray *sf_c9_kalman_verifier_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("SsStM4xzmpPlB0xvo1dAwG");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,6,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
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
      pr[0] = (double)(1);
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
      pr[0] = (double)(20);
      pr[1] = (double)(1);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,5,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,5,"type",mxType);
    }

    mxSetField(mxData,5,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
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
      pr[0] = (double)(20);
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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c9_kalman_verifier_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c9_kalman_verifier_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c9_kalman_verifier(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[13],T\"memory\",},{M[1],M[7],T\"override\",},{M[1],M[14],T\"sum\",},{M[8],M[0],T\"is_active_c9_kalman_verifier\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c9_kalman_verifier_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc9_kalman_verifierInstanceStruct *chartInstance;
    chartInstance = (SFc9_kalman_verifierInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _kalman_verifierMachineNumber_,
           9,
           1,
           1,
           9,
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
          _SFD_SET_DATA_PROPS(0,2,0,1,"override");
          _SFD_SET_DATA_PROPS(1,1,1,0,"sensor");
          _SFD_SET_DATA_PROPS(2,1,1,0,"kalman");
          _SFD_SET_DATA_PROPS(3,1,1,0,"start");
          _SFD_SET_DATA_PROPS(4,1,1,0,"threshold");
          _SFD_SET_DATA_PROPS(5,1,1,0,"lastmemory");
          _SFD_SET_DATA_PROPS(6,1,1,0,"lastsum");
          _SFD_SET_DATA_PROPS(7,2,0,1,"memory");
          _SFD_SET_DATA_PROPS(8,2,0,1,"sum");
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
        _SFD_CV_INIT_EML(0,1,1,2,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,482);
        _SFD_CV_INIT_EML_IF(0,1,0,99,113,412,481);
        _SFD_CV_INIT_EML_IF(0,1,1,228,252,343,406);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)c9_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 20;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_b_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 20;
          _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c9_b_sf_marshallOut,(MexInFcnForType)
            c9_b_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(8,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c9_sf_marshallOut,(MexInFcnForType)c9_sf_marshallIn);

        {
          real_T *c9_override;
          real_T *c9_sensor;
          real_T *c9_kalman;
          real_T *c9_start;
          real_T *c9_threshold;
          real_T *c9_lastsum;
          real_T *c9_sum;
          real_T (*c9_lastmemory)[20];
          real_T (*c9_memory)[20];
          c9_sum = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c9_memory = (real_T (*)[20])ssGetOutputPortSignal(chartInstance->S, 2);
          c9_lastsum = (real_T *)ssGetInputPortSignal(chartInstance->S, 5);
          c9_lastmemory = (real_T (*)[20])ssGetInputPortSignal(chartInstance->S,
            4);
          c9_threshold = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c9_start = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c9_kalman = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c9_sensor = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          c9_override = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, c9_override);
          _SFD_SET_DATA_VALUE_PTR(1U, c9_sensor);
          _SFD_SET_DATA_VALUE_PTR(2U, c9_kalman);
          _SFD_SET_DATA_VALUE_PTR(3U, c9_start);
          _SFD_SET_DATA_VALUE_PTR(4U, c9_threshold);
          _SFD_SET_DATA_VALUE_PTR(5U, *c9_lastmemory);
          _SFD_SET_DATA_VALUE_PTR(6U, c9_lastsum);
          _SFD_SET_DATA_VALUE_PTR(7U, *c9_memory);
          _SFD_SET_DATA_VALUE_PTR(8U, c9_sum);
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
  return "w95MhgZeeWWu8m7zL2KyV";
}

static void sf_opaque_initialize_c9_kalman_verifier(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc9_kalman_verifierInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c9_kalman_verifier((SFc9_kalman_verifierInstanceStruct*)
    chartInstanceVar);
  initialize_c9_kalman_verifier((SFc9_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c9_kalman_verifier(void *chartInstanceVar)
{
  enable_c9_kalman_verifier((SFc9_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c9_kalman_verifier(void *chartInstanceVar)
{
  disable_c9_kalman_verifier((SFc9_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c9_kalman_verifier(void *chartInstanceVar)
{
  sf_c9_kalman_verifier((SFc9_kalman_verifierInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c9_kalman_verifier(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c9_kalman_verifier
    ((SFc9_kalman_verifierInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c9_kalman_verifier();/* state var info */
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

extern void sf_internal_set_sim_state_c9_kalman_verifier(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c9_kalman_verifier();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c9_kalman_verifier((SFc9_kalman_verifierInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c9_kalman_verifier(SimStruct* S)
{
  return sf_internal_get_sim_state_c9_kalman_verifier(S);
}

static void sf_opaque_set_sim_state_c9_kalman_verifier(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c9_kalman_verifier(S, st);
}

static void sf_opaque_terminate_c9_kalman_verifier(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc9_kalman_verifierInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_kalman_verifier_optimization_info();
    }

    finalize_c9_kalman_verifier((SFc9_kalman_verifierInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc9_kalman_verifier((SFc9_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c9_kalman_verifier(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c9_kalman_verifier((SFc9_kalman_verifierInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c9_kalman_verifier(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_kalman_verifier_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      9);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,9,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,9,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,9);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 5, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,9,6);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,9,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 6; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,9);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2586542717U));
  ssSetChecksum1(S,(2941680305U));
  ssSetChecksum2(S,(1691095416U));
  ssSetChecksum3(S,(2712392064U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c9_kalman_verifier(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c9_kalman_verifier(SimStruct *S)
{
  SFc9_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc9_kalman_verifierInstanceStruct *)utMalloc(sizeof
    (SFc9_kalman_verifierInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc9_kalman_verifierInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c9_kalman_verifier;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c9_kalman_verifier;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c9_kalman_verifier;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c9_kalman_verifier;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c9_kalman_verifier;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c9_kalman_verifier;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c9_kalman_verifier;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c9_kalman_verifier;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c9_kalman_verifier;
  chartInstance->chartInfo.mdlStart = mdlStart_c9_kalman_verifier;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c9_kalman_verifier;
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

void c9_kalman_verifier_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c9_kalman_verifier(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c9_kalman_verifier(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c9_kalman_verifier(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c9_kalman_verifier_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
