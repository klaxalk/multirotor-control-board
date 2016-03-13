/* Include files */

#include <stddef.h>
#include "blas.h"
#include "kalman_verifier_sfun.h"
#include "c6_kalman_verifier.h"
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
static const char * c6_debug_family_names[9] = { "nargin", "nargout", "input",
  "threshold", "lastGood", "cycles", "good", "confidence", "cyclesBlind" };

/* Function Declarations */
static void initialize_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance);
static void initialize_params_c6_kalman_verifier
  (SFc6_kalman_verifierInstanceStruct *chartInstance);
static void enable_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance);
static void disable_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance);
static void c6_update_debugger_state_c6_kalman_verifier
  (SFc6_kalman_verifierInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c6_kalman_verifier
  (SFc6_kalman_verifierInstanceStruct *chartInstance);
static void set_sim_state_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct *
  chartInstance, const mxArray *c6_st);
static void finalize_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance);
static void sf_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance);
static void c6_chartstep_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance);
static void initSimStructsc6_kalman_verifier(SFc6_kalman_verifierInstanceStruct *
  chartInstance);
static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber);
static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData);
static real_T c6_emlrt_marshallIn(SFc6_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c6_cyclesBlind, const char_T *c6_identifier);
static real_T c6_b_emlrt_marshallIn(SFc6_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static void c6_info_helper(const mxArray **c6_info);
static const mxArray *c6_emlrt_marshallOut(char * c6_u);
static const mxArray *c6_b_emlrt_marshallOut(uint32_T c6_u);
static void c6_eml_scalar_eg(SFc6_kalman_verifierInstanceStruct *chartInstance);
static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData);
static int32_T c6_c_emlrt_marshallIn(SFc6_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData);
static uint8_T c6_d_emlrt_marshallIn(SFc6_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_kalman_verifier, const char_T
  *c6_identifier);
static uint8_T c6_e_emlrt_marshallIn(SFc6_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId);
static void init_dsm_address_info(SFc6_kalman_verifierInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance)
{
  chartInstance->c6_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c6_is_active_c6_kalman_verifier = 0U;
}

static void initialize_params_c6_kalman_verifier
  (SFc6_kalman_verifierInstanceStruct *chartInstance)
{
}

static void enable_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c6_update_debugger_state_c6_kalman_verifier
  (SFc6_kalman_verifierInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c6_kalman_verifier
  (SFc6_kalman_verifierInstanceStruct *chartInstance)
{
  const mxArray *c6_st;
  const mxArray *c6_y = NULL;
  real_T c6_hoistedGlobal;
  real_T c6_u;
  const mxArray *c6_b_y = NULL;
  real_T c6_b_hoistedGlobal;
  real_T c6_b_u;
  const mxArray *c6_c_y = NULL;
  real_T c6_c_hoistedGlobal;
  real_T c6_c_u;
  const mxArray *c6_d_y = NULL;
  uint8_T c6_d_hoistedGlobal;
  uint8_T c6_d_u;
  const mxArray *c6_e_y = NULL;
  real_T *c6_confidence;
  real_T *c6_cyclesBlind;
  real_T *c6_good;
  c6_cyclesBlind = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c6_confidence = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c6_good = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c6_st = NULL;
  c6_st = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_createcellarray(4), FALSE);
  c6_hoistedGlobal = *c6_confidence;
  c6_u = c6_hoistedGlobal;
  c6_b_y = NULL;
  sf_mex_assign(&c6_b_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 0, c6_b_y);
  c6_b_hoistedGlobal = *c6_cyclesBlind;
  c6_b_u = c6_b_hoistedGlobal;
  c6_c_y = NULL;
  sf_mex_assign(&c6_c_y, sf_mex_create("y", &c6_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 1, c6_c_y);
  c6_c_hoistedGlobal = *c6_good;
  c6_c_u = c6_c_hoistedGlobal;
  c6_d_y = NULL;
  sf_mex_assign(&c6_d_y, sf_mex_create("y", &c6_c_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 2, c6_d_y);
  c6_d_hoistedGlobal = chartInstance->c6_is_active_c6_kalman_verifier;
  c6_d_u = c6_d_hoistedGlobal;
  c6_e_y = NULL;
  sf_mex_assign(&c6_e_y, sf_mex_create("y", &c6_d_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c6_y, 3, c6_e_y);
  sf_mex_assign(&c6_st, c6_y, FALSE);
  return c6_st;
}

static void set_sim_state_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct *
  chartInstance, const mxArray *c6_st)
{
  const mxArray *c6_u;
  real_T *c6_confidence;
  real_T *c6_cyclesBlind;
  real_T *c6_good;
  c6_cyclesBlind = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c6_confidence = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c6_good = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c6_doneDoubleBufferReInit = TRUE;
  c6_u = sf_mex_dup(c6_st);
  *c6_confidence = c6_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c6_u, 0)), "confidence");
  *c6_cyclesBlind = c6_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c6_u, 1)), "cyclesBlind");
  *c6_good = c6_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c6_u,
    2)), "good");
  chartInstance->c6_is_active_c6_kalman_verifier = c6_d_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c6_u, 3)),
     "is_active_c6_kalman_verifier");
  sf_mex_destroy(&c6_u);
  c6_update_debugger_state_c6_kalman_verifier(chartInstance);
  sf_mex_destroy(&c6_st);
}

static void finalize_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance)
{
}

static void sf_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance)
{
  real_T *c6_input;
  real_T *c6_threshold;
  real_T *c6_good;
  real_T *c6_lastGood;
  real_T *c6_confidence;
  real_T *c6_cycles;
  real_T *c6_cyclesBlind;
  c6_cyclesBlind = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c6_cycles = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c6_confidence = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c6_lastGood = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c6_good = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c6_threshold = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c6_input = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 4U, chartInstance->c6_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c6_input, 0U);
  _SFD_DATA_RANGE_CHECK(*c6_threshold, 1U);
  _SFD_DATA_RANGE_CHECK(*c6_good, 2U);
  _SFD_DATA_RANGE_CHECK(*c6_lastGood, 3U);
  _SFD_DATA_RANGE_CHECK(*c6_confidence, 4U);
  _SFD_DATA_RANGE_CHECK(*c6_cycles, 5U);
  _SFD_DATA_RANGE_CHECK(*c6_cyclesBlind, 6U);
  chartInstance->c6_sfEvent = CALL_EVENT;
  c6_chartstep_c6_kalman_verifier(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_kalman_verifierMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c6_chartstep_c6_kalman_verifier(SFc6_kalman_verifierInstanceStruct
  *chartInstance)
{
  real_T c6_hoistedGlobal;
  real_T c6_b_hoistedGlobal;
  real_T c6_c_hoistedGlobal;
  real_T c6_d_hoistedGlobal;
  real_T c6_input;
  real_T c6_threshold;
  real_T c6_lastGood;
  real_T c6_cycles;
  uint32_T c6_debug_family_var_map[9];
  real_T c6_nargin = 4.0;
  real_T c6_nargout = 3.0;
  real_T c6_good;
  real_T c6_confidence;
  real_T c6_cyclesBlind;
  real_T c6_A;
  real_T c6_x;
  real_T c6_b_x;
  real_T c6_y;
  real_T c6_a;
  real_T c6_b_a;
  real_T c6_c_a;
  real_T c6_ak;
  real_T c6_d_a;
  real_T c6_e_a;
  real_T c6_b;
  real_T c6_c;
  real_T c6_c_x;
  real_T c6_d_x;
  real_T c6_b_y;
  real_T c6_b_b;
  real_T c6_c_b;
  real_T c6_d_b;
  real_T c6_bk;
  real_T c6_e_b;
  real_T c6_br;
  real_T c6_b_c;
  real_T c6_b_A;
  real_T c6_e_x;
  real_T c6_f_x;
  real_T *c6_b_cyclesBlind;
  real_T *c6_b_confidence;
  real_T *c6_b_good;
  real_T *c6_b_cycles;
  real_T *c6_b_lastGood;
  real_T *c6_b_threshold;
  real_T *c6_b_input;
  c6_b_cyclesBlind = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
  c6_b_cycles = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
  c6_b_confidence = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c6_b_lastGood = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c6_b_good = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c6_b_threshold = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c6_b_input = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 4U, chartInstance->c6_sfEvent);
  c6_hoistedGlobal = *c6_b_input;
  c6_b_hoistedGlobal = *c6_b_threshold;
  c6_c_hoistedGlobal = *c6_b_lastGood;
  c6_d_hoistedGlobal = *c6_b_cycles;
  c6_input = c6_hoistedGlobal;
  c6_threshold = c6_b_hoistedGlobal;
  c6_lastGood = c6_c_hoistedGlobal;
  c6_cycles = c6_d_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c6_debug_family_names,
    c6_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargin, 0U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_nargout, 1U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_input, 2U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_threshold, 3U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_lastGood, 4U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c6_cycles, 5U, c6_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_good, 6U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_confidence, 7U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c6_cyclesBlind, 8U, c6_sf_marshallOut,
    c6_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 3);
  if (CV_EML_IF(0, 1, 0, c6_input == c6_lastGood)) {
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 4);
    if (CV_EML_IF(0, 1, 1, c6_cycles < 16.0)) {
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 5);
      c6_good = c6_lastGood;
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 6);
      c6_A = c6_cycles;
      c6_x = c6_A;
      c6_b_x = c6_x;
      c6_y = c6_b_x / 16.0;
      c6_a = c6_y;
      c6_b_a = c6_a;
      c6_c_a = c6_b_a;
      c6_eml_scalar_eg(chartInstance);
      c6_ak = c6_c_a;
      c6_d_a = c6_ak;
      c6_eml_scalar_eg(chartInstance);
      c6_e_a = c6_d_a;
      c6_b = c6_d_a;
      c6_c = c6_e_a * c6_b;
      c6_confidence = 1.0 - c6_c;
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 7);
      c6_cyclesBlind = c6_cycles + 1.0;
    } else {
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 9);
      c6_good = c6_lastGood;
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 10);
      c6_confidence = 0.0;
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 11);
      c6_cyclesBlind = c6_cycles + 1.0;
    }
  } else {
    _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 14);
    c6_c_x = c6_input - c6_lastGood;
    c6_d_x = c6_c_x;
    c6_b_y = muDoubleScalarAbs(c6_d_x);
    if (CV_EML_IF(0, 1, 2, c6_b_y > c6_threshold)) {
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 15);
      if (CV_EML_IF(0, 1, 3, c6_cycles < 16.0)) {
        _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 16);
        c6_good = c6_lastGood;
        _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 17);
        c6_b_b = c6_cycles;
        c6_c_b = c6_b_b;
        c6_d_b = c6_c_b;
        c6_eml_scalar_eg(chartInstance);
        c6_bk = c6_d_b;
        c6_e_b = c6_bk;
        c6_eml_scalar_eg(chartInstance);
        c6_br = c6_e_b;
        c6_b_c = muDoubleScalarPower(2.0, c6_br);
        c6_b_A = c6_b_c;
        c6_e_x = c6_b_A;
        c6_f_x = c6_e_x;
        c6_confidence = c6_f_x / 65536.0;
        _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 18);
        c6_cyclesBlind = c6_cycles + 1.0;
      } else {
        _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 20);
        c6_good = c6_input;
        _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 21);
        c6_confidence = 1.0;
        _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 22);
        c6_cyclesBlind = 0.0;
      }
    } else {
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 25);
      c6_good = c6_input;
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 26);
      c6_confidence = 1.0;
      _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, 27);
      c6_cyclesBlind = 0.0;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c6_sfEvent, -27);
  _SFD_SYMBOL_SCOPE_POP();
  *c6_b_good = c6_good;
  *c6_b_confidence = c6_confidence;
  *c6_b_cyclesBlind = c6_cyclesBlind;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c6_sfEvent);
}

static void initSimStructsc6_kalman_verifier(SFc6_kalman_verifierInstanceStruct *
  chartInstance)
{
}

static void init_script_number_translation(uint32_T c6_machineNumber, uint32_T
  c6_chartNumber)
{
}

static const mxArray *c6_sf_marshallOut(void *chartInstanceVoid, void *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  real_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc6_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(real_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static real_T c6_emlrt_marshallIn(SFc6_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c6_cyclesBlind, const char_T *c6_identifier)
{
  real_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_cyclesBlind),
    &c6_thisId);
  sf_mex_destroy(&c6_cyclesBlind);
  return c6_y;
}

static real_T c6_b_emlrt_marshallIn(SFc6_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  real_T c6_y;
  real_T c6_d0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_d0, 1, 0, 0U, 0, 0U, 0);
  c6_y = c6_d0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_cyclesBlind;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  real_T c6_y;
  SFc6_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc6_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c6_cyclesBlind = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_cyclesBlind),
    &c6_thisId);
  sf_mex_destroy(&c6_cyclesBlind);
  *(real_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

const mxArray *sf_c6_kalman_verifier_get_eml_resolved_functions_info(void)
{
  const mxArray *c6_nameCaptureInfo = NULL;
  c6_nameCaptureInfo = NULL;
  sf_mex_assign(&c6_nameCaptureInfo, sf_mex_createstruct("structure", 2, 22, 1),
                FALSE);
  c6_info_helper(&c6_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c6_nameCaptureInfo);
  return c6_nameCaptureInfo;
}

static void c6_info_helper(const mxArray **c6_info)
{
  const mxArray *c6_rhs0 = NULL;
  const mxArray *c6_lhs0 = NULL;
  const mxArray *c6_rhs1 = NULL;
  const mxArray *c6_lhs1 = NULL;
  const mxArray *c6_rhs2 = NULL;
  const mxArray *c6_lhs2 = NULL;
  const mxArray *c6_rhs3 = NULL;
  const mxArray *c6_lhs3 = NULL;
  const mxArray *c6_rhs4 = NULL;
  const mxArray *c6_lhs4 = NULL;
  const mxArray *c6_rhs5 = NULL;
  const mxArray *c6_lhs5 = NULL;
  const mxArray *c6_rhs6 = NULL;
  const mxArray *c6_lhs6 = NULL;
  const mxArray *c6_rhs7 = NULL;
  const mxArray *c6_lhs7 = NULL;
  const mxArray *c6_rhs8 = NULL;
  const mxArray *c6_lhs8 = NULL;
  const mxArray *c6_rhs9 = NULL;
  const mxArray *c6_lhs9 = NULL;
  const mxArray *c6_rhs10 = NULL;
  const mxArray *c6_lhs10 = NULL;
  const mxArray *c6_rhs11 = NULL;
  const mxArray *c6_lhs11 = NULL;
  const mxArray *c6_rhs12 = NULL;
  const mxArray *c6_lhs12 = NULL;
  const mxArray *c6_rhs13 = NULL;
  const mxArray *c6_lhs13 = NULL;
  const mxArray *c6_rhs14 = NULL;
  const mxArray *c6_lhs14 = NULL;
  const mxArray *c6_rhs15 = NULL;
  const mxArray *c6_lhs15 = NULL;
  const mxArray *c6_rhs16 = NULL;
  const mxArray *c6_lhs16 = NULL;
  const mxArray *c6_rhs17 = NULL;
  const mxArray *c6_lhs17 = NULL;
  const mxArray *c6_rhs18 = NULL;
  const mxArray *c6_lhs18 = NULL;
  const mxArray *c6_rhs19 = NULL;
  const mxArray *c6_lhs19 = NULL;
  const mxArray *c6_rhs20 = NULL;
  const mxArray *c6_lhs20 = NULL;
  const mxArray *c6_rhs21 = NULL;
  const mxArray *c6_lhs21 = NULL;
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("mrdivide"), "name", "name", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1373306508U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1319729966U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c6_rhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("rdivide"), "name", "name", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c6_rhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c6_rhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c6_rhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_div"), "name", "name", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713866U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c6_rhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("mpower"), "name", "name", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c6_rhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c6_rhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("ismatrix"), "name", "name", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1331304858U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c6_rhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("power"), "name", "name", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713880U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c6_rhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c6_rhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c6_rhs10, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs10, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1358186140U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c6_rhs11, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs11, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("floor"), "name", "name", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713854U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c6_rhs12, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs12, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c6_rhs13, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs13, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818726U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c6_rhs14, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs14, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818796U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c6_rhs15, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs15, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("mtimes"), "name", "name", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713878U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c6_rhs16, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs16, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m!common_checks"),
                  "context", "context", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c6_rhs17, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs17, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(""), "context", "context", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("abs"), "name", "name", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363713852U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c6_rhs18, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs18, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1363714556U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c6_rhs19, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs19, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1286818712U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c6_rhs20, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs20, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("eml_error"), "name", "name",
                  21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c6_info, c6_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(1343830358U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c6_info, c6_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c6_rhs21, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c6_lhs21, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c6_info, sf_mex_duplicatearraysafe(&c6_lhs21), "lhs", "lhs",
                  21);
  sf_mex_destroy(&c6_rhs0);
  sf_mex_destroy(&c6_lhs0);
  sf_mex_destroy(&c6_rhs1);
  sf_mex_destroy(&c6_lhs1);
  sf_mex_destroy(&c6_rhs2);
  sf_mex_destroy(&c6_lhs2);
  sf_mex_destroy(&c6_rhs3);
  sf_mex_destroy(&c6_lhs3);
  sf_mex_destroy(&c6_rhs4);
  sf_mex_destroy(&c6_lhs4);
  sf_mex_destroy(&c6_rhs5);
  sf_mex_destroy(&c6_lhs5);
  sf_mex_destroy(&c6_rhs6);
  sf_mex_destroy(&c6_lhs6);
  sf_mex_destroy(&c6_rhs7);
  sf_mex_destroy(&c6_lhs7);
  sf_mex_destroy(&c6_rhs8);
  sf_mex_destroy(&c6_lhs8);
  sf_mex_destroy(&c6_rhs9);
  sf_mex_destroy(&c6_lhs9);
  sf_mex_destroy(&c6_rhs10);
  sf_mex_destroy(&c6_lhs10);
  sf_mex_destroy(&c6_rhs11);
  sf_mex_destroy(&c6_lhs11);
  sf_mex_destroy(&c6_rhs12);
  sf_mex_destroy(&c6_lhs12);
  sf_mex_destroy(&c6_rhs13);
  sf_mex_destroy(&c6_lhs13);
  sf_mex_destroy(&c6_rhs14);
  sf_mex_destroy(&c6_lhs14);
  sf_mex_destroy(&c6_rhs15);
  sf_mex_destroy(&c6_lhs15);
  sf_mex_destroy(&c6_rhs16);
  sf_mex_destroy(&c6_lhs16);
  sf_mex_destroy(&c6_rhs17);
  sf_mex_destroy(&c6_lhs17);
  sf_mex_destroy(&c6_rhs18);
  sf_mex_destroy(&c6_lhs18);
  sf_mex_destroy(&c6_rhs19);
  sf_mex_destroy(&c6_lhs19);
  sf_mex_destroy(&c6_rhs20);
  sf_mex_destroy(&c6_lhs20);
  sf_mex_destroy(&c6_rhs21);
  sf_mex_destroy(&c6_lhs21);
}

static const mxArray *c6_emlrt_marshallOut(char * c6_u)
{
  const mxArray *c6_y = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", c6_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c6_u)), FALSE);
  return c6_y;
}

static const mxArray *c6_b_emlrt_marshallOut(uint32_T c6_u)
{
  const mxArray *c6_y = NULL;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 7, 0U, 0U, 0U, 0), FALSE);
  return c6_y;
}

static void c6_eml_scalar_eg(SFc6_kalman_verifierInstanceStruct *chartInstance)
{
}

static const mxArray *c6_b_sf_marshallOut(void *chartInstanceVoid, void
  *c6_inData)
{
  const mxArray *c6_mxArrayOutData = NULL;
  int32_T c6_u;
  const mxArray *c6_y = NULL;
  SFc6_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc6_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c6_mxArrayOutData = NULL;
  c6_u = *(int32_T *)c6_inData;
  c6_y = NULL;
  sf_mex_assign(&c6_y, sf_mex_create("y", &c6_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c6_mxArrayOutData, c6_y, FALSE);
  return c6_mxArrayOutData;
}

static int32_T c6_c_emlrt_marshallIn(SFc6_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  int32_T c6_y;
  int32_T c6_i0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_i0, 1, 6, 0U, 0, 0U, 0);
  c6_y = c6_i0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void c6_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c6_mxArrayInData, const char_T *c6_varName, void *c6_outData)
{
  const mxArray *c6_b_sfEvent;
  const char_T *c6_identifier;
  emlrtMsgIdentifier c6_thisId;
  int32_T c6_y;
  SFc6_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc6_kalman_verifierInstanceStruct *)chartInstanceVoid;
  c6_b_sfEvent = sf_mex_dup(c6_mxArrayInData);
  c6_identifier = c6_varName;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c6_b_sfEvent),
    &c6_thisId);
  sf_mex_destroy(&c6_b_sfEvent);
  *(int32_T *)c6_outData = c6_y;
  sf_mex_destroy(&c6_mxArrayInData);
}

static uint8_T c6_d_emlrt_marshallIn(SFc6_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c6_b_is_active_c6_kalman_verifier, const char_T
  *c6_identifier)
{
  uint8_T c6_y;
  emlrtMsgIdentifier c6_thisId;
  c6_thisId.fIdentifier = c6_identifier;
  c6_thisId.fParent = NULL;
  c6_y = c6_e_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c6_b_is_active_c6_kalman_verifier), &c6_thisId);
  sf_mex_destroy(&c6_b_is_active_c6_kalman_verifier);
  return c6_y;
}

static uint8_T c6_e_emlrt_marshallIn(SFc6_kalman_verifierInstanceStruct
  *chartInstance, const mxArray *c6_u, const emlrtMsgIdentifier *c6_parentId)
{
  uint8_T c6_y;
  uint8_T c6_u0;
  sf_mex_import(c6_parentId, sf_mex_dup(c6_u), &c6_u0, 1, 3, 0U, 0, 0U, 0);
  c6_y = c6_u0;
  sf_mex_destroy(&c6_u);
  return c6_y;
}

static void init_dsm_address_info(SFc6_kalman_verifierInstanceStruct
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

void sf_c6_kalman_verifier_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4223810328U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(728792612U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(99196444U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2309490777U);
}

mxArray *sf_c6_kalman_verifier_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("ITVcDGNYBuUL8eu6O51ffE");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

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
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c6_kalman_verifier_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c6_kalman_verifier_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c6_kalman_verifier(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[7],T\"confidence\",},{M[1],M[10],T\"cyclesBlind\",},{M[1],M[5],T\"good\",},{M[8],M[0],T\"is_active_c6_kalman_verifier\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c6_kalman_verifier_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc6_kalman_verifierInstanceStruct *chartInstance;
    chartInstance = (SFc6_kalman_verifierInstanceStruct *) ((ChartInfoStruct *)
      (ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _kalman_verifierMachineNumber_,
           6,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"input");
          _SFD_SET_DATA_PROPS(1,1,1,0,"threshold");
          _SFD_SET_DATA_PROPS(2,2,0,1,"good");
          _SFD_SET_DATA_PROPS(3,1,1,0,"lastGood");
          _SFD_SET_DATA_PROPS(4,2,0,1,"confidence");
          _SFD_SET_DATA_PROPS(5,1,1,0,"cycles");
          _SFD_SET_DATA_PROPS(6,2,0,1,"cyclesBlind");
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
        _SFD_CV_INIT_EML(0,1,1,4,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,771);
        _SFD_CV_INIT_EML_IF(0,1,0,90,112,380,770);
        _SFD_CV_INIT_EML_IF(0,1,1,121,137,261,374);
        _SFD_CV_INIT_EML_IF(0,1,2,389,427,683,766);
        _SFD_CV_INIT_EML_IF(0,1,3,436,452,571,670);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)c6_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)c6_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c6_sf_marshallOut,(MexInFcnForType)c6_sf_marshallIn);

        {
          real_T *c6_input;
          real_T *c6_threshold;
          real_T *c6_good;
          real_T *c6_lastGood;
          real_T *c6_confidence;
          real_T *c6_cycles;
          real_T *c6_cyclesBlind;
          c6_cyclesBlind = (real_T *)ssGetOutputPortSignal(chartInstance->S, 3);
          c6_cycles = (real_T *)ssGetInputPortSignal(chartInstance->S, 3);
          c6_confidence = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c6_lastGood = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c6_good = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c6_threshold = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c6_input = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c6_input);
          _SFD_SET_DATA_VALUE_PTR(1U, c6_threshold);
          _SFD_SET_DATA_VALUE_PTR(2U, c6_good);
          _SFD_SET_DATA_VALUE_PTR(3U, c6_lastGood);
          _SFD_SET_DATA_VALUE_PTR(4U, c6_confidence);
          _SFD_SET_DATA_VALUE_PTR(5U, c6_cycles);
          _SFD_SET_DATA_VALUE_PTR(6U, c6_cyclesBlind);
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
  return "4OOvsWPv7SXjPXyG2WqL1F";
}

static void sf_opaque_initialize_c6_kalman_verifier(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc6_kalman_verifierInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c6_kalman_verifier((SFc6_kalman_verifierInstanceStruct*)
    chartInstanceVar);
  initialize_c6_kalman_verifier((SFc6_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c6_kalman_verifier(void *chartInstanceVar)
{
  enable_c6_kalman_verifier((SFc6_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c6_kalman_verifier(void *chartInstanceVar)
{
  disable_c6_kalman_verifier((SFc6_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c6_kalman_verifier(void *chartInstanceVar)
{
  sf_c6_kalman_verifier((SFc6_kalman_verifierInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c6_kalman_verifier(SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c6_kalman_verifier
    ((SFc6_kalman_verifierInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_kalman_verifier();/* state var info */
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

extern void sf_internal_set_sim_state_c6_kalman_verifier(SimStruct* S, const
  mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c6_kalman_verifier();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c6_kalman_verifier((SFc6_kalman_verifierInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c6_kalman_verifier(SimStruct* S)
{
  return sf_internal_get_sim_state_c6_kalman_verifier(S);
}

static void sf_opaque_set_sim_state_c6_kalman_verifier(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c6_kalman_verifier(S, st);
}

static void sf_opaque_terminate_c6_kalman_verifier(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc6_kalman_verifierInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_kalman_verifier_optimization_info();
    }

    finalize_c6_kalman_verifier((SFc6_kalman_verifierInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc6_kalman_verifier((SFc6_kalman_verifierInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c6_kalman_verifier(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c6_kalman_verifier((SFc6_kalman_verifierInstanceStruct*)
      (((ChartInfoStruct *)ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c6_kalman_verifier(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_kalman_verifier_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      6);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,6,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,6,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,6);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,6,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,6,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,6);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1027929495U));
  ssSetChecksum1(S,(2528271092U));
  ssSetChecksum2(S,(21119473U));
  ssSetChecksum3(S,(1991113000U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c6_kalman_verifier(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c6_kalman_verifier(SimStruct *S)
{
  SFc6_kalman_verifierInstanceStruct *chartInstance;
  chartInstance = (SFc6_kalman_verifierInstanceStruct *)utMalloc(sizeof
    (SFc6_kalman_verifierInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc6_kalman_verifierInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c6_kalman_verifier;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c6_kalman_verifier;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c6_kalman_verifier;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c6_kalman_verifier;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c6_kalman_verifier;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c6_kalman_verifier;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c6_kalman_verifier;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c6_kalman_verifier;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c6_kalman_verifier;
  chartInstance->chartInfo.mdlStart = mdlStart_c6_kalman_verifier;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c6_kalman_verifier;
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

void c6_kalman_verifier_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c6_kalman_verifier(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c6_kalman_verifier(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c6_kalman_verifier(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c6_kalman_verifier_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
