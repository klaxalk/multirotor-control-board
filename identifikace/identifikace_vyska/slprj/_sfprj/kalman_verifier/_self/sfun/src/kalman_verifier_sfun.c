/* Include files */

#include "kalman_verifier_sfun.h"
#include "kalman_verifier_sfun_debug_macros.h"
#include "c1_kalman_verifier.h"
#include "c2_kalman_verifier.h"
#include "c3_kalman_verifier.h"
#include "c4_kalman_verifier.h"
#include "c5_kalman_verifier.h"
#include "c6_kalman_verifier.h"
#include "c8_kalman_verifier.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _kalman_verifierMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void kalman_verifier_initializer(void)
{
}

void kalman_verifier_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_kalman_verifier_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_kalman_verifier_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_kalman_verifier_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_kalman_verifier_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_kalman_verifier_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_kalman_verifier_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_kalman_verifier_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==8) {
    c8_kalman_verifier_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_kalman_verifier_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(4273874023U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1164031813U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1781065820U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1100199764U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3497703509U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2094898502U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4079818515U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2376494707U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_kalman_verifier_get_check_sum(mxArray *plhs[]);
          sf_c1_kalman_verifier_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_kalman_verifier_get_check_sum(mxArray *plhs[]);
          sf_c2_kalman_verifier_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_kalman_verifier_get_check_sum(mxArray *plhs[]);
          sf_c3_kalman_verifier_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_kalman_verifier_get_check_sum(mxArray *plhs[]);
          sf_c4_kalman_verifier_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_kalman_verifier_get_check_sum(mxArray *plhs[]);
          sf_c5_kalman_verifier_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_kalman_verifier_get_check_sum(mxArray *plhs[]);
          sf_c6_kalman_verifier_get_check_sum(plhs);
          break;
        }

       case 8:
        {
          extern void sf_c8_kalman_verifier_get_check_sum(mxArray *plhs[]);
          sf_c8_kalman_verifier_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3031367619U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4001028638U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3978939492U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(838979348U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1223440995U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(698140664U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(130746021U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3010728674U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_kalman_verifier_autoinheritance_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "gr5zxT1xJsLKczHrIAXwvG") == 0) {
          extern mxArray *sf_c1_kalman_verifier_get_autoinheritance_info(void);
          plhs[0] = sf_c1_kalman_verifier_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "MBHZ6gaNTwAwpAbPJRJWRG") == 0) {
          extern mxArray *sf_c2_kalman_verifier_get_autoinheritance_info(void);
          plhs[0] = sf_c2_kalman_verifier_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "PQGFIEctg058bUo5AZcQOF") == 0) {
          extern mxArray *sf_c3_kalman_verifier_get_autoinheritance_info(void);
          plhs[0] = sf_c3_kalman_verifier_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "3rVDfIwSa49zJ5Gl8hdt5F") == 0) {
          extern mxArray *sf_c4_kalman_verifier_get_autoinheritance_info(void);
          plhs[0] = sf_c4_kalman_verifier_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "ZWGHeTnJwql4DYU3mfYCxD") == 0) {
          extern mxArray *sf_c5_kalman_verifier_get_autoinheritance_info(void);
          plhs[0] = sf_c5_kalman_verifier_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "ITVcDGNYBuUL8eu6O51ffE") == 0) {
          extern mxArray *sf_c6_kalman_verifier_get_autoinheritance_info(void);
          plhs[0] = sf_c6_kalman_verifier_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 8:
      {
        if (strcmp(aiChksum, "BrW08qBUnxGAhLjZIIHRp") == 0) {
          extern mxArray *sf_c8_kalman_verifier_get_autoinheritance_info(void);
          plhs[0] = sf_c8_kalman_verifier_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_kalman_verifier_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray
          *sf_c1_kalman_verifier_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_kalman_verifier_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_kalman_verifier_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_kalman_verifier_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_kalman_verifier_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_kalman_verifier_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_kalman_verifier_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_kalman_verifier_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_kalman_verifier_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_kalman_verifier_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_kalman_verifier_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_kalman_verifier_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 8:
      {
        extern const mxArray
          *sf_c8_kalman_verifier_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c8_kalman_verifier_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_kalman_verifier_third_party_uses_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "zIrVbHjshSrCrPTKxanCsE") == 0) {
          extern mxArray *sf_c1_kalman_verifier_third_party_uses_info(void);
          plhs[0] = sf_c1_kalman_verifier_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "OEHXCDQoCWMpWxhmAj69LB") == 0) {
          extern mxArray *sf_c2_kalman_verifier_third_party_uses_info(void);
          plhs[0] = sf_c2_kalman_verifier_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "Gf7TsLNjxuaaGT3TygkJ7C") == 0) {
          extern mxArray *sf_c3_kalman_verifier_third_party_uses_info(void);
          plhs[0] = sf_c3_kalman_verifier_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "8JC65lwDb0QtCurqtWerp") == 0) {
          extern mxArray *sf_c4_kalman_verifier_third_party_uses_info(void);
          plhs[0] = sf_c4_kalman_verifier_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "0HfCEgdVO0X3NOXVQDEWZH") == 0) {
          extern mxArray *sf_c5_kalman_verifier_third_party_uses_info(void);
          plhs[0] = sf_c5_kalman_verifier_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "4OOvsWPv7SXjPXyG2WqL1F") == 0) {
          extern mxArray *sf_c6_kalman_verifier_third_party_uses_info(void);
          plhs[0] = sf_c6_kalman_verifier_third_party_uses_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "pgHjqzkznwNnuQlFh2s2bC") == 0) {
          extern mxArray *sf_c8_kalman_verifier_third_party_uses_info(void);
          plhs[0] = sf_c8_kalman_verifier_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_kalman_verifier_updateBuildInfo_args_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "zIrVbHjshSrCrPTKxanCsE") == 0) {
          extern mxArray *sf_c1_kalman_verifier_updateBuildInfo_args_info(void);
          plhs[0] = sf_c1_kalman_verifier_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "OEHXCDQoCWMpWxhmAj69LB") == 0) {
          extern mxArray *sf_c2_kalman_verifier_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_kalman_verifier_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "Gf7TsLNjxuaaGT3TygkJ7C") == 0) {
          extern mxArray *sf_c3_kalman_verifier_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_kalman_verifier_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "8JC65lwDb0QtCurqtWerp") == 0) {
          extern mxArray *sf_c4_kalman_verifier_updateBuildInfo_args_info(void);
          plhs[0] = sf_c4_kalman_verifier_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "0HfCEgdVO0X3NOXVQDEWZH") == 0) {
          extern mxArray *sf_c5_kalman_verifier_updateBuildInfo_args_info(void);
          plhs[0] = sf_c5_kalman_verifier_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "4OOvsWPv7SXjPXyG2WqL1F") == 0) {
          extern mxArray *sf_c6_kalman_verifier_updateBuildInfo_args_info(void);
          plhs[0] = sf_c6_kalman_verifier_updateBuildInfo_args_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "pgHjqzkznwNnuQlFh2s2bC") == 0) {
          extern mxArray *sf_c8_kalman_verifier_updateBuildInfo_args_info(void);
          plhs[0] = sf_c8_kalman_verifier_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void kalman_verifier_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _kalman_verifierMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "kalman_verifier","sfun",0,7,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _kalman_verifierMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _kalman_verifierMachineNumber_,0);
}

void kalman_verifier_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_kalman_verifier_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("kalman_verifier",
      "kalman_verifier");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_kalman_verifier_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
