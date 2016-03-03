/* Include files */

#include "kalman_sfun.h"
#include "kalman_sfun_debug_macros.h"
#include "c1_kalman.h"
#include "c2_kalman.h"
#include "c3_kalman.h"
#include "c4_kalman.h"
#include "c5_kalman.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _kalmanMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void kalman_initializer(void)
{
}

void kalman_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_kalman_method_dispatcher(SimStruct *simstructPtr, unsigned int
  chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_kalman_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_kalman_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_kalman_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_kalman_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_kalman_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_kalman_process_check_sum_call( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
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
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(847824431U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4241216134U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1013362732U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(801718593U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2805641891U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1489595852U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(172546080U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2708722222U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_kalman_get_check_sum(mxArray *plhs[]);
          sf_c1_kalman_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_kalman_get_check_sum(mxArray *plhs[]);
          sf_c2_kalman_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_kalman_get_check_sum(mxArray *plhs[]);
          sf_c3_kalman_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_kalman_get_check_sum(mxArray *plhs[]);
          sf_c4_kalman_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_kalman_get_check_sum(mxArray *plhs[]);
          sf_c5_kalman_get_check_sum(plhs);
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
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3896193060U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(951580278U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1051565694U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(864142986U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_kalman_autoinheritance_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
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
        if (strcmp(aiChksum, "T1zIey9WkHsT7ArVRmhaeC") == 0) {
          extern mxArray *sf_c1_kalman_get_autoinheritance_info(void);
          plhs[0] = sf_c1_kalman_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "SplOGjPfRNfb6mFECay8iD") == 0) {
          extern mxArray *sf_c2_kalman_get_autoinheritance_info(void);
          plhs[0] = sf_c2_kalman_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "uo80lXlHFSR6TuKtbyUA8B") == 0) {
          extern mxArray *sf_c3_kalman_get_autoinheritance_info(void);
          plhs[0] = sf_c3_kalman_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "3w92ynWhtxvscTv2H7Fd2E") == 0) {
          extern mxArray *sf_c4_kalman_get_autoinheritance_info(void);
          plhs[0] = sf_c4_kalman_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "1brxsE6JvQUtKzPp0JDexE") == 0) {
          extern mxArray *sf_c5_kalman_get_autoinheritance_info(void);
          plhs[0] = sf_c5_kalman_get_autoinheritance_info();
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

unsigned int sf_kalman_get_eml_resolved_functions_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
        extern const mxArray *sf_c1_kalman_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_kalman_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray *sf_c2_kalman_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_kalman_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray *sf_c3_kalman_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_kalman_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray *sf_c4_kalman_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_kalman_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray *sf_c5_kalman_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_kalman_get_eml_resolved_functions_info();
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

unsigned int sf_kalman_third_party_uses_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
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
        if (strcmp(tpChksum, "VYK5mNBTrl2o4EFwIUaUPD") == 0) {
          extern mxArray *sf_c1_kalman_third_party_uses_info(void);
          plhs[0] = sf_c1_kalman_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "GMedJlsnYuTn9uIuIzYiIH") == 0) {
          extern mxArray *sf_c2_kalman_third_party_uses_info(void);
          plhs[0] = sf_c2_kalman_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "Q9WpCxRjv85mudMNxBu6lC") == 0) {
          extern mxArray *sf_c3_kalman_third_party_uses_info(void);
          plhs[0] = sf_c3_kalman_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "906XdoM6PF9YMaY9wGcSKH") == 0) {
          extern mxArray *sf_c4_kalman_third_party_uses_info(void);
          plhs[0] = sf_c4_kalman_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "AnT4f1UXo1uDQaqfCUFJeE") == 0) {
          extern mxArray *sf_c5_kalman_third_party_uses_info(void);
          plhs[0] = sf_c5_kalman_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_kalman_updateBuildInfo_args_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
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
        if (strcmp(tpChksum, "VYK5mNBTrl2o4EFwIUaUPD") == 0) {
          extern mxArray *sf_c1_kalman_updateBuildInfo_args_info(void);
          plhs[0] = sf_c1_kalman_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "GMedJlsnYuTn9uIuIzYiIH") == 0) {
          extern mxArray *sf_c2_kalman_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_kalman_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "Q9WpCxRjv85mudMNxBu6lC") == 0) {
          extern mxArray *sf_c3_kalman_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_kalman_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "906XdoM6PF9YMaY9wGcSKH") == 0) {
          extern mxArray *sf_c4_kalman_updateBuildInfo_args_info(void);
          plhs[0] = sf_c4_kalman_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "AnT4f1UXo1uDQaqfCUFJeE") == 0) {
          extern mxArray *sf_c5_kalman_updateBuildInfo_args_info(void);
          plhs[0] = sf_c5_kalman_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void kalman_debug_initialize(struct SfDebugInstanceStruct* debugInstance)
{
  _kalmanMachineNumber_ = sf_debug_initialize_machine(debugInstance,"kalman",
    "sfun",0,5,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,_kalmanMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,_kalmanMachineNumber_,0);
}

void kalman_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_kalman_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("kalman",
      "kalman");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_kalman_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
