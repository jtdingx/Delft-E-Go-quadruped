/* written by Yanlong, 19-Oct-2017*/
/* this file declares the kmp class*/ 

#ifndef kmp_h
#define kmp_h

#include <iostream>
#include <armadillo>
using namespace std;
using namespace arma;

class kmp
{
   private:
	double lamda, kh;
        int    len,  inDim, outDim;
        mat    invK, W;
	mat    data;
	int    pvFlag;					   
   public: 
     	kmp();
	virtual ~kmp() {};
     
        void kmp_initialize(mat& oriData, int n2, int n3, int flag, double value1, double value2);  ////initialize

	int kernel_extend      ( vec a, vec b, mat& kext );	  // calculate an extended matrix
	int kmp_estimateMatrix ();                                // estimate kmp relevant matrix (initilize)  
	int kmp_prediction     ( vec query, vec& mean);// prediction mean
	int kmp_insertPoint    ( vec newPoint );                  // insert new point and then update kmp

/*        kmp(mat& oriData, int n2, int n3, int flag, double value1, double value2)
        {
		len    = oriData.n_rows;   // length of data
		inDim  = n2;		   // dim of inut
		outDim = n3;		   // dim of output

		pvFlag = flag;		   // pvFlag=0:model position, pvFlag=1, model position and velocity
		lamda  = value1;           // kmp parameter
		kh     = value2;	   // kmp parameter
		data   = oriData;	   // reference trajectory

		kmp_estimateMatrix (); 	   // initialize kmp		 
        }*/

//         kmp(mat& oriData, int n2, int n3, int flag, double value1, double value2)
//         {
// 		len    = oriData.n_rows;   // length of data
// 		inDim  = n2;		   // dim of inut
// 		outDim = n3;		   // dim of output
// 
// 		pvFlag = flag;		   // pvFlag=0:model position, pvFlag=1, model position and velocity
// 		lamda  = value1;           // kmp parameter
// 		kh     = value2;	   // kmp parameter
// 		data   = oriData;	   // reference trajectory
// 
// 		kmp_estimateMatrix (); 	   // initialize kmp		 
//         }

};

#endif
