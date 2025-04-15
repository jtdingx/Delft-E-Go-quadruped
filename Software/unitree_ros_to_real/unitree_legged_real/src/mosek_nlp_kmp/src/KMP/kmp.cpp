/* written by Yanlong, 19-Oct-2017*/
/* this file defines the main functions for kmp*/ 

#include "kmp.h"


kmp::kmp()                    ///declaration function
{

}



/********************KMP**********************************/
void kmp::kmp_initialize(mat& oriData, int n2, int n3, int flag, double value1, double value2)
{
  
  len    = oriData.n_rows;   // length of data
  inDim  = n2;		   // dim of inut
  outDim = n3;		   // dim of output

  pvFlag = flag;		   // pvFlag=0:model position, pvFlag=1, model position and velocity
  lamda  = value1;           // kmp parameter
  kh     = value2;	   // kmp parameter
  data   = oriData;	   // reference trajectory

 // kmp_estimateMatrix(); 	   // initialize kmp	  
//  cout<<"data_row:"<<oriData.n_rows<<endl;
//  cout<<"data_row:"<<oriData.n_cols<<endl;
}






/*********************kernel_extend*********************/
// estimate a extended kernel matrix
// input: vec a, b
// output, mat kext
int kmp::kernel_extend( vec a, vec b, mat& kext )
{
	int i;
	kext = zeros( outDim , outDim );

	if (pvFlag==0)		//input:  time/high-dimensional variable, output: position
	{	
	  double temp;
	  temp = dot( a-b , a-b );
	  temp = exp( -kh * temp );
	  
	  for ( i = 0; i < outDim; i++ )
	  kext(i,i) = temp;        
	}

	if( pvFlag==1 ) 	// input: time, output: position and velocity
	{

		double ta,tb,tadt,tbdt;
		double dt=0.005; //////// determine the smoothness: the bigger the smoother

		ta=a(0);
		tb=b(0);
		tadt=ta+dt;
		tbdt=tb+dt;

		double kt_t,kt_dt_temp,kt_dt;
		double kdt_t_temp,kdt_t;
		double kdt_dt_temp,kdt_dt;
		kt_t=exp(-kh*(ta-tb)*(ta-tb)); 	         // k(ta,tb)

		kt_dt_temp=exp(-kh*(ta-tbdt)*(ta-tbdt)); // k(ta,tb+dt)
		kt_dt=(kt_dt_temp-kt_t)/dt; 		 // (k(ta,tb+dt)-k(ta,tb))/dt

		kdt_t_temp=exp(-kh*(tadt-tb)*(tadt-tb)); // k(ta+dt,tb)
		kdt_t=(kdt_t_temp-kt_t)/dt; 		 // (k(ta+dt,tb)-k(ta,tb))/dt

		kdt_dt_temp=exp(-kh*(tadt-tbdt)*(tadt-tbdt));          // k(ta+dt,tb+dt)
		kdt_dt=(kdt_dt_temp-kt_dt_temp-kdt_t_temp+kt_t)/dt/dt; // (k(ta+dt,tb+dt)-k(ta,tb+dt)-k(ta+dt,tb)+k(ta,tb))/dt/dt

		int halfDim;
		halfDim=outDim/2;
		for ( i = 0; i < halfDim; i++ )
		{
			kext(i,i)                 = kt_t; 
			kext(i,i+halfDim)         = kt_dt; 
			kext(halfDim+i,i)         = kdt_t;
			kext(halfDim+i,halfDim+i) = kdt_dt;
		}
	}    

	return 0;
}

/*********************kmp_estimateMatrix*********************/
// estimate matrix K,Y
// then, invK and W=invK * Y are updated
int kmp::kmp_estimateMatrix()
{
	//cout<<"data"<<endl<<data<<endl;
	//cout<<"len:"<<len<<endl;

	if (len<=0) {cout<<"error1:initial kmp"<<endl;return 0;}

	int i, j, k, index;
	mat Y    (len*outDim, 1);
	mat K    (len*outDim, len*outDim);
	mat C    (outDim, outDim);
	vec temp1(inDim), temp2(inDim);
	mat temp (outDim, outDim);

	invK = zeros(len*outDim, len*outDim);
	W    = zeros(len*outDim, 1);

	for ( i = 0; i < len; i++ )
	{
	  for ( j = 0; j < len; j++ )
	  {
	    for ( k = 0; k < inDim; k++ ) temp1(k) = data(i,k);
	    for ( k = 0; k < inDim; k++ ) temp2(k) = data(j,k);
	    kernel_extend (temp1, temp2, temp); // k(s_i,s_j)

	    if(i == j)
	    {	
	      for(k=0; k<outDim; k++)
	      {
		index = inDim + outDim + k*outDim;
		C( k, span( 0, outDim - 1 ) ) = data( i, span( index, index + outDim - 1 ) ); // C dim: outDim × outDim
	      }
	      //cout<<C<<endl;
	      temp = temp + lamda*C;	
	    }

	    K( span( i*outDim, (i+1)*outDim - 1 ), span( j*outDim, (j+1)*outDim - 1 ) ) = temp;  // K dim: (len*outDim) × (len*outDim)
	  }
	  Y( span( i*outDim, (i+1)*outDim - 1), 0 ) = trans( data(i, span( inDim, inDim + outDim - 1) ) );   //Y dim: (len*outDim)  1
	}

	invK = inv(K);            // invK=K^{-1}
	W = invK * Y;		// W=K^{-1} * Y

	//cout<<"Y"<<"\n"<<Y<<endl;
	//cout<<"K"<<"\n"<<K<<endl;
	//cout<<"invK"<<"\n"<<invK<<endl;
	//cout<<"W"<<"\n"<<W<<endl;
	return 0;
}

/*********************kmp_prediction*********************/ 
// predict the mean using kmp
// input: vec query
// output: vec mean
int kmp::kmp_prediction( vec query, vec& mean)
{
	int i, k;
	mat Ks   (outDim, outDim*len);
	vec temp1(inDim);
	mat temp (outDim, outDim);

	for ( i = 0; i < len; i++ )   // calculate Ks
	{
		for ( k = 0; k < inDim; k++ ) temp1(k) = data(i,k);	
		kernel_extend ( query, temp1, temp ); // k(s^{*},s_i)
		Ks( span( 0, outDim - 1 ), span( i*outDim, (i+1)*outDim - 1 ) ) = temp;
	}

	mean = Ks*W; // predict mean
	
	return 0;
}

//int kmp::kmp_insertPoints(vec inPoint, vec outPoint, mat outVar)  
// point format, mean: 1×inDim, output:1×outDim, sigma:[first row, second row, ...], i.e., [1×outDim 1×outDim ...1×outDim]
int kmp::kmp_insertPoint(vec newPoint)
{	
	int    i, j, size;
	int    replaceFlag, replaceNum=-1;
	double dis, minDis=100;
	vec    temp1(inDim),temp2(inDim);

	/*vec newPoint(size);	
	/for(i=0;i<inDim;i++)
		newPoint(i)=inPoint(i);
	for(i=0;i<outDim;i++)
		newPoint(inDim+i)=outPoint(i);
	for(i=0;i<outDim;i++)
		newPoint(span(inDim+outDim+i,inDim+outDim+outDim-1)=outVar(i,span(0,outDim-1));
	*/

	size  = inDim + outDim * (outDim+1);
	temp1 = newPoint( span( 0, inDim-1 ) );
	for ( i = 0; i < len; i++ )
	{	
		for ( j = 0; j<inDim; j++ ) temp2(j) = data(i,j);
    		dis = dot( temp1-temp2, temp1-temp2 );
    		dis = sqrt( dis );
    		if( dis < minDis)
   		{   
			minDis=dis;
        		replaceNum = i;
    		}
	}
	if ( minDis < 0.1 ) replaceFlag = 1;
	else		     replaceFlag = 0;
	//cout<<"minDis:"<<minDis<<endl;
	//cout<<"replaceNum:"<<replaceNum<<endl;

	if (replaceFlag)
	{
    		//cout<<"data before 0:"<<endl<<data<<endl;
		data( replaceNum, span( 0, size-1 ) ) = trans( newPoint ); 
		//cout<<"data after 0:"<<endl<<data<<endl;
	}
	else
	{
    		//cout<<"data before 1:"<<endl<<data<<endl;
		data.resize ( len+1, size );
		len = len + 1;   // increase the length of database
    		data( len-1, span( 0,size-1 ) ) = trans( newPoint ); 
		//cout<<"data after 1:"<<endl<<data<<endl;
	}
//	cout<<"\n len:"<<len<<endl;
	//kmp_estimateMatrix (); //update KMP

	return 0;
}


