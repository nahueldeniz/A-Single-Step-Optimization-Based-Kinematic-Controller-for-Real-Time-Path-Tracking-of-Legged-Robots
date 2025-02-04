/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    Control w0;
    Control vf;
    Control vl;
    Control vz;
    DifferentialState x;
    DifferentialState y;
    DifferentialState z;
    DifferentialState theta;
    BMatrix acadodata_M1;
    acadodata_M1.read( "mpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "mpc_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << x;
    acadodata_f2 << y;
    acadodata_f2 << z;
    acadodata_f2 << theta;
    Function acadodata_f3;
    acadodata_f3 << x;
    acadodata_f3 << y;
    acadodata_f3 << z;
    acadodata_f3 << theta;
    BMatrix acadodata_M3;
    acadodata_M3.read( "mpc_data_acadodata_M3.txt" );
    BMatrix acadodata_M4;
    acadodata_M4.read( "mpc_data_acadodata_M4.txt" );
    Function acadodata_f5;
    acadodata_f5 << x;
    acadodata_f5 << y;
    acadodata_f5 << z;
    acadodata_f5 << theta;
    acadodata_f5 << w0;
    acadodata_f5 << vf;
    acadodata_f5 << vl;
    acadodata_f5 << vz;
    Function acadodata_f6;
    acadodata_f6 << x;
    acadodata_f6 << y;
    acadodata_f6 << z;
    acadodata_f6 << theta;
    OCP ocp2(0, 0.5, 10);
    ocp2.minimizeLSQ(acadodata_M3, acadodata_f5);
    ocp2.minimizeLSQEndTerm(acadodata_M4, acadodata_f6);
    ocp2.subjectTo((-1.57079632679489655800e+00) <= w0 <= 1.57079632679489655800e+00);
    ocp2.subjectTo((-5.00000000000000000000e-01) <= vf <= 5.00000000000000000000e-01);
    ocp2.subjectTo((-5.00000000000000000000e-01) <= vl <= 5.00000000000000000000e-01);
    ocp2.subjectTo((-5.00000000000000000000e-01) <= vz <= 5.00000000000000000000e-01);
    DifferentialEquation acadodata_f7;
    acadodata_f7 << (-cos(theta)*vf+dot(x)-sin(theta)*vl) == 0.00000000000000000000e+00;
    acadodata_f7 << (cos(theta)*vl+dot(y)-sin(theta)*vf) == 0.00000000000000000000e+00;
    acadodata_f7 << (dot(z)-vz) == 0.00000000000000000000e+00;
    acadodata_f7 << (dot(theta)-w0) == 0.00000000000000000000e+00;

    ocp2.setModel( acadodata_f7 );


    ocp2.setNU( 4 );
    ocp2.setNP( 0 );
    ocp2.setNOD( 0 );
    OCPexport ExportModule3( ocp2 );
    ExportModule3.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule3.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule3.set( LEVENBERG_MARQUARDT, 1.000000E-12 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule3.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule3.set( SPARSE_QP_SOLUTION, CONDENSING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule3.set( INTEGRATOR_TYPE, INT_IRK_GL4 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule3.set( NUM_INTEGRATOR_STEPS, 11 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule3.set( QP_SOLVER, QP_QPOASES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    uint export_flag;
    export_flag = ExportModule3.exportCode( "export_MPC" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

