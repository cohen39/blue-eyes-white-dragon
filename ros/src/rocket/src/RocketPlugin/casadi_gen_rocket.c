/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CODEGEN_PREFIX
  #define NAMESPACE_CONCAT(NS, ID) _NAMESPACE_CONCAT(NS, ID)
  #define _NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) casadi_gen_rocket_ ## ID
#endif

#include <math.h>
#include <casadi/mem.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_f3 CASADI_PREFIX(f3)
#define casadi_f4 CASADI_PREFIX(f4)
#define casadi_f5 CASADI_PREFIX(f5)
#define casadi_f6 CASADI_PREFIX(f6)
#define casadi_f7 CASADI_PREFIX(f7)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[18] = {14, 1, 0, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[19] = {15, 1, 0, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static const casadi_int casadi_s3[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s4[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};

casadi_real casadi_sq(casadi_real x) { return x*x;}

/* rocket_aero_forces:(x[14],u[4],p[15])->(FA_b[3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a4, a5, a6, a7, a8, a9;
  a0=arg[2] ? arg[2][8] : 0;
  a1=arg[2] ? arg[2][7] : 0;
  a2=1.0000000000000000e-03;
  a3=arg[0] ? arg[0][9] : 0;
  a4=(-a3);
  a4=fabs(a4);
  a4=(a2<a4);
  a5=arg[0] ? arg[0][7] : 0;
  a6=fabs(a5);
  a6=(a2<a6);
  a4=(a4&&a6);
  a6=(a3/a5);
  a6=(-a6);
  a6=atan(a6);
  a6=(-a6);
  a4=(a4?a6:0);
  a6=arg[1] ? arg[1][1] : 0;
  a7=arg[1] ? arg[1][2] : 0;
  a8=(a6+a7);
  a4=(a4+a8);
  a4=(a1*a4);
  a8=(a0+a4);
  a9=5.0000000000000000e-01;
  a10=arg[2] ? arg[2][12] : 0;
  a9=(a9*a10);
  a10=casadi_sq(a5);
  a11=arg[0] ? arg[0][8] : 0;
  a12=casadi_sq(a11);
  a10=(a10+a12);
  a12=casadi_sq(a3);
  a10=(a10+a12);
  a9=(a9*a10);
  a8=(a8*a9);
  a12=arg[2] ? arg[2][11] : 0;
  a8=(a8*a12);
  a10=sqrt(a10);
  a13=fabs(a10);
  a13=(a2<a13);
  a14=(a3/a10);
  a15=(a13?a14:0);
  a16=casadi_sq(a15);
  a17=(a5/a10);
  a17=(a13?a17:0);
  a18=(!a13);
  a19=-1.;
  a18=(a18?a19:0);
  a17=(a17+a18);
  a18=casadi_sq(a17);
  a16=(a16+a18);
  a16=sqrt(a16);
  a15=(a15/a16);
  a15=(a8*a15);
  a18=arg[2] ? arg[2][9] : 0;
  a20=arg[2] ? arg[2][10] : 0;
  a4=casadi_sq(a4);
  a4=(a20*a4);
  a4=(a18+a4);
  a4=(a4*a9);
  a4=(a4*a12);
  a21=(a4*a17);
  a15=(a15-a21);
  a21=fabs(a11);
  a21=(a2<a21);
  a22=fabs(a5);
  a22=(a2<a22);
  a21=(a21&&a22);
  a22=(a11/a5);
  a22=atan(a22);
  a22=(-a22);
  a21=(a21?a22:0);
  a22=arg[1] ? arg[1][3] : 0;
  a23=(a6+a22);
  a21=(a21+a23);
  a21=(a1*a21);
  a23=(a0+a21);
  a23=(a23*a9);
  a23=(a23*a12);
  a24=fabs(a10);
  a24=(a2<a24);
  a25=(a11/a10);
  a26=(a24?a25:0);
  a27=casadi_sq(a26);
  a28=(a5/a10);
  a28=(a24?a28:0);
  a29=(!a24);
  a29=(a29?a19:0);
  a28=(a28+a29);
  a29=casadi_sq(a28);
  a27=(a27+a29);
  a27=sqrt(a27);
  a26=(a26/a27);
  a26=(a23*a26);
  a21=casadi_sq(a21);
  a21=(a20*a21);
  a21=(a18+a21);
  a21=(a21*a9);
  a21=(a21*a12);
  a29=(a21*a28);
  a26=(a26+a29);
  a15=(a15-a26);
  a26=(-a11);
  a26=fabs(a26);
  a26=(a2<a26);
  a29=fabs(a5);
  a29=(a2<a29);
  a26=(a26&&a29);
  a29=(a11/a5);
  a29=(-a29);
  a29=atan(a29);
  a29=(-a29);
  a26=(a26?a29:0);
  a22=(a6-a22);
  a26=(a26+a22);
  a26=(a1*a26);
  a22=(a0+a26);
  a22=(a22*a9);
  a22=(a22*a12);
  a29=fabs(a10);
  a29=(a2<a29);
  a30=(a11/a10);
  a31=(-a30);
  a31=(a29?a31:0);
  a32=casadi_sq(a31);
  a33=(a5/a10);
  a33=(a29?a33:0);
  a34=(!a29);
  a34=(a34?a19:0);
  a33=(a33+a34);
  a34=casadi_sq(a33);
  a32=(a32+a34);
  a32=sqrt(a32);
  a31=(a31/a32);
  a31=(a22*a31);
  a26=casadi_sq(a26);
  a26=(a20*a26);
  a26=(a18+a26);
  a26=(a26*a9);
  a26=(a26*a12);
  a34=(a26*a33);
  a31=(a31+a34);
  a15=(a15-a31);
  a31=fabs(a3);
  a31=(a2<a31);
  a34=fabs(a5);
  a34=(a2<a34);
  a31=(a31&&a34);
  a34=(a3/a5);
  a34=atan(a34);
  a34=(-a34);
  a31=(a31?a34:0);
  a6=(a6-a7);
  a31=(a31+a6);
  a1=(a1*a31);
  a0=(a0+a1);
  a0=(a0*a9);
  a0=(a0*a12);
  a31=fabs(a10);
  a2=(a2<a31);
  a31=(a3/a10);
  a6=(-a31);
  a6=(a2?a6:0);
  a7=casadi_sq(a6);
  a5=(a5/a10);
  a5=(a2?a5:0);
  a34=(!a2);
  a34=(a34?a19:0);
  a5=(a5+a34);
  a34=casadi_sq(a5);
  a7=(a7+a34);
  a7=sqrt(a7);
  a6=(a6/a7);
  a6=(a0*a6);
  a1=casadi_sq(a1);
  a20=(a20*a1);
  a18=(a18+a20);
  a18=(a18*a9);
  a18=(a18*a12);
  a12=(a18*a5);
  a6=(a6-a12);
  a15=(a15+a6);
  if (res[0]!=0) res[0][0]=a15;
  a28=(a28/a27);
  a23=(a23*a28);
  a25=(a25*a21);
  a25=(a24?a25:0);
  a23=(a23-a25);
  a25=(a11/a10);
  a25=(a25*a4);
  a25=(a13?a25:0);
  a23=(a23-a25);
  a33=(a33/a32);
  a22=(a22*a33);
  a30=(a30*a26);
  a30=(a29?a30:0);
  a22=(a22+a30);
  a23=(a23-a22);
  a11=(a11/a10);
  a11=(a11*a18);
  a11=(a2?a11:0);
  a23=(a23-a11);
  if (res[0]!=0) res[0][1]=a23;
  a5=(a5/a7);
  a0=(a0*a5);
  a31=(a31*a18);
  a2=(a2?a31:0);
  a0=(a0-a2);
  a2=(a3/a10);
  a2=(a2*a21);
  a24=(a24?a2:0);
  a17=(a17/a16);
  a8=(a8*a17);
  a14=(a14*a4);
  a13=(a13?a14:0);
  a8=(a8+a13);
  a24=(a24+a8);
  a3=(a3/a10);
  a3=(a3*a26);
  a29=(a29?a3:0);
  a24=(a24+a29);
  a0=(a0-a24);
  if (res[0]!=0) res[0][2]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int rocket_aero_forces(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void rocket_aero_forces_incref(void) {
}

CASADI_SYMBOL_EXPORT void rocket_aero_forces_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int rocket_aero_forces_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int rocket_aero_forces_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* rocket_aero_forces_name_in(casadi_int i){
  switch (i) {
    case 0: return "x";
    case 1: return "u";
    case 2: return "p";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rocket_aero_forces_name_out(casadi_int i){
  switch (i) {
    case 0: return "FA_b";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rocket_aero_forces_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rocket_aero_forces_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int rocket_aero_forces_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT casadi_functions* rocket_aero_forces_functions(void) {
  static casadi_functions fun = {
    rocket_aero_forces_incref,
    rocket_aero_forces_decref,
    rocket_aero_forces_n_in,
    rocket_aero_forces_n_out,
    rocket_aero_forces_name_in,
    rocket_aero_forces_name_out,
    rocket_aero_forces_sparsity_in,
    rocket_aero_forces_sparsity_out,
    rocket_aero_forces_work,
    rocket_aero_forces
  };
  return &fun;
}
/* rocket_aero_moments:(x[14],u[4],p[15])->(MA_b[3]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a4, a5, a6, a7, a8, a9;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[2] ? arg[2][6] : 0;
  a1=arg[2] ? arg[2][8] : 0;
  a2=arg[2] ? arg[2][7] : 0;
  a3=1.0000000000000000e-03;
  a4=arg[0] ? arg[0][9] : 0;
  a5=fabs(a4);
  a5=(a3<a5);
  a6=arg[0] ? arg[0][7] : 0;
  a7=fabs(a6);
  a7=(a3<a7);
  a5=(a5&&a7);
  a7=(a4/a6);
  a7=atan(a7);
  a7=(-a7);
  a5=(a5?a7:0);
  a7=arg[1] ? arg[1][1] : 0;
  a8=arg[1] ? arg[1][2] : 0;
  a9=(a7-a8);
  a5=(a5+a9);
  a5=(a2*a5);
  a9=(a1+a5);
  a10=5.0000000000000000e-01;
  a11=arg[2] ? arg[2][12] : 0;
  a10=(a10*a11);
  a11=casadi_sq(a6);
  a12=arg[0] ? arg[0][8] : 0;
  a13=casadi_sq(a12);
  a11=(a11+a13);
  a13=casadi_sq(a4);
  a11=(a11+a13);
  a10=(a10*a11);
  a9=(a9*a10);
  a13=arg[2] ? arg[2][11] : 0;
  a9=(a9*a13);
  a11=sqrt(a11);
  a14=fabs(a11);
  a14=(a3<a14);
  a15=(a6/a11);
  a15=(a14?a15:0);
  a16=(!a14);
  a17=-1.;
  a16=(a16?a17:0);
  a15=(a15+a16);
  a16=(a4/a11);
  a18=(-a16);
  a18=(a14?a18:0);
  a18=casadi_sq(a18);
  a19=casadi_sq(a15);
  a18=(a18+a19);
  a18=sqrt(a18);
  a15=(a15/a18);
  a9=(a9*a15);
  a15=arg[2] ? arg[2][9] : 0;
  a18=arg[2] ? arg[2][10] : 0;
  a5=casadi_sq(a5);
  a5=(a18*a5);
  a5=(a15+a5);
  a5=(a5*a10);
  a5=(a5*a13);
  a16=(a16*a5);
  a16=(a14?a16:0);
  a9=(a9-a16);
  a16=fabs(a11);
  a16=(a3<a16);
  a19=(a4/a11);
  a20=fabs(a12);
  a20=(a3<a20);
  a21=fabs(a6);
  a21=(a3<a21);
  a20=(a20&&a21);
  a21=(a12/a6);
  a21=atan(a21);
  a21=(-a21);
  a20=(a20?a21:0);
  a21=arg[1] ? arg[1][3] : 0;
  a22=(a7+a21);
  a20=(a20+a22);
  a20=(a2*a20);
  a22=casadi_sq(a20);
  a22=(a18*a22);
  a22=(a15+a22);
  a22=(a22*a10);
  a22=(a22*a13);
  a19=(a19*a22);
  a23=(a16?a19:0);
  a24=(-a4);
  a24=fabs(a24);
  a24=(a3<a24);
  a25=fabs(a6);
  a25=(a3<a25);
  a24=(a24&&a25);
  a25=(a4/a6);
  a25=(-a25);
  a25=atan(a25);
  a25=(-a25);
  a24=(a24?a25:0);
  a8=(a7+a8);
  a24=(a24+a8);
  a24=(a2*a24);
  a8=(a1+a24);
  a8=(a8*a10);
  a8=(a8*a13);
  a25=fabs(a11);
  a25=(a3<a25);
  a26=(a6/a11);
  a26=(a25?a26:0);
  a27=(!a25);
  a27=(a27?a17:0);
  a26=(a26+a27);
  a27=(a4/a11);
  a28=(a25?a27:0);
  a28=casadi_sq(a28);
  a29=casadi_sq(a26);
  a28=(a28+a29);
  a28=sqrt(a28);
  a26=(a26/a28);
  a8=(a8*a26);
  a24=casadi_sq(a24);
  a24=(a18*a24);
  a24=(a15+a24);
  a24=(a24*a10);
  a24=(a24*a13);
  a27=(a27*a24);
  a27=(a25?a27:0);
  a8=(a8+a27);
  a23=(a23+a8);
  a8=fabs(a11);
  a8=(a3<a8);
  a4=(a4/a11);
  a27=(-a12);
  a27=fabs(a27);
  a27=(a3<a27);
  a26=fabs(a6);
  a3=(a3<a26);
  a27=(a27&&a3);
  a3=(a12/a6);
  a3=(-a3);
  a3=atan(a3);
  a3=(-a3);
  a27=(a27?a3:0);
  a7=(a7-a21);
  a27=(a27+a7);
  a2=(a2*a27);
  a27=casadi_sq(a2);
  a18=(a18*a27);
  a15=(a15+a18);
  a15=(a15*a10);
  a15=(a15*a13);
  a4=(a4*a15);
  a4=(a8?a4:0);
  a4=(a23+a4);
  a9=(a9-a4);
  a9=(a0*a9);
  a19=(a19*a0);
  a19=(a16?a19:0);
  a23=(a0*a23);
  a19=(a19+a23);
  a4=(a0*a4);
  a19=(a19+a4);
  a9=(a9-a19);
  if (res[0]!=0) res[0][1]=a9;
  a20=(a1+a20);
  a20=(a20*a10);
  a20=(a20*a13);
  a9=(a6/a11);
  a9=(a16?a9:0);
  a19=(!a16);
  a19=(a19?a17:0);
  a9=(a9+a19);
  a19=(a12/a11);
  a4=(a16?a19:0);
  a4=casadi_sq(a4);
  a23=casadi_sq(a9);
  a4=(a4+a23);
  a4=sqrt(a4);
  a9=(a9/a4);
  a20=(a20*a9);
  a19=(a19*a22);
  a16=(a16?a19:0);
  a20=(a20-a16);
  a16=(a0*a20);
  a19=(a12/a11);
  a19=(a19*a24);
  a25=(a25?a19:0);
  a20=(a20-a25);
  a25=(a0*a20);
  a16=(a16+a25);
  a1=(a1+a2);
  a1=(a1*a10);
  a1=(a1*a13);
  a6=(a6/a11);
  a6=(a8?a6:0);
  a13=(!a8);
  a13=(a13?a17:0);
  a6=(a6+a13);
  a13=(a12/a11);
  a17=(-a13);
  a17=(a8?a17:0);
  a17=casadi_sq(a17);
  a10=casadi_sq(a6);
  a17=(a17+a10);
  a17=sqrt(a17);
  a6=(a6/a17);
  a1=(a1*a6);
  a13=(a13*a15);
  a8=(a8?a13:0);
  a1=(a1+a8);
  a20=(a20-a1);
  a1=(a0*a20);
  a16=(a16+a1);
  a12=(a12/a11);
  a12=(a12*a5);
  a14=(a14?a12:0);
  a20=(a20-a14);
  a0=(a0*a20);
  a16=(a16+a0);
  a16=(-a16);
  if (res[0]!=0) res[0][2]=a16;
  return 0;
}

CASADI_SYMBOL_EXPORT int rocket_aero_moments(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f1(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void rocket_aero_moments_incref(void) {
}

CASADI_SYMBOL_EXPORT void rocket_aero_moments_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int rocket_aero_moments_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int rocket_aero_moments_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* rocket_aero_moments_name_in(casadi_int i){
  switch (i) {
    case 0: return "x";
    case 1: return "u";
    case 2: return "p";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rocket_aero_moments_name_out(casadi_int i){
  switch (i) {
    case 0: return "MA_b";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rocket_aero_moments_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rocket_aero_moments_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int rocket_aero_moments_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT casadi_functions* rocket_aero_moments_functions(void) {
  static casadi_functions fun = {
    rocket_aero_moments_incref,
    rocket_aero_moments_decref,
    rocket_aero_moments_n_in,
    rocket_aero_moments_n_out,
    rocket_aero_moments_name_in,
    rocket_aero_moments_name_out,
    rocket_aero_moments_sparsity_in,
    rocket_aero_moments_sparsity_out,
    rocket_aero_moments_work,
    rocket_aero_moments
  };
  return &fun;
}
/* rocket_prop_forces:(x[14],u[4],p[15])->(FP_b[3]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0, a1, a2, a3;
  a0=0.;
  a1=arg[0] ? arg[0][13] : 0;
  a1=(a0<a1);
  a2=arg[1] ? arg[1][0] : 0;
  a3=arg[2] ? arg[2][5] : 0;
  a2=(a2*a3);
  a1=(a1?a2:0);
  if (res[0]!=0) res[0][0]=a1;
  if (res[0]!=0) res[0][1]=a0;
  if (res[0]!=0) res[0][2]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int rocket_prop_forces(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f2(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void rocket_prop_forces_incref(void) {
}

CASADI_SYMBOL_EXPORT void rocket_prop_forces_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int rocket_prop_forces_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int rocket_prop_forces_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* rocket_prop_forces_name_in(casadi_int i){
  switch (i) {
    case 0: return "x";
    case 1: return "u";
    case 2: return "p";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rocket_prop_forces_name_out(casadi_int i){
  switch (i) {
    case 0: return "FP_b";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rocket_prop_forces_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rocket_prop_forces_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int rocket_prop_forces_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT casadi_functions* rocket_prop_forces_functions(void) {
  static casadi_functions fun = {
    rocket_prop_forces_incref,
    rocket_prop_forces_decref,
    rocket_prop_forces_n_in,
    rocket_prop_forces_n_out,
    rocket_prop_forces_name_in,
    rocket_prop_forces_name_out,
    rocket_prop_forces_sparsity_in,
    rocket_prop_forces_sparsity_out,
    rocket_prop_forces_work,
    rocket_prop_forces
  };
  return &fun;
}
/* rocket_prop_moments:(x[14],u[4],p[15])->(MP_b[3]) */
static int casadi_f3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  if (res[0]!=0) res[0][2]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int rocket_prop_moments(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f3(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void rocket_prop_moments_incref(void) {
}

CASADI_SYMBOL_EXPORT void rocket_prop_moments_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int rocket_prop_moments_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int rocket_prop_moments_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* rocket_prop_moments_name_in(casadi_int i){
  switch (i) {
    case 0: return "x";
    case 1: return "u";
    case 2: return "p";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* rocket_prop_moments_name_out(casadi_int i){
  switch (i) {
    case 0: return "MP_b";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rocket_prop_moments_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* rocket_prop_moments_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int rocket_prop_moments_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT casadi_functions* rocket_prop_moments_functions(void) {
  static casadi_functions fun = {
    rocket_prop_moments_incref,
    rocket_prop_moments_decref,
    rocket_prop_moments_n_in,
    rocket_prop_moments_n_out,
    rocket_prop_moments_name_in,
    rocket_prop_moments_name_out,
    rocket_prop_moments_sparsity_in,
    rocket_prop_moments_sparsity_out,
    rocket_prop_moments_work,
    rocket_prop_moments
  };
  return &fun;
}
/* quat2mrp:(q[4])->(mrp[4]) */
static int casadi_f4(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6;
  a0=1.;
  a1=arg[0] ? arg[0][1] : 0;
  a2=arg[0] ? arg[0][0] : 0;
  a2=(a0+a2);
  a1=(a1/a2);
  a3=casadi_sq(a1);
  a4=arg[0] ? arg[0][2] : 0;
  a4=(a4/a2);
  a5=casadi_sq(a4);
  a3=(a3+a5);
  a5=arg[0] ? arg[0][3] : 0;
  a5=(a5/a2);
  a2=casadi_sq(a5);
  a3=(a3+a2);
  a3=sqrt(a3);
  a0=(a0<a3);
  a3=casadi_sq(a1);
  a2=casadi_sq(a4);
  a3=(a3+a2);
  a2=casadi_sq(a5);
  a3=(a3+a2);
  a2=(a1/a3);
  a2=(-a2);
  a2=(a0?a2:0);
  a6=(!a0);
  a1=(a6?a1:0);
  a2=(a2+a1);
  if (res[0]!=0) res[0][0]=a2;
  a2=(a4/a3);
  a2=(-a2);
  a2=(a0?a2:0);
  a4=(a6?a4:0);
  a2=(a2+a4);
  if (res[0]!=0) res[0][1]=a2;
  a3=(a5/a3);
  a3=(-a3);
  a0=(a0?a3:0);
  a6=(a6?a5:0);
  a0=(a0+a6);
  if (res[0]!=0) res[0][2]=a0;
  a0=0.;
  if (res[0]!=0) res[0][3]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int quat2mrp(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f4(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void quat2mrp_incref(void) {
}

CASADI_SYMBOL_EXPORT void quat2mrp_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quat2mrp_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int quat2mrp_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* quat2mrp_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quat2mrp_name_out(casadi_int i){
  switch (i) {
    case 0: return "mrp";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quat2mrp_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quat2mrp_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quat2mrp_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT casadi_functions* quat2mrp_functions(void) {
  static casadi_functions fun = {
    quat2mrp_incref,
    quat2mrp_decref,
    quat2mrp_n_in,
    quat2mrp_n_out,
    quat2mrp_name_in,
    quat2mrp_name_out,
    quat2mrp_sparsity_in,
    quat2mrp_sparsity_out,
    quat2mrp_work,
    quat2mrp
  };
  return &fun;
}
/* quat2dcm:(q[4])->(dcm[3x3]) */
static int casadi_f5(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0] ? arg[0][0] : 0;
  a1=casadi_sq(a0);
  a2=arg[0] ? arg[0][1] : 0;
  a3=casadi_sq(a2);
  a4=(a1+a3);
  a5=arg[0] ? arg[0][2] : 0;
  a6=casadi_sq(a5);
  a4=(a4-a6);
  a7=arg[0] ? arg[0][3] : 0;
  a8=casadi_sq(a7);
  a4=(a4-a8);
  if (res[0]!=0) res[0][0]=a4;
  a4=2.;
  a9=(a2*a5);
  a10=(a0*a7);
  a11=(a9+a10);
  a11=(a4*a11);
  if (res[0]!=0) res[0][1]=a11;
  a11=(a2*a7);
  a12=(a0*a5);
  a13=(a11-a12);
  a13=(a4*a13);
  if (res[0]!=0) res[0][2]=a13;
  a9=(a9-a10);
  a9=(a4*a9);
  if (res[0]!=0) res[0][3]=a9;
  a9=(a1+a6);
  a9=(a9-a3);
  a9=(a9-a8);
  if (res[0]!=0) res[0][4]=a9;
  a5=(a5*a7);
  a0=(a0*a2);
  a2=(a5+a0);
  a2=(a4*a2);
  if (res[0]!=0) res[0][5]=a2;
  a11=(a11+a12);
  a11=(a4*a11);
  if (res[0]!=0) res[0][6]=a11;
  a5=(a5-a0);
  a4=(a4*a5);
  if (res[0]!=0) res[0][7]=a4;
  a1=(a1+a8);
  a1=(a1-a3);
  a1=(a1-a6);
  if (res[0]!=0) res[0][8]=a1;
  return 0;
}

CASADI_SYMBOL_EXPORT int quat2dcm(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f5(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void quat2dcm_incref(void) {
}

CASADI_SYMBOL_EXPORT void quat2dcm_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int quat2dcm_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int quat2dcm_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* quat2dcm_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* quat2dcm_name_out(casadi_int i){
  switch (i) {
    case 0: return "dcm";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quat2dcm_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* quat2dcm_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int quat2dcm_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT casadi_functions* quat2dcm_functions(void) {
  static casadi_functions fun = {
    quat2dcm_incref,
    quat2dcm_decref,
    quat2dcm_n_in,
    quat2dcm_n_out,
    quat2dcm_name_in,
    quat2dcm_name_out,
    quat2dcm_sparsity_in,
    quat2dcm_sparsity_out,
    quat2dcm_work,
    quat2dcm
  };
  return &fun;
}
/* tf_linvel:(v_e[3],q[4])->(v_b[3]) */
static int casadi_f6(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[1] ? arg[1][0] : 0;
  a1=casadi_sq(a0);
  a2=arg[1] ? arg[1][1] : 0;
  a3=casadi_sq(a2);
  a4=(a1+a3);
  a5=arg[1] ? arg[1][2] : 0;
  a6=casadi_sq(a5);
  a4=(a4-a6);
  a7=arg[1] ? arg[1][3] : 0;
  a8=casadi_sq(a7);
  a4=(a4-a8);
  a9=arg[0] ? arg[0][0] : 0;
  a4=(a4*a9);
  a10=2.;
  a11=(a2*a5);
  a12=(a0*a7);
  a13=(a11-a12);
  a13=(a10*a13);
  a14=arg[0] ? arg[0][1] : 0;
  a13=(a13*a14);
  a4=(a4+a13);
  a13=(a2*a7);
  a15=(a0*a5);
  a16=(a13+a15);
  a16=(a10*a16);
  a17=arg[0] ? arg[0][2] : 0;
  a16=(a16*a17);
  a4=(a4+a16);
  if (res[0]!=0) res[0][0]=a4;
  a11=(a11+a12);
  a11=(a10*a11);
  a11=(a11*a9);
  a12=(a1+a6);
  a12=(a12-a3);
  a12=(a12-a8);
  a12=(a12*a14);
  a11=(a11+a12);
  a5=(a5*a7);
  a0=(a0*a2);
  a2=(a5-a0);
  a2=(a10*a2);
  a2=(a2*a17);
  a11=(a11+a2);
  if (res[0]!=0) res[0][1]=a11;
  a13=(a13-a15);
  a13=(a10*a13);
  a13=(a13*a9);
  a5=(a5+a0);
  a10=(a10*a5);
  a10=(a10*a14);
  a13=(a13+a10);
  a1=(a1+a8);
  a1=(a1-a3);
  a1=(a1-a6);
  a1=(a1*a17);
  a13=(a13+a1);
  if (res[0]!=0) res[0][2]=a13;
  return 0;
}

CASADI_SYMBOL_EXPORT int tf_linvel(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f6(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void tf_linvel_incref(void) {
}

CASADI_SYMBOL_EXPORT void tf_linvel_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int tf_linvel_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int tf_linvel_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* tf_linvel_name_in(casadi_int i){
  switch (i) {
    case 0: return "v_e";
    case 1: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* tf_linvel_name_out(casadi_int i){
  switch (i) {
    case 0: return "v_b";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* tf_linvel_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* tf_linvel_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int tf_linvel_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT casadi_functions* tf_linvel_functions(void) {
  static casadi_functions fun = {
    tf_linvel_incref,
    tf_linvel_decref,
    tf_linvel_n_in,
    tf_linvel_n_out,
    tf_linvel_name_in,
    tf_linvel_name_out,
    tf_linvel_sparsity_in,
    tf_linvel_sparsity_out,
    tf_linvel_work,
    tf_linvel
  };
  return &fun;
}
/* tf_angvel:(omega_e[3],q[4])->(omega_b[3]) */
static int casadi_f7(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[1] ? arg[1][0] : 0;
  a1=casadi_sq(a0);
  a2=arg[1] ? arg[1][1] : 0;
  a3=casadi_sq(a2);
  a4=(a1+a3);
  a5=arg[1] ? arg[1][2] : 0;
  a6=casadi_sq(a5);
  a4=(a4-a6);
  a7=arg[1] ? arg[1][3] : 0;
  a8=casadi_sq(a7);
  a4=(a4-a8);
  a9=arg[0] ? arg[0][0] : 0;
  a4=(a4*a9);
  a10=2.;
  a11=(a2*a5);
  a12=(a0*a7);
  a13=(a11-a12);
  a13=(a10*a13);
  a14=arg[0] ? arg[0][1] : 0;
  a13=(a13*a14);
  a4=(a4+a13);
  a13=(a2*a7);
  a15=(a0*a5);
  a16=(a13+a15);
  a16=(a10*a16);
  a17=arg[0] ? arg[0][2] : 0;
  a16=(a16*a17);
  a4=(a4+a16);
  if (res[0]!=0) res[0][0]=a4;
  a11=(a11+a12);
  a11=(a10*a11);
  a11=(a11*a9);
  a12=(a1+a6);
  a12=(a12-a3);
  a12=(a12-a8);
  a12=(a12*a14);
  a11=(a11+a12);
  a5=(a5*a7);
  a0=(a0*a2);
  a2=(a5-a0);
  a2=(a10*a2);
  a2=(a2*a17);
  a11=(a11+a2);
  if (res[0]!=0) res[0][1]=a11;
  a13=(a13-a15);
  a13=(a10*a13);
  a13=(a13*a9);
  a5=(a5+a0);
  a10=(a10*a5);
  a10=(a10*a14);
  a13=(a13+a10);
  a1=(a1+a8);
  a1=(a1-a3);
  a1=(a1-a6);
  a1=(a1*a17);
  a13=(a13+a1);
  if (res[0]!=0) res[0][2]=a13;
  return 0;
}

CASADI_SYMBOL_EXPORT int tf_angvel(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f7(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void tf_angvel_incref(void) {
}

CASADI_SYMBOL_EXPORT void tf_angvel_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int tf_angvel_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int tf_angvel_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* tf_angvel_name_in(casadi_int i){
  switch (i) {
    case 0: return "omega_e";
    case 1: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* tf_angvel_name_out(casadi_int i){
  switch (i) {
    case 0: return "omega_b";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* tf_angvel_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* tf_angvel_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int tf_angvel_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT casadi_functions* tf_angvel_functions(void) {
  static casadi_functions fun = {
    tf_angvel_incref,
    tf_angvel_decref,
    tf_angvel_n_in,
    tf_angvel_n_out,
    tf_angvel_name_in,
    tf_angvel_name_out,
    tf_angvel_sparsity_in,
    tf_angvel_sparsity_out,
    tf_angvel_work,
    tf_angvel
  };
  return &fun;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
