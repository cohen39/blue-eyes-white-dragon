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
  #define CASADI_PREFIX(ID) jit_tmp_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
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
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

casadi_real casadi_sq(casadi_real x) { return x*x;}

/* predict:(i0[14],i1[4],i2[15],i3,i4)->(o0[14]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a11, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a12, a120, a121, a122, a123, a124, a125, a126, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=arg[0] ? arg[0][0] : 0;
  a1=arg[4] ? arg[4][0] : 0;
  a2=arg[2] ? arg[2][3] : 0;
  a3=arg[2] ? arg[2][2] : 0;
  a4=arg[0] ? arg[0][13] : 0;
  a5=arg[2] ? arg[2][14] : 0;
  a6=casadi_sq(a5);
  a7=(a4*a6);
  a7=(a3+a7);
  a8=(a2*a7);
  a9=arg[2] ? arg[2][1] : 0;
  a5=casadi_sq(a5);
  a10=(a4*a5);
  a10=(a9+a10);
  a11=(a10*a2);
  a12=arg[2] ? arg[2][4] : 0;
  a13=casadi_sq(a12);
  a11=(a11-a13);
  a11=(a7*a11);
  a8=(a8/a11);
  a14=arg[0] ? arg[0][1] : 0;
  a15=(a12*a0);
  a16=arg[0] ? arg[0][2] : 0;
  a17=(a2*a16);
  a15=(a15+a17);
  a17=(a14*a15);
  a18=(a7*a14);
  a19=(a16*a18);
  a17=(a17-a19);
  a8=(a8*a17);
  a19=(a12*a7);
  a19=(a19/a11);
  a20=1.0000000000000000e-03;
  a21=arg[0] ? arg[0][7] : 0;
  a22=casadi_sq(a21);
  a23=arg[0] ? arg[0][8] : 0;
  a24=casadi_sq(a23);
  a22=(a22+a24);
  a24=arg[0] ? arg[0][9] : 0;
  a25=casadi_sq(a24);
  a22=(a22+a25);
  a22=sqrt(a22);
  a25=fabs(a22);
  a25=(a20<a25);
  a26=fabs(a21);
  a26=(a20<a26);
  a27=(a23/a21);
  a27=atan(a27);
  a27=(-a27);
  a26=(a26?a27:0);
  a27=fabs(a26);
  a28=4.0000000000000002e-01;
  a27=(a27<a28);
  a29=arg[2] ? arg[2][8] : 0;
  a30=arg[2] ? arg[2][7] : 0;
  a31=arg[1] ? arg[1][1] : 0;
  a32=arg[1] ? arg[1][3] : 0;
  a33=(a31+a32);
  a26=(a26+a33);
  a26=(a30*a26);
  a26=(a29+a26);
  a34=5.0000000000000000e-01;
  a35=arg[2] ? arg[2][12] : 0;
  a34=(a34*a35);
  a35=casadi_sq(a21);
  a36=casadi_sq(a23);
  a35=(a35+a36);
  a36=casadi_sq(a24);
  a35=(a35+a36);
  a35=(a34*a35);
  a36=(a26*a35);
  a37=arg[2] ? arg[2][11] : 0;
  a36=(a36*a37);
  a38=(a23/a22);
  a39=casadi_sq(a38);
  a40=(a21/a22);
  a41=casadi_sq(a40);
  a39=(a39+a41);
  a39=sqrt(a39);
  a41=fabs(a39);
  a41=(a20<a41);
  a42=(a40/a39);
  a42=(a41?a42:0);
  a43=(!a41);
  a44=1.;
  a43=(a43?a44:0);
  a42=(a42+a43);
  a42=(a36*a42);
  a42=(a27?a42:0);
  a43=arg[2] ? arg[2][9] : 0;
  a45=arg[2] ? arg[2][10] : 0;
  a26=(a27?a26:0);
  a26=(a26-a29);
  a26=casadi_sq(a26);
  a26=(a45*a26);
  a26=(a43+a26);
  a26=(a26*a35);
  a26=(a26*a37);
  a46=(a26*a38);
  a42=(a42-a46);
  a46=fabs(a21);
  a46=(a20<a46);
  a47=(a24/a21);
  a47=(-a47);
  a47=atan(a47);
  a47=(-a47);
  a46=(a46?a47:0);
  a47=fabs(a46);
  a47=(a47<a28);
  a48=arg[1] ? arg[1][2] : 0;
  a49=(a31+a48);
  a46=(a46+a49);
  a46=(a30*a46);
  a46=(a29+a46);
  a50=(a47?a46:0);
  a50=(a50-a29);
  a50=casadi_sq(a50);
  a50=(a45*a50);
  a50=(a43+a50);
  a50=(a50*a35);
  a50=(a50*a37);
  a51=(a50*a38);
  a42=(a42-a51);
  a51=fabs(a21);
  a51=(a20<a51);
  a52=(a23/a21);
  a52=(-a52);
  a52=atan(a52);
  a52=(-a52);
  a51=(a51?a52:0);
  a52=fabs(a51);
  a52=(a52<a28);
  a32=(a31-a32);
  a51=(a51+a32);
  a51=(a30*a51);
  a51=(a29+a51);
  a53=(a51*a35);
  a53=(a53*a37);
  a54=casadi_sq(a38);
  a55=casadi_sq(a40);
  a54=(a54+a55);
  a54=sqrt(a54);
  a55=fabs(a54);
  a55=(a20<a55);
  a56=(a40/a54);
  a56=(-a56);
  a56=(a55?a56:0);
  a57=(!a55);
  a58=-1.;
  a57=(a57?a58:0);
  a56=(a56+a57);
  a56=(a53*a56);
  a56=(a52?a56:0);
  a51=(a52?a51:0);
  a51=(a51-a29);
  a51=casadi_sq(a51);
  a51=(a45*a51);
  a51=(a43+a51);
  a51=(a51*a35);
  a51=(a51*a37);
  a57=(a51*a38);
  a56=(a56-a57);
  a42=(a42+a56);
  a56=fabs(a21);
  a56=(a20<a56);
  a57=(a24/a21);
  a57=atan(a57);
  a57=(-a57);
  a56=(a56?a57:0);
  a57=fabs(a56);
  a57=(a57<a28);
  a31=(a31-a48);
  a56=(a56+a31);
  a56=(a30*a56);
  a56=(a29+a56);
  a48=(a57?a56:0);
  a48=(a48-a29);
  a48=casadi_sq(a48);
  a48=(a45*a48);
  a48=(a43+a48);
  a48=(a48*a35);
  a48=(a48*a37);
  a59=(a48*a38);
  a42=(a42-a59);
  a59=arg[2] ? arg[2][6] : 0;
  a60=(a42*a59);
  a60=(-a60);
  a60=(a25?a60:0);
  a18=(a0*a18);
  a61=(a10*a0);
  a62=(a12*a16);
  a61=(a61+a62);
  a62=(a14*a61);
  a18=(a18-a62);
  a60=(a60-a18);
  a19=(a19*a60);
  a8=(a8+a19);
  a8=(a1*a8);
  a19=2.;
  a18=0.;
  a62=(a18<a4);
  a63=arg[1] ? arg[1][0] : 0;
  a64=(a63*a1);
  a64=(a62?a64:0);
  a65=(a64/a19);
  a65=(a4-a65);
  a66=(a65*a6);
  a66=(a3+a66);
  a67=(a2*a66);
  a68=(a65*a5);
  a68=(a9+a68);
  a69=(a68*a2);
  a69=(a69-a13);
  a69=(a66*a69);
  a67=(a67/a69);
  a70=(a10*a2);
  a71=casadi_sq(a12);
  a70=(a70-a71);
  a70=(a70/a11);
  a46=(a46*a35);
  a46=(a46*a37);
  a22=(a24/a22);
  a72=casadi_sq(a22);
  a73=casadi_sq(a40);
  a72=(a72+a73);
  a72=sqrt(a72);
  a73=fabs(a72);
  a73=(a20<a73);
  a74=(a40/a72);
  a74=(-a74);
  a74=(a73?a74:0);
  a75=(!a73);
  a75=(a75?a58:0);
  a74=(a74+a75);
  a74=(a46*a74);
  a74=(a47?a74:0);
  a75=(a50*a22);
  a74=(a74-a75);
  a75=(a26*a22);
  a74=(a74-a75);
  a75=(a51*a22);
  a74=(a74-a75);
  a56=(a56*a35);
  a56=(a56*a37);
  a35=casadi_sq(a22);
  a75=casadi_sq(a40);
  a35=(a35+a75);
  a35=sqrt(a35);
  a75=fabs(a35);
  a75=(a20<a75);
  a76=(a40/a35);
  a76=(a75?a76:0);
  a77=(!a75);
  a77=(a77?a44:0);
  a76=(a76+a77);
  a76=(a56*a76);
  a76=(a57?a76:0);
  a77=(a48*a22);
  a76=(a76-a77);
  a74=(a74+a76);
  a76=(a74*a59);
  a76=(-a76);
  a76=(a25?a76:0);
  a61=(a16*a61);
  a15=(a0*a15);
  a61=(a61-a15);
  a76=(a76+a61);
  a70=(a70*a76);
  a70=(a1*a70);
  a76=(a70/a19);
  a76=(a14-a76);
  a61=(a8/a19);
  a61=(a0-a61);
  a15=(a12*a61);
  a77=(a12*a7);
  a77=(a77/a11);
  a77=(a77*a17);
  a10=(a10*a7);
  a10=(a10/a11);
  a10=(a10*a60);
  a77=(a77+a10);
  a77=(a1*a77);
  a10=(a77/a19);
  a10=(a16+a10);
  a60=(a2*a10);
  a15=(a15+a60);
  a60=(a76*a15);
  a11=(a66*a76);
  a7=(a10*a11);
  a60=(a60-a7);
  a67=(a67*a60);
  a7=(a12*a66);
  a7=(a7/a69);
  a39=(a38/a39);
  a39=(a39*a36);
  a39=(-a39);
  a41=(a41?a39:0);
  a27=(a27?a41:0);
  a26=(a26*a40);
  a27=(a27-a26);
  a72=(a22/a72);
  a72=(a72*a46);
  a73=(a73?a72:0);
  a47=(a47?a73:0);
  a50=(a50*a40);
  a47=(a47-a50);
  a27=(a27+a47);
  a38=(a38/a54);
  a38=(a38*a53);
  a55=(a55?a38:0);
  a52=(a52?a55:0);
  a51=(a51*a40);
  a52=(a52-a51);
  a27=(a27+a52);
  a22=(a22/a35);
  a22=(a22*a56);
  a22=(-a22);
  a75=(a75?a22:0);
  a57=(a57?a75:0);
  a48=(a48*a40);
  a57=(a57-a48);
  a27=(a27+a57);
  a27=(a25?a27:0);
  a57=arg[2] ? arg[2][5] : 0;
  a57=(a63*a57);
  a62=(a62?a57:0);
  a27=(a27+a62);
  a62=8.;
  a48=arg[0] ? arg[0][5] : 0;
  a40=arg[0] ? arg[0][3] : 0;
  a75=(a48*a40);
  a75=(a62*a75);
  a22=4.;
  a56=casadi_sq(a40);
  a35=arg[0] ? arg[0][4] : 0;
  a52=casadi_sq(a35);
  a56=(a56+a52);
  a52=casadi_sq(a48);
  a56=(a56+a52);
  a52=(a44-a56);
  a52=(a22*a52);
  a51=(a52*a35);
  a75=(a75-a51);
  a56=(a44+a56);
  a56=casadi_sq(a56);
  a75=(a75/a56);
  a51=arg[2] ? arg[2][13] : 0;
  a55=(a51+a4);
  a38=arg[2] ? arg[2][0] : 0;
  a53=(a55*a38);
  a54=(a75*a53);
  a27=(a27+a54);
  a27=(a27/a55);
  a54=(a14*a24);
  a47=(a16*a23);
  a54=(a54-a47);
  a27=(a27-a54);
  a27=(a1*a27);
  a54=(a27/a19);
  a54=(a21+a54);
  a47=casadi_sq(a54);
  a42=(a25?a42:0);
  a50=(a48*a35);
  a50=(a62*a50);
  a73=(a52*a40);
  a50=(a50+a73);
  a50=(a50/a56);
  a73=(a50*a53);
  a42=(a42+a73);
  a42=(a42/a55);
  a73=(a16*a21);
  a72=(a0*a24);
  a73=(a73-a72);
  a42=(a42-a73);
  a42=(a1*a42);
  a73=(a42/a19);
  a73=(a23+a73);
  a72=casadi_sq(a73);
  a47=(a47+a72);
  a25=(a25?a74:0);
  a74=casadi_sq(a35);
  a72=casadi_sq(a40);
  a74=(a74+a72);
  a74=(a62*a74);
  a74=(a74/a56);
  a74=(a44-a74);
  a53=(a74*a53);
  a25=(a25+a53);
  a25=(a25/a55);
  a55=(a0*a23);
  a53=(a14*a21);
  a55=(a55-a53);
  a25=(a25-a55);
  a25=(a1*a25);
  a55=(a25/a19);
  a55=(a24+a55);
  a53=casadi_sq(a55);
  a47=(a47+a53);
  a47=sqrt(a47);
  a53=fabs(a47);
  a53=(a20<a53);
  a72=fabs(a54);
  a72=(a20<a72);
  a46=(a73/a54);
  a46=atan(a46);
  a46=(-a46);
  a72=(a72?a46:0);
  a46=fabs(a72);
  a46=(a46<a28);
  a72=(a72+a33);
  a72=(a30*a72);
  a72=(a29+a72);
  a26=casadi_sq(a54);
  a41=casadi_sq(a73);
  a26=(a26+a41);
  a41=casadi_sq(a55);
  a26=(a26+a41);
  a26=(a34*a26);
  a41=(a72*a26);
  a41=(a41*a37);
  a39=(a73/a47);
  a36=casadi_sq(a39);
  a17=(a54/a47);
  a78=casadi_sq(a17);
  a36=(a36+a78);
  a36=sqrt(a36);
  a78=fabs(a36);
  a78=(a20<a78);
  a79=(a17/a36);
  a79=(a78?a79:0);
  a80=(!a78);
  a80=(a80?a44:0);
  a79=(a79+a80);
  a79=(a41*a79);
  a79=(a46?a79:0);
  a72=(a46?a72:0);
  a72=(a72-a29);
  a72=casadi_sq(a72);
  a72=(a45*a72);
  a72=(a43+a72);
  a72=(a72*a26);
  a72=(a72*a37);
  a80=(a72*a39);
  a79=(a79-a80);
  a80=fabs(a54);
  a80=(a20<a80);
  a81=(a55/a54);
  a81=(-a81);
  a81=atan(a81);
  a81=(-a81);
  a80=(a80?a81:0);
  a81=fabs(a80);
  a81=(a81<a28);
  a80=(a80+a49);
  a80=(a30*a80);
  a80=(a29+a80);
  a82=(a81?a80:0);
  a82=(a82-a29);
  a82=casadi_sq(a82);
  a82=(a45*a82);
  a82=(a43+a82);
  a82=(a82*a26);
  a82=(a82*a37);
  a83=(a82*a39);
  a79=(a79-a83);
  a83=fabs(a54);
  a83=(a20<a83);
  a84=(a73/a54);
  a84=(-a84);
  a84=atan(a84);
  a84=(-a84);
  a83=(a83?a84:0);
  a84=fabs(a83);
  a84=(a84<a28);
  a83=(a83+a32);
  a83=(a30*a83);
  a83=(a29+a83);
  a85=(a83*a26);
  a85=(a85*a37);
  a86=casadi_sq(a39);
  a87=casadi_sq(a17);
  a86=(a86+a87);
  a86=sqrt(a86);
  a87=fabs(a86);
  a87=(a20<a87);
  a88=(a17/a86);
  a88=(-a88);
  a88=(a87?a88:0);
  a89=(!a87);
  a89=(a89?a58:0);
  a88=(a88+a89);
  a88=(a85*a88);
  a88=(a84?a88:0);
  a83=(a84?a83:0);
  a83=(a83-a29);
  a83=casadi_sq(a83);
  a83=(a45*a83);
  a83=(a43+a83);
  a83=(a83*a26);
  a83=(a83*a37);
  a89=(a83*a39);
  a88=(a88-a89);
  a79=(a79+a88);
  a88=fabs(a54);
  a88=(a20<a88);
  a89=(a55/a54);
  a89=atan(a89);
  a89=(-a89);
  a88=(a88?a89:0);
  a89=fabs(a88);
  a89=(a89<a28);
  a88=(a88+a31);
  a88=(a30*a88);
  a88=(a29+a88);
  a90=(a89?a88:0);
  a90=(a90-a29);
  a90=casadi_sq(a90);
  a90=(a45*a90);
  a90=(a43+a90);
  a90=(a90*a26);
  a90=(a90*a37);
  a91=(a90*a39);
  a79=(a79-a91);
  a91=(a79*a59);
  a91=(-a91);
  a91=(a53?a91:0);
  a11=(a61*a11);
  a92=(a68*a61);
  a93=(a12*a10);
  a92=(a92+a93);
  a93=(a76*a92);
  a11=(a11-a93);
  a91=(a91-a11);
  a7=(a7*a91);
  a67=(a67+a7);
  a67=(a1*a67);
  a7=(a19*a67);
  a8=(a8+a7);
  a7=(a18<a65);
  a11=(a63*a1);
  a93=(a7?a11:0);
  a93=(a93/a19);
  a93=(a4-a93);
  a94=(a93*a6);
  a94=(a3+a94);
  a95=(a2*a94);
  a96=(a93*a5);
  a96=(a9+a96);
  a97=(a96*a2);
  a97=(a97-a13);
  a97=(a94*a97);
  a95=(a95/a97);
  a98=(a68*a2);
  a98=(a98-a71);
  a98=(a98/a69);
  a80=(a80*a26);
  a80=(a80*a37);
  a47=(a55/a47);
  a99=casadi_sq(a47);
  a100=casadi_sq(a17);
  a99=(a99+a100);
  a99=sqrt(a99);
  a100=fabs(a99);
  a100=(a20<a100);
  a101=(a17/a99);
  a101=(-a101);
  a101=(a100?a101:0);
  a102=(!a100);
  a102=(a102?a58:0);
  a101=(a101+a102);
  a101=(a80*a101);
  a101=(a81?a101:0);
  a102=(a82*a47);
  a101=(a101-a102);
  a102=(a72*a47);
  a101=(a101-a102);
  a102=(a83*a47);
  a101=(a101-a102);
  a88=(a88*a26);
  a88=(a88*a37);
  a26=casadi_sq(a47);
  a102=casadi_sq(a17);
  a26=(a26+a102);
  a26=sqrt(a26);
  a102=fabs(a26);
  a102=(a20<a102);
  a103=(a17/a26);
  a103=(a102?a103:0);
  a104=(!a102);
  a104=(a104?a44:0);
  a103=(a103+a104);
  a103=(a88*a103);
  a103=(a89?a103:0);
  a104=(a90*a47);
  a103=(a103-a104);
  a101=(a101+a103);
  a103=(a101*a59);
  a103=(-a103);
  a103=(a53?a103:0);
  a92=(a10*a92);
  a15=(a61*a15);
  a92=(a92-a15);
  a103=(a103+a92);
  a98=(a98*a103);
  a98=(a1*a98);
  a103=(a98/a19);
  a103=(a14-a103);
  a67=(a67/a19);
  a67=(a0-a67);
  a92=(a12*a67);
  a15=(a12*a66);
  a15=(a15/a69);
  a15=(a15*a60);
  a68=(a68*a66);
  a68=(a68/a69);
  a68=(a68*a91);
  a15=(a15+a68);
  a15=(a1*a15);
  a68=(a15/a19);
  a68=(a16+a68);
  a91=(a2*a68);
  a92=(a92+a91);
  a91=(a103*a92);
  a69=(a94*a103);
  a66=(a68*a69);
  a91=(a91-a66);
  a95=(a95*a91);
  a66=(a12*a94);
  a66=(a66/a97);
  a36=(a39/a36);
  a36=(a36*a41);
  a36=(-a36);
  a78=(a78?a36:0);
  a46=(a46?a78:0);
  a72=(a72*a17);
  a46=(a46-a72);
  a99=(a47/a99);
  a99=(a99*a80);
  a100=(a100?a99:0);
  a81=(a81?a100:0);
  a82=(a82*a17);
  a81=(a81-a82);
  a46=(a46+a81);
  a39=(a39/a86);
  a39=(a39*a85);
  a87=(a87?a39:0);
  a84=(a84?a87:0);
  a83=(a83*a17);
  a84=(a84-a83);
  a46=(a46+a84);
  a47=(a47/a26);
  a47=(a47*a88);
  a47=(-a47);
  a102=(a102?a47:0);
  a89=(a89?a102:0);
  a90=(a90*a17);
  a89=(a89-a90);
  a46=(a46+a89);
  a46=(a53?a46:0);
  a89=(a7?a57:0);
  a46=(a46+a89);
  a89=2.5000000000000000e-01;
  a90=(a48*a40);
  a90=(a19*a90);
  a17=(a19*a35);
  a90=(a90-a17);
  a90=(a89*a90);
  a90=(a90*a0);
  a17=(a19*a40);
  a102=(a48*a35);
  a102=(a19*a102);
  a17=(a17+a102);
  a17=(a89*a17);
  a17=(a17*a14);
  a90=(a90+a17);
  a17=casadi_sq(a40);
  a102=casadi_sq(a35);
  a17=(a17+a102);
  a102=casadi_sq(a48);
  a17=(a17+a102);
  a17=(a44-a17);
  a102=casadi_sq(a48);
  a102=(a19*a102);
  a102=(a17+a102);
  a102=(a89*a102);
  a102=(a102*a16);
  a90=(a90+a102);
  a90=(a1*a90);
  a102=(a90/a19);
  a102=(a48+a102);
  a47=casadi_sq(a40);
  a47=(a19*a47);
  a47=(a17+a47);
  a47=(a89*a47);
  a47=(a47*a0);
  a88=(a40*a35);
  a88=(a19*a88);
  a26=(a19*a48);
  a88=(a88-a26);
  a88=(a89*a88);
  a88=(a88*a14);
  a47=(a47+a88);
  a88=(a19*a35);
  a26=(a40*a48);
  a26=(a19*a26);
  a88=(a88+a26);
  a88=(a89*a88);
  a88=(a88*a16);
  a47=(a47+a88);
  a47=(a1*a47);
  a88=(a47/a19);
  a88=(a40+a88);
  a26=(a102*a88);
  a26=(a62*a26);
  a84=casadi_sq(a88);
  a83=(a19*a48);
  a87=(a35*a40);
  a87=(a19*a87);
  a83=(a83+a87);
  a83=(a89*a83);
  a83=(a83*a0);
  a87=casadi_sq(a35);
  a87=(a19*a87);
  a17=(a17+a87);
  a17=(a89*a17);
  a17=(a17*a14);
  a83=(a83+a17);
  a17=(a35*a48);
  a17=(a19*a17);
  a87=(a19*a40);
  a17=(a17-a87);
  a17=(a89*a17);
  a17=(a17*a16);
  a83=(a83+a17);
  a83=(a1*a83);
  a17=(a83/a19);
  a17=(a35+a17);
  a87=casadi_sq(a17);
  a84=(a84+a87);
  a87=casadi_sq(a102);
  a84=(a84+a87);
  a87=(a44-a84);
  a87=(a22*a87);
  a39=(a87*a17);
  a26=(a26-a39);
  a84=(a44+a84);
  a84=casadi_sq(a84);
  a26=(a26/a84);
  a65=(a51+a65);
  a39=(a65*a38);
  a85=(a26*a39);
  a46=(a46+a85);
  a46=(a46/a65);
  a85=(a76*a55);
  a86=(a10*a73);
  a85=(a85-a86);
  a46=(a46-a85);
  a46=(a1*a46);
  a85=(a46/a19);
  a85=(a21+a85);
  a86=casadi_sq(a85);
  a79=(a53?a79:0);
  a81=(a102*a17);
  a81=(a62*a81);
  a82=(a87*a88);
  a81=(a81+a82);
  a81=(a81/a84);
  a82=(a81*a39);
  a79=(a79+a82);
  a79=(a79/a65);
  a82=(a10*a54);
  a100=(a61*a55);
  a82=(a82-a100);
  a79=(a79-a82);
  a79=(a1*a79);
  a82=(a79/a19);
  a82=(a23+a82);
  a100=casadi_sq(a82);
  a86=(a86+a100);
  a53=(a53?a101:0);
  a101=casadi_sq(a17);
  a100=casadi_sq(a88);
  a101=(a101+a100);
  a101=(a62*a101);
  a101=(a101/a84);
  a101=(a44-a101);
  a39=(a101*a39);
  a53=(a53+a39);
  a53=(a53/a65);
  a65=(a61*a73);
  a39=(a76*a54);
  a65=(a65-a39);
  a53=(a53-a65);
  a53=(a1*a53);
  a65=(a53/a19);
  a65=(a24+a65);
  a39=casadi_sq(a65);
  a86=(a86+a39);
  a86=sqrt(a86);
  a39=fabs(a86);
  a39=(a20<a39);
  a100=fabs(a85);
  a100=(a20<a100);
  a99=(a82/a85);
  a99=atan(a99);
  a99=(-a99);
  a100=(a100?a99:0);
  a99=fabs(a100);
  a99=(a99<a28);
  a100=(a100+a33);
  a100=(a30*a100);
  a100=(a29+a100);
  a80=casadi_sq(a85);
  a72=casadi_sq(a82);
  a80=(a80+a72);
  a72=casadi_sq(a65);
  a80=(a80+a72);
  a80=(a34*a80);
  a72=(a100*a80);
  a72=(a72*a37);
  a78=(a82/a86);
  a36=casadi_sq(a78);
  a41=(a85/a86);
  a60=casadi_sq(a41);
  a36=(a36+a60);
  a36=sqrt(a36);
  a60=fabs(a36);
  a60=(a20<a60);
  a104=(a41/a36);
  a104=(a60?a104:0);
  a105=(!a60);
  a105=(a105?a44:0);
  a104=(a104+a105);
  a104=(a72*a104);
  a104=(a99?a104:0);
  a100=(a99?a100:0);
  a100=(a100-a29);
  a100=casadi_sq(a100);
  a100=(a45*a100);
  a100=(a43+a100);
  a100=(a100*a80);
  a100=(a100*a37);
  a105=(a100*a78);
  a104=(a104-a105);
  a105=fabs(a85);
  a105=(a20<a105);
  a106=(a65/a85);
  a106=(-a106);
  a106=atan(a106);
  a106=(-a106);
  a105=(a105?a106:0);
  a106=fabs(a105);
  a106=(a106<a28);
  a105=(a105+a49);
  a105=(a30*a105);
  a105=(a29+a105);
  a107=(a106?a105:0);
  a107=(a107-a29);
  a107=casadi_sq(a107);
  a107=(a45*a107);
  a107=(a43+a107);
  a107=(a107*a80);
  a107=(a107*a37);
  a108=(a107*a78);
  a104=(a104-a108);
  a108=fabs(a85);
  a108=(a20<a108);
  a109=(a82/a85);
  a109=(-a109);
  a109=atan(a109);
  a109=(-a109);
  a108=(a108?a109:0);
  a109=fabs(a108);
  a109=(a109<a28);
  a108=(a108+a32);
  a108=(a30*a108);
  a108=(a29+a108);
  a110=(a108*a80);
  a110=(a110*a37);
  a111=casadi_sq(a78);
  a112=casadi_sq(a41);
  a111=(a111+a112);
  a111=sqrt(a111);
  a112=fabs(a111);
  a112=(a20<a112);
  a113=(a41/a111);
  a113=(-a113);
  a113=(a112?a113:0);
  a114=(!a112);
  a114=(a114?a58:0);
  a113=(a113+a114);
  a113=(a110*a113);
  a113=(a109?a113:0);
  a108=(a109?a108:0);
  a108=(a108-a29);
  a108=casadi_sq(a108);
  a108=(a45*a108);
  a108=(a43+a108);
  a108=(a108*a80);
  a108=(a108*a37);
  a114=(a108*a78);
  a113=(a113-a114);
  a104=(a104+a113);
  a113=fabs(a85);
  a113=(a20<a113);
  a114=(a65/a85);
  a114=atan(a114);
  a114=(-a114);
  a113=(a113?a114:0);
  a114=fabs(a113);
  a114=(a114<a28);
  a113=(a113+a31);
  a113=(a30*a113);
  a113=(a29+a113);
  a115=(a114?a113:0);
  a115=(a115-a29);
  a115=casadi_sq(a115);
  a115=(a45*a115);
  a115=(a43+a115);
  a115=(a115*a80);
  a115=(a115*a37);
  a116=(a115*a78);
  a104=(a104-a116);
  a116=(a104*a59);
  a116=(-a116);
  a116=(a39?a116:0);
  a69=(a67*a69);
  a117=(a96*a67);
  a118=(a12*a68);
  a117=(a117+a118);
  a118=(a103*a117);
  a69=(a69-a118);
  a116=(a116-a69);
  a66=(a66*a116);
  a95=(a95+a66);
  a95=(a1*a95);
  a66=(a19*a95);
  a8=(a8+a66);
  a66=(a18<a93);
  a69=(a63*a1);
  a118=(a66?a69:0);
  a118=(a4-a118);
  a6=(a118*a6);
  a3=(a3+a6);
  a6=(a2*a3);
  a5=(a118*a5);
  a9=(a9+a5);
  a5=(a9*a2);
  a5=(a5-a13);
  a5=(a3*a5);
  a6=(a6/a5);
  a13=(a96*a2);
  a13=(a13-a71);
  a13=(a13/a97);
  a105=(a105*a80);
  a105=(a105*a37);
  a86=(a65/a86);
  a119=casadi_sq(a86);
  a120=casadi_sq(a41);
  a119=(a119+a120);
  a119=sqrt(a119);
  a120=fabs(a119);
  a120=(a20<a120);
  a121=(a41/a119);
  a121=(-a121);
  a121=(a120?a121:0);
  a122=(!a120);
  a122=(a122?a58:0);
  a121=(a121+a122);
  a121=(a105*a121);
  a121=(a106?a121:0);
  a122=(a107*a86);
  a121=(a121-a122);
  a122=(a100*a86);
  a121=(a121-a122);
  a122=(a108*a86);
  a121=(a121-a122);
  a113=(a113*a80);
  a113=(a113*a37);
  a80=casadi_sq(a86);
  a122=casadi_sq(a41);
  a80=(a80+a122);
  a80=sqrt(a80);
  a122=fabs(a80);
  a122=(a20<a122);
  a123=(a41/a80);
  a123=(a122?a123:0);
  a124=(!a122);
  a124=(a124?a44:0);
  a123=(a123+a124);
  a123=(a113*a123);
  a123=(a114?a123:0);
  a124=(a115*a86);
  a123=(a123-a124);
  a121=(a121+a123);
  a123=(a121*a59);
  a123=(-a123);
  a123=(a39?a123:0);
  a117=(a68*a117);
  a92=(a67*a92);
  a117=(a117-a92);
  a123=(a123+a117);
  a13=(a13*a123);
  a13=(a1*a13);
  a123=(a14-a13);
  a95=(a0-a95);
  a117=(a12*a95);
  a92=(a12*a94);
  a92=(a92/a97);
  a92=(a92*a91);
  a96=(a96*a94);
  a96=(a96/a97);
  a96=(a96*a116);
  a92=(a92+a96);
  a92=(a1*a92);
  a96=(a16+a92);
  a116=(a2*a96);
  a117=(a117+a116);
  a116=(a123*a117);
  a97=(a3*a123);
  a94=(a96*a97);
  a116=(a116-a94);
  a6=(a6*a116);
  a94=(a12*a3);
  a94=(a94/a5);
  a36=(a78/a36);
  a36=(a36*a72);
  a36=(-a36);
  a60=(a60?a36:0);
  a99=(a99?a60:0);
  a100=(a100*a41);
  a99=(a99-a100);
  a119=(a86/a119);
  a119=(a119*a105);
  a120=(a120?a119:0);
  a106=(a106?a120:0);
  a107=(a107*a41);
  a106=(a106-a107);
  a99=(a99+a106);
  a78=(a78/a111);
  a78=(a78*a110);
  a112=(a112?a78:0);
  a109=(a109?a112:0);
  a108=(a108*a41);
  a109=(a109-a108);
  a99=(a99+a109);
  a86=(a86/a80);
  a86=(a86*a113);
  a86=(-a86);
  a122=(a122?a86:0);
  a114=(a114?a122:0);
  a115=(a115*a41);
  a114=(a114-a115);
  a99=(a99+a114);
  a99=(a39?a99:0);
  a114=(a66?a57:0);
  a99=(a99+a114);
  a114=(a102*a88);
  a114=(a19*a114);
  a115=(a19*a17);
  a114=(a114-a115);
  a114=(a89*a114);
  a114=(a114*a61);
  a115=(a19*a88);
  a41=(a102*a17);
  a41=(a19*a41);
  a115=(a115+a41);
  a115=(a89*a115);
  a115=(a115*a76);
  a114=(a114+a115);
  a115=casadi_sq(a88);
  a41=casadi_sq(a17);
  a115=(a115+a41);
  a41=casadi_sq(a102);
  a115=(a115+a41);
  a115=(a44-a115);
  a41=casadi_sq(a102);
  a41=(a19*a41);
  a41=(a115+a41);
  a41=(a89*a41);
  a41=(a41*a10);
  a114=(a114+a41);
  a114=(a1*a114);
  a41=(a114/a19);
  a41=(a48+a41);
  a122=casadi_sq(a88);
  a122=(a19*a122);
  a122=(a115+a122);
  a122=(a89*a122);
  a122=(a122*a61);
  a86=(a88*a17);
  a86=(a19*a86);
  a113=(a19*a102);
  a86=(a86-a113);
  a86=(a89*a86);
  a86=(a86*a76);
  a122=(a122+a86);
  a86=(a19*a17);
  a113=(a88*a102);
  a113=(a19*a113);
  a86=(a86+a113);
  a86=(a89*a86);
  a86=(a86*a10);
  a122=(a122+a86);
  a122=(a1*a122);
  a86=(a122/a19);
  a86=(a40+a86);
  a113=(a41*a86);
  a113=(a62*a113);
  a80=casadi_sq(a86);
  a109=(a19*a102);
  a108=(a17*a88);
  a108=(a19*a108);
  a109=(a109+a108);
  a109=(a89*a109);
  a109=(a109*a61);
  a61=casadi_sq(a17);
  a61=(a19*a61);
  a115=(a115+a61);
  a115=(a89*a115);
  a115=(a115*a76);
  a109=(a109+a115);
  a115=(a17*a102);
  a115=(a19*a115);
  a76=(a19*a88);
  a115=(a115-a76);
  a115=(a89*a115);
  a115=(a115*a10);
  a109=(a109+a115);
  a109=(a1*a109);
  a115=(a109/a19);
  a115=(a35+a115);
  a10=casadi_sq(a115);
  a80=(a80+a10);
  a10=casadi_sq(a41);
  a80=(a80+a10);
  a10=(a44-a80);
  a10=(a22*a10);
  a76=(a10*a115);
  a113=(a113-a76);
  a80=(a44+a80);
  a80=casadi_sq(a80);
  a113=(a113/a80);
  a93=(a51+a93);
  a76=(a93*a38);
  a61=(a113*a76);
  a99=(a99+a61);
  a99=(a99/a93);
  a61=(a103*a65);
  a108=(a68*a82);
  a61=(a61-a108);
  a99=(a99-a61);
  a99=(a1*a99);
  a61=(a21+a99);
  a108=casadi_sq(a61);
  a104=(a39?a104:0);
  a112=(a41*a115);
  a112=(a62*a112);
  a78=(a10*a86);
  a112=(a112+a78);
  a112=(a112/a80);
  a78=(a112*a76);
  a104=(a104+a78);
  a104=(a104/a93);
  a78=(a68*a85);
  a110=(a67*a65);
  a78=(a78-a110);
  a104=(a104-a78);
  a104=(a1*a104);
  a78=(a23+a104);
  a110=casadi_sq(a78);
  a108=(a108+a110);
  a39=(a39?a121:0);
  a121=casadi_sq(a115);
  a110=casadi_sq(a86);
  a121=(a121+a110);
  a121=(a62*a121);
  a121=(a121/a80);
  a121=(a44-a121);
  a76=(a121*a76);
  a39=(a39+a76);
  a39=(a39/a93);
  a93=(a67*a82);
  a76=(a103*a85);
  a93=(a93-a76);
  a39=(a39-a93);
  a39=(a1*a39);
  a93=(a24+a39);
  a76=casadi_sq(a93);
  a108=(a108+a76);
  a108=sqrt(a108);
  a76=fabs(a108);
  a76=(a20<a76);
  a110=fabs(a61);
  a110=(a20<a110);
  a111=(a78/a61);
  a111=atan(a111);
  a111=(-a111);
  a110=(a110?a111:0);
  a111=fabs(a110);
  a111=(a111<a28);
  a110=(a110+a33);
  a110=(a30*a110);
  a110=(a29+a110);
  a33=casadi_sq(a61);
  a106=casadi_sq(a78);
  a33=(a33+a106);
  a106=casadi_sq(a93);
  a33=(a33+a106);
  a34=(a34*a33);
  a33=(a110*a34);
  a33=(a33*a37);
  a106=(a78/a108);
  a107=casadi_sq(a106);
  a120=(a61/a108);
  a119=casadi_sq(a120);
  a107=(a107+a119);
  a107=sqrt(a107);
  a119=fabs(a107);
  a119=(a20<a119);
  a105=(a120/a107);
  a105=(a119?a105:0);
  a100=(!a119);
  a100=(a100?a44:0);
  a105=(a105+a100);
  a105=(a33*a105);
  a105=(a111?a105:0);
  a110=(a111?a110:0);
  a110=(a110-a29);
  a110=casadi_sq(a110);
  a110=(a45*a110);
  a110=(a43+a110);
  a110=(a110*a34);
  a110=(a110*a37);
  a100=(a110*a106);
  a105=(a105-a100);
  a100=fabs(a61);
  a100=(a20<a100);
  a60=(a93/a61);
  a60=(-a60);
  a60=atan(a60);
  a60=(-a60);
  a100=(a100?a60:0);
  a60=fabs(a100);
  a60=(a60<a28);
  a100=(a100+a49);
  a100=(a30*a100);
  a100=(a29+a100);
  a49=(a60?a100:0);
  a49=(a49-a29);
  a49=casadi_sq(a49);
  a49=(a45*a49);
  a49=(a43+a49);
  a49=(a49*a34);
  a49=(a49*a37);
  a36=(a49*a106);
  a105=(a105-a36);
  a36=fabs(a61);
  a36=(a20<a36);
  a72=(a78/a61);
  a72=(-a72);
  a72=atan(a72);
  a72=(-a72);
  a36=(a36?a72:0);
  a72=fabs(a36);
  a72=(a72<a28);
  a36=(a36+a32);
  a36=(a30*a36);
  a36=(a29+a36);
  a32=(a36*a34);
  a32=(a32*a37);
  a91=casadi_sq(a106);
  a124=casadi_sq(a120);
  a91=(a91+a124);
  a91=sqrt(a91);
  a124=fabs(a91);
  a124=(a20<a124);
  a125=(a120/a91);
  a125=(-a125);
  a125=(a124?a125:0);
  a126=(!a124);
  a126=(a126?a58:0);
  a125=(a125+a126);
  a125=(a32*a125);
  a125=(a72?a125:0);
  a36=(a72?a36:0);
  a36=(a36-a29);
  a36=casadi_sq(a36);
  a36=(a45*a36);
  a36=(a43+a36);
  a36=(a36*a34);
  a36=(a36*a37);
  a126=(a36*a106);
  a125=(a125-a126);
  a105=(a105+a125);
  a125=fabs(a61);
  a125=(a20<a125);
  a126=(a93/a61);
  a126=atan(a126);
  a126=(-a126);
  a125=(a125?a126:0);
  a126=fabs(a125);
  a126=(a126<a28);
  a125=(a125+a31);
  a30=(a30*a125);
  a30=(a29+a30);
  a125=(a126?a30:0);
  a125=(a125-a29);
  a125=casadi_sq(a125);
  a45=(a45*a125);
  a43=(a43+a45);
  a43=(a43*a34);
  a43=(a43*a37);
  a45=(a43*a106);
  a105=(a105-a45);
  a45=(a105*a59);
  a45=(-a45);
  a45=(a76?a45:0);
  a97=(a95*a97);
  a125=(a9*a95);
  a29=(a12*a96);
  a125=(a125+a29);
  a29=(a123*a125);
  a97=(a97-a29);
  a45=(a45-a97);
  a94=(a94*a45);
  a6=(a6+a94);
  a6=(a1*a6);
  a8=(a8+a6);
  a6=6.;
  a8=(a8/a6);
  a0=(a0-a8);
  if (res[0]!=0) res[0][0]=a0;
  a98=(a19*a98);
  a70=(a70+a98);
  a13=(a19*a13);
  a70=(a70+a13);
  a2=(a9*a2);
  a2=(a2-a71);
  a2=(a2/a5);
  a100=(a100*a34);
  a100=(a100*a37);
  a108=(a93/a108);
  a71=casadi_sq(a108);
  a13=casadi_sq(a120);
  a71=(a71+a13);
  a71=sqrt(a71);
  a13=fabs(a71);
  a13=(a20<a13);
  a98=(a120/a71);
  a98=(-a98);
  a98=(a13?a98:0);
  a0=(!a13);
  a0=(a0?a58:0);
  a98=(a98+a0);
  a98=(a100*a98);
  a98=(a60?a98:0);
  a0=(a49*a108);
  a98=(a98-a0);
  a0=(a110*a108);
  a98=(a98-a0);
  a0=(a36*a108);
  a98=(a98-a0);
  a30=(a30*a34);
  a30=(a30*a37);
  a37=casadi_sq(a108);
  a34=casadi_sq(a120);
  a37=(a37+a34);
  a37=sqrt(a37);
  a34=fabs(a37);
  a20=(a20<a34);
  a34=(a120/a37);
  a34=(a20?a34:0);
  a0=(!a20);
  a0=(a0?a44:0);
  a34=(a34+a0);
  a34=(a30*a34);
  a34=(a126?a34:0);
  a0=(a43*a108);
  a34=(a34-a0);
  a98=(a98+a34);
  a59=(a98*a59);
  a59=(-a59);
  a59=(a76?a59:0);
  a125=(a96*a125);
  a117=(a95*a117);
  a125=(a125-a117);
  a59=(a59+a125);
  a2=(a2*a59);
  a2=(a1*a2);
  a70=(a70+a2);
  a70=(a70/a6);
  a14=(a14-a70);
  if (res[0]!=0) res[0][1]=a14;
  a15=(a19*a15);
  a77=(a77+a15);
  a92=(a19*a92);
  a77=(a77+a92);
  a12=(a12*a3);
  a12=(a12/a5);
  a12=(a12*a116);
  a9=(a9*a3);
  a9=(a9/a5);
  a9=(a9*a45);
  a12=(a12+a9);
  a12=(a1*a12);
  a77=(a77+a12);
  a77=(a77/a6);
  a16=(a16+a77);
  if (res[0]!=0) res[0][2]=a16;
  a122=(a19*a122);
  a47=(a47+a122);
  a122=casadi_sq(a86);
  a16=casadi_sq(a115);
  a122=(a122+a16);
  a16=casadi_sq(a41);
  a122=(a122+a16);
  a122=(a44-a122);
  a16=casadi_sq(a86);
  a16=(a19*a16);
  a16=(a122+a16);
  a16=(a89*a16);
  a16=(a16*a67);
  a77=(a86*a115);
  a77=(a19*a77);
  a12=(a19*a41);
  a77=(a77-a12);
  a77=(a89*a77);
  a77=(a77*a103);
  a16=(a16+a77);
  a77=(a19*a115);
  a12=(a86*a41);
  a12=(a19*a12);
  a77=(a77+a12);
  a77=(a89*a77);
  a77=(a77*a68);
  a16=(a16+a77);
  a16=(a1*a16);
  a77=(a19*a16);
  a47=(a47+a77);
  a16=(a40+a16);
  a77=casadi_sq(a16);
  a12=(a19*a41);
  a9=(a115*a86);
  a9=(a19*a9);
  a12=(a12+a9);
  a12=(a89*a12);
  a12=(a12*a67);
  a9=casadi_sq(a115);
  a9=(a19*a9);
  a9=(a122+a9);
  a9=(a89*a9);
  a9=(a9*a103);
  a12=(a12+a9);
  a9=(a115*a41);
  a9=(a19*a9);
  a45=(a19*a86);
  a9=(a9-a45);
  a9=(a89*a9);
  a9=(a9*a68);
  a12=(a12+a9);
  a12=(a1*a12);
  a9=(a35+a12);
  a45=casadi_sq(a9);
  a77=(a77+a45);
  a45=(a41*a86);
  a45=(a19*a45);
  a5=(a19*a115);
  a45=(a45-a5);
  a45=(a89*a45);
  a45=(a45*a67);
  a67=(a19*a86);
  a5=(a41*a115);
  a5=(a19*a5);
  a67=(a67+a5);
  a67=(a89*a67);
  a67=(a67*a103);
  a45=(a45+a67);
  a67=casadi_sq(a41);
  a67=(a19*a67);
  a122=(a122+a67);
  a122=(a89*a122);
  a122=(a122*a68);
  a45=(a45+a122);
  a45=(a1*a45);
  a122=(a48+a45);
  a68=casadi_sq(a122);
  a77=(a77+a68);
  a77=(a44-a77);
  a68=casadi_sq(a16);
  a68=(a19*a68);
  a68=(a77+a68);
  a68=(a89*a68);
  a68=(a68*a95);
  a67=(a16*a9);
  a67=(a19*a67);
  a103=(a19*a122);
  a67=(a67-a103);
  a67=(a89*a67);
  a67=(a67*a123);
  a68=(a68+a67);
  a67=(a19*a9);
  a103=(a16*a122);
  a103=(a19*a103);
  a67=(a67+a103);
  a67=(a89*a67);
  a67=(a67*a96);
  a68=(a68+a67);
  a68=(a1*a68);
  a47=(a47+a68);
  a47=(a47/a6);
  a47=(a40+a47);
  a68=casadi_sq(a47);
  a109=(a19*a109);
  a83=(a83+a109);
  a12=(a19*a12);
  a83=(a83+a12);
  a12=(a19*a122);
  a109=(a9*a16);
  a109=(a19*a109);
  a12=(a12+a109);
  a12=(a89*a12);
  a12=(a12*a95);
  a109=casadi_sq(a9);
  a109=(a19*a109);
  a109=(a77+a109);
  a109=(a89*a109);
  a109=(a109*a123);
  a12=(a12+a109);
  a109=(a9*a122);
  a109=(a19*a109);
  a67=(a19*a16);
  a109=(a109-a67);
  a109=(a89*a109);
  a109=(a109*a96);
  a12=(a12+a109);
  a12=(a1*a12);
  a83=(a83+a12);
  a83=(a83/a6);
  a83=(a35+a83);
  a12=casadi_sq(a83);
  a68=(a68+a12);
  a114=(a19*a114);
  a90=(a90+a114);
  a45=(a19*a45);
  a90=(a90+a45);
  a45=(a122*a16);
  a45=(a19*a45);
  a114=(a19*a9);
  a45=(a45-a114);
  a45=(a89*a45);
  a45=(a45*a95);
  a114=(a19*a16);
  a12=(a122*a9);
  a12=(a19*a12);
  a114=(a114+a12);
  a114=(a89*a114);
  a114=(a114*a123);
  a45=(a45+a114);
  a114=casadi_sq(a122);
  a114=(a19*a114);
  a77=(a77+a114);
  a89=(a89*a77);
  a89=(a89*a96);
  a45=(a45+a89);
  a45=(a1*a45);
  a90=(a90+a45);
  a90=(a90/a6);
  a90=(a48+a90);
  a45=casadi_sq(a90);
  a68=(a68+a45);
  a68=sqrt(a68);
  a68=(a44<a68);
  a45=casadi_sq(a47);
  a89=casadi_sq(a83);
  a45=(a45+a89);
  a89=casadi_sq(a90);
  a45=(a45+a89);
  a89=(a47/a45);
  a89=(-a89);
  a89=(a68?a89:0);
  a77=(!a68);
  a47=(a77?a47:0);
  a89=(a89+a47);
  if (res[0]!=0) res[0][3]=a89;
  a89=(a83/a45);
  a89=(-a89);
  a89=(a68?a89:0);
  a83=(a77?a83:0);
  a89=(a89+a83);
  if (res[0]!=0) res[0][4]=a89;
  a45=(a90/a45);
  a45=(-a45);
  a45=(a68?a45:0);
  a90=(a77?a90:0);
  a45=(a45+a90);
  if (res[0]!=0) res[0][5]=a45;
  a45=arg[0] ? arg[0][6] : 0;
  a90=(!a45);
  a68=(a68?a90:0);
  a77=(a77?a45:0);
  a68=(a68+a77);
  if (res[0]!=0) res[0][6]=a68;
  a46=(a19*a46);
  a27=(a27+a46);
  a99=(a19*a99);
  a27=(a27+a99);
  a107=(a106/a107);
  a107=(a107*a33);
  a107=(-a107);
  a119=(a119?a107:0);
  a111=(a111?a119:0);
  a110=(a110*a120);
  a111=(a111-a110);
  a71=(a108/a71);
  a71=(a71*a100);
  a13=(a13?a71:0);
  a60=(a60?a13:0);
  a49=(a49*a120);
  a60=(a60-a49);
  a111=(a111+a60);
  a106=(a106/a91);
  a106=(a106*a32);
  a124=(a124?a106:0);
  a72=(a72?a124:0);
  a36=(a36*a120);
  a72=(a72-a36);
  a111=(a111+a72);
  a108=(a108/a37);
  a108=(a108*a30);
  a108=(-a108);
  a20=(a20?a108:0);
  a126=(a126?a20:0);
  a43=(a43*a120);
  a126=(a126-a43);
  a111=(a111+a126);
  a111=(a76?a111:0);
  a18=(a18<a118);
  a57=(a18?a57:0);
  a111=(a111+a57);
  a57=(a122*a16);
  a57=(a62*a57);
  a126=casadi_sq(a16);
  a43=casadi_sq(a9);
  a126=(a126+a43);
  a43=casadi_sq(a122);
  a126=(a126+a43);
  a43=(a44-a126);
  a22=(a22*a43);
  a43=(a22*a9);
  a57=(a57-a43);
  a126=(a44+a126);
  a126=casadi_sq(a126);
  a57=(a57/a126);
  a51=(a51+a118);
  a38=(a51*a38);
  a118=(a57*a38);
  a111=(a111+a118);
  a111=(a111/a51);
  a118=(a123*a93);
  a43=(a96*a78);
  a118=(a118-a43);
  a111=(a111-a118);
  a111=(a1*a111);
  a27=(a27+a111);
  a27=(a27/a6);
  a27=(a21+a27);
  if (res[0]!=0) res[0][7]=a27;
  a79=(a19*a79);
  a42=(a42+a79);
  a104=(a19*a104);
  a42=(a42+a104);
  a105=(a76?a105:0);
  a104=(a122*a9);
  a104=(a62*a104);
  a79=(a22*a16);
  a104=(a104+a79);
  a104=(a104/a126);
  a79=(a104*a38);
  a105=(a105+a79);
  a105=(a105/a51);
  a96=(a96*a61);
  a79=(a95*a93);
  a96=(a96-a79);
  a105=(a105-a96);
  a105=(a1*a105);
  a42=(a42+a105);
  a42=(a42/a6);
  a42=(a23+a42);
  if (res[0]!=0) res[0][8]=a42;
  a53=(a19*a53);
  a25=(a25+a53);
  a39=(a19*a39);
  a25=(a25+a39);
  a76=(a76?a98:0);
  a98=casadi_sq(a9);
  a39=casadi_sq(a16);
  a98=(a98+a39);
  a98=(a62*a98);
  a98=(a98/a126);
  a98=(a44-a98);
  a38=(a98*a38);
  a76=(a76+a38);
  a76=(a76/a51);
  a95=(a95*a78);
  a123=(a123*a61);
  a95=(a95-a123);
  a76=(a76-a95);
  a76=(a1*a76);
  a25=(a25+a76);
  a25=(a25/a6);
  a25=(a24+a25);
  if (res[0]!=0) res[0][9]=a25;
  a25=arg[0] ? arg[0][10] : 0;
  a76=casadi_sq(a48);
  a95=casadi_sq(a35);
  a76=(a76+a95);
  a76=(a62*a76);
  a76=(a76/a56);
  a76=(a44-a76);
  a76=(a76*a21);
  a95=(a40*a35);
  a95=(a62*a95);
  a123=(a52*a48);
  a95=(a95-a123);
  a95=(a95/a56);
  a95=(a95*a23);
  a76=(a76+a95);
  a95=(a40*a48);
  a95=(a62*a95);
  a123=(a52*a35);
  a95=(a95+a123);
  a95=(a95/a56);
  a95=(a95*a24);
  a76=(a76+a95);
  a76=(a1*a76);
  a95=casadi_sq(a102);
  a123=casadi_sq(a17);
  a95=(a95+a123);
  a95=(a62*a95);
  a95=(a95/a84);
  a95=(a44-a95);
  a95=(a95*a54);
  a123=(a88*a17);
  a123=(a62*a123);
  a51=(a87*a102);
  a123=(a123-a51);
  a123=(a123/a84);
  a123=(a123*a73);
  a95=(a95+a123);
  a123=(a88*a102);
  a123=(a62*a123);
  a51=(a87*a17);
  a123=(a123+a51);
  a123=(a123/a84);
  a123=(a123*a55);
  a95=(a95+a123);
  a95=(a1*a95);
  a95=(a19*a95);
  a76=(a76+a95);
  a95=casadi_sq(a41);
  a123=casadi_sq(a115);
  a95=(a95+a123);
  a95=(a62*a95);
  a95=(a95/a80);
  a95=(a44-a95);
  a95=(a95*a85);
  a123=(a86*a115);
  a123=(a62*a123);
  a51=(a10*a41);
  a123=(a123-a51);
  a123=(a123/a80);
  a123=(a123*a82);
  a95=(a95+a123);
  a123=(a86*a41);
  a123=(a62*a123);
  a51=(a10*a115);
  a123=(a123+a51);
  a123=(a123/a80);
  a123=(a123*a65);
  a95=(a95+a123);
  a95=(a1*a95);
  a95=(a19*a95);
  a76=(a76+a95);
  a95=casadi_sq(a122);
  a123=casadi_sq(a9);
  a95=(a95+a123);
  a95=(a62*a95);
  a95=(a95/a126);
  a95=(a44-a95);
  a95=(a95*a61);
  a123=(a16*a9);
  a123=(a62*a123);
  a51=(a22*a122);
  a123=(a123-a51);
  a123=(a123/a126);
  a123=(a123*a78);
  a95=(a95+a123);
  a123=(a16*a122);
  a123=(a62*a123);
  a51=(a22*a9);
  a123=(a123+a51);
  a123=(a123/a126);
  a123=(a123*a93);
  a95=(a95+a123);
  a95=(a1*a95);
  a76=(a76+a95);
  a76=(a76/a6);
  a25=(a25+a76);
  if (res[0]!=0) res[0][10]=a25;
  a25=arg[0] ? arg[0][11] : 0;
  a76=(a35*a40);
  a76=(a62*a76);
  a95=(a52*a48);
  a76=(a76+a95);
  a76=(a76/a56);
  a76=(a76*a21);
  a95=casadi_sq(a48);
  a123=casadi_sq(a40);
  a95=(a95+a123);
  a95=(a62*a95);
  a95=(a95/a56);
  a95=(a44-a95);
  a95=(a95*a23);
  a76=(a76+a95);
  a35=(a35*a48);
  a35=(a62*a35);
  a52=(a52*a40);
  a35=(a35-a52);
  a35=(a35/a56);
  a35=(a35*a24);
  a76=(a76+a35);
  a76=(a1*a76);
  a35=(a17*a88);
  a35=(a62*a35);
  a56=(a87*a102);
  a35=(a35+a56);
  a35=(a35/a84);
  a35=(a35*a54);
  a56=casadi_sq(a102);
  a52=casadi_sq(a88);
  a56=(a56+a52);
  a56=(a62*a56);
  a56=(a56/a84);
  a56=(a44-a56);
  a56=(a56*a73);
  a35=(a35+a56);
  a17=(a17*a102);
  a17=(a62*a17);
  a87=(a87*a88);
  a17=(a17-a87);
  a17=(a17/a84);
  a17=(a17*a55);
  a35=(a35+a17);
  a35=(a1*a35);
  a35=(a19*a35);
  a76=(a76+a35);
  a35=(a115*a86);
  a35=(a62*a35);
  a17=(a10*a41);
  a35=(a35+a17);
  a35=(a35/a80);
  a35=(a35*a85);
  a17=casadi_sq(a41);
  a84=casadi_sq(a86);
  a17=(a17+a84);
  a17=(a62*a17);
  a17=(a17/a80);
  a17=(a44-a17);
  a17=(a17*a82);
  a35=(a35+a17);
  a115=(a115*a41);
  a115=(a62*a115);
  a10=(a10*a86);
  a115=(a115-a10);
  a115=(a115/a80);
  a115=(a115*a65);
  a35=(a35+a115);
  a35=(a1*a35);
  a35=(a19*a35);
  a76=(a76+a35);
  a35=(a9*a16);
  a35=(a62*a35);
  a115=(a22*a122);
  a35=(a35+a115);
  a35=(a35/a126);
  a35=(a35*a61);
  a115=casadi_sq(a122);
  a80=casadi_sq(a16);
  a115=(a115+a80);
  a115=(a62*a115);
  a115=(a115/a126);
  a44=(a44-a115);
  a44=(a44*a78);
  a35=(a35+a44);
  a9=(a9*a122);
  a62=(a62*a9);
  a22=(a22*a16);
  a62=(a62-a22);
  a62=(a62/a126);
  a62=(a62*a93);
  a35=(a35+a62);
  a35=(a1*a35);
  a76=(a76+a35);
  a76=(a76/a6);
  a25=(a25+a76);
  if (res[0]!=0) res[0][11]=a25;
  a25=arg[0] ? arg[0][12] : 0;
  a75=(a75*a21);
  a50=(a50*a23);
  a75=(a75+a50);
  a74=(a74*a24);
  a75=(a75+a74);
  a75=(a1*a75);
  a26=(a26*a54);
  a81=(a81*a73);
  a26=(a26+a81);
  a101=(a101*a55);
  a26=(a26+a101);
  a26=(a1*a26);
  a26=(a19*a26);
  a75=(a75+a26);
  a113=(a113*a85);
  a112=(a112*a82);
  a113=(a113+a112);
  a121=(a121*a65);
  a113=(a113+a121);
  a113=(a1*a113);
  a113=(a19*a113);
  a75=(a75+a113);
  a57=(a57*a61);
  a104=(a104*a78);
  a57=(a57+a104);
  a98=(a98*a93);
  a57=(a57+a98);
  a57=(a1*a57);
  a75=(a75+a57);
  a75=(a75/a6);
  a25=(a25+a75);
  if (res[0]!=0) res[0][12]=a25;
  a11=(a19*a11);
  a7=(a7?a11:0);
  a64=(a64+a7);
  a19=(a19*a69);
  a66=(a66?a19:0);
  a64=(a64+a66);
  a63=(a63*a1);
  a18=(a18?a63:0);
  a64=(a64+a18);
  a64=(a64/a6);
  a4=(a4-a64);
  if (res[0]!=0) res[0][13]=a4;
  return 0;
}

CASADI_SYMBOL_EXPORT int predict(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT void predict_incref(void) {
}

CASADI_SYMBOL_EXPORT void predict_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int predict_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int predict_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* predict_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* predict_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* predict_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s3;
    case 4: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* predict_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int predict_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif