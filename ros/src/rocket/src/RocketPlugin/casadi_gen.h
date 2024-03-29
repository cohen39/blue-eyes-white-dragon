/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

#include <casadi/mem.h>
int state_from_gz(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem);
void state_from_gz_incref(void);
void state_from_gz_decref(void);
casadi_int state_from_gz_n_out(void);
casadi_int state_from_gz_n_in(void);
const char* state_from_gz_name_in(casadi_int i);
const char* state_from_gz_name_out(casadi_int i);
const casadi_int* state_from_gz_sparsity_in(casadi_int i);
const casadi_int* state_from_gz_sparsity_out(casadi_int i);
int state_from_gz_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
casadi_functions* state_from_gz_functions(void);
int rocket_force_moment(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem);
void rocket_force_moment_incref(void);
void rocket_force_moment_decref(void);
casadi_int rocket_force_moment_n_out(void);
casadi_int rocket_force_moment_n_in(void);
const char* rocket_force_moment_name_in(casadi_int i);
const char* rocket_force_moment_name_out(casadi_int i);
const casadi_int* rocket_force_moment_sparsity_in(casadi_int i);
const casadi_int* rocket_force_moment_sparsity_out(casadi_int i);
int rocket_force_moment_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
casadi_functions* rocket_force_moment_functions(void);
int pitch_ctrl(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem);
void pitch_ctrl_incref(void);
void pitch_ctrl_decref(void);
casadi_int pitch_ctrl_n_out(void);
casadi_int pitch_ctrl_n_in(void);
const char* pitch_ctrl_name_in(casadi_int i);
const char* pitch_ctrl_name_out(casadi_int i);
const casadi_int* pitch_ctrl_sparsity_in(casadi_int i);
const casadi_int* pitch_ctrl_sparsity_out(casadi_int i);
int pitch_ctrl_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
casadi_functions* pitch_ctrl_functions(void);
int rocket_u_to_fin(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, void* mem);
void rocket_u_to_fin_incref(void);
void rocket_u_to_fin_decref(void);
casadi_int rocket_u_to_fin_n_out(void);
casadi_int rocket_u_to_fin_n_in(void);
const char* rocket_u_to_fin_name_in(casadi_int i);
const char* rocket_u_to_fin_name_out(casadi_int i);
const casadi_int* rocket_u_to_fin_sparsity_in(casadi_int i);
const casadi_int* rocket_u_to_fin_sparsity_out(casadi_int i);
int rocket_u_to_fin_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
casadi_functions* rocket_u_to_fin_functions(void);
#ifdef __cplusplus
} /* extern "C" */
#endif
