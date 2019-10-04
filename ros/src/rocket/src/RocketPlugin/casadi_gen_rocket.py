import casadi as ca
import numpy as np
import sys

sys.path.insert(0, '../../../../../python/pyecca')
from pyecca.lie import so3
from pyecca.util import rk4

def rockt_eval_func(jit=True):
    x = ca.SX.sym('x', 14)
    u = ca.SX.sym('u', 4)
    p = ca.SX.sym('p', 15)

    # State: x
    omega_b = x[0:3]  # inertial angular velocity expressed in body frame
    r_nb = x[3:7]  # modified rodrigues parameters
    v_b = x[7:10]  # inertial velocity expressed in body components
    p_n = x[10:13]  # positon in nav frame
    m_fuel = x[13]  # mass
    
    # Input: u
    m_dot = ca.if_else(m_fuel > 0, u[0], 0)
    aileron = u[1]
    elevator = u[2]
    rudder = u[3]
    
    # Parameters: p
    g = p[0]  # gravity
    Jx = p[1]  # moment of inertia
    Jy = p[2]
    Jz = p[3]
    Jxz = p[4]
    ve = p[5]
    l_fin = p[6]
    CL_alpha = p[7]
    CL0 = p[8]
    CD0 = p[9]
    K = p[10]
    s_fin = p[11]
    rho = p[12]
    m_empty = p[13]
    l_motor = p[14]

    # Calculations
    m = m_empty + m_fuel
    J_b = ca.SX.zeros(3, 3)
    J_b[0, 0] = Jx + m_fuel*l_motor**2
    J_b[1, 1] = Jy + m_fuel*l_motor**2
    J_b[2, 2] = Jz
    J_b[0, 2] = J_b[2, 0] = Jxz
    C_nb = so3.Dcm.from_mrp(r_nb)
    g_n = ca.vertcat(0, 0, g)
    v_n = ca.mtimes(C_nb, v_b)

    # aerodynamics
    VT = ca.norm_2(v_b)
    q = 0.5*rho*VT**2
    fins = {
        'top': {
            'fwd': [1, 0, 0],
            'up': [0, 1, 0],
            'mix': aileron + rudder,
        },
        'left': {
            'fwd': [1, 0, 0],
            'up': [0, 0, -1],
            'mix': aileron + elevator,
        },
        'down': {
            'fwd': [1, 0, 0],
            'up': [0, -1, 0],
            'mix': aileron - rudder,
        },
        'right': {
            'fwd': [1, 0, 0],
            'up': [0, 0, 1],
            'mix': aileron - elevator,
        },
    }

    vel_tol = 1e-3
    FA_b = ca.SX.zeros(3)
    MA_b = ca.SX.zeros(3) 
    for key, data in fins.items():
        fwd = data['fwd']
        up = data['up']
        mix = data['mix']
        U = ca.dot(fwd, v_b)
        W = ca.dot(up, v_b)
        alpha = ca.if_else(
            ca.logic_and(ca.fabs(W) > vel_tol, ca.fabs(U) > vel_tol),
            -ca.atan(W/U), 0)
        rel_wind_dir = ca.if_else(ca.fabs(VT) > vel_tol, v_b/VT, -ca.DM(fwd))
        perp_wind_dir = ca.cross(ca.cross(fwd, up), rel_wind_dir)
        perp_wind_dir = perp_wind_dir/ca.norm_2(perp_wind_dir)
        CL = CL0 + CL_alpha*(alpha + mix)
        CD = CD0 + K*(CL - CL0)**2
        L = CL*q*s_fin
        D = CD*q*s_fin
        FA_b += L*perp_wind_dir - D*rel_wind_dir
        # MA_b += ca.cross(ca.vertcat(-l_fin, 0, 0), FA_b)

    # propulsion
    FP_b = ca.vertcat(m_dot*ve, 0, 0)
    MP_b = ca.vertcat(0, 0, 0)
    
    # force and momental total
    F_b = FA_b + FP_b + ca.mtimes(C_nb.T, g_n)
    M_b = MA_b + MP_b

    # Casadi Functions
    rocket_aero_forces = ca.Function(
        'rocket_aero_forces',[x,u,p],[FA_b],['x','u','p'],['FA_b'])
    rocket_aero_moments = ca.Function(
        'rocket_aero_moments',[x,u,p],[MA_b],['x','u','p'],['MA_b'])
    rocket_prop_forces = ca.Function(
        'rocket_prop_forces',[x,u,p],[FP_b],['x','u','p'],['FP_b'])
    rocket_prop_moment = ca.Function(
        'rocket_prop_moments',[x,u,p],[MP_b],['x','u','p'],['MP_b'])
    rocket_functions = [rocket_aero_forces, rocket_aero_moments,
            rocket_prop_forces, rocket_prop_moment]
    return rocket_functions


def pyecca_so3_quat(jit=True):
    q = ca.SX.sym('q', 4)
    mrp = so3.Mrp.from_quat(q)
    dcm = so3.Dcm.from_quat(q)
    quat2mrp = ca.Function('quat2mrp',[q],[mrp],['q'],['mrp'])
    quat2dcm = ca.Function('quat2dcm',[q],[dcm],['q'],['dcm'])

    v_e = ca.SX.sym('v_e',3)
    v_b = ca.SX.sym('v_b',3)
    omega_e = ca.SX.sym('omega_e',3)
    omega_b = ca.SX.sym('omega_b',3)
    v_b = ca.mtimes(dcm,v_e)
    omega_b = ca.mtimes(dcm,omega_e)

    tf_linvel = ca.Function('tf_linvel',[v_e,q],[v_b],['v_e','q'],['v_b'])
    tf_angvel = ca.Function('tf_angvel',[omega_e,q],[omega_b],['omega_e','q'],['omega_b'])
    return [quat2mrp,quat2dcm,tf_linvel,tf_angvel]

def ENU_state_2_NED_state(jit = True):
    x_ENU = ca.SX.sym('x_ENU',14)

     # State: x
    omega_neu = x_ENU[0:3]  # inertial angular velocity returned by gazebo
    q_enu_trf = x_ENU[3:7]  # quaternions returned by gazebo
    v_enu = x_ENU[7:10]  # inertial velocity expressed in body components
    p_enu = x_ENU[10:13]  # positon in nav frame

    m_fuel = x_ENU[13]  # mass, not changed
    
    C_ned_enu = np.array([[0,1,0],[1,0,0],[0,0,-1]]) # dcm from ENU 2 NED, world frame
    C_trf_frb = np.array([[0,0,-1],[0,1,0],[1,0,0]]) # dcm from FRB 2 TRF, body frame
    C_neu_trf = so3.Dcm.from_quat(q_enu_trf) # dcm from TRF 2 NEU

    C_ned_frb = ca.mtimes(C_ned_enu,ca.mtimes(C_neu_trf,C_trf_frb)) # dcm from FRB 2 NED

    r_ned_frb = so3.Mrp.from_dcm(C_ned_frb)
    v_frb = ca.mtimes(C_ned_frb,v_enu)
    omega_frb = ca.mtimes(C_ned_frb,omega_neu)
    p_ned = ca.mtimes(C_ned_enu,p_enu)

    x_NED = ca.vertcat(omega_frb,r_ned_frb,v_frb,p_ned,m_fuel)

    state_ENU2NED = ca.Function('state_ENU2NED',[x_ENU],[x_NED],['x_ENU'],['x_NED'])

    return state_ENU2NED


functions = rockt_eval_func()
#so3_quat_func = pyecca_so3_quat()
state_tf = ENU_state_2_NED_state()

functions.append(state_tf)
print(len(functions))
gen = ca.CodeGenerator('casadi_gen_rocket.c', {'main': False, 'mex': False, 'with_header': True, 'with_mem': True})
for i in functions:
    gen.add(i)
gen.generate()

