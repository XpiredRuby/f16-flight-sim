import numpy as np
from scipy.integrate import solve_ivp
from f16sim.controls.autopilot import longitudinal_autopilot, industry_autopilot


def _aero_long(u, w, q, delta_e, c):
    V = np.sqrt(u**2 + w**2)
    alpha = np.arctan2(w, u)
    qbar = 0.5 * c["rho"] * V**2
    CL = c["CL0"] + c["CL_alpha"] * alpha + c["CL_q"] * (q * c["cbar"] / (2 * V)) + c["CL_de"] * delta_e
    CD = c["CD0"] + c["k"] * CL**2
    Cm = c["Cm0"] + c["Cm_alpha"] * alpha + c["Cm_q"] * (q * c["cbar"] / (2 * V)) + c["Cm_de"] * delta_e
    return qbar * c["S"] * CL, qbar * c["S"] * CD, qbar * c["S"] * c["cbar"] * Cm, alpha


def run_6dof(cfg):
    def eq(_, s):
        u,v,w,p,q,r,phi,theta,psi,x,y,z=s
        Lift, Drag, My, alpha = _aero_long(u,w,q,cfg["delta_e"],cfg)
        X = cfg["thrust"] - Drag*np.cos(alpha) + Lift*np.sin(alpha)
        Z = -Lift*np.cos(alpha) - Drag*np.sin(alpha)
        return [X/cfg["mass"] - cfg["g"]*np.sin(theta)-q*w,0.0,Z/cfg["mass"] + cfg["g"]*np.cos(theta)-q*u,0.0,My/cfg["Iy"],0.0,p,q,r,u*np.cos(theta)*np.cos(psi)+w*np.sin(theta),0.0,-u*np.sin(theta)+w*np.cos(theta)]
    return solve_ivp(eq, cfg["t_span"], cfg["state0"], t_eval=cfg["t_eval"], rtol=1e-8, atol=1e-8)


def run_autopilot(cfg):
    def eq(t,s):
        u,w,q,theta,z=s
        altitude=-z
        de=longitudinal_autopilot(theta,q,altitude,cfg)
        Lift,Drag,My,alpha=_aero_long(u,w,q,de,cfg)
        X = cfg["thrust"] - Drag*np.cos(alpha) + Lift*np.sin(alpha)
        Z = -Lift*np.cos(alpha) - Drag*np.sin(alpha)
        return [X/cfg["mass"]-cfg["g"]*np.sin(theta)-q*w,Z/cfg["mass"]+cfg["g"]*np.cos(theta)-q*u,My/cfg["Iy"],q,-u*np.sin(theta)+w*np.cos(theta)]
    return solve_ivp(eq, cfg["t_span"], cfg["state0"], t_eval=cfg["t_eval"], rtol=1e-8, atol=1e-8)


def run_industry(cfg):
    def gust(t): return 2.0*np.sin(0.2*t)
    def aero(u,v,w,p,q,r,de,da,dr):
        V=np.sqrt(u**2+v**2+w**2); alpha=np.arctan2(w,u); beta=np.arcsin(v/V); qbar=0.5*cfg["rho"]*V**2
        CL=cfg["CL0"]+cfg["CL_alpha"]*alpha+cfg["CL_q"]*(q*cfg["cbar"]/(2*V))+cfg["CL_de"]*de
        CD=cfg["CD0"]+cfg["k"]*CL**2
        Cm=cfg["Cm0"]+cfg["Cm_alpha"]*alpha+cfg["Cm_q"]*(q*cfg["cbar"]/(2*V))+cfg["Cm_de"]*de
        CY=cfg["CY_beta"]*beta
        Cl=cfg["Cl_beta"]*beta+cfg["Cl_p"]*(p*cfg["b"]/(2*V))+cfg["Cl_da"]*da
        Cn=cfg["Cn_beta"]*beta+cfg["Cn_r"]*(r*cfg["b"]/(2*V))+cfg["Cn_dr"]*dr
        return qbar*cfg["S"]*CL,qbar*cfg["S"]*CD,qbar*cfg["S"]*CY,qbar*cfg["S"]*cfg["b"]*Cl,qbar*cfg["S"]*cfg["cbar"]*Cm,qbar*cfg["S"]*cfg["b"]*Cn,alpha
    def eq(t,s):
        u,v,w,p,q,r,phi,theta,psi,z=s
        altitude=-z
        de,da,dr=industry_autopilot(phi,p,r,theta,q,altitude,cfg)
        Lift,Drag,Side,L,My,N,alpha=aero(u,v,w+gust(t),p,q,r,de,da,dr)
        X=cfg["thrust"]-Drag*np.cos(alpha)+Lift*np.sin(alpha); Y=Side; Z=-Lift*np.cos(alpha)-Drag*np.sin(alpha)
        return [X/cfg["mass"]-cfg["g"]*np.sin(theta)-q*w+r*v,Y/cfg["mass"]+p*w-r*u,Z/cfg["mass"]+cfg["g"]*np.cos(theta)-p*v+q*u,L/cfg["Ix"],My/cfg["Iy"],N/cfg["Iz"],p,q,r,-u*np.sin(theta)+w*np.cos(theta)]
    return solve_ivp(eq, cfg["t_span"], cfg["state0"], t_eval=cfg["t_eval"], rtol=1e-8, atol=1e-8)
