import numpy
import ctypes

name = "ControlToOrigin_solver"
requires_callback = False
lib = "lib/libControlToOrigin_solver.so"
lib_static = "lib/libControlToOrigin_solver.a"
c_header = "include/ControlToOrigin_solver.h"
nstages = 10

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("minusA_times_x0"     , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("u0"                  , ""      , ""               , ctypes.c_double, numpy.float64,     (  1,),    1)]

# Info Struct Fields
info = \
[('it', ctypes.c_int32),
('it2opt', ctypes.c_int32),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('pobj', ctypes.c_double),
('dobj', ctypes.c_double),
('dgap', ctypes.c_double),
('rdgap', ctypes.c_double),
('gradient_lag_norm', ctypes.c_double),
('mu', ctypes.c_double),
('mu_aff', ctypes.c_double),
('sigma', ctypes.c_double),
('lsit_aff', ctypes.c_int32),
('lsit_cc', ctypes.c_int32),
('step_aff', ctypes.c_double),
('step_cc', ctypes.c_double),
('solvetime', ctypes.c_double)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(3, 2, 0, 0, 3, 3, 0, 0), 
	(3, 2, 0, 0, 3, 3, 0, 0), 
	(3, 2, 0, 0, 3, 3, 0, 0), 
	(3, 2, 0, 0, 3, 3, 0, 0), 
	(3, 2, 0, 0, 3, 3, 0, 0), 
	(3, 2, 0, 0, 3, 3, 0, 0), 
	(3, 2, 0, 0, 3, 3, 0, 0), 
	(3, 2, 0, 0, 3, 3, 0, 0), 
	(3, 2, 0, 0, 3, 3, 0, 0), 
	(3, 2, 0, 0, 3, 3, 0, 0)
]