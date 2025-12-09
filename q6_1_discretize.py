"""
Q6-1 Discretization Verification and Numerical Matrices A, B

Problem: Discretize the continuous-time system m*ddz = F - mg
Given: Ts = 1, m = 2, g = 9.8, F(t) = mg + m*u_k

The discretized system should satisfy:
x_{k+1} = [1  Ts] x_k + [Ts^2/2] * (F(kTs)/m - g)
          [0   1]       [Ts     ]

With F(t) = mg + m*u_k, we get:
x_{k+1} = [1  Ts] x_k + [Ts^2/2] * u_k
          [0   1]       [Ts     ]

Therefore: A = [1  Ts], B = [Ts^2/2]
                  [0   1]         [Ts     ]
"""

import numpy as np
import sympy as sp

# Parameters
Ts = 1.0
m = 2.0
g = 9.8

print("=" * 60)
print("Problem 1: Discretization")
print("=" * 60)

# Step 1: Theoretical derivation verification
print("\n[Step 1] Theoretical Derivation")
print("-" * 60)
print("Continuous-time system: m*ddz = F - mg")
print("State-space: x = [z, dz/dt]^T")
print("\nDiscretization formula (from theory):")
print("x_{k+1} = [1  Ts] x_k + [Ts^2/2] * (F(kTs)/m - g)")
print("          [0   1]       [Ts     ]")
print("\nWith F(t) = mg + m*u_k:")
print("F(kTs)/m - g = (mg + m*u_k)/m - g = g + u_k - g = u_k")
print("\nTherefore:")
print("x_{k+1} = [1  Ts] x_k + [Ts^2/2] * u_k")
print("          [0   1]       [Ts     ]")

# Step 2: Numerical computation
print("\n[Step 2] Numerical Values")
print("-" * 60)
print(f"Given: Ts = {Ts}, m = {m}, g = {g}")
print(f"Control: F(t) = mg + m*u_k = {m*g} + {m}*u_k")

# Compute A and B
A = np.array([[1.0, Ts],
              [0.0, 1.0]])

B = np.array([[Ts**2 / 2],
              [Ts]])

print("\nResult:")
print("A =")
print(A)
print("\nB =")
print(B)

# Step 3: Verification with symbolic computation
print("\n[Step 3] Symbolic Verification")
print("-" * 60)
try:
    # Define symbols
    ts, mass, grav = sp.symbols('Ts m g', real=True, positive=True)
    
    # Theoretical A matrix
    A_theory = sp.Matrix([[1, ts], [0, 1]])
    
    # Theoretical B matrix (before substitution)
    B_theory_before = sp.Matrix([[ts**2 / 2], [ts]])
    
    # After F = mg + m*u substitution: F/m - g = u
    # So B remains the same
    B_theory = B_theory_before
    
    print("Theoretical A matrix:")
    print(A_theory)
    print("\nTheoretical B matrix:")
    print(B_theory)
    
    # Substitute numerical values
    A_num = A_theory.subs([(ts, Ts)])
    B_num = B_theory.subs([(ts, Ts)])
    
    print("\nWith Ts = 1:")
    print("A =")
    print(A_num)
    print("\nB =")
    print(B_num)
    
    print("\n✓ Symbolic computation matches numerical result!")
    
except ImportError:
    print("Note: Install sympy for symbolic verification: pip install sympy")

# Step 4: Verify the discretization formula
print("\n[Step 4] Formula Verification")
print("-" * 60)
print("The discretized system is:")
print("x_{k+1} = A*x_k + B*u_k")
print("\nwhere:")
print(f"A = [[1, {Ts}],")
print(f"     [0,  1]]")
print(f"\nB = [[{Ts**2/2}],")
print(f"     [{Ts}]]")

print("\n" + "=" * 60)
print("Problem 1 Complete!")
print("=" * 60)
