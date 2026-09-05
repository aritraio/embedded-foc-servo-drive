# SVPWM Derivation

Reference vector V* = Vα + jVβ, magnitude Vref, angle θ ∈ [0, 360°).

1. Sector k = ⌊θ/60°⌋ + 1 (1..6); local angle φ = θ − (k−1)·60°.
2. Modulation index m = √3·Vref/Vdc.
3. Dwell times: T1 = m·Ts·sin(60°−φ), T2 = m·Ts·sin(φ), T0 = Ts − T1 − T2.
4. Overmodulation: if T1+T2 > Ts, scale T1,T2 by Ts/(T1+T2), T0 = 0.
5. Center-aligned 7-segment on-times (sector 1 shown):
   Ta = T1+T2+T0/2, Tb = T2+T0/2, Tc = T0/2 (÷Ts for duty).
   Remaining sectors permute symmetrically; see `svpwm.c`.

Linear limit |V*| ≤ Vdc/√3 vs SPWM Vdc/2 → gain 2/√3 ≈ +15.47%.
