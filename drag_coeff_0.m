function out = drag_coeff_0(C_f, FF, S_wet, S_ref)
C_D_0 = C_f * FF * S_wet / S_ref;
out = C_D_0;
end