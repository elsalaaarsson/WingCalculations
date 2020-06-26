function out = drag_coefficient_0(C_f, FF, S_wet, S_ref)
C_0 = C_f * FF * S_wet / S_ref;
out = C_0;
end