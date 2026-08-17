[basic]
type = axmodel
model_npu = compiled_320.axmodel
model_vnpu = compiled_320_vnpu.axmodel

[extra]
model_type = yolo11
type = obb
input_type = rgb
labels = ccap, cled, cres
input_cache = true
output_cache = true
input_cache_flush = false
output_cache_inval = true
mean = 0,0,0
scale = 1,1,1
