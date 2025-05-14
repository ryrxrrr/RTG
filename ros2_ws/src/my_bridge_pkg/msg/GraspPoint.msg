# GraspPoint.msg
#
# 每个字段改成数组形式，把多目标的同类信息放在同一个数组下标上。
# 例如 grasps第i个目标 => object_label[i], x[i], y[i], ...
#

string[] object_label
float32[] x
float32[] y
float32[] z
float32[] angle
float32[] width
float32[] score
float32[] confidence
string[] frame_id
