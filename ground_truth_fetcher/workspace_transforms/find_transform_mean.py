#usr/bin/env python

"""
Wed Oct 26 17:06:25 EEST 2016, Nikos Koukis

Import a csv files, fetch the values that we are interested in and compute the
mean of each one.

"""

from __future__ import print_function

import pandas as pd
import sys
import os

if len(sys.argv) != 2:
    print("Illegal number of arguments. Exiting...")

fname = sys.argv[1]
assert(os.path.isfile(fname))

trans_mat = pd.read_csv(fname)
prefix = "field.transform."

fields_to_parse = [
    "translation.x",
    "translation.y",
    "translation.z",
    "rotation.x",
    "rotation.y",
    "rotation.z",
    "rotation.w"]

mean_vals = {}
for field in fields_to_parse:
    mean_vals[field] = trans_mat[prefix + field].mean()

results_fname = "results_" + fname.split("/")[-1]
print(results_fname)
with open(results_fname, "w") as results_f:
    for key in mean_vals.keys():
        results_f.write("{}\t".format(key))
    results_f.write("\n")
    for val in mean_vals.values():
        results_f.write("{:.8f}\t".format(val))


