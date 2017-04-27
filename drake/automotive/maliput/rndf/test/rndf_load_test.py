#!/usr/bin/env python

"""Attempts to load each non-blacklisted *.rndf file from parent directory,
returning a non-zero exit code if any file fails.
"""

import glob
import subprocess
import os
import unittest
import sys

_THIS_FILE = os.path.abspath(__file__)
_THIS_DIR = os.path.dirname(_THIS_FILE)


class TestRNDFLoading(unittest.TestCase):
    RNDF_LOAD = "rndf_load"

    def test_rndf_files(self):
        rndf_dir = os.path.dirname(_THIS_DIR)

        self.assertTrue(os.path.exists(self.RNDF_LOAD),
                        self.RNDF_LOAD + " not found")

        rndf_files = glob.glob(os.path.join(rndf_dir + '/maps', '*.rndf'))
        blacklist = []
        test_rndf_files = [f for f in rndf_files
                           if not any([b in f for b in blacklist])]
        self.assertTrue(len(test_rndf_files) > 0)

        for rf in test_rndf_files:
            subprocess.check_call([self.RNDF_LOAD, "-rndf_file", rf])


if __name__ == '__main__':
    TestRNDFLoading.RNDF_LOAD = sys.argv.pop()
    unittest.main()
