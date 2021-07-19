# -*- coding: utf-8 -*-
"""
Created on Fri Jul  9 10:08:57 2021

@author: Administrator
"""

# setup.py
from setuptools import setup, Extension
import os


os.environ["CC"] = "g++"

ext = Extension(
      'c_extension',
      sources = ['c_extension.c'])

setup(name='c_extension',
       version='1.0',
       description='This is a demo package',
       ext_modules=[ext])