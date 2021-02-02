# Setup file for using low_cost_lkdriver.cpp by low_cost_lkdriver.py
#
# Aidan Lethaby
# 22 April 2020

from distutils.core import setup, Extension # Import the setup and extention modules

module = Extension("lk_module", # Name
                   include_dirs = ["/usr/local/include"], # Path to external (non-standard) include files
                   libraries = ["pcanbasic"], # Name of external library to be used
                   library_dirs = ["/usr/local/lib"], # Directory location of external library
                   sources = ["low_cost_lkdriver.cpp"]) # List of source files

setup(name = "lk_package", version = "1.0", description = "Package holding lk_module", ext_modules = [module])