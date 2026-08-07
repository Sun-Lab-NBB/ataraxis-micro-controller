# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'ataraxis-micro-controller'
copyright = '2026, Sun (NeuroAI) lab'
author = 'Ivan Kondratyev, Jasmine Si'
release = '4.0.1'

# -- General configuration ---------------------------------------------------
extensions = [
    'breathe',             # To read doxygen-generated xml files (to parse C++ documentation).
]

# Breathe configuration
breathe_projects = {"ataraxis-micro-controller": "./doxygen/xml"}
breathe_default_project = "ataraxis-micro-controller"

# -- Options for HTML output -------------------------------------------------
html_theme = 'furo'
