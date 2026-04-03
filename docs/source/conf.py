# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'ataraxis-micro-controller'
# noinspection PyShadowingBuiltins
copyright = '2026, Sun (NeuroAI) lab'
authors = ['Ivan Kondratyev', 'Jasmine Si']
release = '3.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'breathe',             # To read doxygen-generated xml files (to parse C++ documentation).
]

# Breathe configuration
breathe_projects = {"ataraxis-micro-controller": "./doxygen/xml"}
breathe_default_project = "ataraxis-micro-controller"
breathe_doxygen_config_options = {
    'ENABLE_PREPROCESSING': 'YES',
    'MACRO_EXPANSION': 'YES',
    'EXPAND_ONLY_PREDEF': 'NO',
    'PREDEFINED': 'PACKED_STRUCT='
}

# -- Options for HTML output -------------------------------------------------
html_theme = 'furo'
