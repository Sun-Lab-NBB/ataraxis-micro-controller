# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'ataraxis-tl-microcontroller'
# noinspection PyShadowingBuiltins
copyright = '2024, Ivan Kondratyev (Inkaros)'
author = 'Ivan Kondratyev (Inkaros)'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'breathe',  # To read doxygen-generated XML files (so that C++ documentation can be parsed)
    'sphinx_rtd_theme',  # To format the documentation HTML using ReadTheDocs format
]

# Breathe configuration
# Specifies the source of the C++ documentation
breathe_projects = {"ataraxis-tl-microcontroller": "./doxygen/xml"}
# Specifies default project name if C++ documentation is not available
breathe_default_project = "ataraxis-tl-microcontroller"

templates_path = ['_templates']
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'  # Directs sphinx to use RTD theme
